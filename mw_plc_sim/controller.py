"""A minimal PROFINET IO-Controller: find the device, give it an IP, connect,
parametrise, answer ApplicationReady, then exchange cyclic data.

Sequence and frame layout follow the repo's pnio_connection/connection.py;
the differences are what p-net actually requires or what made that one
Windows-bound: the interface is a parameter everywhere (it hardcoded
"Ethernet" for the cyclic sender), cyclic frames go out with sendto() at the
negotiated cycle instead of a blocking srp() once a second, the output CR's
FrameID comes from the device's Connect response, and ApplicationReady is
answered from a real UDP server socket on 34964.

Needs raw sockets (root or CAP_NET_RAW), like any PROFINET stack.
"""
import fcntl
import socket
import struct
import threading
import time

from scapy.all import load_contrib, srp

load_contrib("pnio")
load_contrib("pnio_dcp")
from scapy.contrib.pnio_dcp import ProfinetDCP  # noqa: E402

from messages.sim_pnio_dcp import get_ident_msg, get_set_ip_msg  # the repo's DCP builders  # noqa: E402
from helper.gsdml_parser import XMLDevice  # noqa: E402

from . import rpc  # noqa: E402

ETH_P_ALL = 0x0003
ETH_P_PNIO = 0x8892
IOXS_GOOD = 0x80
# Primary | valid | run | no problem -- what a PLC in RUN puts on its outputs.
DATA_STATUS_RUN = 0x35


def iface_mac(iface):
    # ioctl, not /sys/class/net: sysfs shows the namespace it was mounted in,
    # which inside a fresh network namespace is the wrong one.
    SIOCGIFHWADDR = 0x8927
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            info = fcntl.ioctl(s.fileno(), SIOCGIFHWADDR, struct.pack("256s", iface.encode()[:15]))
        except OSError:
            raise RuntimeError(f"no network interface '{iface}'")
    return ":".join(f"{b:02x}" for b in info[18:24])


def _ident(value):
    v = value[0] if isinstance(value, tuple) else value
    return int(v, 0)


class PnController:
    def __init__(self, iface, station_name, device_ip, gsdml_path, netmask="255.255.255.0",
                 cycle_ms=32, watchdog_factor=10, name="mw-plc-sim", log=print):
        if cycle_ms not in (1, 2, 4, 8, 16, 32, 64, 128, 256, 512):
            raise ValueError("cycle_ms must be a power of two, 1..512 (the GSDML's reduction ratios)")
        self.iface, self.station_name, self.device_ip = iface, station_name, device_ip
        self.netmask, self.cycle_ms, self.watchdog_factor, self.name = netmask, cycle_ms, watchdog_factor, name
        self.log = log
        self.device = XMLDevice(gsdml_path)
        ident = self.device.body.device_identity
        self.vendor_id, self.device_id = _ident(ident.vendor_id), _ident(ident.device_id)
        self.mac = iface_mac(iface)
        self.device_mac = None
        self.ar_uuid = rpc.new_uuid()
        self.session_key = 1
        self.frame = None
        self.in_frame_id = 0x8001
        self.out_frame_id = None

        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._out_image = None          # module output data (PLC -> device)
        self._in_image = None           # module input data (device -> PLC)
        self._in_valid = False
        self._last_rx = 0.0
        self.on_cycle = None            # callable(in_image | None) -> out_image, runs every cycle
        self.stats = {"tx": 0, "rx": 0, "rx_other_frame_id": 0, "last_iops": None, "last_status": None,
                      "seen": {}}  # (src, frame id) -> count, for every PNIO frame on the wire
        self._threads = []

    # ── bring-up ─────────────────────────────────────────────────────────────

    def identify(self, timeout=30.0):
        """DCP Identify by station name; the answer's source MAC is the device."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ans, _ = srp(get_ident_msg(src=self.mac, name_of_station=self.station_name),
                         iface=self.iface, timeout=1, multi=True, verbose=False)
            for _, rsp in ans:
                if rsp.haslayer(ProfinetDCP) and rsp.src.lower() != self.mac.lower():
                    self.device_mac = rsp.src
                    self.log(f"DCP: '{self.station_name}' is {self.device_mac}")
                    return self.device_mac
        raise TimeoutError(f"no device named '{self.station_name}' answered DCP Identify on {self.iface}")

    def set_ip(self):
        ans, _ = srp(get_set_ip_msg(src=self.mac, dst=self.device_mac, ip=self.device_ip,
                                    netmask=self.netmask),
                     iface=self.iface, timeout=2, multi=True, verbose=False)
        for _, rsp in ans:
            if rsp.haslayer(ProfinetDCP) and rsp[ProfinetDCP].service_id == 0x04 \
                    and rsp[ProfinetDCP].service_type == 0x01:
                self.log(f"DCP: device IP set to {self.device_ip}")
                return
        raise RuntimeError("device did not confirm DCP Set IP")

    def connect(self, timeout=20.0):
        """Connect -> start cyclic -> PrmEnd -> answer ApplicationReady."""
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("", rpc.PNIO_RPC_PORT))   # the device sends ApplicationReady here
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.bind(("", 0))
        client.settimeout(2.0)
        act_id = rpc.new_uuid()

        req, self.frame = rpc.connect_request(
            device=self.device, ar_uuid=self.ar_uuid, session_key=self.session_key,
            controller_mac=self.mac, controller_name=self.name, vendor_id=self.vendor_id,
            device_id=self.device_id, reduction_ratio=self.cycle_ms,
            watchdog_factor=self.watchdog_factor, act_id=act_id)
        rsp = self._call(client, req, timeout, "Connect")
        pdu = rsp.payload
        blocks = getattr(pdu, "blocks", [])
        if pdu.status != 0:
            raise RuntimeError(f"Connect refused, PNIO status 0x{pdu.status:08x}")
        for block in blocks:
            name = type(block).__name__
            if name == "IOCRBlockRes" and block.IOCRType == 2:
                self.out_frame_id = block.FrameID
            elif name == "IOCRBlockRes" and block.IOCRType == 1:
                self.in_frame_id = block.FrameID
            elif name == "ModuleDiffBlock":
                raise RuntimeError("device reports a module difference -- GSDML and device disagree")
        if self.out_frame_id is None:
            raise RuntimeError(f"Connect response has no output IOCR block: {[type(b).__name__ for b in blocks]}")
        self.log(f"CM: connected (input frame 0x{self.in_frame_id:04x}, output frame 0x{self.out_frame_id:04x}, "
                 f"cycle {self.cycle_ms} ms, watchdog x{self.watchdog_factor})")

        self._start_cyclic()

        prm = rpc.prm_end_request(ar_uuid=self.ar_uuid, session_key=self.session_key,
                                  vendor_id=self.vendor_id, device_id=self.device_id,
                                  act_id=act_id, seqnum=1)
        rsp = self._call(client, prm, 5.0, "PrmEnd")
        if rsp.payload.status != 0:
            raise RuntimeError(f"PrmEnd refused, PNIO status 0x{rsp.payload.status:08x}")
        self.log("CM: parameter end")

        server.settimeout(timeout)
        while True:
            try:
                data, addr = server.recvfrom(4096)
            except socket.timeout:
                raise TimeoutError("device never sent ApplicationReady")
            msg = rpc.parse(data)
            block = next((b for b in getattr(msg.payload, "blocks", [])
                          if type(b).__name__ == "IODControlReq"), None)
            if block is not None and block.ControlCommand_ApplicationReady:
                server.sendto(rpc.application_ready_response(
                    msg, ar_uuid=self.ar_uuid, session_key=self.session_key), addr)
                self.log("CM: application ready -- data exchange running")
                break
        server.close()
        client.close()

    def _call(self, sock, request, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            sock.sendto(request, (self.device_ip, rpc.PNIO_RPC_PORT))
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                continue
            msg = rpc.parse(data)
            if msg.ptype == 2:
                return msg
        raise TimeoutError(f"no answer to {what} from {self.device_ip}")

    # ── cyclic data ──────────────────────────────────────────────────────────

    def _start_cyclic(self):
        mod = [m for m in self.device.body.dap_list[0].usable_modules if m.used_in_slots][0]
        self._out_image = bytearray(mod.output_length)
        self._in_len = mod.input_length
        for target in (self._rx_loop, self._tx_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)

    def _tx_loop(self):
        sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_PNIO))
        sock.bind((self.iface, 0))
        header = bytes.fromhex(self.device_mac.replace(":", "")) + bytes.fromhex(self.mac.replace(":", "")) \
            + struct.pack(">HH", ETH_P_PNIO, self.out_frame_id)
        f = self.frame
        counter = 0
        period = self.cycle_ms / 1000.0
        next_t = time.monotonic()
        while not self._stop.is_set():
            if self.on_cycle is not None:
                with self._lock:
                    image = self._in_image if self._in_valid else None
                out = self.on_cycle(image)
                with self._lock:
                    self._out_image[:] = out
            c_sdu = bytearray(f.out_data_len)
            for off in f.out_iocs:
                c_sdu[off] = IOXS_GOOD
            with self._lock:
                c_sdu[f.out_data:f.out_data + len(self._out_image)] = self._out_image
            c_sdu[f.out_data_iops] = IOXS_GOOD
            # Cycle counter runs in 31.25 us units: send clock (32) x reduction ratio per frame.
            counter = (counter + 32 * self.cycle_ms) & 0xFFFF
            sock.send(header + bytes(c_sdu) + struct.pack(">HBB", counter, DATA_STATUS_RUN, 0))
            self.stats["tx"] += 1
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))
        sock.close()

    def _rx_loop(self):
        sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_ALL))
        sock.bind((self.iface, 0))
        sock.settimeout(0.2)
        dev = bytes.fromhex(self.device_mac.replace(":", ""))
        f = self.frame
        while not self._stop.is_set():
            try:
                pkt = sock.recv(1600)
            except socket.timeout:
                continue
            off = 12
            ethertype = struct.unpack(">H", pkt[off:off + 2])[0]
            if ethertype == 0x8100:                     # VLAN-tagged RT frame
                off += 4
                ethertype = struct.unpack(">H", pkt[off:off + 2])[0]
            if ethertype != ETH_P_PNIO:
                continue
            key = (pkt[6:12].hex(":"), f"0x{struct.unpack('>H', pkt[off + 2:off + 4])[0]:04x}", len(pkt))
            seen = self.stats["seen"]
            if key in seen or len(seen) < 16:
                seen[key] = seen.get(key, 0) + 1
            if pkt[6:12] != dev:
                continue
            if struct.unpack(">H", pkt[off + 2:off + 4])[0] != self.in_frame_id:
                self.stats["rx_other_frame_id"] += 1
                continue
            c_sdu = pkt[off + 4:off + 4 + f.in_data_len]
            status = pkt[off + 4 + f.in_data_len + 2] if len(pkt) > off + 4 + f.in_data_len + 2 else 0
            valid = bool(c_sdu[f.in_data_iops] & IOXS_GOOD) and bool(status & 0x04)  # IOPS good, DataValid
            self.stats["rx"] += 1
            self.stats["last_iops"] = c_sdu[f.in_data_iops]
            self.stats["last_status"] = status
            with self._lock:
                self._in_image = bytes(c_sdu[f.in_data:f.in_data + self._in_len])
                self._in_valid = valid
                self._last_rx = time.monotonic()
        sock.close()

    @property
    def link_ok(self):
        """Device frames arriving within the watchdog time, with valid data."""
        with self._lock:
            fresh = time.monotonic() - self._last_rx < self.cycle_ms * self.watchdog_factor / 1000.0
            return fresh and self._in_valid

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
