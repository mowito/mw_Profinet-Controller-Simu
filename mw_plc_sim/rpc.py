"""PROFINET CM (DCE/RPC) messages the IO-Controller side needs, on scapy 2.5.

The repo's messages/sim_pnio_cm.py builds the same blocks against a pre-2.5
scapy (contrib.dce_rpc) that no released version ships any more. This keeps
its structure -- same blocks, same frame-offset scheme -- on scapy 2.5's
DceRpc4, and returns the frame layout it asked for, so the cyclic frames are
built from exactly what the device agreed to rather than a second copy.
"""
import uuid
from dataclasses import dataclass

from scapy.all import load_contrib, raw
from scapy.layers.dcerpc import DceRpc4

load_contrib("pnio")
load_contrib("pnio_rpc")
from scapy.contrib.pnio_rpc import (  # noqa: E402
    AlarmCRBlockReq,
    ARBlockReq,
    ExpectedSubmodule,
    ExpectedSubmoduleAPI,
    ExpectedSubmoduleBlockReq,
    ExpectedSubmoduleDataDescription,
    IOCRAPI,
    IOCRAPIObject,
    IOCRBlockReq,
    IODControlReq,
    IODControlRes,
    PNIOServiceReqPDU,
    PNIOServiceResPDU,
)

UUID_IO_DEVICE_IF = "dea00001-6c97-11d1-8271-00a02442df7d"
UUID_IO_CONTROLLER_IF = "dea00002-6c97-11d1-8271-00a02442df7d"
OPNUM_CONNECT, OPNUM_CONTROL = 0, 4
PNIO_RPC_PORT = 34964
MIN_C_SDU = 40  # RT frames carry at least 40 bytes of C_SDU; shorter data is padded


def object_uuid(vendor_id, device_id, instance=1):
    """PNIO object UUID: dea00000-6c97-11d1-8271-<instance><device><vendor>."""
    return f"dea00000-6c97-11d1-8271-{instance:04x}{device_id:04x}{vendor_id:04x}"


@dataclass
class FrameLayout:
    """Where things sit in the two C_SDUs, as requested in the Connect."""
    in_data_len: int      # device -> controller C_SDU length (DataLength of the input CR)
    out_data_len: int     # controller -> device C_SDU length
    in_iops: list         # offsets of the DAP IOPS in the input CR
    in_data: int          # offset of the module's input data (device -> PLC)
    in_data_iops: int
    in_iocs: int          # device's consumer status for our output data
    out_iocs: list        # our consumer status for the DAP + module input
    out_data: int         # offset of the module's output data (PLC -> device)
    out_data_iops: int


def _dce(opnum, act_id, obj, seqnum, if_id=UUID_IO_DEVICE_IF, ptype=0):
    # flags1 0x20 idempotent | 0x08 no fragment ack, as the repo sends; little endian.
    return DceRpc4(ptype=ptype, flags1=0x28 if ptype == 0 else 0x0A, endian=1, opnum=opnum,
                   object=obj, if_id=if_id, act_id=act_id, seqnum=seqnum)


def connect_request(*, device, ar_uuid, session_key, controller_mac, controller_name,
                    vendor_id, device_id, reduction_ratio, watchdog_factor, act_id, seqnum=0):
    """-> (bytes, FrameLayout). `device` is a helper.gsdml_parser.XMLDevice."""
    dap = device.body.dap_list[0]
    interface = dap.interface_submodule_item.subslot_ident_number
    port = dap.port_submodule_item.subslot_ident_number
    modules = [m for m in dap.usable_modules if m.used_in_slots != ""]
    if len(modules) != 1:
        raise ValueError(f"expected exactly one I/O module in the GSDML, found {len(modules)}")
    mod = modules[0]
    slot = int(mod.used_in_slots)

    # Input CR (device -> controller): DAP IOPS, module data + IOPS, IOCS for our output.
    in_iops = [0, 1, 2]
    in_data = 3
    in_data_iops = in_data + mod.input_length
    in_iocs = in_data_iops + 1
    in_len = max(MIN_C_SDU, in_iocs + 1)
    # Output CR (controller -> device): IOCS for DAP + module input, module data + IOPS.
    out_iocs = [0, 1, 2, 3]
    out_data = 4
    out_data_iops = out_data + mod.output_length
    out_len = max(MIN_C_SDU, out_data_iops + 1)

    dap_objects = [
        IOCRAPIObject(SlotNumber=0, SubslotNumber=0x1, FrameOffset=0),
        IOCRAPIObject(SlotNumber=0, SubslotNumber=interface, FrameOffset=1),
        IOCRAPIObject(SlotNumber=0, SubslotNumber=port, FrameOffset=2),
    ]
    timing = dict(IOCRProperties_RTClass=0x2, SendClockFactor=32, ReductionRatio=reduction_ratio,
                  Phase=1, WatchdogFactor=watchdog_factor, DataHoldFactor=watchdog_factor)
    input_cr = IOCRBlockReq(
        IOCRType=0x1, IOCRReference=1, FrameID=0x8001, DataLength=in_len, **timing,
        APIs=[IOCRAPI(
            IODataObjects=dap_objects + [
                IOCRAPIObject(SlotNumber=slot, SubslotNumber=1, FrameOffset=in_data)],
            IOCSs=[IOCRAPIObject(SlotNumber=slot, SubslotNumber=1, FrameOffset=in_iocs)],
        )],
    )
    output_cr = IOCRBlockReq(
        IOCRType=0x2, IOCRReference=2, FrameID=0xFFFF, DataLength=out_len, **timing,
        APIs=[IOCRAPI(
            IODataObjects=[IOCRAPIObject(SlotNumber=slot, SubslotNumber=1, FrameOffset=out_data)],
            IOCSs=[IOCRAPIObject(SlotNumber=o.SlotNumber, SubslotNumber=o.SubslotNumber,
                                 FrameOffset=o.FrameOffset) for o in dap_objects]
            + [IOCRAPIObject(SlotNumber=slot, SubslotNumber=1, FrameOffset=3)],
        )],
    )

    def no_io():
        return [ExpectedSubmoduleDataDescription(DataDescription=1, SubmoduleDataLength=0,
                                                 LengthIOCS=1, LengthIOPS=1)]

    expected_dap = ExpectedSubmoduleBlockReq(APIs=[ExpectedSubmoduleAPI(
        SlotNumber=0, ModuleIdentNumber=0x1,
        Submodules=[
            ExpectedSubmodule(SubslotNumber=0x1, SubmoduleIdentNumber=0x1,
                              SubmoduleProperties_Type="NO_IO", DataDescription=no_io()),
            ExpectedSubmodule(SubslotNumber=interface, SubmoduleIdentNumber=interface,
                              SubmoduleProperties_Type="NO_IO", DataDescription=no_io()),
            ExpectedSubmodule(SubslotNumber=port, SubmoduleIdentNumber=port,
                              SubmoduleProperties_Type="NO_IO", DataDescription=no_io()),
        ])])
    expected_io = ExpectedSubmoduleBlockReq(APIs=[ExpectedSubmoduleAPI(
        SlotNumber=slot, ModuleIdentNumber=mod.module_ident_number,
        Submodules=[ExpectedSubmodule(
            SubslotNumber=0x1, SubmoduleIdentNumber=mod.submodule_ident_number,
            SubmoduleProperties_Type="INPUT_OUTPUT",
            DataDescription=[
                ExpectedSubmoduleDataDescription(DataDescription=1, SubmoduleDataLength=mod.input_length,
                                                 LengthIOCS=1, LengthIOPS=1),
                ExpectedSubmoduleDataDescription(DataDescription=2, SubmoduleDataLength=mod.output_length,
                                                 LengthIOCS=1, LengthIOPS=1),
            ])])])

    ar = ARBlockReq(
        ARType=1, ARUUID=ar_uuid, SessionKey=session_key, CMInitiatorMacAdd=controller_mac,
        CMInitiatorObjectUUID=object_uuid(vendor_id, device_id),
        ARProperties_State=1, ARProperties_ParametrizationServer=1, ARProperties_StartupMode=1,
        CMInitiatorActivityTimeoutFactor=1000, CMInitiatorUDPRTPort=0x8892,
        CMInitiatorStationName=controller_name.encode(),
    )
    pdu = PNIOServiceReqPDU(args_max=16696, blocks=[
        ar, input_cr, output_cr, AlarmCRBlockReq(), expected_dap, expected_io])
    frame = FrameLayout(in_len, out_len, in_iops, in_data, in_data_iops, in_iocs,
                        out_iocs, out_data, out_data_iops)
    msg = _dce(OPNUM_CONNECT, act_id, object_uuid(vendor_id, device_id), seqnum) / pdu
    return raw(msg), frame


def prm_end_request(*, ar_uuid, session_key, vendor_id, device_id, act_id, seqnum):
    pdu = PNIOServiceReqPDU(args_max=16696, blocks=[IODControlReq(
        block_type=0x0110, ARUUID=ar_uuid, SessionKey=session_key, ControlCommand_PrmEnd=1)])
    return raw(_dce(OPNUM_CONTROL, act_id, object_uuid(vendor_id, device_id), seqnum) / pdu)


def application_ready_response(request, *, ar_uuid, session_key):
    """Answer the device's CControl(ApplicationReady) with the request's own ids."""
    pdu = PNIOServiceResPDU(status=0, blocks=[IODControlRes(
        block_type=0x8112, ARUUID=ar_uuid, SessionKey=session_key, ControlCommand_Done=1)])
    rsp = DceRpc4(ptype=2, flags1=0x0A, endian=request.endian, opnum=request.opnum,
                  object=request.object, if_id=request.if_id, act_id=request.act_id,
                  seqnum=request.seqnum, server_boot=request.server_boot) / pdu
    return raw(rsp)


def parse(data):
    return DceRpc4(data)


def new_uuid():
    return str(uuid.uuid4())
