# mw_plc_sim — a PLC for testing the Mowito robot PC

Plays the line PLC against `mw_profinet_bridge` over real PROFINET. It acts as
the IO-Controller and runs the PLC programme's side of every handshake: start
and stop, fault reset, and the nutrunner request, tightening and ack. The
robot side runs unmodified (p-net, the bridge, the state machine), so what
passes here is the code path the cell runs.

It builds on this repo: the GSDML parser (`helper/gsdml_parser.py`), the DCP
builders (`messages/sim_pnio_dcp.py`), and the connection sequence and frame
layout of `pnio_connection/connection.py`. `rpc.py` redoes the CM messages on
scapy 2.5, because `messages/sim_pnio_cm.py` targets a pre-2.5 scapy
(`scapy.contrib.dce_rpc`) that no released version ships.

## Setup

```bash
python3 -m venv .venv && .venv/bin/pip install -r mw_plc_sim/requirements.txt
```

## Loopback test: PLC and robot PC on one machine, no sudo

```bash
mw_plc_sim/loopback_test.sh --cycles 3                 # exit 0 when all 3 cycles are DONE
mw_plc_sim/loopback_test.sh --cycles 2 --torque-fail   # fault path: servo_fault, fault reset
mw_plc_sim/loopback_test.sh                            # interactive: start / stop / reset / torque ok|fail / status / quit
```

The script sets up two machines and a cable:

- **PLC:** a network namespace running `mw_plc_sim`.
- **Robot PC:** a second namespace running the bridge (`transport:=pnet`) and
  `mw_fuse_insertion_sm` with `plc_single_cycle:=true` and
  `fixtures/robot_one_fasten.xml`. That tree fires the nutrunner through the
  PLC once per cycle.
- **Cable:** a veth pair between them.

It all runs inside an unprivileged user namespace, so it needs no root. It
is also its own PID namespace, so nothing outlives a run.

Robot-side logs are kept under `/tmp/mw-loopback.*`.

| env | |
|---|---|
| `ROS_SETUP` | setup files for the robot side, colon-separated. Default: `~/mw_ws/install/setup.bash:~/fuse_insetion_ws/install/setup.bash` |
| `ROBOT=bridge` | bridge only, no state machine |
| `CAPTURE=1` | pcap of the robot side of the cable |
| `BRIDGE_WRAP`, `BRIDGE_PRELOAD` | run the bridge under strace or gdb, or preload `libasan.so` for a sanitizer build |

**One harness-only workaround.** A veth reports 10 Gbit/s, and p-net v0.2.0
refuses it at PrmEnd ("no local Ethernet port with high enough speed").
`fake_link_speed.c` is preloaded into the bridge in this test only and
reports 1 Gbit/s for `pn-dev`. A real 100M or 1G port never needs it.

## Against a real robot PC

```bash
sudo .venv/bin/python -m mw_plc_sim --iface <nic cabled to the robot> \
    --gsdml <mw_profinet_bridge>/share/mw_profinet_bridge/gsdml/GSDML-*.xml --cycles 3
```

Raw sockets need root or CAP_NET_RAW. Give the PLC NIC an address in the same
subnet as `--device-ip` (default 192.168.0.50).

Other options:

| option | |
|---|---|
| `--cycle-ms` | bus cycle: a power of two from 1 to 512 |
| `--watchdog` | watchdog factor, in cycles |
| `--fasten-s` | simulated tightening time: seconds until the set torque is reached (default 2.0) |
| `--layout file.json` | override I/O offsets, using the bridge's names without `_bit`/`_byte`, e.g. `{"stop_cycle": "0.6"}` |

## Files

| | |
|---|---|
| `__main__.py` | CLI: scripted `--cycles N` (exit code 0/1) or interactive HMI |
| `controller.py` | IO-Controller: DCP Identify and Set IP, Connect, PrmEnd, ApplicationReady, cyclic exchange |
| `rpc.py` | DCE/RPC PNIO messages (scapy 2.5) |
| `plc_program.py` | the PLC programme: command holds, nutrunner "result first, ack last" |
| `layout.py` | I/O offsets, matching the bridge's `IoMap` and the GSDML |
| `loopback_test.sh`, `fake_link_speed.c`, `fixtures/` | the loopback test |
