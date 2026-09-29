"""PLC side of the Mowito robot PC PROFINET link, for testing without a PLC.

  python -m mw_plc_sim --iface eth1 --gsdml GSDML-...xml               # interactive HMI
  python -m mw_plc_sim --iface eth1 --gsdml GSDML-...xml --cycles 3    # run 3 cycles, exit 0/1

Interactive commands:  start | stop | reset | torque ok | torque fail |
                       fasten <s> | status | quit
"""
import argparse
import sys
import time
from pathlib import Path

# The repo's own modules (messages/, helper/) live one level up.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from .controller import PnController  # noqa: E402
from .layout import Layout  # noqa: E402
from .plc_program import PlcProgram  # noqa: E402


def log(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def run_cycles(plc, ctl, n, timeout):
    """One PLC-driven cycle per iteration: wait ready, start, wait done/fault."""
    results = []
    for i in range(1, n + 1):
        if not plc.wait_for(lambda r: r.servo_ready or r.servo_fault, timeout):
            log(f"cycle {i}: robot never became ready (robot {plc.robot.short() if plc.robot else 'no valid data'}; "
                f"link {ctl.stats})")
            return results, False
        if plc.robot.servo_fault:
            log(f"cycle {i}: robot is faulted -- fault reset first")
            plc.press_fault_reset()
            if not plc.wait_for(lambda r: r.servo_ready, 10.0):
                log(f"cycle {i}: fault did not clear")
                return results, False
        tightenings_before = len(plc.tightenings)
        plc.press_start()
        if not plc.wait_for(lambda r: r.servo_busy, 10.0):
            log(f"cycle {i}: robot did not go busy after start")
            return results, False
        end = plc.wait_for(lambda r: not r.servo_busy and (r.cycle_done or r.servo_fault), timeout)
        if end is None:
            log(f"cycle {i}: no cycle_done / servo_fault within {timeout} s")
            return results, False
        done = end.cycle_done and not end.servo_fault
        results.append((i, done, plc.tightenings[tightenings_before:]))
        log(f"cycle {i}: {'DONE' if done else 'FAULT'}, tightenings {plc.tightenings[tightenings_before:]}")
        if not ctl.link_ok:
            log("link lost")
            return results, False
    return results, True


def repl(plc, ctl):
    for line in sys.stdin:
        cmd = line.strip().split()
        if not cmd:
            continue
        if cmd[0] == "start":
            plc.press_start()
        elif cmd[0] == "stop":
            plc.press_stop()
        elif cmd[0] == "reset":
            plc.press_fault_reset()
        elif cmd[0] == "torque" and len(cmd) == 2 and cmd[1] in ("ok", "fail"):
            plc.set_torque_result(cmd[1] == "ok")
        elif cmd[0] == "fasten" and len(cmd) == 2:
            plc.fasten_s = float(cmd[1])
        elif cmd[0] == "status":
            log(f"link {'OK' if ctl.link_ok else 'DOWN'} {ctl.stats}; robot {plc.robot.short() if plc.robot else '?'}")
        elif cmd[0] in ("quit", "exit"):
            return
        else:
            log(__doc__.split("Interactive commands:")[1].strip())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", required=True, help="NIC cabled to the robot PC")
    ap.add_argument("--gsdml", required=True, help="the robot PC's GSDML (mw_profinet_bridge/gsdml/)")
    ap.add_argument("--station", default="mw-robot-pc", help="device name, as in the bridge's station_name")
    ap.add_argument("--device-ip", default="192.168.0.50", help="IP assigned to the robot PC over DCP")
    ap.add_argument("--netmask", default="255.255.255.0")
    ap.add_argument("--cycle-ms", type=int, default=32, help="bus cycle: 1..512, power of two")
    ap.add_argument("--watchdog", type=int, default=10, help="watchdog factor (cycles)")
    ap.add_argument("--layout", help="JSON overriding the default I/O offsets")
    ap.add_argument("--fasten-s", type=float, default=1.5, help="simulated tightening time")
    ap.add_argument("--torque-fail", action="store_true", help="report every tightening as NOT OK")
    ap.add_argument("--cycles", type=int, help="run this many cycles non-interactively, then exit")
    ap.add_argument("--cycle-timeout", type=float, default=120.0)
    args = ap.parse_args()

    plc = PlcProgram(Layout.load(args.layout), fasten_s=args.fasten_s, log=log)
    if args.torque_fail:
        plc.torque_ok = False
    ctl = PnController(args.iface, args.station, args.device_ip, args.gsdml, netmask=args.netmask,
                       cycle_ms=args.cycle_ms, watchdog_factor=args.watchdog, log=log)
    ctl.on_cycle = plc.step
    try:
        ctl.identify()
        ctl.set_ip()
        ctl.connect()
        if args.cycles:
            results, ok = run_cycles(plc, ctl, args.cycles, args.cycle_timeout)
            done = sum(1 for _, d, _ in results if d)
            log(f"SUMMARY: {done}/{args.cycles} cycles done, {len(results) - done} faulted"
                f"{'' if ok else ', stopped early'}")
            return 0 if ok and done == args.cycles else 1
        log("PLC running -- type 'start', 'stop', 'reset', 'torque ok|fail', 'status', 'quit'")
        repl(plc, ctl)
        return 0
    except (TimeoutError, RuntimeError) as exc:
        log(f"ERROR: {exc}")
        return 2
    finally:
        ctl.stop()


if __name__ == "__main__":
    sys.exit(main())
