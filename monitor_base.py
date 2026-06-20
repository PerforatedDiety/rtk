#!/usr/bin/env python3
"""monitor_base.py - Live status monitor for a u-blox C94-M8P (NEO-M8P) BASE.

Shows two things, refreshed every couple of seconds:
  * Survey-in progress  (UBX-NAV-SVIN): elapsed time, observations, current mean
    position accuracy, and whether survey-in is active / complete.
  * Satellite status    (UBX-NAV-SAT):  per-constellation counts, how many are
    used, and C/N0 signal levels, plus the strongest satellites.

Survey-in completes when BOTH the minimum duration AND the accuracy target are
met; once valid, the base enters TIME mode and starts broadcasting corrections.
Strong C/N0 on many satellites means a faster, tighter survey.

Connect the BASE board to this computer by USB.

Install:  pip install pyserial pyubx2
Run:      python monitor_base.py COM5
          python monitor_base.py /dev/ttyACM0 -i 2 --acc 3.0 --min-dur 300
"""
import argparse
import struct
import time
from collections import defaultdict

from serial import Serial
from pyubx2 import UBXMessage, UBXReader, SET, UBX_PROTOCOL

CFG = b"\x06"
ID_MSG = b"\x01"
GNSS_NAME = {0: "GPS", 1: "SBAS", 2: "GAL", 3: "BDS", 5: "QZSS", 6: "GLO"}
GNSS_LET = {0: "G", 1: "S", 2: "E", 3: "B", 5: "J", 6: "R"}


def get_attr(msg, *names, default=0):
    for n in names:
        if hasattr(msg, n):
            return getattr(msg, n)
    return default


def enable_on_usb(stream, msg_cls, msg_id):
    rates = [0, 0, 0, 1, 0, 0]      # DDC, UART1, UART2, USB, SPI, reserved
    payload = struct.pack("<BB6B", msg_cls, msg_id, *rates)
    stream.write(UBXMessage(CFG, ID_MSG, SET, payload=payload).serialize())


def parse_sats(msg):
    sats = []
    for i in range(1, get_attr(msg, "numSvs") + 1):
        s = f"{i:02d}"
        sats.append({
            "gnss": get_attr(msg, f"gnssId_{s}"),
            "sv": get_attr(msg, f"svId_{s}"),
            "cno": get_attr(msg, f"cno_{s}"),
            "used": bool(get_attr(msg, f"svUsed_{s}")),
        })
    return sats


def print_block(t0, svin, sats, acc_target, dur_target):
    el = time.strftime("%H:%M:%S", time.gmtime(time.time() - t0))
    print("\n" + "=" * 56)
    print(f" BASE STATUS   running {el}")
    print("-" * 56)

    # Survey-in
    if svin is None:
        print(" Survey-in : (no NAV-SVIN yet)")
    elif svin["valid"]:
        print(f" Survey-in : COMPLETE - base in TIME mode, broadcasting.")
        print(f"             {svin['dur']} s, {svin['obs']} obs, "
              f"meanAcc {svin['acc_m']:.2f} m")
    elif svin["active"]:
        dur_ok = "y" if svin["dur"] >= dur_target else "n"
        acc_ok = "y" if svin["acc_m"] <= acc_target else "n"
        print(f" Survey-in : ACTIVE")
        print(f"   duration : {svin['dur']:>5} s  (target {dur_target} s, met: {dur_ok})")
        print(f"   meanAcc  : {svin['acc_m']:>5.2f} m  (target {acc_target} m, met: {acc_ok})")
        print(f"   obs used : {svin['obs']}")
    else:
        print(" Survey-in : not running (base in fixed/disabled mode?)")

    # Satellites
    tracked = [s for s in sats if s["cno"] > 0]
    used = [s for s in tracked if s["used"]]
    print("-" * 56)
    print(f" Satellites: {len(tracked)} tracked, {len(used)} used")
    by = defaultdict(list)
    for s in tracked:
        by[s["gnss"]].append(s)
    for g in sorted(by):
        grp = by[g]
        cnos = [s["cno"] for s in grp]
        nused = sum(1 for s in grp if s["used"])
        print(f"   {GNSS_NAME.get(g, g):<5}: {len(grp):>2} sats  used {nused:>2}  "
              f"C/N0 max {max(cnos):>2}  avg {sum(cnos)//len(cnos):>2}")
    if tracked:
        top = sorted(tracked, key=lambda s: s["cno"], reverse=True)[:8]
        strongest = "  ".join(
            f"{GNSS_LET.get(s['gnss'], '?')}{s['sv']}:{s['cno']}" for s in top)
        print(f"   strongest: {strongest}")
    print("=" * 56)


def main():
    ap = argparse.ArgumentParser(description="Live status monitor for a C94-M8P base.")
    ap.add_argument("port", help="USB serial port of the BASE, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=9600, help="USB baud (default 9600)")
    ap.add_argument("-i", "--interval", type=float, default=2.0,
                    help="Seconds between status refreshes (default 2)")
    ap.add_argument("--acc", type=float, default=3.0,
                    help="Survey-in accuracy target in m, display only (default 3.0)")
    ap.add_argument("--min-dur", type=int, default=300,
                    help="Survey-in min duration in s, display only (default 300)")
    args = ap.parse_args()

    svin = None
    sats = []
    t0 = time.time()
    last_print = 0.0

    print(f"Monitoring BASE on {args.port} ... (Ctrl-C to stop)")
    with Serial(args.port, args.baud, timeout=1) as stream:
        enable_on_usb(stream, 0x01, 0x3B)   # NAV-SVIN (survey-in)
        enable_on_usb(stream, 0x01, 0x35)   # NAV-SAT (per-satellite)
        ubr = UBXReader(stream, protfilter=UBX_PROTOCOL)
        try:
            while True:
                try:
                    _, msg = ubr.read()
                except Exception:
                    msg = None
                if msg is not None:
                    if msg.identity == "NAV-SVIN":
                        svin = {
                            "dur": get_attr(msg, "dur"),
                            "obs": get_attr(msg, "obs"),
                            # meanAcc is reported in 0.1 mm units -> metres
                            "acc_m": get_attr(msg, "meanAcc") * 0.0001,
                            "valid": bool(get_attr(msg, "valid")),
                            "active": bool(get_attr(msg, "active")),
                        }
                    elif msg.identity == "NAV-SAT":
                        sats = parse_sats(msg)

                now = time.time()
                if now - last_print >= args.interval:
                    print_block(t0, svin, sats, args.acc, args.min_dur)
                    last_print = now
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
