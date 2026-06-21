#!/usr/bin/env python3
"""rxm_rtcm_monitor.py - Live ROVER dashboard for a u-blox C94-M8P (NEO-M8P):
link health, RTK status, and the relative position (distance + N/E/D).

Refreshes a status block every second from three messages on USB:
  UBX-RXM-RTCM      correction message flow + CRC integrity
  UBX-NAV-PVT       satellite count + basic fix status
  UBX-NAV-RELPOSNED RTK solution status, accuracy, and relative position vector

The block shows:
  * a status line: sats, fix, soln (none/float/FIXED), diff, rel, correction age,
    and hAcc (horizontal relative accuracy in mm);
  * RTCM health: which message types are arriving and total CRC failures;
  * a relative-position table - North, East, Down and the straight-line and
    horizontal distance from base to rover - each in metres and feet+inches.

Trust the position only at soln=FIXED (centimetre). At float it is decimetre,
at none it is not a carrier solution at all.

Connect the ROVER to this computer by USB (base powered and past survey-in).

Install:  pip install pyserial pyubx2
Run:      python rxm_rtcm_monitor.py COM6
          python rxm_rtcm_monitor.py /dev/ttyACM0 -i 1
"""
import argparse
import math
import struct
import time
from collections import Counter

from serial import Serial
from pyubx2 import UBXMessage, UBXReader, SET, UBX_PROTOCOL

CFG = b"\x06"
ID_MSG = b"\x01"
SOLN = {0: "none", 1: "float", 2: "FIXED"}
OBS_TYPES = {1074, 1077, 1084, 1087, 1124, 1127}
IN_PER_M = 39.37007874015748


def get_attr(msg, *names, default=0):
    for n in names:
        if hasattr(msg, n):
            return getattr(msg, n)
    return default


def yn(v):
    return "y" if v else "n"


def ftin(v_m):
    """Format a length in metres as feet + inches, keeping sign."""
    sign = "-" if v_m < 0 else ""
    total_in = abs(v_m) * IN_PER_M
    feet = int(total_in // 12)
    inch = total_in - feet * 12
    return f"{sign}{feet}' {inch:.1f}\""


def enable_on_usb(stream, msg_cls, msg_id):
    rates = [0, 0, 0, 1, 0, 0]      # DDC, UART1, UART2, USB, SPI, reserved
    payload = struct.pack("<BB6B", msg_cls, msg_id, *rates)
    stream.write(UBXMessage(CFG, ID_MSG, SET, payload=payload).serialize())


def crc_failed(msg):
    v = get_attr(msg, "crcFailed", default=None)
    if v is not None:
        return bool(v)
    return bool(get_attr(msg, "flags") & 0x01)


def print_block(t0, ix, rate, counts, crc_total, st, last_obs):
    el = time.strftime("%H:%M:%S", time.gmtime(time.time() - t0))
    age = f"{time.time() - last_obs:.1f}s" if last_obs else "-"
    print("\n" + "=" * 60)
    print(f" ROVER MONITOR   running {el}      RTCM {ix} msgs ({rate:.1f}/s)")
    print("-" * 60)
    print(f" sats {st['sats']:<3} fix {yn(st['fix'])}   soln {SOLN.get(st['soln'], '?'):<5} "
          f"diff {yn(st['diff'])}   rel {yn(st['rel'])}   age {age:<5} "
          f"hAcc {('%dmm' % st['hacc']) if st['hacc'] is not None else '-'}")
    types = "  ".join(f"{t}" for t in sorted(counts)) or "(none yet)"
    print(f" RTCM types: {types}    crc_fail {crc_total}")
    print("-" * 60)
    if st["rel"] and st["pos"] is not None:
        n, e, d = st["pos"]
        dist = math.sqrt(n * n + e * e + d * d)
        horiz = math.hypot(n, e)
        print(" Relative position (rover relative to base):")
        print(f"   {'axis':<11}{'meters':>13}      {'ft + in':>12}")
        for label, v in (("North", n), ("East", e), ("Down", d)):
            print(f"   {label:<11}{v:>+12.4f} m   {ftin(v):>12}")
        print("   " + "-" * 42)
        print(f"   {'Distance':<11}{dist:>12.4f} m   {ftin(dist):>12}")
        print(f"   {'Horizontal':<11}{horiz:>12.4f} m   {ftin(horiz):>12}")
        # Horizontal slope = bearing of the N/E vector (deg from North, CW to East)
        bearing = math.degrees(math.atan2(e, n)) % 360.0
        # Vertical slope = inclination from horizontal distance and Down
        vert = math.degrees(math.atan2(d, horiz)) if (horiz or d) else 0.0
        grade = f"{d / horiz * 100:+.1f}%" if horiz > 1e-9 else "vertical"
        print("   " + "-" * 42)
        print(f"   {'Horiz slope':<11}{bearing:>12.1f} deg  (bearing N->E)")
        print(f"   {'Vert slope':<11}{vert:>+12.1f} deg  ({grade}, + = rover below base)")
    else:
        print(" Relative position: (waiting for a valid relative solution)")
    print("=" * 60)


def main():
    ap = argparse.ArgumentParser(
        description="Live ROVER dashboard: link, RTK status, and relative position.")
    ap.add_argument("port", help="USB serial port of the ROVER, e.g. COM6 or /dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=9600, help="USB baud (default 9600)")
    ap.add_argument("-i", "--interval", type=float, default=1.0,
                    help="Seconds between refreshes (default 1)")
    ap.add_argument("-t", "--time", type=float, default=0,
                    help="Seconds to run then stop (0 = until Ctrl-C)")
    args = ap.parse_args()

    counts = Counter()
    crc_total = 0
    ix = 0
    last_obs = None
    st = {"sats": 0, "fix": 0, "soln": 0, "diff": 0, "rel": 0, "hacc": None, "pos": None}
    t0 = time.time()
    last_print = 0.0

    def expired():
        return args.time and (time.time() - t0) >= args.time

    print(f"Monitoring ROVER on {args.port} ... (Ctrl-C to stop)")
    with Serial(args.port, args.baud, timeout=1) as stream:
        enable_on_usb(stream, 0x02, 0x32)   # RXM-RTCM
        enable_on_usb(stream, 0x01, 0x07)   # NAV-PVT
        enable_on_usb(stream, 0x01, 0x3C)   # NAV-RELPOSNED
        ubr = UBXReader(stream, protfilter=UBX_PROTOCOL)
        try:
            while not expired():
                try:
                    _, msg = ubr.read()
                except Exception:
                    msg = None

                if msg is not None:
                    ident = msg.identity
                    if ident == "NAV-PVT":
                        st["sats"] = get_attr(msg, "numSV")
                        st["fix"] = get_attr(msg, "gnssFixOk", "gnssFixOK")
                        st["soln"] = get_attr(msg, "carrSoln")
                        st["diff"] = get_attr(msg, "diffSoln")
                    elif ident == "NAV-RELPOSNED":
                        st["soln"] = get_attr(msg, "carrSoln")
                        st["diff"] = get_attr(msg, "diffSoln")
                        st["rel"] = get_attr(msg, "relPosValid")
                        st["hacc"] = math.hypot(get_attr(msg, "accN"), get_attr(msg, "accE"))
                        # pyubx2 returns relPosN/E/D in cm (HP folded in) -> metres
                        st["pos"] = (get_attr(msg, "relPosN") / 100.0,
                                     get_attr(msg, "relPosE") / 100.0,
                                     get_attr(msg, "relPosD") / 100.0)
                    elif ident == "RXM-RTCM":
                        ix += 1
                        mtype = get_attr(msg, "msgType")
                        bad = crc_failed(msg)
                        counts[mtype] += 1
                        if bad:
                            crc_total += 1
                        elif mtype in OBS_TYPES:
                            last_obs = time.time()

                now = time.time()
                if now - last_print >= args.interval:
                    rate = ix / max(now - t0, 1e-6)
                    print_block(t0, ix, rate, counts, crc_total, st, last_obs)
                    last_print = now
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
