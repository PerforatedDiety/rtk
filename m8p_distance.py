#!/usr/bin/env python3
"""
m8p_distance.py - Measure the distance between two points with a u-blox
C94-M8P (NEO-M8P) RTK pair, using a laptop connected to the ROVER over USB.

HOW IT WORKS
  RTK gives the rover's position as a vector relative to the base
  (UBX-NAV-RELPOSNED, in North / East / Down). Put the base antenna over
  point A and the rover antenna over point B: the length of that vector IS
  the distance between the two points. This reads the message, waits for an
  RTK *fixed* solution, averages the vector over N samples, and reports the
  horizontal and 3D (slope) distance. Every sample is also logged to CSV.

  Because it's a *relative* measurement, the base's absolute position does
  not matter - you can let it survey-in to a rough location.

SETUP BEFORE RUNNING
  1. Base over point A, rover over point B, UHF radio link active.
  2. Rover connected to this laptop via USB.
  3. Enable UBX-NAV-RELPOSNED on the rover (u-center or PyGPSClient:
     Messages -> NAV -> RELPOSNED -> enable). The script also *requests* it
     on startup, but enabling it manually once is the reliable path.

INSTALL
  pip install pyserial pyubx2

RUN
  python m8p_distance.py COM5            # Windows
  python m8p_distance.py /dev/ttyACM0    # Linux/Mac
  python m8p_distance.py COM5 -n 120 -o pointA_to_pointB.csv
"""

import argparse
import csv
import math
import sys

from serial import Serial
from pyubx2 import UBXReader, UBXMessage, SET, UBX_PROTOCOL

CARR = {0: "none", 1: "float", 2: "FIXED"}


def relposned_to_meters(msg):
    """Combine the cm field and the 0.1 mm high-precision residual into metres."""
    n = msg.relPosN * 0.01 + getattr(msg, "relPosHPN", 0) * 0.0001
    e = msg.relPosE * 0.01 + getattr(msg, "relPosHPE", 0) * 0.0001
    d = msg.relPosD * 0.01 + getattr(msg, "relPosHPD", 0) * 0.0001
    return n, e, d


def main():
    ap = argparse.ArgumentParser(
        description="Measure base->rover distance from UBX-NAV-RELPOSNED.")
    ap.add_argument("port", help="Serial port of the ROVER, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=9600, help="Baud rate (default 9600)")
    ap.add_argument("-n", "--samples", type=int, default=60,
                    help="Number of fixed-mode samples to average (default 60)")
    ap.add_argument("--allow-float", action="store_true",
                    help="Also accept FLOAT samples (less accurate)")
    ap.add_argument("-o", "--csv", default="relposned_log.csv", help="CSV log filename")
    args = ap.parse_args()

    accepted = {1, 2} if args.allow_float else {2}
    samples = []  # (n, e, d) in metres

    want = "fixed/float" if args.allow_float else "fixed"
    print(f"Opening {args.port} @ {args.baud} ... waiting for RTK {want} solution.\n")

    with Serial(args.port, args.baud, timeout=2) as stream, \
         open(args.csv, "w", newline="") as fcsv:

        # Best-effort request to emit NAV-RELPOSNED (class 0x01, id 0x3C) every epoch.
        # If this fails, just enable the message manually in u-center / PyGPSClient.
        try:
            cfg = UBXMessage("CFG", "CFG-MSG", SET, msgClass=0x01, msgID=0x3C,
                             rateUART1=1, rateUSB=1)
            stream.write(cfg.serialize())
        except Exception:
            pass

        writer = csv.writer(fcsv)
        writer.writerow(["iTOW", "carrSoln", "N_m", "E_m", "D_m",
                         "horiz_m", "slope_m", "accN_mm", "accE_mm", "accD_mm"])

        ubr = UBXReader(stream, protfilter=UBX_PROTOCOL)
        try:
            while len(samples) < args.samples:
                _, msg = ubr.read()
                if msg is None or msg.identity != "NAV-RELPOSNED":
                    continue

                carr = getattr(msg, "carrSoln", 0)
                n, e, d = relposned_to_meters(msg)
                horiz = math.hypot(n, e)
                slope = math.sqrt(n * n + e * e + d * d)
                accN = getattr(msg, "accN", 0) * 0.1
                accE = getattr(msg, "accE", 0) * 0.1
                accD = getattr(msg, "accD", 0) * 0.1

                writer.writerow([getattr(msg, "iTOW", ""), CARR.get(carr, carr),
                                 f"{n:.4f}", f"{e:.4f}", f"{d:.4f}",
                                 f"{horiz:.4f}", f"{slope:.4f}",
                                 f"{accN:.1f}", f"{accE:.1f}", f"{accD:.1f}"])

                status = CARR.get(carr, str(carr))
                if carr in accepted:
                    samples.append((n, e, d))
                    print(f"[{len(samples):>3}/{args.samples}] {status:>5}  "
                          f"horiz={horiz:7.3f} m  slope={slope:7.3f} m  "
                          f"(+/-{accN:.0f}/{accE:.0f}/{accD:.0f} mm)")
                else:
                    print(f"   ...   {status:>5}  waiting for fix  "
                          f"(+/-{accN:.0f}/{accE:.0f}/{accD:.0f} mm)   ", end="\r")
        except KeyboardInterrupt:
            print("\nStopped early.")

    if not samples:
        print("No usable samples collected. Check that NAV-RELPOSNED is enabled "
              "and the rover has corrections.")
        sys.exit(1)

    k = len(samples)
    mn = sum(s[0] for s in samples) / k
    me = sum(s[1] for s in samples) / k
    md = sum(s[2] for s in samples) / k
    horiz = math.hypot(mn, me)
    slope = math.sqrt(mn * mn + me * me + md * md)

    # Spread of per-sample horizontal distance = rough repeatability figure.
    hs = [math.hypot(s[0], s[1]) for s in samples]
    mh = sum(hs) / k
    sd = (sum((h - mh) ** 2 for h in hs) / k) ** 0.5

    print("\n" + "=" * 50)
    print(f" Samples averaged : {k}")
    print(f" Mean N / E / D   : {mn:+.4f} / {me:+.4f} / {md:+.4f} m")
    print(f" Horizontal dist. : {horiz:.4f} m")
    print(f" 3D slope dist.   : {slope:.4f} m")
    print(f" Horiz. std dev   : {sd * 1000:.1f} mm")
    print(f" CSV log          : {args.csv}")
    print("=" * 50)


if __name__ == "__main__":
    main()
