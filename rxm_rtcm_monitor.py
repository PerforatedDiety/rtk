#!/usr/bin/env python3
"""rxm_rtcm_monitor.py - Verify a C94-M8P ROVER is receiving AND applying RTCM
corrections, and diagnose why it might not be reaching a fix.

Reads three messages on USB and prints a combined live table:
  UBX-RXM-RTCM      each correction message (type, station ID, CRC status)
  UBX-NAV-PVT       satellite count + basic fix status
  UBX-NAV-RELPOSNED RTK solution status + relative-position accuracy

Columns:
  ix     count of correction messages received (should climb steadily)
  type   RTCM message type (want 1005 + observations 1074/1084 or 1077/1087)
  stnID  reference station ID
  CRC    ok / FAIL  - integrity of the received correction
  sats   satellites used in the nav solution (numSV)
  fix    y/n  - gnssFixOk: receiver has a valid basic GNSS fix
  soln   none / float / FIXED  - RTK carrier solution (carrSoln)
  diff   y/n  - diffSoln: differential corrections are being applied
  rel    y/n  - relPosValid: the relative position is valid
  age    seconds since the last observation message arrived (link freshness)
  hAcc   horizontal relative-position accuracy estimate, in mm

Using hAcc to diagnose 'float but never fixed': as the receiver approaches a
fix, hAcc shrinks from decimetres (hundreds of mm in float) toward ~10-30 mm.
If hAcc hovers in the hundreds and never tightens, conditions are too poor for
ambiguity resolution - almost always multipath / antenna environment on the
single-frequency M8P. Move antennas to open sky, on ground planes, away from
walls/metal/vehicles, and give it several minutes static.

  age computed here (not NAV-PVT lastCorrectionAge, which the M8P doesn't fill)
  as time since the last carrier-phase observation (1074/1077/1084/1087).

Connect the ROVER to this computer by USB (base powered and past survey-in).

Install:  pip install pyserial pyubx2
Run:      python rxm_rtcm_monitor.py COM6
          python rxm_rtcm_monitor.py /dev/ttyACM0 -t 30
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
OBS_TYPES = {1074, 1077, 1084, 1087, 1124, 1127}   # carrier-phase observations


def get_attr(msg, *names, default=0):
    for n in names:
        if hasattr(msg, n):
            return getattr(msg, n)
    return default


def yn(v):
    return "y" if v else "n"


def enable_on_usb(stream, msg_cls, msg_id):
    rates = [0, 0, 0, 1, 0, 0]      # DDC, UART1, UART2, USB, SPI, reserved
    payload = struct.pack("<BB6B", msg_cls, msg_id, *rates)
    stream.write(UBXMessage(CFG, ID_MSG, SET, payload=payload).serialize())


def crc_failed(msg):
    v = get_attr(msg, "crcFailed", default=None)
    if v is not None:
        return bool(v)
    return bool(get_attr(msg, "flags") & 0x01)


def main():
    ap = argparse.ArgumentParser(
        description="Monitor RTCM reception + RTK status on a C94-M8P rover.")
    ap.add_argument("port", help="USB serial port of the ROVER, e.g. COM6 or /dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=9600, help="USB baud (default 9600)")
    ap.add_argument("-t", "--time", type=float, default=0,
                    help="Seconds to run then summarise (0 = until Ctrl-C)")
    args = ap.parse_args()

    counts = Counter()
    crc_bad = Counter()
    ix = 0
    sats = fix = soln = diff = rel = 0
    best = 0                 # best (highest) carrSoln reached
    hacc = None              # horizontal relative accuracy (mm)
    last_obs = None
    start = time.time()

    def expired():
        return args.time and (time.time() - start) >= args.time

    def hacc_str():
        return f"{hacc:.0f}mm" if hacc is not None else "-"

    print(f"Listening on {args.port} ... (Ctrl-C to stop)\n")
    hdr = (f"{'ix':>5} {'type':>5} {'stnID':>5} {'CRC':>4} {'sats':>4} "
           f"{'fix':>3} {'soln':>5} {'diff':>4} {'rel':>3} {'age':>5} {'hAcc':>7}")
    print(hdr)
    print("-" * len(hdr))

    with Serial(args.port, args.baud, timeout=1) as stream:
        enable_on_usb(stream, 0x02, 0x32)   # RXM-RTCM
        enable_on_usb(stream, 0x01, 0x07)   # NAV-PVT (sats + fix)
        enable_on_usb(stream, 0x01, 0x3C)   # NAV-RELPOSNED (solution + accuracy)
        ubr = UBXReader(stream, protfilter=UBX_PROTOCOL)
        try:
            while not expired():
                try:
                    _, msg = ubr.read()
                except Exception:
                    continue
                if msg is None:
                    continue

                ident = msg.identity
                if ident == "NAV-PVT":
                    sats = get_attr(msg, "numSV")
                    fix = get_attr(msg, "gnssFixOk", "gnssFixOK")
                    soln = get_attr(msg, "carrSoln")
                    best = max(best, soln)
                    diff = get_attr(msg, "diffSoln")
                    continue
                if ident == "NAV-RELPOSNED":
                    soln = get_attr(msg, "carrSoln")
                    best = max(best, soln)
                    diff = get_attr(msg, "diffSoln")
                    rel = get_attr(msg, "relPosValid")
                    accN = get_attr(msg, "accN")   # pyubx2 returns these in mm
                    accE = get_attr(msg, "accE")
                    hacc = math.hypot(accN, accE)
                    continue
                if ident != "RXM-RTCM":
                    continue

                now = time.time()
                ix += 1
                mtype = get_attr(msg, "msgType")
                stn = get_attr(msg, "refStation", "refStationId")
                bad = crc_failed(msg)
                counts[mtype] += 1
                if bad:
                    crc_bad[mtype] += 1
                if mtype in OBS_TYPES and not bad:
                    last_obs = now

                age = f"{now - last_obs:.1f}s" if last_obs is not None else "-"
                print(f"{ix:>5} {mtype:>5} {stn:>5} {'FAIL' if bad else 'ok':>4} "
                      f"{sats:>4} {yn(fix):>3} {SOLN.get(soln, '?'):>5} "
                      f"{yn(diff):>4} {yn(rel):>3} {age:>5} {hacc_str():>7}")
        except KeyboardInterrupt:
            print("\nStopped.")

    elapsed = max(time.time() - start, 1e-6)
    print("\n" + "=" * 58)
    print(f" Run time       : {elapsed:.1f} s")
    print(f" Total messages : {ix}  ({ix / elapsed:.1f}/s)")
    print(f" Satellites     : {sats}   gnssFixOk: {yn(fix)}")
    print(f" Final solution : {SOLN.get(soln, '?')}   diff: {yn(diff)}   "
          f"relValid: {yn(rel)}   hAcc: {hacc_str()}")
    print(f" Best reached   : {SOLN.get(best, '?')}")
    if counts:
        print(f"\n {'type':>6} {'count':>6} {'crc_fail':>8}")
        for mtype in sorted(counts):
            print(f" {mtype:>6} {counts[mtype]:>6} {crc_bad[mtype]:>8}")
        obs_seen = sum(counts[t] for t in counts if t in OBS_TYPES)
        if not fix:
            print("\n Diagnosis: no basic GNSS fix -> sky-view / satellite problem.")
            print(" Get both antennas outside with clear sky and ground planes.")
        elif soln == 0 and obs_seen and diff:
            print("\n Diagnosis: corrections applied but no carrier RTK -> radio")
            print(" bandwidth. Lighten the base stream (MSM4) or use --gps-only.")
        elif soln == 0:
            print("\n Diagnosis: have a fix but RTK not engaging. Check 1005 present,")
            print(" base survey-in done, matched constellations, observations arriving.")
        elif soln == 1:
            print("\n Diagnosis: float but not fixing. Watch hAcc: if it doesn't fall")
            print(" toward ~10-30 mm, it's multipath/antenna environment. Open sky,")
            print(" ground planes, away from walls/metal; try --gps-only; wait minutes.")
        else:
            print("\n Looks healthy: fixed RTK solution with corrections applied.")
    else:
        print("\n No RTCM received. Check: base past survey-in (NAV-SVIN 'TIME'),")
        print(" radios on the same channel, rover UART1 'Protocol in = RTCM3'.")
    print("=" * 58)


if __name__ == "__main__":
    main()
