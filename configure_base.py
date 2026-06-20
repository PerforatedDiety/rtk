#!/usr/bin/env python3
"""configure_base.py - Configure a u-blox C94-M8P (NEO-M8P) board as an RTK BASE.

Sends, over USB:
  1) UART1 (radio link): 19200 baud, input protocols = none, output = RTCM3 only
  2) TMODE3: survey-in, default 300 s minimum duration and 3.0 m accuracy target
  3) RTCM3 output on UART1 - using MSM4 (lighter than MSM7) to fit the C94 radio:
        default (GPS+GLONASS): 1005, 1074, 1084, and 1230 every 5 s
        --gps-only:            1005, 1074
     Any previously enabled MSM7 messages (1077/1087) are DISABLED first, since
     mixing MSM4 and MSM7 causes incorrect behaviour.
  4) Save the configuration to flash / battery-backed RAM (CFG-CFG)

Why MSM4: at high satellite counts a full GPS+GLONASS MSM7 stream can exceed the
19200-baud UHF link, so observation messages arrive late or dropped and the
rover stays in code-differential (DGNSS) without ever reaching RTK float. MSM4
cuts the load to roughly 300 bytes/s. If float still won't engage at very high
satellite counts, re-run with --gps-only.

Connect the BASE board to this computer by USB.
Install:  pip install pyserial pyubx2
Run:      python configure_base.py COM5
          python configure_base.py /dev/ttyACM0 --gps-only --min-dur 300 --acc 3.0
"""
import argparse
import struct
import time

from serial import Serial
from pyubx2 import UBXMessage, UBXReader, SET, UBX_PROTOCOL

CFG = b"\x06"
ID_PRT, ID_TMODE3, ID_MSG, ID_CFG = b"\x00", b"\x71", b"\x01", b"\x09"

MODE_8N1 = 0x000008D0

# RTCM3 message ids (RTCM class is 0xF5; the id byte is the type minus 1000).
RTCM_ID = {"1005": 0x05, "1074": 0x4A, "1077": 0x4D,
           "1084": 0x54, "1087": 0x57, "1230": 0xE6}

# Target sets as (name, rate-in-nav-epochs). MSM4 observations.
SET_FULL = [("1005", 1), ("1074", 1), ("1084", 1), ("1230", 5)]
SET_GPS = [("1005", 1), ("1074", 1)]


def cfg_msg(msg_cls, msg_id, port_index, rate):
    """CFG-MSG payload: set a message rate on one port (rate 0 disables)."""
    rates = [0, 0, 0, 0, 0, 0]      # DDC, UART1, UART2, USB, SPI, reserved
    rates[port_index] = rate
    return struct.pack("<BB6B", msg_cls, msg_id, *rates)


def wait_ack(ubr, cls_byte, id_byte, timeout=2.0):
    end = time.time() + timeout
    while time.time() < end:
        try:
            _, msg = ubr.read()
        except Exception:
            continue
        if msg is None:
            continue
        if msg.identity in ("ACK-ACK", "ACK-NAK") and \
                msg.clsID == cls_byte[0] and msg.msgID == id_byte[0]:
            return msg.identity == "ACK-ACK"
    return None


def send(stream, ubr, cls, mid, payload, label):
    stream.write(UBXMessage(cls, mid, SET, payload=payload).serialize())
    ack = wait_ack(ubr, cls, mid)
    status = "OK" if ack else ("NAK" if ack is False else "no-ack")
    print(f"  {label:<36} {status}")


def main():
    ap = argparse.ArgumentParser(description="Configure a C94-M8P board as RTK base (MSM4).")
    ap.add_argument("port", help="USB serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=9600,
                    help="USB baud (default 9600; USB CDC ignores it)")
    ap.add_argument("--gps-only", action="store_true",
                    help="Send GPS-only corrections (lightest radio load)")
    ap.add_argument("--min-dur", type=int, default=300,
                    help="Survey-in minimum duration in seconds (default 300)")
    ap.add_argument("--acc", type=float, default=3.0,
                    help="Survey-in accuracy target in metres (default 3.0)")
    args = ap.parse_args()

    target = dict(SET_GPS if args.gps_only else SET_FULL)

    print(f"Configuring BASE on {args.port} "
          f"({'GPS-only' if args.gps_only else 'GPS+GLONASS'} MSM4) ...")
    with Serial(args.port, args.baud, timeout=1) as stream:
        ubr = UBXReader(stream, protfilter=UBX_PROTOCOL)

        # 1) UART1 -> 19200 baud, in: none, out: RTCM3 (0x20)
        prt = struct.pack("<BBHIIHHHH", 1, 0, 0, MODE_8N1, 19200, 0x0000, 0x0020, 0, 0)
        send(stream, ubr, CFG, ID_PRT, prt, "UART1 port (RTCM3 out only)")

        # 2) TMODE3 survey-in (svinAccLimit is in units of 0.1 mm)
        acc_units = int(round(args.acc * 10000))
        tmode = struct.pack("<BBHiiibbbBIII8s",
                            0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                            args.min_dur, acc_units, b"\x00" * 8)
        send(stream, ubr, CFG, ID_TMODE3, tmode,
             f"TMODE3 survey-in {args.min_dur}s / {args.acc} m")

        # 3) Disable any RTCM type not in the target set (kills leftover MSM7),
        #    then enable the target MSM4 set on UART1.
        for name, rid in RTCM_ID.items():
            if name not in target:
                send(stream, ubr, CFG, ID_MSG, cfg_msg(0xF5, rid, 1, 0),
                     f"disable RTCM {name}")
        for name, rate in target.items():
            suffix = f" @1/{rate}s" if rate > 1 else ""
            send(stream, ubr, CFG, ID_MSG, cfg_msg(0xF5, RTCM_ID[name], 1, rate),
                 f"RTCM {name} on UART1{suffix}")

        # 4) Save current configuration to BBR + flash
        save = struct.pack("<IIIB", 0x00000000, 0x0000FFFF, 0x00000000, 0x17)
        send(stream, ubr, CFG, ID_CFG, save, "Save configuration")

    print("\nDone. Monitor the survey with UBX-NAV-SVIN ('TIME' when complete).")
    print("On the rover, watch rxm_rtcm_monitor.py: 'age' should stay near 1 s and")
    print("'soln' should climb to float then FIXED now that the stream is lighter.")


if __name__ == "__main__":
    main()
