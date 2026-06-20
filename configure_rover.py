#!/usr/bin/env python3
"""configure_rover.py - Configure a u-blox C94-M8P (NEO-M8P) board as an RTK ROVER.

Sends, over USB, the same settings as the KumarRobotics c94_m8p_rover.yaml:
  1) UART1 (radio link): 19200 baud, input = RTCM3 only, output = none
  2) DGNSS mode = RTK fixed (best accuracy for static distance measurement)
  3) Enable UBX-NAV-RELPOSNED on USB at 1 Hz (the message m8p_distance.py reads)
  4) Save the configuration to flash / battery-backed RAM (CFG-CFG)

GNSS constellations are left at the M8P default (GPS + GLONASS), which already
matches the base and the YAML. Change them in u-center only if your base differs.

The USB connection is NOT touched - only UART1 (the wire to the UHF radio) is
reconfigured - so this session stays stable while it runs.

Connect the ROVER board to this computer by USB.
Install:  pip install pyserial pyubx2
Run:      python configure_rover.py COM6
"""
import argparse
import struct
import time

from serial import Serial
from pyubx2 import UBXMessage, UBXReader, SET, UBX_PROTOCOL

CFG = b"\x06"
ID_PRT, ID_DGNSS, ID_MSG, ID_CFG = b"\x00", b"\x70", b"\x01", b"\x09"

MODE_8N1 = 0x000008D0


def cfg_msg(msg_cls, msg_id, port_index):
    """CFG-MSG payload: enable a message at rate 1 on one port.
    port_index: 0=DDC/I2C, 1=UART1, 2=UART2, 3=USB, 4=SPI, 5=reserved."""
    rates = [0, 0, 0, 0, 0, 0]
    rates[port_index] = 1
    return struct.pack("<BB6B", msg_cls, msg_id, *rates)


def wait_ack(ubr, cls_byte, id_byte, timeout=2.0):
    """Return True on ACK-ACK, False on ACK-NAK, None on timeout, for a given cls/id."""
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
    ap = argparse.ArgumentParser(description="Configure a C94-M8P board as RTK rover.")
    ap.add_argument("port", help="USB serial port, e.g. COM6 or /dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=9600,
                    help="USB baud (default 9600; USB CDC ignores it)")
    args = ap.parse_args()

    print(f"Configuring ROVER on {args.port} ...")
    with Serial(args.port, args.baud, timeout=1) as stream:
        ubr = UBXReader(stream, protfilter=UBX_PROTOCOL)

        # 1) UART1 -> 19200 baud, in: RTCM3 (0x20), out: none
        prt = struct.pack("<BBHIIHHHH", 1, 0, 0, MODE_8N1, 19200, 0x0020, 0x0000, 0, 0)
        send(stream, ubr, CFG, ID_PRT, prt, "UART1 port (RTCM3 in only)")

        # 2) DGNSS mode = 3 (RTK fixed)
        dgnss = struct.pack("<B3s", 3, b"\x00\x00\x00")
        send(stream, ubr, CFG, ID_DGNSS, dgnss, "DGNSS mode = RTK fixed")

        # 3) NAV-RELPOSNED (class 0x01, id 0x3C) on USB (port index 3) @ 1 Hz
        send(stream, ubr, CFG, ID_MSG, cfg_msg(0x01, 0x3C, 3), "NAV-RELPOSNED on USB")

        # 4) Save current configuration to BBR + flash
        save = struct.pack("<IIIB", 0x00000000, 0x0000FFFF, 0x00000000, 0x17)
        send(stream, ubr, CFG, ID_CFG, save, "Save configuration")

    print("\nDone. Power both boards with the radio link up; the rover's green LED")
    print("goes solid at RTK FIXED. Then run m8p_distance.py to read the distance.")


if __name__ == "__main__":
    main()
