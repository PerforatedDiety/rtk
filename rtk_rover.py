#!/usr/bin/env python3
"""rtk_rover.py - GUI rover monitor + point recorder for a u-blox C94-M8P.

A tkinter front-end built on the rxm_rtcm_monitor logic. It shows the live RTK
status and relative position (N/E/D, distance, horizontal, slopes) in metres and
feet+inches, plus a map view. Buttons:

  Record  - snapshot the current relative coordinates (with full detail) into an
            in-memory list.
  Save    - write the recorded points to a CSV file.
  Clear   - empty the in-memory list.

The map plots the base station at the origin and every recorded point, joined by
a line in the order they were recorded; the current rover position is shown as a
hollow marker. North is up, East is right, drawn to equal aspect with a scale bar.

Only the values at soln=FIXED are centimetre-accurate; record at FIXED for best
results (the recorded soln is stored so you know each point's quality).

Connect the ROVER to this computer by USB (base powered and past survey-in).

Install:  pip install pyserial pyubx2     (tkinter ships with Python)
Run:      python rtk_rover.py
          python rtk_rover.py COM6
          python rtk_rover.py /dev/ttyACM0 -b 9600
"""
import argparse
import csv
import math
import struct
import threading
import time
from datetime import datetime

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

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


def derived(n, e, d):
    horiz = math.hypot(n, e)
    dist = math.sqrt(n * n + e * e + d * d)
    bearing = math.degrees(math.atan2(e, n)) % 360.0
    vert = math.degrees(math.atan2(d, horiz)) if (horiz or d) else 0.0
    grade = (d / horiz * 100.0) if horiz > 1e-9 else float("inf")
    return {"N": n, "E": e, "D": d, "horiz": horiz, "dist": dist,
            "bearing": bearing, "vert": vert, "grade": grade}


def nice_len(target):
    best = 0.02
    for x in (0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000):
        if x <= target:
            best = x
    return best


def fmt_len(m):
    return f"{m * 100:.0f} cm" if m < 1 else f"{m:g} m"


CSV_COLS = ["time", "soln", "sats", "hAcc_mm", "N", "E", "D", "horiz", "dist",
            "N_ftin", "E_ftin", "D_ftin", "horiz_ftin", "dist_ftin",
            "bearing_deg", "vert_deg", "grade_pct"]


class RTKRover:
    W = 460
    H = 460

    def __init__(self, root, port, baud):
        self.root = root
        self.lock = threading.Lock()
        self.stream = None
        self.reader = None
        self.running = False
        self.state = {"sats": 0, "fix": 0, "soln": 0, "diff": 0, "rel": 0,
                      "hacc": None, "pos": None, "ix": 0, "crc": 0,
                      "last_obs": None, "counts": {}}
        self.points = []        # recorded snapshots (main-thread only)
        self._build_ui(port, baud)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(250, self._refresh)

    # ---------- UI ----------
    def _build_ui(self, port, baud):
        self.root.title("RTK Rover")
        top = ttk.Frame(self.root, padding=6)
        top.grid(row=0, column=0, sticky="ew")
        ttk.Label(top, text="Port:").grid(row=0, column=0)
        self.port_var = tk.StringVar(value=port or "")
        ttk.Entry(top, textvariable=self.port_var, width=16).grid(row=0, column=1, padx=4)
        ttk.Label(top, text="Baud:").grid(row=0, column=2)
        self.baud_var = tk.StringVar(value=str(baud))
        ttk.Entry(top, textvariable=self.baud_var, width=8).grid(row=0, column=3, padx=4)
        self.conn_btn = ttk.Button(top, text="Connect", command=self._toggle_conn)
        self.conn_btn.grid(row=0, column=4, padx=6)

        mid = ttk.Frame(self.root, padding=6)
        mid.grid(row=1, column=0, sticky="nsew")
        self.info_var = tk.StringVar(value="Not connected.")
        ttk.Label(mid, textvariable=self.info_var, font=("Courier", 10),
                  justify="left", anchor="nw").grid(row=0, column=0, sticky="nw", padx=(0, 10))
        self.canvas = tk.Canvas(mid, width=self.W, height=self.H, bg="white",
                                highlightthickness=1, highlightbackground="#999")
        self.canvas.grid(row=0, column=1, sticky="nsew")

        bot = ttk.Frame(self.root, padding=6)
        bot.grid(row=2, column=0, sticky="ew")
        ttk.Button(bot, text="Record", command=self._record).grid(row=0, column=0, padx=4)
        ttk.Button(bot, text="Save", command=self._save).grid(row=0, column=1, padx=4)
        ttk.Button(bot, text="Clear", command=self._clear).grid(row=0, column=2, padx=4)
        self.count_var = tk.StringVar(value="0 recorded")
        ttk.Label(bot, textvariable=self.count_var).grid(row=0, column=3, padx=10)
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(bot, textvariable=self.status_var).grid(row=0, column=4, padx=10)

    # ---------- connection ----------
    def _toggle_conn(self):
        self._disconnect() if self.running else self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("No port", "Enter the rover's serial port.")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = 9600
        try:
            self.stream = Serial(port, baud, timeout=1)
        except Exception as ex:
            messagebox.showerror("Connect failed", str(ex))
            return
        for c, i in ((0x02, 0x32), (0x01, 0x07), (0x01, 0x3C)):
            try:
                enable_on_usb(self.stream, c, i)
            except Exception:
                pass
        self.running = True
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()
        self.conn_btn.config(text="Disconnect")
        self.status_var.set(f"Connected {port} @ {baud}")

    def _disconnect(self):
        self.running = False
        if self.reader:
            self.reader.join(timeout=2)
            self.reader = None
        if self.stream:
            try:
                self.stream.close()
            except Exception:
                pass
            self.stream = None
        self.conn_btn.config(text="Connect")
        self.status_var.set("Disconnected")

    def _reader_loop(self):
        ubr = UBXReader(self.stream, protfilter=UBX_PROTOCOL)
        while self.running:
            try:
                _, msg = ubr.read()
            except Exception:
                continue
            if msg is None:
                continue
            ident = msg.identity
            with self.lock:
                s = self.state
                if ident == "NAV-PVT":
                    s["sats"] = get_attr(msg, "numSV")
                    s["fix"] = get_attr(msg, "gnssFixOk", "gnssFixOK")
                    s["soln"] = get_attr(msg, "carrSoln")
                    s["diff"] = get_attr(msg, "diffSoln")
                elif ident == "NAV-RELPOSNED":
                    s["soln"] = get_attr(msg, "carrSoln")
                    s["diff"] = get_attr(msg, "diffSoln")
                    s["rel"] = get_attr(msg, "relPosValid")
                    s["hacc"] = math.hypot(get_attr(msg, "accN"), get_attr(msg, "accE"))
                    s["pos"] = (get_attr(msg, "relPosN") / 100.0,
                                get_attr(msg, "relPosE") / 100.0,
                                get_attr(msg, "relPosD") / 100.0)
                elif ident == "RXM-RTCM":
                    s["ix"] += 1
                    mt = get_attr(msg, "msgType")
                    s["counts"][mt] = s["counts"].get(mt, 0) + 1
                    if crc_failed(msg):
                        s["crc"] += 1
                    elif mt in OBS_TYPES:
                        s["last_obs"] = time.time()

    # ---------- refresh ----------
    def _refresh(self):
        with self.lock:
            snap = dict(self.state)
            snap["counts"] = dict(self.state["counts"])
        self.info_var.set(self._status_text(snap))
        self._draw_map(snap)
        self.root.after(250, self._refresh)

    def _status_text(self, s):
        age = f"{time.time() - s['last_obs']:.1f}s" if s["last_obs"] else "-"
        hacc = f"{s['hacc']:.0f} mm" if s["hacc"] is not None else "-"
        lines = [
            f"sats {s['sats']}    fix {yn(s['fix'])}    soln {SOLN.get(s['soln'], '?')}",
            f"diff {yn(s['diff'])}    rel {yn(s['rel'])}    age {age}    hAcc {hacc}",
            f"RTCM {s['ix']} msgs    crc_fail {s['crc']}",
            "",
        ]
        if s["pos"] is not None and s["rel"]:
            n, e, d = s["pos"]
            dd = derived(n, e, d)
            lines.append(f"{'axis':<11}{'meters':>12}   {'ft+in':>11}")
            for lbl, v in (("North", n), ("East", e), ("Down", d)):
                lines.append(f"{lbl:<11}{v:>+12.4f}   {ftin(v):>11}")
            lines.append(f"{'Distance':<11}{dd['dist']:>12.4f}   {ftin(dd['dist']):>11}")
            lines.append(f"{'Horizontal':<11}{dd['horiz']:>12.4f}   {ftin(dd['horiz']):>11}")
            grade = f"{dd['grade']:+.1f}%" if math.isfinite(dd["grade"]) else "vertical"
            lines.append(f"{'H slope':<11}{dd['bearing']:>12.1f} deg (N->E)")
            lines.append(f"{'V slope':<11}{dd['vert']:>+12.1f} deg ({grade})")
        else:
            lines.append("(waiting for a valid relative position)")
        return "\n".join(lines)

    # ---------- map ----------
    def _draw_map(self, snap):
        c = self.canvas
        c.delete("all")
        W, H, margin = self.W, self.H, 36
        recorded = [(p["E"], p["N"]) for p in self.points]
        live = None
        if snap["pos"] is not None:
            n, e, d = snap["pos"]
            live = (e, n)
        allpts = [(0.0, 0.0)] + recorded + ([live] if live else [])
        xs = [p[0] for p in allpts]
        ys = [p[1] for p in allpts]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        spanx = max(maxx - minx, 1.0)
        spany = max(maxy - miny, 1.0)
        minx -= spanx * 0.15; maxx += spanx * 0.15
        miny -= spany * 0.15; maxy += spany * 0.15
        scale = min((W - 2 * margin) / (maxx - minx), (H - 2 * margin) / (maxy - miny))
        cx, cy = (minx + maxx) / 2, (miny + maxy) / 2

        def to_screen(e, n):
            return W / 2 + (e - cx) * scale, H / 2 - (n - cy) * scale

        # polyline through recorded points in order
        if len(recorded) >= 2:
            coords = []
            for e, n in recorded:
                coords.extend(to_screen(e, n))
            c.create_line(*coords, fill="#d33", width=2)
        # recorded points
        for i, (e, n) in enumerate(recorded, 1):
            sx, sy = to_screen(e, n)
            c.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, fill="#d33", outline="")
            c.create_text(sx + 6, sy - 6, text=str(i), fill="#a00",
                          anchor="w", font=("TkDefaultFont", 8))
        # base marker at origin
        bx, by = to_screen(0, 0)
        c.create_rectangle(bx - 5, by - 5, bx + 5, by + 5, fill="#1a9", outline="black")
        c.create_text(bx + 8, by + 8, text="BASE", fill="#0a7",
                      anchor="w", font=("TkDefaultFont", 8, "bold"))
        # live rover marker
        if live:
            lx, ly = to_screen(*live)
            c.create_oval(lx - 5, ly - 5, lx + 5, ly + 5, outline="#36c", width=2)
        # north arrow
        c.create_line(W - 20, 30, W - 20, 14, arrow=tk.LAST, width=2)
        c.create_text(W - 20, 40, text="N", font=("TkDefaultFont", 9, "bold"))
        # scale bar
        bar_m = nice_len((W - 2 * margin) / scale / 4)
        px = bar_m * scale
        x0, y0 = margin, H - margin
        c.create_line(x0, y0, x0 + px, y0, fill="black", width=2)
        c.create_text(x0 + px / 2, y0 - 8, text=fmt_len(bar_m), font=("TkDefaultFont", 8))

    # ---------- buttons ----------
    def _record(self):
        with self.lock:
            snap = dict(self.state)
        if snap["pos"] is None or not snap["rel"]:
            messagebox.showwarning("No position",
                                   "No valid relative position to record yet.")
            return
        n, e, d = snap["pos"]
        dd = derived(n, e, d)
        rec = {
            "time": datetime.now().isoformat(timespec="seconds"),
            "soln": SOLN.get(snap["soln"], "?"),
            "sats": snap["sats"],
            "hAcc_mm": round(snap["hacc"], 1) if snap["hacc"] is not None else "",
            "N": round(n, 4), "E": round(e, 4), "D": round(d, 4),
            "horiz": round(dd["horiz"], 4), "dist": round(dd["dist"], 4),
            "N_ftin": ftin(n), "E_ftin": ftin(e), "D_ftin": ftin(d),
            "horiz_ftin": ftin(dd["horiz"]), "dist_ftin": ftin(dd["dist"]),
            "bearing_deg": round(dd["bearing"], 1),
            "vert_deg": round(dd["vert"], 1),
            "grade_pct": (round(dd["grade"], 1) if math.isfinite(dd["grade"]) else "inf"),
        }
        self.points.append(rec)
        self.count_var.set(f"{len(self.points)} recorded")
        if snap["soln"] != 2:
            self.status_var.set(f"Recorded point {len(self.points)} "
                                f"(WARNING: soln={SOLN.get(snap['soln'], '?')}, not FIXED)")
        else:
            self.status_var.set(f"Recorded point {len(self.points)} (FIXED)")

    def _save(self):
        if not self.points:
            messagebox.showinfo("Nothing to save", "No recorded points yet.")
            return
        fn = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=f"rtk_points_{datetime.now():%Y%m%d_%H%M%S}.csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if not fn:
            return
        try:
            with open(fn, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=CSV_COLS)
                w.writeheader()
                for p in self.points:
                    w.writerow({k: p.get(k, "") for k in CSV_COLS})
            self.status_var.set(f"Saved {len(self.points)} points")
            messagebox.showinfo("Saved", f"Wrote {len(self.points)} points to\n{fn}")
        except Exception as ex:
            messagebox.showerror("Save failed", str(ex))

    def _clear(self):
        if self.points and not messagebox.askyesno(
                "Clear", f"Discard {len(self.points)} recorded points?"):
            return
        self.points.clear()
        self.count_var.set("0 recorded")

    def _on_close(self):
        self._disconnect()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description="GUI RTK rover monitor + recorder.")
    ap.add_argument("port", nargs="?", default="", help="Rover serial port (optional)")
    ap.add_argument("-b", "--baud", type=int, default=9600, help="USB baud (default 9600)")
    args = ap.parse_args()

    root = tk.Tk()
    RTKRover(root, args.port, args.baud)
    root.mainloop()


if __name__ == "__main__":
    main()
