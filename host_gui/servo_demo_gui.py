#!/usr/bin/env python3
"""
Desktop GUI to slowly nudge two RDS5180 servos over USB serial.
No Wi-Fi / no hotspot — talks to the ESP32 on /dev/cu.usbserial-* .

Run:
    python3 host_gui/servo_demo_gui.py
    python3 host_gui/servo_demo_gui.py --port /dev/cu.usbserial-0001

Requires: pyserial   (pip3 install pyserial)

Firmware: servo_demo/servo_demo.ino  (motor 0 -> GPIO18, motor 1 -> GPIO19)
Protocol:
    Host  -> board:  PING | MOVE <id> <deg> | JOG <id> <ddeg> | SPEED <id> <dps> |
                     CENTER [id] | STOP [id] | LIMITS <id> <lo> <hi> | STATUS
    Board -> host :  STATUS online=1 m0_pos=.. m0_target=.. m0_speed=.. m0_min=..
                     m0_max=.. m0_moving=.. m1_pos=.. ... | OK <cmd> | ERR <..> | PONG
"""
import argparse
import glob
import json
import os
import queue
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import serial  # pyserial
    from serial.tools import list_ports
except ImportError:
    raise SystemExit("pyserial is required:  pip3 install pyserial")

BAUD = 115200

# --------------------------------------------------------------------------- #
#  Color palette
# --------------------------------------------------------------------------- #
#  macOS renders native tk.Button faces light-grey and ignores `bg`, so all
#  colored buttons look identical. ColorButton below is a Frame+Label that
#  honors its colors on every platform.
COL_WINDOW   = "#0d1117"   # app background (deep slate, not pure black)
COL_CARD_A   = "#1b2430"   # servo card
COL_CARD_B   = "#15212e"   # stepper card
COL_BORDER   = "#3a4757"   # card outline so cards separate from background
COL_TEXT     = "#e6edf3"
COL_MUTED    = "#8b97a6"
COL_ACCENT   = "#4ea1ff"
COL_NEG      = "#b4561f"   # jog negative (amber-brown)
COL_POS      = "#1f9d57"   # jog positive (green)
COL_PRIMARY  = "#2b6cb0"   # center / set-zero (blue)
COL_STOP     = "#b3331f"   # stop (red)
COL_ESTOP    = "#000000"   # emergency stop (black, red border)


class ColorButton(tk.Frame):
    """A flat colored button that renders identically on macOS/Linux/Windows.

    Native tk.Button on macOS ignores `bg`; this wraps a Label in a Frame and
    binds clicks so the requested color is always shown.
    """

    def __init__(self, parent, text, bg, command, fg="white",
                 font=("Helvetica", 13, "bold"), pady=10,
                 border=None, border_w=0):
        outline = border or bg
        super().__init__(parent, bg=outline, highlightthickness=0,
                         bd=0, padx=border_w, pady=border_w)
        self._bg = bg
        self._hover = self._tint(bg, 1.18)
        self._press = self._tint(bg, 0.82)
        self.lbl = tk.Label(self, text=text, bg=bg, fg=fg, font=font,
                            pady=pady, cursor="pointinghand")
        self.lbl.pack(fill="both", expand=True)
        self._cmd = command
        for w in (self, self.lbl):
            w.bind("<Enter>", lambda e: self.lbl.config(bg=self._hover))
            w.bind("<Leave>", lambda e: self.lbl.config(bg=self._bg))
            w.bind("<ButtonPress-1>", lambda e: self.lbl.config(bg=self._press))
            w.bind("<ButtonRelease-1>", self._release)

    def _release(self, _e):
        self.lbl.config(bg=self._hover)
        if self._cmd:
            self._cmd()

    @staticmethod
    def _tint(hexcol, factor):
        hexcol = hexcol.lstrip("#")
        r, g, b = (int(hexcol[i:i + 2], 16) for i in (0, 2, 4))
        f = lambda v: max(0, min(255, int(v * factor)))  # noqa: E731
        return f"#{f(r):02x}{f(g):02x}{f(b):02x}"


NUM_MOTORS = 2


def autodetect_port():
    candidates = [p.device for p in list_ports.comports()]
    candidates += glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.usbmodem*")
    seen, out = set(), []
    for c in candidates:
        if c and c not in seen:
            seen.add(c)
            out.append(c)
    for c in out:
        if any(t in c for t in ("usbserial", "usbmodem", "wchusb", "SLAB")):
            return c
    return out[0] if out else None


class SerialLink:
    """Background reader/writer that owns all serial I/O and auto-reconnects."""

    def __init__(self):
        self.ser = None
        self.port = None
        self.rx = queue.Queue()
        self.tx = queue.Queue()
        self._stop = threading.Event()
        self._thread = None
        self._connected = threading.Event()

    def open(self, port):
        self.close()
        self.port = port
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
        self._connected.clear()
        self.port = None
        try:
            while True:
                self.tx.get_nowait()
        except queue.Empty:
            pass

    def is_open(self):
        return self._connected.is_set()

    def send(self, text):
        self.tx.put(text.strip())

    def _worker(self):
        buf = b""
        announced_wait = False
        while not self._stop.is_set():
            if self.ser is None:
                try:
                    ser = serial.Serial(self.port, BAUD, timeout=0.15)
                except Exception:
                    if not announced_wait:
                        self.rx.put(f"-- waiting for {self.port} --")
                        announced_wait = True
                    self._connected.clear()
                    time.sleep(0.8)
                    continue
                self.ser = ser
                announced_wait = False
                time.sleep(1.5)   # ESP32 reboots when the port opens
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass
                buf = b""
                self._connected.set()
                self.rx.put(f"-- connected {self.port} --")
                self.tx.put("STATUS")

            try:
                while True:
                    cmd = self.tx.get_nowait()
                    self.ser.write((cmd + "\n").encode())
            except queue.Empty:
                pass
            except Exception:
                self._drop("-- link lost (write) --"); buf = b""; continue

            try:
                data = self.ser.read(256)
            except Exception:
                self._drop("-- link lost (read) --"); buf = b""; continue

            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode(errors="replace").strip()
                    if s:
                        self.rx.put(s)

        self._drop(None)

    def _drop(self, notice):
        self._connected.clear()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        if notice:
            self.rx.put(notice)


def parse_status(line):
    d = {}
    for tok in line.split()[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            d[k] = v
    return d


# --------------------------------------------------------------------------- #
#  Tic T249 stepper — controlled DIRECTLY over USB via ticcmd (not the ESP32)
# --------------------------------------------------------------------------- #
TIC_PATHS = [
    "/Applications/Pololu Tic Stepper Motor Controller.app/Contents/MacOS/ticcmd",
    "/usr/local/bin/ticcmd",
    "/opt/homebrew/bin/ticcmd",
    "ticcmd",
]

# Motion geometry: NEMA23 200 full-steps/rev, Tic step-mode 1/32, 5:1 gearbox.
#   microsteps per OUTPUT-shaft degree = 200 * 32 * GEAR_RATIO / 360
# Finest microstepping the T249 supports (1/32) → smoothest motion, least
# resonance, fewest lost steps → most accurate output angle.
FULL_STEPS_PER_REV = 200
STEP_MODE = 32                 # 1/32 microstepping (T249 max)
# Measured: nominal 5:1 over-travelled ~5x, so effective ratio is 1:1.
GEAR_RATIO = 1.0               # calibrated — divide travel by 5 vs old 5.0
USTEPS_PER_OUT_DEG = FULL_STEPS_PER_REV * STEP_MODE * GEAR_RATIO / 360.0  # 17.778
TIC_CURRENT_MA = 2880          # NEMA23 rated 2.9 A — full torque (AGC stays off)
ACCEL_OUT_DPS2 = 6.0           # gentle output-shaft acceleration, deg/s^2


def find_ticcmd():
    import shutil
    for p in TIC_PATHS:
        if os.path.isabs(p) and os.path.exists(p):
            return p
        if not os.path.isabs(p) and shutil.which(p):
            return p
    return None


class TicLink:
    """Background worker that drives the Tic T249 via the ticcmd CLI.

    GUI thread enqueues argument lists; a worker thread runs them serially so
    the Tkinter loop never blocks on subprocess calls. Status is polled
    periodically and pushed to `rx` as a dict.
    """

    def __init__(self):
        self.ticcmd = find_ticcmd()
        self.tx = queue.Queue()
        self.rx = queue.Queue()
        self.speed_dps = 15.0
        self._run = False
        self._thread = None
        self._last_poll = 0.0

    def available(self):
        return self.ticcmd is not None

    def start(self):
        if self._run or not self.ticcmd:
            return
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        # One-time configuration + energize.
        self.configure()

    def stop(self):
        self._run = False

    # ---- public command API (output-shaft degrees) ----
    def configure(self):
        accel = int(round(ACCEL_OUT_DPS2 * USTEPS_PER_OUT_DEG * 100))
        self._send([
            "--current", str(TIC_CURRENT_MA),
            "--step-mode", str(STEP_MODE),
            "--agc-mode", "off",          # full current = max torque/holding
            "--max-accel", str(accel),    # gentle ramp = no skipped steps
            "--max-decel", str(accel),
            "--resume",
        ])
        self.set_speed(self.speed_dps)

    def set_speed(self, dps):
        self.speed_dps = max(0.1, float(dps))
        # Tic --max-speed unit = microsteps/10000 s
        msteps_per_s = self.speed_dps * USTEPS_PER_OUT_DEG
        self._send(["--max-speed", str(int(round(msteps_per_s * 10000)))])

    def move_to(self, deg):
        self._send(["--exit-safe-start", "--position",
                    str(int(round(deg * USTEPS_PER_OUT_DEG)))])

    def jog(self, ddeg):
        self._send(["--exit-safe-start", "--position-relative",
                    str(int(round(ddeg * USTEPS_PER_OUT_DEG)))])

    def zero(self):
        self._send(["--halt-and-set-position", "0"])

    def halt(self):
        self._send(["--halt-and-hold"])

    # ---- internals ----
    def _send(self, args):
        self.tx.put(args)

    def _run_ticcmd(self, args):
        try:
            out = subprocess.run([self.ticcmd] + args, capture_output=True,
                                 text=True, timeout=6)
            return out.stdout, out.stderr
        except Exception as e:  # noqa: BLE001
            return "", str(e)

    def _poll_status(self):
        out, err = self._run_ticcmd(["--status"])
        if err and not out:
            self.rx.put({"st_online": "0", "st_err": err.strip()[:60]})
            return
        d = {"st_online": "1"}
        for line in out.splitlines():
            if ":" not in line:
                continue
            key, _, val = line.partition(":")
            key, val = key.strip().lower(), val.strip()
            if key == "current position":
                try:
                    d["st_pos_deg"] = int(val) / USTEPS_PER_OUT_DEG
                except ValueError:
                    pass
            elif key == "target position":
                try:
                    d["st_target_deg"] = int(val) / USTEPS_PER_OUT_DEG
                except ValueError:
                    pass
            elif key == "operation state":
                d["st_state"] = val
            elif key == "energized":
                d["st_energized"] = val
            elif key == "vin voltage":
                d["st_vin"] = val
            elif key in ("error status", "errors currently stopping the motor"):
                if val and val not in ("0", "None", "-"):
                    d["st_err"] = val
        cp = d.get("st_pos_deg")
        tp = d.get("st_target_deg")
        d["st_moving"] = "1" if (cp is not None and tp is not None
                                 and abs(cp - tp) > 0.05) else "0"
        self.rx.put(d)

    def _loop(self):
        while self._run:
            ran = False
            try:
                while True:
                    args = self.tx.get_nowait()
                    self._run_ticcmd(args)
                    ran = True
            except queue.Empty:
                pass
            now = time.time()
            if ran or (now - self._last_poll) > 0.7:
                self._poll_status()
                self._last_poll = now
            time.sleep(0.05)


class MotorPanel:
    """One card of controls for a single motor."""

    def __init__(self, parent, app, mid, title):
        self.app = app
        self.mid = mid
        self.pos = 135.0

        card = tk.Frame(parent, bg=COL_CARD_A, highlightbackground=COL_BORDER,
                        highlightthickness=1, bd=0)
        card.pack(side="left", expand=True, fill="both", padx=6)

        head = tk.Frame(card, bg=COL_CARD_A)
        head.pack(fill="x", padx=12, pady=(10, 0))
        tk.Label(head, text=title, bg=COL_CARD_A, fg=COL_TEXT,
                 font=("Helvetica", 13, "bold")).pack(side="left")
        self.pos_lbl = tk.Label(head, text="--\u00b0", bg=COL_CARD_A, fg=COL_ACCENT,
                                font=("Helvetica", 20, "bold"))
        self.pos_lbl.pack(side="right")

        # Jog buttons
        row = tk.Frame(card, bg=COL_CARD_A)
        row.pack(fill="x", padx=6, pady=(8, 4))
        for label, delta in (("-5\u00b0", -5), ("-1\u00b0", -1),
                             ("+1\u00b0", 1), ("+5\u00b0", 5)):
            col = COL_NEG if delta < 0 else COL_POS
            ColorButton(row, label, col,
                        command=lambda d=delta: self.app.cmd(f"JOG {self.mid} {d}"),
                        font=("Helvetica", 13, "bold")).pack(
                            side="left", expand=True, fill="x", padx=2)

        # Emergency stop — halt in place but stay energized (holding torque).
        estop = tk.Frame(card, bg=COL_CARD_A)
        estop.pack(fill="x", padx=6, pady=(4, 2))
        ColorButton(estop, "\u26d4  E-STOP  (stop but stay powered)", COL_ESTOP,
                    command=self.estop, fg="#ff5a5a",
                    font=("Helvetica", 14, "bold"),
                    border="#ff3b3b", border_w=2).pack(fill="x", padx=2)

        # Speed slider
        self.speed = self._slider(card, "Speed \u00b0/s (small=slower)", 1, 60, 8,
                                  lambda v: self.app.cmd(f"SPEED {self.mid} {v}"), "{}")
        # Absolute target slider (0..270 deg full travel)
        self.target = self._slider(card, "Go to angle", 0, 270, 135,
                                   lambda v: self.app.cmd(f"MOVE {self.mid} {v}"), "{}\u00b0")
        # Manual numeric target — type an exact angle and press GO / Enter
        self._goto_row(card, 0, 270)

        # Center / Stop
        act = tk.Frame(card, bg=COL_CARD_A)
        act.pack(fill="x", padx=6, pady=(2, 8))
        ColorButton(act, "CENTER", COL_PRIMARY,
                    command=lambda: self.app.cmd(f"CENTER {self.mid}"),
                    font=("Helvetica", 12, "bold")).pack(
                        side="left", expand=True, fill="x", padx=2)
        ColorButton(act, "STOP", COL_STOP,
                    command=lambda: self.app.cmd(f"STOP {self.mid}"),
                    font=("Helvetica", 12, "bold")).pack(
                        side="left", expand=True, fill="x", padx=2)

        # Per-motor status line
        self.info = tk.Label(card, text="", bg=COL_CARD_A, fg=COL_MUTED,
                             anchor="w", justify="left", font=("Menlo", 9))
        self.info.pack(fill="x", padx=12, pady=(0, 10))

    def _slider(self, card, title, lo, hi, init, cb, fmt):
        head = tk.Frame(card, bg=COL_CARD_A)
        head.pack(fill="x", padx=12, pady=(6, 0))
        tk.Label(head, text=title, bg=COL_CARD_A, fg=COL_TEXT,
                 font=("Helvetica", 11)).pack(side="left")
        val = tk.Label(head, text=fmt.format(init), bg=COL_CARD_A, fg=COL_ACCENT,
                       font=("Helvetica", 12, "bold"))
        val.pack(side="right")
        s = tk.Scale(card, from_=lo, to=hi, orient="horizontal", bg=COL_CARD_A,
                     fg=COL_TEXT, troughcolor="#0e1620", highlightthickness=0,
                     activebackground=COL_ACCENT, showvalue=False)
        s.set(init)
        s.pack(fill="x", padx=12, pady=(0, 6))
        s.configure(command=lambda v: val.config(text=fmt.format(int(float(v)))))
        s.bind("<ButtonRelease-1>", lambda e: cb(int(s.get())))
        return s

    def _goto_row(self, card, lo, hi):
        """Row with a numeric entry: type an exact angle, GO/Enter to move there."""
        row = tk.Frame(card, bg=COL_CARD_A)
        row.pack(fill="x", padx=12, pady=(0, 6))
        tk.Label(row, text="Type angle \u25b8", bg=COL_CARD_A, fg=COL_TEXT,
                 font=("Helvetica", 11)).pack(side="left")
        self.goto_var = tk.StringVar()
        ent = tk.Entry(row, textvariable=self.goto_var, width=8, justify="right",
                       bg="#0e1620", fg=COL_TEXT, insertbackground=COL_TEXT,
                       relief="flat", highlightthickness=1,
                       highlightbackground=COL_BORDER)
        ent.pack(side="left", padx=6)
        ent.bind("<Return>", lambda e: self._goto(lo, hi))
        ColorButton(row, "GO", COL_PRIMARY, command=lambda: self._goto(lo, hi),
                    font=("Helvetica", 11, "bold"), pady=4).pack(side="left")

    def _goto(self, lo, hi):
        try:
            v = float(self.goto_var.get().strip())
        except ValueError:
            return
        v = max(lo, min(hi, v))
        self.target.set(int(round(v)))
        self.app.cmd(f"MOVE {self.mid} {v}")

    def estop(self):
        """Emergency stop: halt this servo where it is, stay energized.

        STOP <id> sets the firmware target to the live position, so motion
        halts immediately but PWM keeps being sent (servo stays powered with
        holding torque). The target slider is snapped to the current position
        so a later touch doesn't fling it back to the old target.
        """
        self.app.cmd(f"STOP {self.mid}")
        try:
            self.target.set(int(round(self.pos)))
        except Exception:  # noqa: BLE001
            pass

    def update_from(self, d):
        p = f"m{self.mid}_"
        try:
            self.pos = float(d.get(p + "pos", self.pos))
        except ValueError:
            pass
        moving = d.get(p + "moving") == "1"
        self.pos_lbl.config(text=f"{self.pos:.1f}\u00b0",
                            fg="#ffd166" if moving else "#4ea1ff")
        self.info.config(text=(
            f"target {d.get(p+'target','?')}\u00b0  "
            f"speed {d.get(p+'speed','?')}\u00b0/s  "
            f"{'MOVING' if moving else 'idle'}\n"
            f"range {d.get(p+'min','?')}–{d.get(p+'max','?')}\u00b0"))

class StepperPanel:
    """Controls for the Tic T249 stepper (NEMA 23, 5:1 gearbox).

    The Tic is connected DIRECTLY to the computer over USB and driven via the
    ticcmd CLI (see TicLink) — it does NOT go through the ESP32.
    """

    def __init__(self, parent, app):
        self.app = app
        self.tic = app.tic
        self.pos = 0.0

        card = tk.Frame(parent, bg=COL_CARD_B, highlightbackground=COL_BORDER,
                        highlightthickness=1, bd=0)
        card.pack(fill="x", padx=10, pady=(2, 6))

        head = tk.Frame(card, bg=COL_CARD_B)
        head.pack(fill="x", padx=12, pady=(8, 0))
        tk.Label(head, text="Stepper  (Tic T249 · NEMA23 · 5:1 · USB)", bg=COL_CARD_B,
                 fg=COL_TEXT, font=("Helvetica", 13, "bold")).pack(side="left")
        self.pos_lbl = tk.Label(head, text="--\u00b0", bg=COL_CARD_B, fg=COL_ACCENT,
                                font=("Helvetica", 20, "bold"))
        self.pos_lbl.pack(side="right")

        # Jog buttons (output-shaft degrees)
        row = tk.Frame(card, bg=COL_CARD_B)
        row.pack(fill="x", padx=6, pady=(8, 4))
        for label, delta in (("-90\u00b0", -90), ("-10\u00b0", -10), ("-1\u00b0", -1),
                             ("+1\u00b0", 1), ("+10\u00b0", 10), ("+90\u00b0", 90)):
            col = COL_NEG if delta < 0 else COL_POS
            ColorButton(row, label, col,
                        command=lambda dd=delta: self.tic.jog(dd),
                        font=("Helvetica", 12, "bold")).pack(
                            side="left", expand=True, fill="x", padx=2)

        # Emergency stop — halt-and-hold: stop instantly but stay energized.
        estop = tk.Frame(card, bg=COL_CARD_B)
        estop.pack(fill="x", padx=6, pady=(4, 2))
        ColorButton(estop, "\u26d4  E-STOP  (stop but stay powered)", COL_ESTOP,
                    command=self.estop, fg="#ff5a5a",
                    font=("Helvetica", 14, "bold"),
                    border="#ff3b3b", border_w=2).pack(fill="x", padx=2)

        # Speed slider (output deg/sec)
        self.speed = self._slider(card, "Speed \u00b0/s (output)", 1, 90, 15,
                                  lambda v: self.tic.set_speed(v), "{}")
        # Absolute target slider (multi-turn output angle)
        self.target = self._slider(card, "Go to angle (output)", -360, 360, 0,
                                   lambda v: self.tic.move_to(v), "{}\u00b0")
        # Manual numeric target — type an exact angle and press GO / Enter
        self._goto_row(card)

        # Zero / Stop
        act = tk.Frame(card, bg=COL_CARD_B)
        act.pack(fill="x", padx=6, pady=(2, 8))
        ColorButton(act, "SET ZERO", COL_PRIMARY,
                    command=lambda: self.tic.zero(),
                    font=("Helvetica", 12, "bold")).pack(
                        side="left", expand=True, fill="x", padx=2)
        ColorButton(act, "STOP", COL_STOP,
                    command=lambda: self.tic.halt(),
                    font=("Helvetica", 12, "bold")).pack(
                        side="left", expand=True, fill="x", padx=2)

        self.info = tk.Label(card, text="", bg=COL_CARD_B, fg=COL_MUTED,
                             anchor="w", justify="left", font=("Menlo", 9))
        self.info.pack(fill="x", padx=12, pady=(0, 10))
        if not self.tic.available():
            self.pos_lbl.config(text="no ticcmd", fg="#ff6b6b")
            self.info.config(text="ticcmd not found — install Pololu Tic Software")

    def _slider(self, card, title, lo, hi, init, cb, fmt):
        head = tk.Frame(card, bg=COL_CARD_B)
        head.pack(fill="x", padx=12, pady=(6, 0))
        tk.Label(head, text=title, bg=COL_CARD_B, fg=COL_TEXT,
                 font=("Helvetica", 11)).pack(side="left")
        val = tk.Label(head, text=fmt.format(init), bg=COL_CARD_B, fg=COL_ACCENT,
                       font=("Helvetica", 12, "bold"))
        val.pack(side="right")
        s = tk.Scale(card, from_=lo, to=hi, orient="horizontal", bg=COL_CARD_B,
                     fg=COL_TEXT, troughcolor="#0e1620", highlightthickness=0,
                     activebackground=COL_ACCENT, showvalue=False)
        s.set(init)
        s.pack(fill="x", padx=12, pady=(0, 6))
        s.configure(command=lambda v: val.config(text=fmt.format(int(float(v)))))
        s.bind("<ButtonRelease-1>", lambda e: cb(int(s.get())))
        return s

    def _goto_row(self, card):
        """Row with a numeric entry: type an exact output angle, GO/Enter to move.

        The stepper is multi-turn, so the typed value is not clamped.
        """
        row = tk.Frame(card, bg=COL_CARD_B)
        row.pack(fill="x", padx=12, pady=(0, 6))
        tk.Label(row, text="Type angle \u25b8", bg=COL_CARD_B, fg=COL_TEXT,
                 font=("Helvetica", 11)).pack(side="left")
        self.goto_var = tk.StringVar()
        ent = tk.Entry(row, textvariable=self.goto_var, width=8, justify="right",
                       bg="#0e1620", fg=COL_TEXT, insertbackground=COL_TEXT,
                       relief="flat", highlightthickness=1,
                       highlightbackground=COL_BORDER)
        ent.pack(side="left", padx=6)
        ent.bind("<Return>", lambda e: self._goto())
        ColorButton(row, "GO", COL_PRIMARY, command=self._goto,
                    font=("Helvetica", 11, "bold"), pady=4).pack(side="left")

    def _goto(self):
        if not self.tic.available():
            return
        try:
            v = float(self.goto_var.get().strip())
        except ValueError:
            return
        # Keep the slider in sync within its visible range; move to exact value.
        self.target.set(int(round(max(-360, min(360, v)))))
        self.tic.move_to(v)

    def estop(self):
        """Emergency stop: halt-and-hold the stepper, stay energized.

        --halt-and-hold abruptly stops the motor but leaves the driver
        energized (full holding torque). The target slider is snapped to the
        live position so a later touch doesn't resume the old move.
        """
        self.tic.halt()
        try:
            self.target.set(int(round(self.pos)))
        except Exception:  # noqa: BLE001
            pass

    def update_from(self, d):
        if "st_pos_deg" in d:
            self.pos = d["st_pos_deg"]
        online = d.get("st_online") == "1"
        moving = d.get("st_moving") == "1"
        if not online:
            self.pos_lbl.config(text="offline", fg="#ff6b6b")
        else:
            self.pos_lbl.config(text=f"{self.pos:.1f}\u00b0",
                                fg="#ffd166" if moving else "#4ea1ff")
        state = d.get("st_state", "?")
        energ = d.get("st_energized", "?")
        vin = d.get("st_vin", "?")
        target = d.get("st_target_deg")
        target_txt = f"{target:.1f}\u00b0" if target is not None else "?"
        err = d.get("st_err", "")
        self.info.config(text=(
            f"motor: {'online' if online else 'NOT DETECTED'}   "
            f"state {state}   energized {energ}   {'MOVING' if moving else 'idle'}\n"
            f"target {target_txt}   speed {self.tic.speed_dps:.0f}\u00b0/s   "
            f"VIN {vin}" + (f"   ERR {err}" if err else "")))

# --------------------------------------------------------------------------- #
#  Editable macro sequences (HOME / OPEN / CLOSE)
# --------------------------------------------------------------------------- #
#  Each step is one of:
#     ["servo", id, deg, speed]   id is 0 or 1
#     ["stepper", deg, speed]
#  Saved to / loaded from SEQ_FILE so edits survive relaunches.
SEQ_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sequences.json")

DEFAULT_SEQUENCES = {
    "HOME": [
        ["stepper", -84, 15],
        ["servo", 1, 0, 10],
        ["servo", 0, 150, 8],
    ],
    "OPEN": [
        ["stepper", -84, 15],
        ["servo", 1, 0, 10],
        ["servo", 0, 55, 8],
    ],
    "CLOSE": [
        ["stepper", -84, 15],
        ["servo", 1, 0, 10],
        ["servo", 0, 195, 8],
        ["stepper", 55, 15],
        ["stepper", 75, 2],
        ["servo", 1, 108, 10],
    ],
}


def load_sequences():
    seqs = {k: [list(s) for s in v] for k, v in DEFAULT_SEQUENCES.items()}
    try:
        with open(SEQ_FILE, "r") as f:
            data = json.load(f)
        for name in ("HOME", "OPEN", "CLOSE"):
            if isinstance(data.get(name), list) and data[name]:
                seqs[name] = [list(s) for s in data[name]]
    except (FileNotFoundError, ValueError, OSError):
        pass
    return seqs


def save_sequences(seqs):
    try:
        with open(SEQ_FILE, "w") as f:
            json.dump(seqs, f, indent=2)
        return True
    except OSError:
        return False


class SequenceEditor(tk.Toplevel):
    """Pop-up editor: add/remove/edit each step of HOME / OPEN / CLOSE."""

    def __init__(self, app):
        super().__init__(app.root, bg=COL_WINDOW)
        self.app = app
        self.title("Edit macro sequences")
        self.geometry("560x640")
        self.configure(bg=COL_WINDOW)
        # working copy: {name: [[kind,...], ...]}
        self.work = {k: [list(s) for s in v] for k, v in app.sequences.items()}
        self.rows = {}   # name -> list of row dicts

        head = tk.Frame(self, bg=COL_WINDOW)
        head.pack(fill="x", padx=12, pady=(12, 4))
        tk.Label(head, text="Edit each step \u00b7 servo id/deg/speed or stepper deg/speed",
                 bg=COL_WINDOW, fg=COL_MUTED, font=("Helvetica", 11)).pack(side="left")

        wrap = tk.Frame(self, bg=COL_WINDOW)
        wrap.pack(fill="both", expand=True, padx=8, pady=4)
        canvas = tk.Canvas(wrap, bg=COL_WINDOW, highlightthickness=0)
        sb = ttk.Scrollbar(wrap, orient="vertical", command=canvas.yview)
        self.inner = tk.Frame(canvas, bg=COL_WINDOW)
        self.inner.bind("<Configure>",
                        lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.inner, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        for name in ("HOME", "OPEN", "CLOSE"):
            self._build_macro_block(name)

        foot = tk.Frame(self, bg=COL_WINDOW)
        foot.pack(fill="x", padx=12, pady=10)
        ColorButton(foot, "SAVE", COL_POS, command=self._save,
                    font=("Helvetica", 12, "bold")).pack(side="right", padx=4)
        ColorButton(foot, "RESET DEFAULTS", COL_STOP, command=self._reset,
                    font=("Helvetica", 12, "bold")).pack(side="left", padx=4)
        ColorButton(foot, "CANCEL", COL_CARD_A, command=self.destroy, fg=COL_TEXT,
                    font=("Helvetica", 12, "bold"),
                    border=COL_BORDER, border_w=1).pack(side="right", padx=4)

    def _build_macro_block(self, name):
        card = tk.Frame(self.inner, bg=COL_CARD_A, highlightbackground=COL_BORDER,
                        highlightthickness=1)
        card.pack(fill="x", padx=6, pady=6)
        bar = tk.Frame(card, bg=COL_CARD_A)
        bar.pack(fill="x", padx=10, pady=(8, 2))
        tk.Label(bar, text=name, bg=COL_CARD_A, fg=COL_ACCENT,
                 font=("Helvetica", 14, "bold")).pack(side="left")
        ColorButton(bar, "+ Add step", COL_PRIMARY,
                    command=lambda n=name: self._add_row(n),
                    font=("Helvetica", 10, "bold"), pady=4).pack(side="right")
        body = tk.Frame(card, bg=COL_CARD_A)
        body.pack(fill="x", padx=10, pady=(0, 8))
        self.rows[name] = []
        self._bodies = getattr(self, "_bodies", {})
        self._bodies[name] = body
        for step in self.work[name]:
            self._add_row(name, step)

    def _add_row(self, name, step=None):
        body = self._bodies[name]
        row = tk.Frame(body, bg=COL_CARD_A)
        row.pack(fill="x", pady=2)
        kind = step[0] if step else "stepper"
        if kind == "servo":
            sid, deg, speed = step[1], step[2], step[3]
        else:
            sid, deg, speed = 0, (step[1] if step else 0), (step[2] if step else 10)

        type_var = tk.StringVar(value=("servo" if kind == "servo" else "stepper"))
        id_var = tk.StringVar(value=str(sid))
        deg_var = tk.StringVar(value=str(deg))
        spd_var = tk.StringVar(value=str(speed))

        type_cb = ttk.Combobox(row, textvariable=type_var, width=8, state="readonly",
                               values=("servo", "stepper"))
        type_cb.pack(side="left", padx=2)
        id_cb = ttk.Combobox(row, textvariable=id_var, width=3, state="readonly",
                             values=("0", "1"))
        id_cb.pack(side="left", padx=2)

        def mk_entry(var, w):
            return tk.Entry(row, textvariable=var, width=w, justify="right",
                            bg="#0e1620", fg=COL_TEXT, insertbackground=COL_TEXT,
                            relief="flat", highlightthickness=1,
                            highlightbackground=COL_BORDER)
        tk.Label(row, text="deg", bg=COL_CARD_A, fg=COL_MUTED).pack(side="left", padx=(6, 1))
        mk_entry(deg_var, 7).pack(side="left", padx=1)
        tk.Label(row, text="spd", bg=COL_CARD_A, fg=COL_MUTED).pack(side="left", padx=(6, 1))
        mk_entry(spd_var, 5).pack(side="left", padx=1)

        rd = {"row": row, "type": type_var, "id": id_var,
              "deg": deg_var, "spd": spd_var, "id_cb": id_cb}

        def sync_id(*_):
            id_cb.configure(state="readonly" if type_var.get() == "servo" else "disabled")
        type_cb.bind("<<ComboboxSelected>>", sync_id)
        sync_id()

        ColorButton(row, "\u2715", COL_STOP,
                    command=lambda: self._del_row(name, rd),
                    font=("Helvetica", 10, "bold"), pady=2).pack(side="right", padx=2)
        self.rows[name].append(rd)

    def _del_row(self, name, rd):
        rd["row"].destroy()
        self.rows[name].remove(rd)

    def _collect(self):
        out = {}
        for name in ("HOME", "OPEN", "CLOSE"):
            steps = []
            for rd in self.rows[name]:
                try:
                    deg = float(rd["deg"].get())
                    spd = float(rd["spd"].get())
                except ValueError:
                    raise ValueError(f"{name}: deg/spd must be numbers")
                deg = int(deg) if deg == int(deg) else deg
                spd = int(spd) if spd == int(spd) else spd
                if rd["type"].get() == "servo":
                    steps.append(["servo", int(rd["id"].get()), deg, spd])
                else:
                    steps.append(["stepper", deg, spd])
            out[name] = steps
        return out

    def _save(self):
        try:
            new = self._collect()
        except ValueError as e:
            self.app.logln(f"[edit error: {e}]")
            return
        self.app.sequences = new
        save_sequences(new)
        self.app.logln("[sequences saved]")
        self.destroy()

    def _reset(self):
        for name in ("HOME", "OPEN", "CLOSE"):
            for rd in list(self.rows[name]):
                rd["row"].destroy()
            self.rows[name] = []
            for step in DEFAULT_SEQUENCES[name]:
                self._add_row(name, list(step))


class App:
    def __init__(self, root, port):
        self.root = root
        self.link = SerialLink()
        self.tic = TicLink()
        self.sequences = load_sequences()
        root.title("ESP32 \u00b7 2 Servos + Tic Stepper (USB)")
        root.configure(bg=COL_WINDOW)
        root.geometry("680x780")

        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        # ---- Connection bar ----
        top = tk.Frame(root, bg=COL_WINDOW)
        top.pack(fill="x", padx=14, pady=(14, 6))
        tk.Label(top, text="Port", bg=COL_WINDOW, fg=COL_MUTED).pack(side="left")
        self.port_var = tk.StringVar(value=port or "")
        tk.Entry(top, textvariable=self.port_var, width=22,
                 bg="#0e1620", fg=COL_TEXT, insertbackground=COL_TEXT,
                 relief="flat", highlightthickness=1,
                 highlightbackground=COL_BORDER).pack(side="left", padx=6)
        self.conn_btn = tk.Button(top, text="Connect", command=self.toggle_conn,
                                  highlightbackground=COL_WINDOW)
        self.conn_btn.pack(side="left")

        # Limit-switch indicator LED (GPIO17). Lit red when triggered.
        lim = tk.Frame(top, bg=COL_WINDOW)
        lim.pack(side="left", padx=(14, 0))
        self.limit_led = tk.Canvas(lim, width=16, height=16, bg=COL_WINDOW,
                                   highlightthickness=0)
        self._limit_dot = self.limit_led.create_oval(2, 2, 14, 14,
                                                     fill="#3a4757", outline="")
        self.limit_led.pack(side="left")
        self.limit_lbl = tk.Label(lim, text="LIMIT \u2014", bg=COL_WINDOW,
                                  fg=COL_MUTED, font=("Helvetica", 11, "bold"))
        self.limit_lbl.pack(side="left", padx=(5, 0))

        ColorButton(top, "STOP BOTH", COL_STOP, command=lambda: self.cmd("STOP"),
                    font=("Helvetica", 11, "bold"), pady=5).pack(side="right", padx=4)
        ColorButton(top, "CENTER BOTH", COL_PRIMARY,
                    command=lambda: self.cmd("CENTER"),
                    font=("Helvetica", 11, "bold"), pady=5).pack(side="right", padx=4)

        # ---- Macro buttons (sequenced multi-motor routines) ----
        macro = tk.Frame(root, bg=COL_WINDOW)
        macro.pack(fill="x", padx=12, pady=(0, 4))
        ColorButton(macro, "\u5f52\u4f4d / HOME", COL_PRIMARY,
                    command=self.macro_home,
                    font=("Helvetica", 12, "bold")).pack(
                        side="left", expand=True, fill="x", padx=3)
        ColorButton(macro, "\u5f00\u95e8 / OPEN", COL_POS,
                    command=self.macro_open,
                    font=("Helvetica", 12, "bold")).pack(
                        side="left", expand=True, fill="x", padx=3)
        ColorButton(macro, "\u5173\u95e8 / CLOSE", COL_NEG,
                    command=self.macro_close,
                    font=("Helvetica", 12, "bold")).pack(
                        side="left", expand=True, fill="x", padx=3)
        ColorButton(macro, "\u2699 EDIT", COL_CARD_A,
                    command=self.open_sequence_editor, fg=COL_TEXT,
                    font=("Helvetica", 12, "bold"),
                    border=COL_BORDER, border_w=1).pack(
                        side="left", fill="x", padx=3)

        # ---- Two motor panels ----
        panels = tk.Frame(root, bg=COL_WINDOW)
        panels.pack(fill="both", expand=True, padx=10, pady=6)
        self.motors = [
            MotorPanel(panels, self, 0, "Motor 0  (GPIO18)"),
            MotorPanel(panels, self, 1, "Motor 1  (GPIO19)"),
        ]

        # ---- Stepper panel (full width) ----
        self.stepper = StepperPanel(root, self)

        # ---- Status + log ----
        self.status_lbl = tk.Label(root, text="not connected", bg=COL_WINDOW,
                                   fg="#ff6b6b", anchor="w", font=("Menlo", 11))
        self.status_lbl.pack(fill="x", padx=16, pady=(2, 0))
        self.log = tk.Text(root, height=5, bg="#0a0f15", fg="#7CFC9A",
                           insertbackground="#7CFC9A", font=("Menlo", 10),
                           relief="flat", highlightthickness=1,
                           highlightbackground=COL_BORDER)
        self.log.pack(fill="both", expand=False, padx=14, pady=(6, 12))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(50, self.pump)
        self.root.after(800, self.poll_status)

        # Start the Tic stepper worker (direct USB, independent of the ESP32).
        self.tic.start()

        if port:
            self.toggle_conn()

    # ---- Actions ----
    def toggle_conn(self):
        if getattr(self, "active", False):
            self.active = False
            self.link.close()
            self.conn_btn.config(text="Connect")
            self.set_status("disconnected", ok=False)
            self.logln("[disconnected]")
            return
        port = self.port_var.get().strip() or autodetect_port()
        if not port:
            self.logln("[no serial port found]")
            return
        self.active = True
        self.link.open(port)
        self.port_var.set(port)
        self.conn_btn.config(text="Disconnect")
        self.set_status("connecting...", ok=False)
        self.logln(f"[opening {port} — auto-reconnect on]")

    def cmd(self, c):
        self.link.send(c)
        self.logln(f"> {c}")

    # ---- Macro routines (run as ordered sequences in a worker thread) ----
    #   step = ("servo", id, deg, speed)  |  ("stepper", deg, speed)
    #   Sequences are editable (EDIT button) and persisted to SEQ_FILE.
    def macro_home(self):
        self.run_macro("HOME", self.sequences["HOME"])

    def macro_open(self):
        self.run_macro("OPEN", self.sequences["OPEN"])

    def macro_close(self):
        self.run_macro("CLOSE", self.sequences["CLOSE"])

    def open_sequence_editor(self):
        SequenceEditor(self)

    def run_macro(self, name, steps):
        if getattr(self, "_macro_running", False):
            self.logln(f"[macro busy \u2014 {name} ignored]")
            return
        if not self.link.is_open():
            self.logln(f"[{name}: not connected]")
            return
        self._macro_running = True
        self.logln(f"[macro {name} started]")
        threading.Thread(target=self._run_macro, args=(name, steps),
                         daemon=True).start()

    def _run_macro(self, name, steps):
        try:
            for st in steps:
                if st[0] == "servo":
                    _, mid, deg, speed = st
                    self.link.send(f"SPEED {mid} {speed}")
                    self.link.send(f"MOVE {mid} {deg}")
                    self.root.after(0, self._macro_log_servo, mid, deg, speed)
                    self._wait_servo(mid, deg)
                else:
                    _, deg, speed = st
                    self.tic.set_speed(speed)
                    self.tic.move_to(deg)
                    self.root.after(0, self._macro_log_stepper, deg, speed)
                    self._wait_stepper(deg)
            self.root.after(0, self.logln, f"[macro {name} done]")
        finally:
            self._macro_running = False

    def _macro_log_servo(self, mid, deg, speed):
        self.logln(f"> servo{mid} \u2192 {deg}\u00b0 @ {speed}\u00b0/s")
        try:
            self.motors[mid].target.set(int(round(deg)))
            self.motors[mid].speed.set(int(round(speed)))
        except Exception:  # noqa: BLE001
            pass

    def _macro_log_stepper(self, deg, speed):
        self.logln(f"> stepper \u2192 {deg}\u00b0 @ {speed}\u00b0/s")
        try:
            self.stepper.target.set(int(round(max(-360, min(360, deg)))))
            self.stepper.speed.set(int(round(speed)))
        except Exception:  # noqa: BLE001
            pass

    def _wait_servo(self, mid, target, tol=0.8, timeout=60):
        time.sleep(0.35)                      # let motion begin
        end = time.time() + timeout
        while time.time() < end:
            if abs(self.motors[mid].pos - target) <= tol:
                time.sleep(0.15)
                return
            time.sleep(0.1)

    def _wait_stepper(self, target, tol=1.0, timeout=240):
        time.sleep(0.35)
        end = time.time() + timeout
        while time.time() < end:
            if abs(self.stepper.pos - target) <= tol:
                time.sleep(0.15)
                return
            time.sleep(0.1)

    # ---- Loop tasks ----
    def poll_status(self):
        if getattr(self, "active", False):
            if self.link.is_open():
                self.link.send("STATUS")
            else:
                self.set_status("reconnecting... (check USB cable)", ok=False)
        self.root.after(300, self.poll_status)

    def pump(self):
        try:
            while True:
                self.handle_line(self.link.rx.get_nowait())
        except queue.Empty:
            pass
        try:
            while True:
                self.stepper.update_from(self.tic.rx.get_nowait())
        except queue.Empty:
            pass
        self.root.after(50, self.pump)

    def handle_line(self, line):
        if line.startswith("STATUS"):
            d = parse_status(line)
            for m in self.motors:
                m.update_from(d)
            self.update_limit_led(d.get("limit"))
            self.set_status("connected", ok=True)
        else:
            self.logln(f"< {line}")

    def update_limit_led(self, val):
        """Color the limit-switch LED: red=triggered, green=open, grey=unknown."""
        if val == "1":
            self.limit_led.itemconfig(self._limit_dot, fill="#ff3b3b")
            self.limit_lbl.config(text="LIMIT HIT", fg="#ff6b6b")
        elif val == "0":
            self.limit_led.itemconfig(self._limit_dot, fill="#1f9d57")
            self.limit_lbl.config(text="LIMIT open", fg="#7CFC9A")
        else:
            self.limit_led.itemconfig(self._limit_dot, fill="#3a4757")
            self.limit_lbl.config(text="LIMIT \u2014", fg=COL_MUTED)

    def set_status(self, text, ok):
        self.status_lbl.config(text=text, fg="#7CFC9A" if ok else "#ff6b6b")

    def logln(self, s):
        self.log.insert("end", s + "\n")
        self.log.see("end")

    def on_close(self):
        self.tic.stop()
        self.link.close()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="serial port (default: autodetect)")
    args = ap.parse_args()
    port = args.port or autodetect_port()
    root = tk.Tk()
    App(root, port)
    root.mainloop()


if __name__ == "__main__":
    main()
