#!/usr/bin/env python3
"""
Desktop GUI to control the ESP32 door controller directly over USB serial.
No Wi-Fi / no hotspot — talks to the board on /dev/cu.usbserial-* .

Run:
    python3 host_gui/door_gui.py
    python3 host_gui/door_gui.py --port /dev/cu.usbserial-0001

Requires: pyserial   (pip3 install pyserial)
GUI uses Tkinter, which ships with the standard macOS python.org Python.

Protocol implemented (matches esp32_servo_tic_demo.ino):
    Host -> board:  PING | OPEN | CLOSE | STOP | STEP <deg> | SERVO <deg> | STATUS
    Board -> host:  STATUS online=.. state=.. err=0x.. vin_mv=.. angle=.. target=.. servo=..
                    OK <cmd> | ERR <reason> | PONG | READY ...
"""
import argparse
import glob
import queue
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


def autodetect_port():
    # Prefer common ESP32 USB-UART names, else first cu.usbserial/usbmodem.
    candidates = []
    for p in list_ports.comports():
        candidates.append(p.device)
    candidates += glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.usbmodem*")
    # de-dup preserving order
    seen, out = set(), []
    for c in candidates:
        if c and c not in seen:
            seen.add(c)
            out.append(c)
    for c in out:
        if "usbserial" in c or "usbmodem" in c or "wchusb" in c or "SLAB" in c:
            return c
    return out[0] if out else None


class SerialLink:
    """Background reader/writer for the board on a worker thread."""

    def __init__(self):
        self.ser = None
        self.port = None            # the port we WANT connected (target)
        self.rx = queue.Queue()     # lines received from board (+ local notices)
        self.tx = queue.Queue()     # commands queued to send
        self._stop = threading.Event()
        self._thread = None
        self._connected = threading.Event()   # set when the port is live

    def open(self, port):
        """Set the target port and start the supervisor (which auto-reconnects)."""
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
        # drain any pending commands so a future session starts clean
        try:
            while True:
                self.tx.get_nowait()
        except queue.Empty:
            pass

    def is_open(self):
        return self._connected.is_set()

    def send(self, text):
        # Never write from the GUI thread; hand off to the worker.
        self.tx.put(text.strip())

    # --- single worker owns ALL serial I/O + reconnection ---
    def _worker(self):
        buf = b""
        announced_wait = False
        while not self._stop.is_set():
            # (Re)connect phase ------------------------------------------------
            if self.ser is None:
                try:
                    ser = serial.Serial(self.port, BAUD, timeout=0.15)
                except Exception:
                    if not announced_wait:
                        self.rx.put(f"-- waiting for {self.port} --")
                        announced_wait = True
                    self._connected.clear()
                    time.sleep(0.8)     # retry the port until it comes back
                    continue
                self.ser = ser
                announced_wait = False
                # ESP32 resets when the port opens; let it boot before talking.
                time.sleep(1.5)
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass
                buf = b""
                self._connected.set()
                self.rx.put(f"-- connected {self.port} --")
                self.tx.put("STATUS")   # refresh UI right after (re)connect

            # Write phase ------------------------------------------------------
            try:
                while True:
                    cmd = self.tx.get_nowait()
                    self.ser.write((cmd + "\n").encode())
            except queue.Empty:
                pass
            except Exception:
                self._drop("-- link lost (write) --")
                buf = b""
                continue

            # Read phase -------------------------------------------------------
            try:
                data = self.ser.read(256)
            except Exception:
                self._drop("-- link lost (read) --")
                buf = b""
                continue

            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode(errors="replace").strip()
                    if s:
                        self.rx.put(s)

        # Clean shutdown
        self._drop(None)

    def _drop(self, notice):
        """Tear down a dead handle so the worker reconnects."""
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
    """'STATUS k=v k=v ...' -> dict."""
    d = {}
    for tok in line.split()[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            d[k] = v
    return d


class App:
    def __init__(self, root, port):
        self.root = root
        self.link = SerialLink()
        root.title("ESP32 Door Control (USB serial)")
        root.configure(bg="#111")
        root.geometry("440x560")

        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        # ---- Connection bar ----
        top = tk.Frame(root, bg="#111")
        top.pack(fill="x", padx=14, pady=(14, 6))
        tk.Label(top, text="Port", bg="#111", fg="#aaa").pack(side="left")
        self.port_var = tk.StringVar(value=port or "")
        self.port_entry = tk.Entry(top, textvariable=self.port_var, width=22)
        self.port_entry.pack(side="left", padx=6)
        self.conn_btn = tk.Button(top, text="Connect", command=self.toggle_conn)
        self.conn_btn.pack(side="left")

        # ---- Big action buttons ----
        btns = tk.Frame(root, bg="#111")
        btns.pack(fill="x", padx=14, pady=10)
        self._mkbtn(btns, "OPEN", "#1f8a4c", lambda: self.cmd("OPEN")).pack(
            side="left", expand=True, fill="x", padx=4)
        self._mkbtn(btns, "CLOSE", "#9a4d1f", lambda: self.cmd("CLOSE")).pack(
            side="left", expand=True, fill="x", padx=4)
        self._mkbtn(btns, "STOP", "#a01f1f", lambda: self.cmd("STOP")).pack(
            side="left", expand=True, fill="x", padx=4)

        # ---- Angle slider (the "point on a line") ----
        self._slider_card(root, "Stepper angle (output)", -180, 180, 0,
                          self.on_step, "step")
        # ---- Servo slider ----
        self._slider_card(root, "Servo (test)", 0, 180, 90,
                          self.on_servo, "servo")

        # ---- Status panel ----
        card = tk.Frame(root, bg="#1e1e1e")
        card.pack(fill="x", padx=14, pady=10)
        tk.Label(card, text="Status", bg="#1e1e1e", fg="#fff",
                 font=("Helvetica", 13, "bold")).pack(anchor="w", padx=12, pady=(10, 4))
        self.status_lbl = tk.Label(card, text="not connected", bg="#1e1e1e",
                                   fg="#ff6b6b", justify="left", anchor="w",
                                   font=("Menlo", 11))
        self.status_lbl.pack(fill="x", padx=12, pady=(0, 12))

        # ---- Log ----
        self.log = tk.Text(root, height=6, bg="#0c0c0c", fg="#7CFC9A",
                           insertbackground="#7CFC9A", font=("Menlo", 10))
        self.log.pack(fill="both", expand=True, padx=14, pady=(0, 12))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(50, self.pump)
        self.root.after(800, self.poll_status)

        if port:
            self.toggle_conn()  # auto-connect if a port was found

    # ---- UI helpers ----
    def _mkbtn(self, parent, text, color, cmd):
        return tk.Button(parent, text=text, command=cmd, bg=color, fg="white",
                         activebackground=color, font=("Helvetica", 15, "bold"),
                         relief="flat", height=2, bd=0)

    def _slider_card(self, root, title, lo, hi, init, cb, key):
        card = tk.Frame(root, bg="#1e1e1e")
        card.pack(fill="x", padx=14, pady=6)
        head = tk.Frame(card, bg="#1e1e1e")
        head.pack(fill="x", padx=12, pady=(10, 0))
        tk.Label(head, text=title, bg="#1e1e1e", fg="#ddd",
                 font=("Helvetica", 12, "bold")).pack(side="left")
        val = tk.Label(head, text=f"{init}\u00b0", bg="#1e1e1e", fg="#4ea1ff",
                       font=("Helvetica", 14, "bold"))
        val.pack(side="right")
        s = tk.Scale(card, from_=lo, to=hi, orient="horizontal", bg="#1e1e1e",
                     fg="#ddd", troughcolor="#333", highlightthickness=0,
                     showvalue=False, length=380)
        s.set(init)
        s.pack(fill="x", padx=12, pady=(0, 12))
        # update label live; send command on release
        s.configure(command=lambda v: val.config(text=f"{int(float(v))}\u00b0"))
        s.bind("<ButtonRelease-1>", lambda e: cb(int(s.get())))
        setattr(self, f"slider_{key}", s)

    # ---- Actions ----
    def toggle_conn(self):
        # Button reflects INTENT; the link itself auto-reconnects while active.
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
        self.link.open(port)               # starts supervisor; reconnects itself
        self.port_var.set(port)
        self.conn_btn.config(text="Disconnect")
        self.set_status("connecting...", ok=False)
        self.logln(f"[opening {port} — auto-reconnect on]")

    def cmd(self, c):
        self.link.send(c)
        self.logln(f"> {c}")

    def on_step(self, deg):
        self.cmd(f"STEP {deg}")

    def on_servo(self, deg):
        self.cmd(f"SERVO {deg}")

    # ---- Loop tasks ----
    def poll_status(self):
        if getattr(self, "active", False):
            if self.link.is_open():
                self.link.send("STATUS")
            else:
                self.set_status("reconnecting... (check USB cable)", ok=False)
        self.root.after(1000, self.poll_status)

    def pump(self):
        try:
            while True:
                line = self.link.rx.get_nowait()
                self.handle_line(line)
        except queue.Empty:
            pass
        self.root.after(50, self.pump)

    def handle_line(self, line):
        if line.startswith("STATUS"):
            d = parse_status(line)
            online = d.get("online") == "1"
            vin = int(d.get("vin_mv", "0")) / 1000.0
            txt = (f"motor: {'online' if online else 'NOT DETECTED'}\n"
                   f"state: {d.get('state','?')}   err: {d.get('err','?')}\n"
                   f"VIN: {vin:.1f} V\n"
                   f"angle: {d.get('angle','?')}\u00b0  target: {d.get('target','?')}\u00b0  "
                   f"servo: {d.get('servo','?')}\u00b0")
            err = d.get("err", "0x0000")
            ok = online and err == "0x0000"
            self.set_status(txt, ok=ok)
        else:
            self.logln(f"< {line}")

    def set_status(self, text, ok):
        self.status_lbl.config(text=text, fg="#7CFC9A" if ok else "#ff6b6b")

    def logln(self, s):
        self.log.insert("end", s + "\n")
        self.log.see("end")

    def on_close(self):
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
