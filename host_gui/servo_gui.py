#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 舵机控制 + 实时电流/电压 GUI
  - 门 (Door, D18/0号) 与 handle (D19/1号) 的滑块 + 预设按钮控制
  - 实时显示 电压 / 电流 / 功率，以及电流曲线图
  - 通过串口向 ESP32 发送 "编号 角度" 命令
运行: /usr/local/bin/python3 servo_gui.py
"""
import tkinter as tk
from tkinter import ttk
import threading, time, re
from collections import deque
import serial

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

PORT = "/dev/cu.usbserial-0001"
BAUD = 115200
SERVO_MAX_DEG = 270   # 270° 舵机

METRIC_RE = re.compile(r"V=([-\d.]+)V\s+I=([-\d.]+)mA\s+P=([-\d.]+)mW")
ANGLE_RE  = re.compile(r"door=(\d+).*?handle=(\d+)")

# 配色（深色主题）
BG      = "#0f1419"
PANEL   = "#1a222c"
CARD    = "#141b23"
FG      = "#e6e6e6"
MUTED   = "#8aa0b3"
GREEN   = "#39d98a"
AMBER   = "#ffb454"
BLUE    = "#59c2ff"
PURPLE  = "#d2a8ff"
RED     = "#ff5f56"


class SerialWorker:
    """后台线程：读串口解析读数、发送舵机命令、断线自动重连。"""
    def __init__(self, port, baud):
        self.port, self.baud = port, baud
        self.ser = None
        self.lock = threading.Lock()
        self.connected = False
        self.v = self.i = self.p = None
        self.fault = False          # 急停状态
        self.fault_reason = ""      # 急停原因
        self.seq_status = None      # 序列执行状态文本
        self.door_switch = False    # D17 门关紧开关
        self.servos_idle = False    # 舵机待机(未上电)
        self.door_actual = None
        self.handle_actual = None
        self.t0 = time.time()
        self.hist = deque(maxlen=600)   # (t, current_mA)
        self.peak = 0.0
        self._stop = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _try_open(self):
        try:
            s = serial.Serial()
            s.port = self.port
            s.baudrate = self.baud
            s.timeout = 1
            # 尽量不复位 ESP32
            s.dtr = False
            s.rts = False
            s.open()
            try:
                s.dtr = False
                s.rts = False
            except Exception:
                pass
            time.sleep(0.2)
            s.reset_input_buffer()
            self.ser = s
            self.connected = True
            return True
        except Exception:
            self.ser = None
            self.connected = False
            return False

    def _reader(self):
        while not self._stop:
            if not self.connected:
                if not self._try_open():
                    time.sleep(1.0)
                    continue
            try:
                raw = self.ser.readline()
            except Exception:
                self._drop()
                continue
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").strip()
            m = METRIC_RE.search(line)
            if m:
                self.v = float(m.group(1))
                self.i = float(m.group(2))
                self.p = float(m.group(3))
                self.hist.append((time.time() - self.t0, self.i))
                if self.i > self.peak:
                    self.peak = self.i
                # 每条遥测都带状态标记：!!ESTOP:reason!! 或 {seq:name:step}
                fm = re.search(r"!!ESTOP:(.+?)!!", line)
                if fm:
                    self.fault = True
                    self.fault_reason = fm.group(1)
                else:
                    self.fault = False
                    self.fault_reason = ""
                sq = re.search(r"\{seq:(\w+):(\d+)\}", line)
                self.seq_status = f"{sq.group(1).capitalize()} · step {sq.group(2)}" if sq else None
                self.servos_idle = "{servos:idle}" in line
            elif "!!! ESTOP" in line:
                self.fault = True
            a = ANGLE_RE.search(line)
            if a:
                self.door_actual = int(a.group(1))
                self.handle_actual = int(a.group(2))
            sw = re.search(r"sw=(\d)", line)
            if sw:
                self.door_switch = sw.group(1) == "1"

    def _drop(self):
        self.connected = False
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def send_line(self, text):
        data = (text.strip() + "\n").encode()
        with self.lock:
            if self.ser and self.connected:
                try:
                    self.ser.write(data)
                    self.ser.flush()
                except Exception:
                    self._drop()

    def send(self, ch, angle):
        angle = max(0, min(SERVO_MAX_DEG, int(angle)))
        self.send_line(f"{ch} {angle}")

    def reset_peak(self):
        self.peak = 0.0

    def stop(self):
        self._stop = True
        self._drop()


class App:
    def __init__(self, root):
        self.root = root
        self.worker = SerialWorker(PORT, BAUD)

        root.title("ESP32 Servo Control + Current Monitor")
        root.configure(bg=BG)
        root.geometry("880x760")
        root.minsize(760, 680)

        self.door_target = tk.IntVar(value=190)   # 门初始=190(关紧位附近)
        self.handle_target = tk.IntVar(value=0)   # handle初始=0(解锁)
        self.last_door = 190
        self.last_handle = 0
        self.speed = tk.IntVar(value=30)     # 转速 度/秒（默认慢速）
        self.last_speed = 30
        self._was_connected = False

        self._build_header()
        self._build_actions()
        self._build_controls()
        self._build_metrics()
        self._build_chart()

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick = 0
        self._update()

    # ---------- UI 构建 ----------
    def _build_header(self):
        f = tk.Frame(self.root, bg=BG)
        f.pack(fill="x", padx=18, pady=(14, 6))
        tk.Label(f, text="ESP32 Servo Console", bg=BG, fg=FG,
                 font=("Helvetica", 20, "bold")).pack(side="left")
        self.status_dot = tk.Label(f, text="●", bg=BG, fg=RED, font=("Helvetica", 16))
        self.status_dot.pack(side="right")
        self.status_txt = tk.Label(f, text="Connecting…", bg=BG, fg=MUTED,
                                   font=("Helvetica", 12))
        self.status_txt.pack(side="right", padx=(0, 6))

    def _build_actions(self):
        row = tk.Frame(self.root, bg=BG)
        row.pack(fill="x", padx=18, pady=(4, 2))

        # 用 Label 模拟按钮：macOS 上 tk.Button 不渲染背景色，Label 可以
        def action_btn(text, cmd, bg_color, fg_color, big=False):
            lbl = tk.Label(row, text=text, bg=bg_color, fg=fg_color,
                           font=("Helvetica", 14 if big else 13, "bold"),
                           padx=20, pady=9, cursor="hand2")
            def press(_e, c=cmd, l=lbl, orig=bg_color):
                l.config(bg="#ffffff")
                l.after(120, lambda: l.config(bg=orig))
                self.worker.send_line(c)
            lbl.bind("<Button-1>", press)
            return lbl

        action_btn("🚪 Open", "open", BLUE, "#000").pack(side="left", padx=(0, 8))
        action_btn("🔒 Close", "close", GREEN, "#000").pack(side="left", padx=8)
        action_btn("🏠 Home", "home", PURPLE, "#000").pack(side="left", padx=8)
        # 门关紧开关(D17)状态
        self.sw_lbl = tk.Label(row, text="⭘ Switch: open", bg=BG, fg=MUTED,
                               font=("Helvetica", 12, "bold"))
        self.sw_lbl.pack(side="left", padx=14)
        # 序列状态显示
        self.seq_lbl = tk.Label(row, text="", bg=BG, fg=AMBER,
                                font=("Helvetica", 12, "bold"))
        self.seq_lbl.pack(side="left", padx=6)
        action_btn("▶ START", "start", "#2ea043", "#000", big=True).pack(side="right", padx=(8, 0))
        action_btn("⏹ STOP", "stop", "#d21f2c", "#fff", big=True).pack(side="right", padx=8)

    def _servo_panel(self, parent, title, sub, var, ch, color,
                     amin=0, amax=SERVO_MAX_DEG, presets=None):
        if presets is None:
            presets = [("0°", 0), ("90°", 90), ("135°", 135), ("180°", 180), ("270°", 270)]
        card = tk.Frame(parent, bg=CARD, bd=0, highlightthickness=1,
                        highlightbackground="#26303b")
        card.pack(side="left", expand=True, fill="both", padx=6)
        head = tk.Frame(card, bg=CARD)
        head.pack(fill="x", padx=14, pady=(12, 0))
        tk.Label(head, text=title, bg=CARD, fg=FG,
                 font=("Helvetica", 15, "bold")).pack(side="left")
        tk.Label(head, text=sub, bg=CARD, fg=MUTED,
                 font=("Helvetica", 10)).pack(side="left", padx=6)

        val = tk.Label(card, textvariable=var, bg=CARD, fg=color,
                       font=("Helvetica", 40, "bold"))
        val.pack(pady=(2, 0))
        tk.Label(card, text="Target angle (°)", bg=CARD, fg=MUTED,
                 font=("Helvetica", 10)).pack()

        actual = tk.Label(card, text="Actual: –°", bg=CARD, fg=MUTED,
                          font=("Helvetica", 10))
        actual.pack(pady=(0, 4))

        # 可拖动滑块（范围按各舵机限位）
        scale = tk.Scale(card, from_=amin, to=amax, orient="horizontal",
                         variable=var, bg=CARD, fg=FG, troughcolor="#0d1218",
                         highlightthickness=0, showvalue=True, length=300,
                         sliderrelief="raised", sliderlength=26,
                         activebackground=color, bd=0, font=("Helvetica", 9))
        scale.pack(padx=14, pady=(2, 4))

        # 手动输入角度：打数字 + 回车 / 点“确定”
        entry_var = tk.StringVar(value=str(var.get()))
        row = tk.Frame(card, bg=CARD)
        row.pack(pady=(2, 6))
        tk.Label(row, text="Set angle:", bg=CARD, fg=MUTED,
                 font=("Helvetica", 10)).pack(side="left")
        entry = tk.Entry(row, textvariable=entry_var, width=5, justify="center",
                         bg="#0d1218", fg=FG, insertbackground=FG,
                         relief="flat", font=("Helvetica", 13))
        entry.pack(side="left", padx=4)
        tk.Label(row, text=f"°  ({amin}-{amax})", bg=CARD, fg=MUTED,
                 font=("Helvetica", 10)).pack(side="left")

        def submit(_=None):
            try:
                a = int(float(entry_var.get()))
            except ValueError:
                entry_var.set(str(var.get()))
                return
            a = max(amin, min(amax, a))
            var.set(a)          # 同步滑块 + 触发发送
            entry_var.set(str(a))
        entry.bind("<Return>", submit)
        tk.Button(row, text="Go", bg=PANEL, fg="#000", activebackground=color,
                  activeforeground="#000", relief="flat", bd=0,
                  font=("Helvetica", 10), command=submit).pack(side="left", padx=(6, 0))

        # 预设按钮
        btns = tk.Frame(card, bg=CARD)
        btns.pack(pady=(0, 14))
        for label, ang in presets:
            b = tk.Button(btns, text=label, width=6,
                          bg=PANEL, fg="#000", activebackground=color,
                          activeforeground="#000", relief="flat", bd=0,
                          font=("Helvetica", 10),
                          command=lambda a=ang: var.set(a))
            b.pack(side="left", padx=3)

        return {"actual": actual, "entry": entry, "entry_var": entry_var, "var": var}

    def _build_controls(self):
        wrap = tk.Frame(self.root, bg=BG)
        wrap.pack(fill="x", padx=12, pady=6)
        self.door_ui = self._servo_panel(
            wrap, "Door", "D18 · ch0 · limits 55–220", self.door_target, 0, GREEN,
            amin=55, amax=220,
            presets=[("55° Open", 55), ("90°", 90), ("135°", 135), ("190°", 190), ("220° Close", 220)])
        self.handle_ui = self._servo_panel(
            wrap, "Handle", "D19 · ch1", self.handle_target, 1, BLUE,
            presets=[("0° Unlock", 0), ("45°", 45), ("125° Lock", 125), ("180°", 180), ("270°", 270)])

        # 转速控制条（两个舵机共用）
        srow = tk.Frame(self.root, bg=BG)
        srow.pack(fill="x", padx=18, pady=(4, 0))
        tk.Label(srow, text="Speed", bg=BG, fg=MUTED,
                 font=("Helvetica", 12, "bold")).pack(side="left")
        tk.Scale(srow, from_=10, to=300, resolution=10, orient="horizontal",
                 variable=self.speed, bg=BG, fg=FG, troughcolor="#0d1218",
                 highlightthickness=0, showvalue=True, length=320,
                 sliderrelief="raised", sliderlength=22,
                 activebackground=AMBER, bd=0,
                 font=("Helvetica", 9)).pack(side="left", padx=10)
        tk.Label(srow, text="°/s   (30 = slow default, 300 = fast)", bg=BG, fg=MUTED,
                 font=("Helvetica", 10)).pack(side="left")

        # 过流保护警告横幅（平时隐藏）
        self.fault_bar = tk.Frame(self.root, bg="#3a1215", highlightthickness=1,
                                  highlightbackground=RED)
        self.fault_lbl = tk.Label(self.fault_bar,
                                  text="⚠️ EMERGENCY STOP — motion frozen",
                                  bg="#3a1215", fg=RED, font=("Helvetica", 13, "bold"))
        self.fault_lbl.pack(side="left", padx=12, pady=8)
        resume_btn = tk.Label(self.fault_bar, text="Resolved — START to resume",
                              bg="#d21f2c", fg="#fff", padx=14, pady=6,
                              font=("Helvetica", 11, "bold"), cursor="hand2")
        resume_btn.bind("<Button-1>", lambda _e: self.worker.send_line("start"))
        resume_btn.pack(side="right", padx=12, pady=6)
        self._fault_visible = False

    def _metric_card(self, parent, title, color):
        card = tk.Frame(parent, bg=CARD, highlightthickness=1,
                        highlightbackground="#26303b")
        card.pack(side="left", expand=True, fill="both", padx=6)
        tk.Label(card, text=title, bg=CARD, fg=MUTED,
                 font=("Helvetica", 11)).pack(pady=(12, 0))
        val = tk.Label(card, text="—", bg=CARD, fg=color,
                       font=("Helvetica", 30, "bold"))
        val.pack(pady=(0, 12))
        return val

    def _build_metrics(self):
        wrap = tk.Frame(self.root, bg=BG)
        wrap.pack(fill="x", padx=12, pady=6)
        self.metrics_row = wrap   # 过流横幅插在这个区块前面
        self.volt_lbl = self._metric_card(wrap, "Voltage (V)", BLUE)
        self.curr_lbl = self._metric_card(wrap, "Current (mA)", AMBER)
        self.pow_lbl = self._metric_card(wrap, "Power (mW)", PURPLE)

        peak_card = tk.Frame(wrap, bg=CARD, highlightthickness=1,
                             highlightbackground="#26303b")
        peak_card.pack(side="left", expand=True, fill="both", padx=6)
        tk.Label(peak_card, text="Peak (mA)", bg=CARD, fg=MUTED,
                 font=("Helvetica", 11)).pack(pady=(12, 0))
        self.peak_lbl = tk.Label(peak_card, text="—", bg=CARD, fg=RED,
                                 font=("Helvetica", 30, "bold"))
        self.peak_lbl.pack()
        tk.Button(peak_card, text="Reset", bg=PANEL, fg="#000", relief="flat", bd=0,
                  font=("Helvetica", 9), command=self.worker.reset_peak).pack(pady=(0, 10))

    def _build_chart(self):
        frame = tk.Frame(self.root, bg=BG)
        frame.pack(fill="both", expand=True, padx=18, pady=(6, 14))
        self.fig = Figure(figsize=(8, 2.8), dpi=100, facecolor=PANEL)
        self.ax = self.fig.add_subplot(111, facecolor="#0d1218")
        self.line, = self.ax.plot([], [], color=AMBER, lw=1.8)
        self.ax.set_xlabel("time (s, last 30s)", color=MUTED, fontsize=9)
        self.ax.set_ylabel("current (mA)", color=MUTED, fontsize=9)
        self.ax.tick_params(colors=MUTED, labelsize=8)
        for sp in self.ax.spines.values():
            sp.set_color("#26303b")
        self.ax.grid(True, color="#1e2831", lw=0.6)
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # ---------- 周期刷新 ----------
    def _update(self):
        w = self.worker
        # 连接状态
        if w.connected:
            self.status_dot.config(fg=GREEN)
            self.status_txt.config(text=f"Connected {PORT}")
        else:
            self.status_dot.config(fg=RED)
            self.status_txt.config(text="Disconnected · reconnecting…")

        # 刚(重)连上时，把 GUI 当前转速同步给固件（固件重启后默认60）
        if w.connected and not self._was_connected:
            w.send_line(f"speed {self.speed.get()}")
        self._was_connected = w.connected

        # 发送滑块变化（节流：每 100ms 检查一次）
        d = self.door_target.get()
        if d != self.last_door:
            w.send(0, d); self.last_door = d
        h = self.handle_target.get()
        if h != self.last_handle:
            w.send(1, h); self.last_handle = h
        sp = self.speed.get()
        if sp != self.last_speed:
            w.send_line(f"speed {sp}"); self.last_speed = sp

        # 急停横幅：触发时插到读数区上方
        if w.fault and not self._fault_visible:
            reason = f" ({w.fault_reason})" if w.fault_reason else ""
            self.fault_lbl.config(text=f"⚠️ EMERGENCY STOP{reason} — motion frozen")
            self.fault_bar.pack(fill="x", padx=18, pady=(6, 0),
                                before=self.metrics_row)
            self._fault_visible = True
        elif not w.fault and self._fault_visible:
            self.fault_bar.pack_forget()
            self._fault_visible = False

        # 序列状态 / 舵机待机提示
        if w.seq_status:
            self.seq_lbl.config(text=f"⚙️ Running: {w.seq_status}", fg=AMBER)
        elif w.servos_idle:
            self.seq_lbl.config(text="💤 Servos idle — any command engages them", fg=MUTED)
        else:
            self.seq_lbl.config(text="", fg=AMBER)

        # D17 门关紧开关状态
        if w.door_switch:
            self.sw_lbl.config(text="● Switch: PRESSED", fg=GREEN)
        else:
            self.sw_lbl.config(text="⭘ Switch: open", fg=MUTED)

        # 读数（电流 >2000mA 变红警示）
        self.volt_lbl.config(text=f"{w.v:.2f}" if w.v is not None else "—")
        overcurrent = w.i is not None and abs(w.i) > 2000
        self.curr_lbl.config(text=f"{w.i:.0f}" if w.i is not None else "—",
                             fg=RED if overcurrent else AMBER)
        self.pow_lbl.config(text=f"{w.p:.0f}" if w.p is not None else "—")
        self.peak_lbl.config(text=f"{w.peak:.0f}" if w.peak else "—")
        if w.door_actual is not None:
            self.door_ui["actual"].config(text=f"Actual: {w.door_actual}°")
        if w.handle_actual is not None:
            self.handle_ui["actual"].config(text=f"Actual: {w.handle_actual}°")

        # 输入框跟随滑块（正在该框内输入时不覆盖）
        focused = self.root.focus_get()
        for ui in (self.door_ui, self.handle_ui):
            if focused is not ui["entry"]:
                cur = str(ui["var"].get())
                if ui["entry_var"].get() != cur:
                    ui["entry_var"].set(cur)

        # 图表（每 2 个 tick 更新一次 ≈ 5Hz）
        self._tick += 1
        if self._tick % 2 == 0 and w.hist:
            data = list(w.hist)
            t_now = data[-1][0]
            xs = [t - t_now for (t, _) in data]
            ys = [c for (_, c) in data]
            self.line.set_data(xs, ys)
            self.ax.set_xlim(-30, 0)
            ymax = max(100.0, max(ys) * 1.15)
            self.ax.set_ylim(min(0, min(ys) * 1.1), ymax)
            self.canvas.draw_idle()

        self.root.after(100, self._update)

    def _on_close(self):
        self.worker.stop()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
