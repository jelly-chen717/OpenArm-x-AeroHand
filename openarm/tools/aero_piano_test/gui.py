"""Tk front end. Only the worker touches serial; Tk only exchanges snapshots."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from .core import AeroTransport, CHANNELS, FINGERS, MODES, Finger, Plan, Runner, Interrupted, number

ROOT = Path(__file__).resolve().parents[2]
CONFIG = Path(__file__).with_name("config.json")


class App:
    def __init__(self, root, demo=False, config_path=CONFIG):
        self.root, self.demo, self.config_path = root, demo, config_path
        self.jobs, self.events = queue.Queue(), queue.Queue()
        self.stop, self.abort = threading.Event(), threading.Event()
        self.shutdown = threading.Event()
        self.connected = self.busy = self.closing = False
        self.last_channels = []
        self.feedback = None
        self.controls = []
        self.rows = {}
        root.title("Aero 右手 · 钢琴击键测试" + ("【演示：不连接硬件】" if demo else ""))
        root.geometry("1120x800")
        root.minsize(1000, 740)
        style = ttk.Style(root)
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 17, "bold"))
        style.configure("TButton", padding=(8, 5))
        outer = ttk.Frame(root, padding=16)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="Aero 右手 · 钢琴击键测试", style="Title.TLabel").pack(anchor="w")
        ttk.Label(outer, text="四指独立测试  /  拇指保持原驱动位置  /  连接不动作  /  不控制机械臂").pack(anchor="w", pady=(4, 12))
        connection = ttk.Frame(outer)
        connection.pack(fill="x")
        self.port = tk.StringVar(value="COM8")
        ttk.Label(connection, text="串口").pack(side="left")
        entry = ttk.Entry(connection, textvariable=self.port, width=12)
        entry.pack(side="left", padx=6)
        self.controls.append(entry)
        ttk.Label(connection, text="921600 baud").pack(side="left", padx=6)
        self.button(connection, "连接 / 只读反馈", lambda: self.submit("connect", self.port.get().strip()))
        self.button(connection, "断开", lambda: self.submit("disconnect"))
        self.button(connection, "保存配置", self.save)
        self.status = tk.StringVar(value="演示模式：无硬件通信" if demo else "未连接；请先断开官方 GUI，释放 COM8")
        ttk.Label(outer, textvariable=self.status, foreground="#176676").pack(anchor="w", pady=8)
        self.armed = tk.BooleanVar(value=False)
        arm = ttk.Checkbutton(outer, text="允许手指动作：已完成归零，手腕已固定，并确认当前位置可运动", variable=self.armed)
        arm.pack(anchor="w", pady=(0, 10))
        self.controls.append(arm)

        frame = ttk.LabelFrame(outer, text="手指参数（官方 SDK 紧凑关节角，单位 °；不是飞特位置计数）", padding=8)
        frame.pack(fill="x")
        headers = ("选择", "手指", "估算角度", "抬起 °", "按下 °", "过渡 s", "记录当前位置", "手动测试")
        for column, title in enumerate(headers):
            ttk.Label(frame, text=title).grid(row=0, column=column, padx=7, pady=4)
        for row, (ch, name) in enumerate(FINGERS.items(), 1):
            variables = dict(enabled=tk.BooleanVar(value=ch == 3), q=tk.StringVar(value="—"),
                             home=tk.StringVar(), press=tk.StringVar(), transition=tk.StringVar(value="0.5"))
            self.rows[ch] = variables
            check = ttk.Checkbutton(frame, variable=variables["enabled"])
            check.grid(row=row, column=0)
            self.controls.append(check)
            ttk.Label(frame, text=name).grid(row=row, column=1, padx=8)
            ttk.Label(frame, textvariable=variables["q"]).grid(row=row, column=2, padx=8)
            for column, key in enumerate(("home", "press", "transition"), 3):
                field = ttk.Entry(frame, width=8, textvariable=variables[key])
                field.grid(row=row, column=column, padx=5, pady=6)
                self.controls.append(field)
            captures = ttk.Frame(frame)
            captures.grid(row=row, column=6)
            self.button(captures, "记抬起", lambda c=ch: self.capture(c, "home"))
            self.button(captures, "记按下", lambda c=ch: self.capture(c, "press"))
            manual = ttk.Frame(frame)
            manual.grid(row=row, column=7)
            self.button(manual, "−1°", lambda c=ch: self.jog(c, -1))
            self.button(manual, "+1°", lambda c=ch: self.jog(c, 1))
            self.button(manual, "抬起", lambda c=ch: self.one(c, "home"))
            self.button(manual, "按下", lambda c=ch: self.one(c, "press"))
        ttk.Label(outer, text="位置初始为空。先用 ±1° 微调并记录抬起/按下位置；过渡时间越大越慢。实物触底后勿继续增加角度。").pack(anchor="w", pady=8)
        speed = ttk.Frame(outer)
        speed.pack(fill="x", pady=(0, 8))
        ttk.Label(speed, text="所选手指过渡：").pack(side="left")
        self.button(speed, "缓慢 0.5 s", lambda: self.set_transition(0.5))
        self.button(speed, "快速 0.05 s", lambda: self.set_transition(0.05))
        self.button(speed, "直接目标 0 s", lambda: self.set_transition(0))
        ttk.Label(speed, text="0 = 一次发送目标，实际速度由驱动决定；停留时间仍有效。").pack(side="left", padx=8)

        cycle = ttk.LabelFrame(outer, text="重复击键", padding=8)
        cycle.pack(fill="x")
        self.mode = tk.StringVar(value=MODES[0])
        mode = ttk.Combobox(cycle, state="readonly", textvariable=self.mode, values=MODES, width=12)
        mode.pack(side="left", padx=5)
        self.controls.append(mode)
        self.press_dwell, self.home_dwell, self.cycles = tk.StringVar(value="0.3"), tk.StringVar(value="0.3"), tk.StringVar(value="10")
        for label, variable in (("按下停留 s", self.press_dwell), ("抬起停留 s", self.home_dwell), ("次数（0=持续）", self.cycles)):
            ttk.Label(cycle, text=label).pack(side="left", padx=(7, 3))
            field = ttk.Entry(cycle, textvariable=variable, width=6)
            field.pack(side="left")
            self.controls.append(field)
        self.button(cycle, "开始循环", self.cycle)
        actions = ttk.Frame(outer)
        actions.pack(fill="x", pady=8)
        self.button(actions, "所选抬起", lambda: self.selected("home"))
        self.button(actions, "所选按下", lambda: self.selected("press"))
        ttk.Button(actions, text="停止并抬起（有已设抬起值）", command=lambda: self.guard(self.stop_and_home)).pack(side="left", padx=5)
        ttk.Button(actions, text="立即中止发送", command=self.hard_stop).pack(side="left", padx=5)
        self.counter = tk.StringVar(value="指令循环：0（不等于成功发声次数）")
        ttk.Label(actions, textvariable=self.counter).pack(side="left", padx=8)

        telemetry = ttk.LabelFrame(outer, text="实时反馈 · 约 5 Hz · 估算关节角不等于独立关节传感器读数", padding=6)
        telemetry.pack(fill="x")
        self.table = ttk.Treeview(telemetry, columns=("name", "act", "current", "temp", "rpm"), show="headings", height=7)
        for col, label in zip(self.table["columns"], ("驱动通道", "驱动角度 °", "电流 mA", "温度 °C", "转速 RPM")):
            self.table.heading(col, text=label)
            self.table.column(col, width=165, anchor="center")
        for ch, name in enumerate(CHANNELS):
            self.table.insert("", "end", iid=str(ch), values=(name, "—", "—", "—", "—"))
        self.table.pack(fill="x")
        self.logbox = tk.Text(outer, height=6, state="disabled", font=("Microsoft YaHei UI", 9))
        self.logbox.pack(fill="both", expand=True, pady=(10, 0))
        self.load()
        root.update_idletasks()
        # Tk's font metrics vary with Windows DPI; keep all action buttons visible.
        width = max(1120, root.winfo_reqwidth())
        height = max(800, root.winfo_reqheight())
        root.geometry(f"{width}x{height}")
        root.minsize(width, height)
        self.log("演示数据，所有动作均为模拟。" if demo else "就绪。连接只读取反馈；不会自动 Homing、改 ID、改速度/电流参数。")
        self.thread = threading.Thread(target=self.worker, name="aero-piano-serial", daemon=True)
        self.thread.start()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(50, self.drain)

    def button(self, parent, text, command):
        button = ttk.Button(parent, text=text, command=lambda: self.guard(command))
        button.pack(side="left", padx=3)
        self.controls.append(button)
        return button

    def guard(self, callback):
        try:
            callback()
        except Exception as exc:
            messagebox.showerror("暂不能执行", str(exc), parent=self.root)

    def require_motion(self):
        if not self.connected or not self.feedback or time.monotonic() - self.feedback["time"] > 1:
            raise ValueError("请先连接并取得新鲜反馈")
        if not self.armed.get():
            raise ValueError("请确认已归零和固定，勾选“允许手指动作”")

    def capture(self, ch, key):
        if not self.connected or not self.feedback or time.monotonic() - self.feedback["time"] > 1:
            raise ValueError("请先连接并读取当前反馈")
        value = number(self.feedback["q"][ch], "估算关节角", 0, 90)
        self.rows[ch][key].set(f"{value:.3f}")
        self.save()

    def durations(self, channels):
        return {ch: number(self.rows[ch]["transition"].get(), "过渡时间", 0, 10) for ch in channels}

    def set_transition(self, seconds):
        if self.busy:
            raise ValueError("请先停止当前动作")
        selected = [row for row in self.rows.values() if row["enabled"].get()]
        if not selected:
            raise ValueError("请至少选择一根手指")
        for row in selected:
            row["transition"].set(str(seconds))
        self.save(silent=True)
        self.log(f"所选手指过渡时间已设为 {seconds} 秒；位置和停留时间保持不变。")

    def releases(self, channels):
        return {ch: number(self.rows[ch]["home"].get(), "抬起角度", 0, 90)
                for ch in channels if self.rows[ch]["home"].get().strip()}

    def jog(self, ch, delta):
        self.require_motion()
        # Worker computes from fresh measured position, not the GUI snapshot.
        self.submit("jog", (ch, delta, self.durations([ch]), self.releases([ch])))

    def one(self, ch, key):
        self.move([ch], key)

    def selected(self, key):
        channels = [ch for ch, row in self.rows.items() if row["enabled"].get()]
        if not channels:
            raise ValueError("请至少选择一根手指")
        self.move(channels, key)

    def move(self, channels, key):
        self.require_motion()
        goals = {ch: number(self.rows[ch][key].get(), "目标角度", 0, 90) for ch in channels}
        self.submit("move", (goals, self.durations(channels), self.releases(channels)))

    def cycle(self):
        self.require_motion()
        fingers = tuple(Finger(ch, float(row["home"].get()), float(row["press"].get()),
                               float(row["transition"].get()))
                        for ch, row in self.rows.items() if row["enabled"].get())
        plan = Plan(fingers, self.mode.get(), float(self.press_dwell.get()), float(self.home_dwell.get()), int(self.cycles.get()))
        self.counter.set("指令循环：0（不等于成功发声次数）")
        self.submit("cycle", plan)

    def submit(self, kind, payload=None):
        if self.busy or self.closing:
            raise ValueError("请等待当前动作结束，或先停止")
        if kind == "connect" and not payload:
            raise ValueError("请输入 COM 端口")
        self.save(silent=True)
        self.stop.clear()
        self.abort.clear()
        if kind == "move":
            self.last_channels = list(payload[0])
        elif kind == "jog":
            self.last_channels = [payload[0]]
        elif kind == "cycle":
            self.last_channels = [f.channel for f in payload.fingers]
        self.busy = True
        for control in self.controls:
            control.state(["disabled"])
        self.jobs.put((kind, payload))

    def stop_and_home(self):
        if self.busy:
            self.stop.set()
            return
        if not self.connected:
            self.log("未连接或连接已故障，不能保证抬起；请观察实物。")
            return
        channels = self.last_channels or [ch for ch, row in self.rows.items() if row["enabled"].get()]
        homes = self.releases(channels)
        if not homes:
            self.log("尚未设置抬起位置，不发送未经标定的返回目标。")
            return
        self.submit("move", (homes, self.durations(homes), homes))

    def hard_stop(self):
        self.abort.set()
        self.stop.set()
        self.armed.set(False)
        self.log("请求立即中止发送；不会再发抬起目标。此按钮不切断电源，驱动可能仍保持最后目标。")

    def worker(self):
        bus = AeroTransport(self.demo)
        emit = lambda kind, data: self.events.put((kind, data))
        app = self

        class AbortableRunner(Runner):
            def poll(self):
                if app.abort.is_set():
                    raise OSError("用户中止发送，已锁定动作；重新连接后才能继续")
                return super().poll()

            def move(self, *args, **kwargs):
                if app.abort.is_set():
                    raise OSError("用户中止发送")
                return super().move(*args, **kwargs)

        # Check before every packet, including return-home packets after Stop.
        original_write = bus.write
        def checked_write(raw):
            if self.abort.is_set():
                raise OSError("用户中止发送")
            original_write(raw)
        bus.write = checked_write
        runner = AbortableRunner(bus, self.stop, emit)
        logfile = None
        writer = None
        try:
            while not self.shutdown.is_set():
                try:
                    kind, payload = self.jobs.get(timeout=0.2)
                except queue.Empty:
                    if bus.connected and not bus.fault:
                        try:
                            if self.abort.is_set():
                                raise OSError("用户中止发送")
                            emit("feedback", bus.read())
                        except Exception as exc:
                            bus.fault = True
                            emit("error", str(exc))
                    continue
                try:
                    if kind == "connect":
                        if bus.connected:
                            bus.close()
                        emit("feedback", bus.connect(payload))
                        emit("connected", True)
                        emit("log", f"已连接 {'演示设备' if self.demo else payload}，只读反馈；请设置或记录手指位置。")
                    elif kind == "disconnect":
                        bus.close()
                        emit("connected", False)
                    elif kind == "cycle":
                        folder = ROOT / "logs" / "aero_piano_test"
                        folder.mkdir(parents=True, exist_ok=True)
                        path = folder / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ("_demo" if self.demo else "") + ".csv")
                        logfile = path.open("w", newline="", encoding="utf-8-sig")
                        writer = csv.writer(logfile)
                        writer.writerow(["wall_time", "monotonic_s", "event", "detail"])
                        writer.writerow([datetime.now().isoformat(), time.monotonic(), "plan", repr(payload)])
                        base_emit = runner.emit
                        def record(event, data):
                            if event in ("cycle", "log"):
                                writer.writerow([datetime.now().isoformat(), time.monotonic(), event, str(data)])
                                logfile.flush()
                            base_emit(event, data)
                        runner.emit = record
                        emit("log", f"循环记录：{path}")
                        try:
                            runner.run(payload)
                            writer.writerow([datetime.now().isoformat(), time.monotonic(), "end", "stopped" if self.stop.is_set() else "completed"])
                        finally:
                            runner.emit = base_emit
                    elif kind in ("move", "jog"):
                        runner.begin()
                        if kind == "jog":
                            ch, delta, durations, homes = payload
                            goals = {ch: number(bus.last["q"][ch] + delta, "微调角度", 0, 90)}
                        else:
                            goals, durations, homes = payload
                        try:
                            runner.move(goals, durations)
                        except Interrupted:
                            if homes:
                                runner.move(homes, {ch: durations[ch] for ch in homes}, interrupt=False)
                            emit("log", "已停止；已设抬起值的本次手指返回抬起目标，其余保持最后指令。")
                    if bus.connected and not bus.fault:
                        emit("feedback", bus.read())
                except Exception as exc:
                    bus.fault = True
                    if writer:
                        writer.writerow([datetime.now().isoformat(), time.monotonic(), "error", str(exc)])
                    emit("error", str(exc))
                finally:
                    if logfile:
                        logfile.close()
                        logfile = writer = None
                    emit("done", None)
        finally:
            bus.close()
            emit("closed", None)

    def drain(self):
        while True:
            try:
                kind, data = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "feedback":
                self.feedback = data
                for ch, row in self.rows.items():
                    row["q"].set(f"{data['q'][ch]:.2f}°")
                for ch, name in enumerate(CHANNELS):
                    self.table.item(str(ch), values=(name, *(f"{data[k][ch]:.2f}" for k in ("actuations", "currents", "temperatures", "speeds"))))
            elif kind == "connected":
                self.connected = data
                self.last_channels = []
                self.armed.set(False)
                self.status.set(("演示已连接" if self.demo else "已连接 · 只在点击动作按钮后运动") if data else "已断开")
                if not data:
                    self.feedback = None
                    for row in self.rows.values():
                        row["q"].set("—")
            elif kind == "error":
                self.connected = False
                self.armed.set(False)
                self.feedback = None
                self.status.set("故障：已停止发送；请观察手部，排查后重新连接")
                for row in self.rows.values():
                    row["q"].set("失效")
                for ch, name in enumerate(CHANNELS):
                    self.table.item(str(ch), values=(name, "失效", "失效", "失效", "失效"))
                self.log(data)
            elif kind == "cycle":
                self.counter.set(f"指令循环：{data[0]}（不等于成功发声次数）")
            elif kind == "log":
                self.log(data)
            elif kind == "done":
                self.busy = False
                if not self.closing:
                    for control in self.controls:
                        control.state(["!disabled"])
            elif kind == "closed":
                self.root.destroy()
                return
        self.root.after(50, self.drain)

    def log(self, text):
        self.logbox.configure(state="normal")
        self.logbox.insert("end", datetime.now().strftime("%H:%M:%S ") + text + "\n")
        self.logbox.see("end")
        self.logbox.configure(state="disabled")

    def save(self, silent=False):
        data = dict(version=1, port=self.port.get(), mode=self.mode.get(), press_dwell=self.press_dwell.get(),
                    home_dwell=self.home_dwell.get(), cycles=self.cycles.get(),
                    fingers={str(ch): {key: row[key].get() for key in ("enabled", "home", "press", "transition")} for ch, row in self.rows.items()})
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.config_path.with_suffix(".tmp")
        temp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        temp.replace(self.config_path)
        if not silent:
            self.log("配置已保存；重新连接后仍需手动允许动作。")

    def load(self):
        if not self.config_path.exists():
            return
        try:
            data = json.loads(self.config_path.read_text(encoding="utf-8"))
            if data.get("version") != 1:
                raise ValueError("未知配置版本")
            for key in ("port", "mode", "press_dwell", "home_dwell", "cycles"):
                if key in data:
                    getattr(self, key).set(data[key])
            for ch, row in self.rows.items():
                for key, value in data.get("fingers", {}).get(str(ch), {}).items():
                    if key in ("enabled", "home", "press", "transition"):
                        row[key].set(value)
        except Exception as exc:
            self.log(f"配置读取失败：{exc}；执行前将重新检查参数。")

    def close(self):
        if self.closing:
            return
        try:
            self.save(silent=True)
        except OSError as exc:
            self.log(f"配置未能保存：{exc}")
        self.closing = True
        self.abort.set()
        self.stop.set()
        self.shutdown.set()
        self.status.set("正在关闭连接；不再下发动作。驱动可能保持最后目标。")
        for control in self.controls:
            control.state(["disabled"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", help="纯软件演示，不连接串口")
    parser.add_argument("--config", type=Path, default=None)
    args = parser.parse_args()
    config = args.config or (CONFIG.with_name("demo_config.json") if args.demo else CONFIG)
    root = tk.Tk()
    App(root, args.demo, config)
    root.mainloop()


if __name__ == "__main__":
    main()
