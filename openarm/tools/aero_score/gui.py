"""Dual-hand TXT rehearsal GUI. No arm service, IK or automatic connection."""
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
from tkinter import filedialog, messagebox, ttk

from tools.aero_piano_test.core import FINGERS, Runner, Interrupted, number
from .core import (SIDES, LABELS, CHARS, HandLink, compile_score, run_dual,
                   validate_calibration, validate_ports)

ROOT = Path(__file__).resolve().parents[2]
CONFIG = Path(__file__).with_name("config.json")
MUSIC = ROOT / "skills" / "paino_skill" / "music_data"
SAMPLE = """# 双手四指短测试；移臂仅作原谱时间/索引校验，不发送到机械臂。
left_arm_move(1,0.1)
right_arm_move(24,0.1)
sleep(0.5)
hand(D,47)
sleep(0.2)
hand(U,47)
sleep(0.3)
hand(D,38)
sleep(0.2)
hand(U,38)
sleep(0.3)
hand(D,29)
sleep(0.2)
hand(U,29)
sleep(0.3)
hand(D,1x)
sleep(0.2)
hand(U,1x)
sleep(0.3)
hand(D,1234789x)
sleep(0.2)
hand(U,1234789x)
sleep(0.3)
"""


def defaults(root=None):
    root = ROOT if root is None else root
    settings = {"version": 1, "sides": {}}
    for side in SIDES:
        settings["sides"][side] = dict(demo=side == "left", port="COM8" if side == "right" else "",
            real={str(ch): dict(home="", press="") for ch in FINGERS},
            simulation={str(ch): dict(home="0", press="20") for ch in FINGERS})
    source = root / "tools" / "aero_piano_test" / "config.json"
    if source.exists():
        try:
            data = json.loads(source.read_text(encoding="utf-8"))
            settings["sides"]["right"]["port"] = data.get("port", "COM8")
            for ch in FINGERS:
                old = data.get("fingers", {}).get(str(ch), {})
                settings["sides"]["right"]["real"][str(ch)] = {k: str(old.get(k, "")) for k in ("home", "press")}
        except (OSError, ValueError, TypeError):
            pass
    return settings


def right_real_setup(settings, root=None):
    """Explicit launcher: import the user's right-hand test positions at startup."""
    source = defaults(root)
    right = source["sides"]["right"]
    validate_calibration(right["real"], tuple(FINGERS))
    validate_ports({"right": dict(demo=False, port=right["port"])})
    settings["sides"]["right"].update(demo=False, port=right["port"], real=right["real"])
    left = settings["sides"]["left"]
    left["demo"] = True
    try:
        validate_calibration(left["simulation"], tuple(FINGERS))
    except ValueError:
        left["simulation"] = source["sides"]["left"]["simulation"]
    return settings


class App:
    def __init__(self, root, config_path=CONFIG, all_demo=False, right_real=False):
        self.root, self.config_path = root, Path(config_path)
        self.events = queue.Queue()
        self.stop, self.abort = threading.Event(), threading.Event()
        self.controls, self.rows, self.hand_widgets = [], {}, {}
        self.connected = {s: False for s in SIDES}
        self.connected_info = {}
        self.feedback = {s: None for s in SIDES}
        self.active_mode = {}
        self.busy = self.closing = False
        self.task_thread = None
        self.settings = defaults()
        if self.config_path.exists():
            data = json.loads(self.config_path.read_text(encoding="utf-8"))
            if data.get("version") != 1:
                raise ValueError("未知配置版本")
            self.settings = data
        if right_real and not all_demo:
            self.settings = right_real_setup(self.settings)
        if all_demo:
            for side in SIDES:
                self.settings["sides"][side]["demo"] = True
        root.title("Aero 双手 TXT · 仅手指排练" + ("【双手模拟】" if all_demo else ""))
        root.geometry("1140x840")
        frame = ttk.Frame(root, padding=14)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="Aero 双手 TXT · 仅手指排练", font=("Microsoft YaHei UI", 18, "bold")).pack(anchor="w")
        ttk.Label(frame, text="不连接机械臂 · 忽略乐谱移臂指令 · 保留原按下/抬起时间 · 固定手腕下不对应原谱音高",
                  foreground="#a35b09").pack(anchor="w", pady=6)
        self.status = tk.StringVar(value="默认左手软件模拟、右手 COM8 实机。连接只读反馈，不自动动作。")
        ttk.Label(frame, textvariable=self.status).pack(anchor="w", pady=4)

        self.demo, self.port, self.armed, self.connection_status, self.command_status = {}, {}, {}, {}, {}
        for side in SIDES:
            stored = self.settings["sides"][side]
            self.demo[side] = tk.BooleanVar(value=stored["demo"])
            self.port[side] = tk.StringVar(value=stored["port"])
            self.armed[side] = tk.BooleanVar(value=False)
            self.connection_status[side] = tk.StringVar(value="未连接")
            self.command_status[side] = tk.StringVar(value="指令：—")
            self.active_mode[side] = "simulation" if stored["demo"] else "real"
            row = ttk.Frame(frame)
            row.pack(fill="x", pady=3)
            ttk.Label(row, text=LABELS[side], width=6).pack(side="left")
            check = ttk.Checkbutton(row, text="软件模拟（不打开串口）", variable=self.demo[side],
                command=lambda s=side: self.guard(lambda: self.change_mode(s)))
            check.pack(side="left")
            self.controls.append(check)
            self.field(row, self.port[side], 9)
            self.button(row, "连接 / 只读", lambda s=side: self.connect(s))
            self.button(row, "断开", lambda s=side: self.disconnect(s))
            ready = ttk.Checkbutton(row, text="实机已归零并固定", variable=self.armed[side])
            ready.pack(side="left", padx=6)
            self.controls.append(ready)
            ttk.Label(row, textvariable=self.connection_status[side], foreground="#176676").pack(side="left", padx=8)

        tabs = ttk.Notebook(frame)
        tabs.pack(fill="both", expand=True, pady=8)
        score_tab = ttk.Frame(tabs, padding=8)
        tabs.add(score_tab, text="TXT 乐谱")
        catalog = ttk.Frame(score_tab)
        catalog.pack(fill="x")
        self.song = tk.StringVar(value="双手短测试")
        songs = ["双手短测试"] + sorted(p.name for p in MUSIC.glob("*.txt"))
        combo = ttk.Combobox(catalog, textvariable=self.song, values=songs, state="readonly", width=30)
        combo.pack(side="left")
        self.controls.append(combo)
        self.button(catalog, "载入曲目", self.load_song)
        self.button(catalog, "打开 TXT", self.open_file)
        self.button(catalog, "检查整谱", self.inspect)
        self.summary = tk.StringVar(value="左 1/2/3/4 = 小/无名/中/食；右 7/8/9/x = 食/中/无名/小。拇指保持原位置。")
        ttk.Label(score_tab, textvariable=self.summary, wraplength=1050).pack(anchor="w", pady=8)
        editor = ttk.Frame(score_tab)
        editor.pack(fill="both", expand=True)
        self.text = tk.Text(editor, height=14, undo=True, font=("Consolas", 10))
        scrollbar = ttk.Scrollbar(editor, command=self.text.yview)
        self.text.configure(yscrollcommand=scrollbar.set)
        self.text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        self.text.insert("1.0", SAMPLE)
        for side in SIDES:
            page = ttk.Frame(tabs, padding=10)
            tabs.add(page, text=f"{LABELS[side]}位置标定")
            ttk.Label(page, text="模拟与实机位置分开保存。实机角度需在实际安装位置上验证；模型估算角不是指尖位置。",
                      wraplength=1000).pack(anchor="w", pady=8)
            table = ttk.Frame(page)
            table.pack(anchor="w")
            for column, label in enumerate(("手指 / 谱符", "估算角 °", "抬起 °", "按下 °", "记录反馈", "手动微调 / 测试")):
                ttk.Label(table, text=label).grid(row=0, column=column, padx=12, pady=6)
            self.rows[side] = {}
            for i, (ch, name) in enumerate(FINGERS.items(), 1):
                values = self.settings["sides"][side][self.active_mode[side]][str(ch)]
                fields = dict(q=tk.StringVar(value="—"), home=tk.StringVar(value=values["home"]),
                              press=tk.StringVar(value=values["press"]))
                self.rows[side][ch] = fields
                ttk.Label(table, text=f"{name} / {CHARS[side][ch]}").grid(row=i, column=0, padx=12)
                ttk.Label(table, textvariable=fields["q"]).grid(row=i, column=1, padx=12)
                for column, key in ((2, "home"), (3, "press")):
                    entry = ttk.Entry(table, width=9, textvariable=fields[key])
                    entry.grid(row=i, column=column, padx=8, pady=8)
                    self.controls.append(entry)
                captures, manual = ttk.Frame(table), ttk.Frame(table)
                captures.grid(row=i, column=4)
                manual.grid(row=i, column=5)
                for key, label in (("home", "记抬起"), ("press", "记按下")):
                    self.button(captures, label, lambda s=side, c=ch, k=key: self.capture(s, c, k))
                for delta in (-1, 1):
                    self.button(manual, f"{delta:+d}°", lambda s=side, c=ch, d=delta: self.manual(s, c, "jog", d))
                self.button(manual, "抬起", lambda s=side, c=ch: self.manual(s, c, "home"))
                self.button(manual, "按下", lambda s=side, c=ch: self.manual(s, c, "press"))
            lower = ttk.Frame(page)
            lower.pack(anchor="w", pady=12)
            self.button(lower, "抬起已配置手指", lambda s=side: self.home_side(s))
            self.button(lower, "保存配置", self.save)
            ttk.Label(page, text="右手实机位置首次从原测试窗口复制；左手实机初始留空。\n手动动作缓动 0.3 s；演奏直接发送目标，无额外过渡和停留。\n开始前须先抬起手指；姿态检查通过后，两手使用同一个起始时刻。",
                      wraplength=1000).pack(anchor="w", pady=8)

        action = ttk.Frame(frame)
        action.pack(fill="x", pady=5)
        self.tolerance = tk.StringVar(value="2")
        ttk.Label(action, text="启动抬起容差 °").pack(side="left")
        self.field(action, self.tolerance, 5)
        self.button(action, "开始指法排练", self.start)
        ttk.Button(action, text="停止并抬起", command=self.stop_score).pack(side="left", padx=5)
        ttk.Button(action, text="立即中止发送", command=self.hard_abort).pack(side="left", padx=5)
        self.progress = tk.StringVar(value="未开始")
        ttk.Label(action, textvariable=self.progress).pack(side="left", padx=8)
        for side in SIDES:
            ttk.Label(frame, textvariable=self.command_status[side]).pack(anchor="w")
        self.logbox = tk.Text(frame, height=5, state="disabled", font=("Microsoft YaHei UI", 9))
        self.logbox.pack(fill="x", pady=8)
        self.links = {s: HandLink(s, lambda kind, data: self.events.put((kind, data))) for s in SIDES}
        self.log("软件模拟不是 COM1，也不读取实物。接好左手后，断开模拟、取消勾选，再配置真实串口及独立位置。")
        if right_real and not all_demo:
            self.save()
            self.status.set("右手实机模式已就绪：先手动连接，再确认归零并固定。左手为软件模拟。")
            self.log("已重新载入右手测试窗口的位置参数；尚未连接 COM，也未发送动作。")
        root.update_idletasks()
        root.minsize(max(1140, root.winfo_reqwidth()), max(800, root.winfo_reqheight()))
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(40, self.drain)

    def field(self, parent, variable, width):
        entry = ttk.Entry(parent, textvariable=variable, width=width)
        entry.pack(side="left", padx=5)
        self.controls.append(entry)

    def button(self, parent, text, callback):
        btn = ttk.Button(parent, text=text, command=lambda: self.guard(callback))
        btn.pack(side="left", padx=4)
        self.controls.append(btn)

    def guard(self, callback):
        try:
            callback()
        except Exception as exc:
            messagebox.showerror("暂不能执行", str(exc), parent=self.root)

    def save_current(self, side):
        self.settings["sides"][side][self.active_mode[side]] = {
            str(ch): {key: row[key].get() for key in ("home", "press")} for ch, row in self.rows[side].items()}

    def save(self):
        for side in SIDES:
            self.save_current(side)
            self.settings["sides"][side].update(demo=self.demo[side].get(), port=self.port[side].get())
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.config_path.with_suffix(".tmp")
        temp.write_text(json.dumps(self.settings, ensure_ascii=False, indent=2), encoding="utf-8")
        temp.replace(self.config_path)

    def change_mode(self, side):
        if self.connected[side] or self.busy:
            self.demo[side].set(self.active_mode[side] == "simulation")
            raise ValueError("请先停止并断开该手，再切换模拟/实机")
        self.save_current(side)
        self.active_mode[side] = "simulation" if self.demo[side].get() else "real"
        for ch, row in self.rows[side].items():
            saved = self.settings["sides"][side][self.active_mode[side]][str(ch)]
            for key in ("home", "press"):
                row[key].set(saved[key])
        self.armed[side].set(False)
        self.save()

    def set_busy(self, busy):
        self.busy = busy
        for widget in self.controls:
            widget.state(["disabled"] if busy else ["!disabled"])
        self.text.configure(state="disabled" if busy else "normal")

    def task(self, callback, label):
        if self.busy or self.closing:
            raise ValueError("当前任务未结束")
        self.save()
        self.stop.clear()
        self.abort.clear()
        self.set_busy(True)
        self.status.set(label)
        def work():
            try:
                callback()
            except Exception as exc:
                self.events.put(("error", str(exc)))
            finally:
                self.events.put(("done", label))
        self.task_thread = threading.Thread(target=work, name="aero-score-coordinator", daemon=True)
        self.task_thread.start()

    def connect(self, side):
        demo, port = self.demo[side].get(), self.port[side].get().strip().upper()
        settings = {s: self.connected_info[s] for s in SIDES if s != side and self.connected[s]}
        settings[side] = dict(demo=demo, port=port)
        validate_ports(settings)
        self.task(lambda: self.links[side].submit(lambda link: link.connect(demo, port)).result(timeout=5),
                  f"连接{LABELS[side]} {'软件模拟' if demo else port}，不发送动作…")

    def disconnect(self, side):
        self.task(lambda: self.links[side].submit(lambda link: link.disconnect()).result(timeout=5), f"断开{LABELS[side]}")

    def require_hand(self, side):
        if not self.connected[side] or not self.feedback[side] or time.monotonic()-self.feedback[side]["time"] > 1:
            raise ValueError(f"{LABELS[side]}需要连接和有效反馈")
        actual = self.connected_info[side]
        if actual["demo"] != self.demo[side].get() or (not actual["demo"] and actual["port"] != self.port[side].get().strip().upper()):
            raise ValueError(f"{LABELS[side]}端口或模式已改变，请重新连接")
        if not self.demo[side].get() and not self.armed[side].get():
            raise ValueError(f"请先确认{LABELS[side]}实机已归零并固定")

    def calibration(self, side):
        return {ch: {k: row[k].get() for k in ("home", "press")} for ch, row in self.rows[side].items()}

    def capture(self, side, ch, key):
        data = self.feedback[side]
        if not self.connected[side] or not data or time.monotonic()-data["time"] > 1:
            raise ValueError("请先连接并读取位置")
        self.rows[side][ch][key].set(f"{number(data['q'][ch], '估算角度', 0, 90):.3f}")
        self.save()

    def manual(self, side, ch, key, delta=0):
        self.require_hand(side)
        goals = None if key == "jog" else {ch: number(self.rows[side][ch][key].get(), "目标角度", 0, 90)}
        homes = {ch: float(self.rows[side][ch]["home"].get())} if self.rows[side][ch]["home"].get().strip() else {}
        self._manual_task(side, goals, homes, ch, delta)

    def home_side(self, side):
        self.require_hand(side)
        homes = {ch: number(row["home"].get(), "抬起角度", 0, 90) for ch, row in self.rows[side].items() if row["home"].get().strip()}
        if not homes:
            raise ValueError("尚未配置抬起位置")
        self._manual_task(side, homes, homes)

    def _manual_task(self, side, goals, homes, ch=None, delta=0):
        def action(link):
            runner = Runner(link.bus, self.stop, lambda kind, data: self.events.put(
                ("feedback", (side, data)) if kind == "feedback" else ("log", data)))
            original = link.bus.write
            def checked(raw):
                if self.abort.is_set():
                    raise OSError("用户中止发送")
                original(raw)
            link.bus.write = checked
            try:
                runner.begin()
                targets = goals if goals is not None else {ch: number(link.bus.last["q"][ch]+delta, "微调角度", 0, 90)}
                try:
                    runner.move(targets, {c: .3 for c in targets})
                except Interrupted:
                    if homes and not self.abort.is_set():
                        runner.move(homes, {c: .3 for c in homes}, interrupt=False)
                link.feedback()
            finally:
                link.bus.write = original
        self.task(lambda: self.links[side].submit(action).result(timeout=10), f"{LABELS[side]}手动动作")

    def load_song(self):
        name = self.song.get()
        content = SAMPLE if name == "双手短测试" else (MUSIC / name).read_text(encoding="utf-8-sig")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", content)
        self.inspect()

    def open_file(self):
        filename = filedialog.askopenfilename(parent=self.root, filetypes=[("TXT 乐谱", "*.txt")])
        if filename:
            content = Path(filename).read_text(encoding="utf-8-sig")
            self.text.delete("1.0", "end")
            self.text.insert("1.0", content)
            self.song.set(Path(filename).name)
            self.inspect()

    def inspect(self):
        score = compile_score(self.text.get("1.0", "end-1c"))
        self.summary.set(f"四指 · {score.duration_us/1e6:.6f} 秒 · {score.note_count} 个按指 · "
                         f"忽略 {score.ignored_moves} 条移臂指令；仅排练指法，不验证音高或实际发声。")
        return score

    def start(self):
        score = self.inspect()
        calibration = {}
        settings = {}
        for side in score.sides:
            self.require_hand(side)
            calibration[side] = validate_calibration(self.calibration(side), score.used[side])
            settings[side] = dict(demo=self.demo[side].get(), port=self.port[side].get())
        validate_ports(settings)
        tolerance = number(self.tolerance.get(), "抬起姿态容差", .1, 5)
        self.progress.set("准备检查抬起位置…")
        text = self.text.get("1.0", "end-1c")
        def play():
            records = []
            lock = threading.Lock()
            def record(event):
                with lock:
                    records.append(event)
                self.events.put(("score-event", event))
            result = None
            error = None
            try:
                result = run_dual(self.links, score, calibration, tolerance, self.stop, self.abort, record)
            except Exception as exc:
                error = str(exc)
                raise
            finally:
                folder = ROOT / "logs" / "aero_score"
                folder.mkdir(parents=True, exist_ok=True)
                stem = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                report = dict(mode="finger-rehearsal-no-arms", settings=settings, calibration=calibration,
                              duration_us=score.duration_us, ignored_moves=score.ignored_moves,
                              result=result, error=error, score=text, records=records)
                path = folder / (stem + ".json")
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
                with (folder / (stem + ".csv")).open("w", newline="", encoding="utf-8-sig") as stream:
                    writer = csv.DictWriter(stream, fieldnames=("side", "demo", "kind", "scheduled_us", "sent_us", "finished_us", "lines", "changes", "outcome", "error", "packets_sent"))
                    writer.writeheader()
                    writer.writerows(records)
                self.events.put(("log", f"排练记录：{path}"))
                self.events.put(("result", result if error is None else error))
        self.task(play, "双手指法排练：移臂指令不执行")

    def stop_score(self):
        if self.busy:
            self.stop.set()
            self.log("已请求停止；通信正常的参与手会发送抬起目标。故障手不能保证释放。")
            return
        def release_idle():
            candidates = {}
            for side in SIDES:
                if self.connected[side] and (self.demo[side].get() or self.armed[side].get()):
                    candidates[side] = {ch: number(row["home"].get(), "抬起角度", 0, 90)
                                        for ch, row in self.rows[side].items() if row["home"].get().strip()}
            # Snapshot all Tk fields here, before creating the worker thread.
            def release(link, homes):
                link.feedback()
                raw = list(link.bus.raw)
                for ch, value in homes.items():
                    raw[ch] = link.bus.target(ch, value)
                if not self.abort.is_set():
                    link.bus.write(raw)
                link.feedback()
            def work():
                futures = [self.links[s].submit(lambda link, h=homes: release(link, h))
                           for s, homes in candidates.items() if homes]
                for future in futures:
                    future.result(timeout=5)
            if candidates:
                self.task(work, "抬起已配置手指")
        self.guard(release_idle)

    def hard_abort(self):
        self.abort.set()
        self.stop.set()
        self.log("已请求中止所有发送，不再补发抬起；驱动可能仍保持最后目标。")

    def log(self, message):
        self.logbox.configure(state="normal")
        self.logbox.insert("end", datetime.now().strftime("%H:%M:%S ") + str(message) + "\n")
        self.logbox.see("end")
        self.logbox.configure(state="disabled")

    def drain(self):
        for _ in range(500):
            try:
                kind, data = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "feedback":
                side, feedback = data
                self.feedback[side] = feedback
                for ch, row in self.rows[side].items():
                    row["q"].set(f"{feedback['q'][ch]:.2f}")
            elif kind == "connected":
                side, demo, port = data
                self.connected[side] = True
                self.connected_info[side] = dict(demo=demo, port=port)
                self.armed[side].set(False)
                self.connection_status[side].set("模拟已连接" if demo else f"实机 {port} 已连接")
            elif kind in ("disconnected", "fault"):
                side = data if kind == "disconnected" else data[0]
                self.connected[side] = False
                self.feedback[side] = None
                self.armed[side].set(False)
                self.connection_status[side].set("已断开" if kind == "disconnected" else "故障；请重新连接")
                for row in self.rows[side].values():
                    row["q"].set("—")
                if kind == "fault":
                    self.stop.set()
                    self.log(f"{LABELS[side]}：{data[1]}")
            elif kind == "score-event":
                side = data["side"]
                if data["kind"] == "hand":
                    label = "模拟" if data["demo"] else "实机"
                    changes = " ".join(CHARS[side][ch]+("↓" if down else "↑") for ch, down in data["changes"])
                    self.command_status[side].set(f"{LABELS[side]} [{label}] 指令：{changes}")
                    self.progress.set(f"谱时 {data['scheduled_us']/1e6:.3f} s")
                elif data["kind"] == "release-failed":
                    self.log(f"{LABELS[side]}未能释放：{data['error']}")
            elif kind in ("error", "log"):
                self.log(data)
                if kind == "error":
                    self.progress.set("未完成，请看错误日志")
            elif kind == "result":
                self.log(f"结果：{data}")
                self.progress.set("完成（指令发送，不代表发声验收）" if isinstance(data, dict) and all(v == "completed" for v in data.values()) else "已停止 / 未完成")
            elif kind == "done":
                if not self.closing:
                    self.set_busy(False)
                    self.status.set("就绪；左手模拟与实机状态见上方。")
        if self.closing:
            if self.task_thread and self.task_thread.is_alive():
                self.root.after(40, self.drain)
                return
            for link in self.links.values():
                link.shutdown.set()
            if all(not link.thread.is_alive() for link in self.links.values()):
                self.root.destroy()
                return
        self.root.after(40, self.drain)

    def close(self):
        if self.closing:
            return
        self.save()
        self.closing = True
        self.set_busy(True)
        self.hard_abort()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", help="双手软件模拟，不连接串口")
    parser.add_argument("--right-real", action="store_true", help="右手实机、左手模拟；重新载入右手测试参数，不自动连接")
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    config = args.config or (CONFIG.with_name("demo_config.json") if args.demo else CONFIG)
    root = tk.Tk()
    App(root, config_path=config, all_demo=args.demo, right_real=args.right_real)
    root.mainloop()


if __name__ == "__main__":
    main()
