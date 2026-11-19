"""Separate desktop controller; no MuJoCo or motor SDK in this process."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from queue import SimpleQueue
import tkinter as tk
from tkinter import ttk
from openarm_app.sim_client import SimClient


class Controller:
    def __init__(self, root, port):
        self.root = root
        self.client = SimClient(port)
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.results = SimpleQueue()
        self.poll_pending = False
        self.action_pending = False
        self.snapshot = None
        self.initialized_fields = False
        root.title("OpenArm 双臂仿真控制端 · 理想位置模式")
        root.geometry("940x760")
        frame = ttk.Frame(root, padding=16)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="OpenArm 双臂仿真控制端", font=("Microsoft YaHei UI", 16, "bold")).pack(anchor="w")
        ttk.Label(frame, text=f"仅连接本机 MuJoCo · 127.0.0.1:{port} · 不连接真实电机").pack(anchor="w", pady=(4, 12))
        self.status = tk.StringVar(value="正在连接仿真端…")
        self.actual = {s: tk.StringVar(value=f"{label}关节：—") for s, label in (("right", "右臂"), ("left", "左臂"))}
        self.pose_status = {s: tk.StringVar(value=f"{label}末端：—") for s, label in (("right", "右臂"), ("left", "左臂"))}
        for variable in (self.status, self.actual["right"], self.pose_status["right"],
                         self.actual["left"], self.pose_status["left"]):
            ttk.Label(frame, textvariable=variable).pack(anchor="w", pady=3)
        self.progress = ttk.Progressbar(frame, maximum=100)
        self.progress.pack(fill="x", pady=8)
        selector = ttk.Frame(frame)
        selector.pack(fill="x")
        ttk.Label(selector, text="控制对象：").pack(side="left")
        self.arm_selection = tk.StringVar(value="right")
        for label, value in (("右臂", "right"), ("左臂", "left"), ("双臂同步", "both")):
            ttk.Radiobutton(selector, text=label, variable=self.arm_selection, value=value,
                            command=self.update_selection).pack(side="left", padx=8)
        tabs = ttk.Notebook(frame)
        tabs.pack(fill="x", pady=8)
        joints = ttk.Frame(tabs, padding=12)
        poses = ttk.Frame(tabs, padding=12)
        tabs.add(joints, text="左右臂目标角度")
        tabs.add(poses, text="末端目标 / 逆运动学")
        self.joints, self.pose, self.entries = {}, {}, {"right": [], "left": []}
        for row, (side, label) in enumerate((("right", "右臂"), ("left", "左臂"))):
            for parent, storage, labels in (
                    (joints, self.joints, [f"J{i} (rad)" for i in range(1, 8)]),
                    (poses, self.pose, ["X (m)","Y (m)","Z (m)","qw","qx","qy","qz"])):
                group = ttk.LabelFrame(parent, text=label, padding=6)
                group.grid(row=row, column=0, columnspan=7, sticky="ew", pady=3)
                storage[side] = self.fields(group, labels, [""]*7, side)
        ttk.Button(joints, text="执行所选关节目标", command=lambda: self.send("move_joints", q=self.read_selected(self.joints))).grid(row=2, column=0, columnspan=3, sticky="w", pady=12)
        ttk.Label(poses, text="相对于 arm_origin；四元数顺序 wxyz；双臂分别求解终点、同步执行。").grid(row=2, column=0, columnspan=7, sticky="w", pady=6)
        ttk.Button(poses, text="IK 预览（不运动）", command=lambda: self.send("preview_pose", pose=self.read_selected(self.pose))).grid(row=3, column=0, columnspan=3, sticky="w")
        ttk.Button(poses, text="执行所选末端目标", command=lambda: self.send("move_pose", pose=self.read_selected(self.pose))).grid(row=3, column=3, columnspan=3, sticky="w")
        bar = ttk.Frame(frame)
        bar.pack(fill="x", pady=12)
        for label, action in (("全部暂停", "pause"), ("继续", "resume"), ("全部停止并保持", "stop"), ("所选臂回 Home", "home")):
            ttk.Button(bar, text=label, command=lambda a=action: self.send(a)).pack(side="left", padx=(0, 6))
        ttk.Button(bar, text="当前值填入输入框", command=self.fill_current).pack(side="left")
        self.message = tk.StringVar(value="连接后自动填入双臂当前值；选择右臂、左臂或双臂同步，再输入目标。")
        ttk.Label(frame, textvariable=self.message, wraplength=880, foreground="#225b85").pack(anchor="w", pady=8)
        ttk.Label(frame, text="默认 0.2 rad/s、0.4 rad/s²。运动中先停止或等待完成。夹爪固定；未启用双臂避碰规划。", wraplength=880).pack(anchor="w")
        self.update_selection()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.report_callback_exception = lambda typ, exc, tb: self.message.set(str(exc))
        root.after(100, self.pump)

    def fields(self, parent, labels, defaults, side):
        variables = []
        for index, (label, value) in enumerate(zip(labels, defaults)):
            ttk.Label(parent, text=label).grid(row=0, column=index, sticky="w")
            variable = tk.StringVar(value=str(value))
            entry = ttk.Entry(parent, textvariable=variable, width=12)
            entry.grid(row=1, column=index, padx=(0, 4), pady=5)
            self.entries[side].append(entry)
            variables.append(variable)
        return variables

    def update_selection(self):
        for side, entries in self.entries.items():
            enabled = self.arm_selection.get() in (side, "both")
            for entry in entries:
                entry.state(["!disabled"] if enabled else ["disabled"])

    def read_selected(self, fields):
        side = self.arm_selection.get()
        if side == "both":
            return {s: self.read(fields[s]) for s in ("right", "left")}
        return self.read(fields[side])

    def read(self, fields):
        try:
            return [float(v.get()) for v in fields]
        except ValueError as exc:
            raise ValueError("请在所有输入框中填写有效数字") from exc

    def submit(self, label, function):
        if label == "poll" and (self.poll_pending or self.action_pending):
            return
        if label != "poll" and self.action_pending:
            self.message.set("正在处理上一条指令，请稍后再试。")
            return
        if label == "poll":
            self.poll_pending = True
        else:
            self.action_pending = True
        def job():
            try:
                self.results.put((label, function(), None))
            except Exception as exc:
                self.results.put((label, None, str(exc)))
        self.worker.submit(job)

    def send(self, action, **values):
        # Capture selection before the asynchronous request; preview replies also
        # carry their arm so switching tabs/selection cannot misapply the result.
        if action in ("move_joints", "move_pose", "preview_pose", "home"):
            values["arm"] = self.arm_selection.get()
        if self.snapshot is None:
            self.message.set("请先连接新版双臂仿真服务。")
            return
        self.submit(action, lambda: self.client.command(action, **values))

    def fill_current(self):
        if self.snapshot:
            for side in ("right", "left"):
                for fields, values in ((self.joints[side], self.snapshot["arms"][side]["q"]),
                                       (self.pose[side], self.snapshot["arms"][side]["pose"])):
                    for field, value in zip(fields, values):
                        field.set(f"{value:.9f}")

    def pump(self):
        while not self.results.empty():
            label, result, error = self.results.get()
            if label == "poll":
                self.poll_pending = False
            else:
                self.action_pending = False
            if error:
                if label == "poll":
                    self.status.set("未连接：请先打开 MuJoCo 仿真端")
                self.message.set(error)
            elif label == "preview_pose":
                side = result["arm"]
                goals = result["q_goal"] if side == "both" else {side: result["q_goal"]}
                for s, goal in goals.items():
                    for field, value in zip(self.joints[s], goal):
                        field.set(f"{value:.9f}")
                self.message.set("IK 预览成功，解已填入对应关节页；机械臂没有运动。")
            else:
                if result.get("api_version") != 2 or "arms" not in result:
                    self.snapshot = None
                    self.status.set("仿真端版本过旧，请关闭旧窗口并重新启动 start_mujoco.cmd")
                    continue
                self.snapshot = result
                names = {"IDLE":"等待目标", "EXECUTING":"执行中", "PAUSED":"已暂停", "COMPLETED":"已完成", "STOPPED":"已停止"}
                self.status.set("已连接 · "+names.get(result["state"], result["state"]))
                for side, name in (("right", "右臂"), ("left", "左臂")):
                    self.actual[side].set(name+"关节 (rad)："+"  ".join(f"{x:.4f}" for x in result["arms"][side]["q"]))
                    self.pose_status[side].set(name+"末端 (m / wxyz)："+"  ".join(f"{x:.4f}" for x in result["arms"][side]["pose"]))
                self.progress["value"] = result["progress"]*100
                if not self.initialized_fields:
                    self.fill_current()
                    self.initialized_fields = True
                if label != "poll":
                    self.message.set("指令已接受："+label)
        if not self.poll_pending and not self.action_pending:
            self.submit("poll", self.client.state)
        self.root.after(300, self.pump)

    def close(self):
        self.worker.shutdown(wait=False, cancel_futures=True)
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8769)
    args = parser.parse_args()
    root = tk.Tk()
    Controller(root, args.port)
    root.mainloop()


if __name__ == "__main__":
    main()
