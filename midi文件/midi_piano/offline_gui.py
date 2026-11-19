"""Four-finger A/B preview. Only simulated devices are constructed."""
import csv
import json
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from pathlib import Path

from .planner import ROOT, read_config, build_plan
from .midi_reader import read_song
from .devices import SimHand, SimArm, StoppableClock
from .executor import execute


class App:
    def __init__(self, window):
        self.window = window
        window.title("七音双琴位 · 离线联动")
        window.geometry("1080x720")
        self.cfg = read_config(ROOT / "config_four_ab.json")
        self.plan = None
        self.running = False
        self.stop_event = threading.Event()
        self.events = queue.Queue()
        self.path = tk.StringVar(value=str(ROOT / "examples" / "seven_note_scale.mid"))
        self.speed = tk.StringVar(value="50")
        self.track = tk.StringVar()
        self.channel = tk.StringVar()
        self.status = tk.StringVar(value="A: do re mi fa ｜ B: sol la si ｜ 换位 2 秒为模拟假设")
        bar = ttk.Frame(window, padding=10); bar.pack(fill="x")
        self.controls = []
        def button(text, command):
            widget = ttk.Button(bar, text=text, command=command); widget.pack(side="left", padx=3)
            self.controls.append(widget)
        button("选择 MIDI", self.choose)
        for label, variable, width in (("速度 %", self.speed, 6), ("音轨(可空)", self.track, 4), ("通道(可空)", self.channel, 4)):
            ttk.Label(bar, text=label).pack(side="left")
            widget=ttk.Entry(bar, textvariable=variable, width=width); widget.pack(side="left"); self.controls.append(widget)
        button("生成计划", self.preview)
        button("导出时间表", self.export)
        button("实时模拟", self.play)
        self.stop_button = ttk.Button(bar, text="停止并释放", command=self.stop_event.set, state="disabled")
        self.stop_button.pack(side="left")
        ttk.Label(window, textvariable=self.path, wraplength=1040).pack(anchor="w", padx=12)
        ttk.Label(window, textvariable=self.status, wraplength=1040).pack(anchor="w", padx=12, pady=8)
        self.stage = tk.Canvas(window, height=95, bg="#eef3f7", highlightthickness=0); self.stage.pack(fill="x", padx=12)
        self.stage.create_text(260, 25, text="A 位 · do / re / mi / fa", font=("Microsoft YaHei", 14))
        self.stage.create_text(780, 25, text="B 位 · sol / la / si", font=("Microsoft YaHei", 14))
        self.marker=self.stage.create_oval(245, 48, 275, 78, fill="#337dba")
        self.tree=ttk.Treeview(window, columns=("time", "duration", "kind", "group", "note"), show="headings", height=14)
        for key,label in zip(self.tree['columns'],("开始(s)", "耗时(s)", "动作", "琴位", "音符/手指")):
            self.tree.heading(key,text=label); self.tree.column(key,width=180)
        scroll=ttk.Scrollbar(window, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set); scroll.pack(side="right",fill="y"); self.tree.pack(fill="both",expand=True,padx=12)
        self.log=tk.Text(window,height=6); self.log.pack(fill="x",padx=12,pady=8)
        window.protocol("WM_DELETE_WINDOW",self.close)
        window.after(40,self.poll)
        self.preview()

    def choose(self):
        path=filedialog.askopenfilename(filetypes=[("MIDI", "*.mid *.midi")])
        if path:
            self.path.set(path); self.speed.set("50"); self.track.set(""); self.channel.set(""); self.preview()

    def preview(self):
        try:
            song=read_song(Path(self.path.get()), int(self.track.get()) if self.track.get() else None, int(self.channel.get()) if self.channel.get() else None)
            self.plan=build_plan(song,self.cfg,float(self.speed.get())/100)
            self.tree.delete(*self.tree.get_children())
            for i,a in enumerate(self.plan['actions']):
                self.tree.insert("", "end", iid=str(i), values=(f"{a['start']:.3f}",f"{a['duration']:.3f}",a['kind'],a.get('group',''),f"{a.get('pitch','')} {a.get('finger','')}"))
            self.status.set(f"请求 {self.plan['requested_speed']:.1%} ｜ 实际 {self.plan['effective_speed']:.1%} ｜ {self.plan['total_seconds']:.2f} 秒 ｜ 换位 {self.plan['group_changes']} 次；换位耗时未实测")
            return True
        except Exception as exc:
            self.plan=None; self.tree.delete(*self.tree.get_children()); self.status.set("计划无效，请检查 MIDI 或参数")
            messagebox.showerror("无法生成计划", str(exc)); return False

    def export(self):
        if not self.preview(): return
        path=filedialog.asksaveasfilename(defaultextension=".plan.json", initialfile=Path(self.path.get()).stem+".plan.json",filetypes=[("计划", "*.plan.json")])
        if not path: return
        output=Path(path)
        if not output.name.endswith('.plan.json') or output.resolve()==Path(self.path.get()).resolve():
            messagebox.showerror("无法导出", "请选择以 .plan.json 结尾的新文件"); return
        try:
            output.write_text(json.dumps(self.plan,ensure_ascii=False,indent=2),encoding="utf-8")
            with output.with_suffix('.csv').open('w',encoding='utf-8-sig',newline='') as f:
                writer=csv.writer(f); writer.writerow(['开始秒','耗时秒','动作','目标琴位','MIDI音高','手指'])
                for a in self.plan['actions']: writer.writerow([a['start'],a['duration'],a['kind'],a.get('group',''),a.get('pitch',''),a.get('finger','')])
            self.status.set(f"已导出 JSON 与 CSV：{output}")
        except Exception as exc: messagebox.showerror("导出失败",str(exc))

    def play(self):
        if self.running or not self.preview(): return
        self.running=True; self.stop_event.clear()
        for widget in self.controls: widget.configure(state='disabled')
        self.stop_button.configure(state='normal'); self.log.delete('1.0','end')
        def worker():
            clock=StoppableClock(self.stop_event); hand=SimHand(clock); arm=SimArm(clock,hand)
            try:
                execute(self.plan,self.cfg,hand,arm,clock,lambda msg:self.events.put(('log',msg)))
                result='模拟完成'
            except Exception as exc: result=str(exc)
            self.events.put(('done',f'{result}；模拟手释放={hand.released}，模拟臂停止={arm.stopped}'))
        threading.Thread(target=worker,daemon=True).start()

    def poll(self):
        try:
            while True:
                kind,msg=self.events.get_nowait()
                self.log.insert('end',msg+'\n'); self.log.see('end')
                if kind=='log' and 'move' in msg:
                    target=msg.split()[-1]; x=260 if target=='A' else 780
                    self.stage.coords(self.marker,x-15,48,x+15,78)
                if kind=='done':
                    self.running=False; self.status.set(msg)
                    for widget in self.controls: widget.configure(state='normal')
                    self.stop_button.configure(state='disabled')
        except queue.Empty: pass
        self.window.after(40,self.poll)

    def close(self):
        if self.running:
            self.stop_event.set(); self.window.after(80,self.close)
        else: self.window.destroy()


def main():
    window=tk.Tk(); App(window); window.mainloop()
