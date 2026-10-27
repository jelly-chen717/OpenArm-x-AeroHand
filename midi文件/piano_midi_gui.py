"""Standalone four-finger MIDI console. Importing or adjusting speed never drives hardware."""
import contextlib
import json
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from pathlib import Path
import piano_four_midi as player

ROOT=Path(__file__).resolve().parent

class Cancelled(Exception): pass

class Console:
    def __init__(self, root):
        self.root=root; self.song=None; self.path=None; self.bus=queue.Queue()
        self.stop_event=threading.Event(); self.busy=False; self.closing=False
        root.title('四指 MIDI 演奏台'); root.geometry('820x640'); root.minsize(720,590)
        style=ttk.Style(); style.theme_use('clam')
        style.configure('.',font=('Microsoft YaHei UI',11))
        style.configure('Title.TLabel',font=('Microsoft YaHei UI',22,'bold'))
        frame=ttk.Frame(root,padding=24); frame.pack(fill='both',expand=True)
        ttk.Label(frame,text='四指 MIDI 演奏台',style='Title.TLabel').pack(anchor='w')
        ttk.Label(frame,text='固定琴位 · do / re / mi / fa · 单旋律',foreground='#58656b').pack(anchor='w',pady=(4,18))
        self.select=ttk.Button(frame,text='选择 MIDI 文件…',command=self.choose); self.select.pack(anchor='w')
        self.filename=tk.StringVar(value='尚未选择曲目')
        ttk.Label(frame,textvariable=self.filename,wraplength=710).pack(anchor='w',pady=10)
        ttk.Separator(frame).pack(fill='x',pady=10)
        row=ttk.Frame(frame); row.pack(fill='x')
        ttk.Label(row,text='原曲速度').pack(side='left')
        self.percent=tk.StringVar(); ttk.Label(row,textvariable=self.percent,font=('Microsoft YaHei UI',18,'bold')).pack(side='right')
        self.speed=tk.DoubleVar(value=50)
        self.slider=ttk.Scale(frame,from_=20,to=150,variable=self.speed,command=self.changed)
        self.slider.pack(fill='x',pady=12)
        presets=ttk.Frame(frame); presets.pack(fill='x')
        self.presets=[]
        for value in (25,50,75,100,105.6,125):
            b=ttk.Button(presets,text=f'{value:g}%',command=lambda v=value:self.set_speed(v)); b.pack(side='left',padx=(0,6)); self.presets.append(b)
        self.detail=tk.StringVar(value='选择曲目后显示时长与实际计划速度')
        ttk.Label(frame,textvariable=self.detail,wraplength=710).pack(anchor='w',pady=14)
        ttk.Label(frame,text='速度在下次播放时生效；播放中锁定。新曲从低速验证。\n实物播放前断开官方上位机，保持手掌固定。',foreground='#72552b').pack(anchor='w',pady=(0,14))
        actions=ttk.Frame(frame); actions.pack(fill='x')
        self.sim=ttk.Button(actions,text='模拟预览',command=lambda:self.start(False));self.sim.pack(side='left',padx=(0,8))
        self.play=ttk.Button(actions,text='开始实物演奏',command=lambda:self.start(True));self.play.pack(side='left',padx=(0,8))
        self.stop=ttk.Button(actions,text='停止并释放',command=self.stop_event.set,state='disabled');self.stop.pack(side='left')
        self.status=tk.StringVar(value='就绪 · 未连接机械手')
        ttk.Label(frame,textvariable=self.status).pack(anchor='w',pady=12)
        self.log=tk.Text(frame,height=6,font=('Microsoft YaHei UI',10),state='disabled',wrap='word')
        self.log.pack(fill='both',expand=True)
        self.cfg=json.loads((ROOT/'piano_four_config.json').read_text(encoding='utf-8'))
        player.hand.validate(self.cfg,True)
        self.changed(); root.after(80,self.poll); root.protocol('WM_DELETE_WINDOW',self.close)

    def set_speed(self,value):
        self.speed.set(value);self.changed()
    def changed(self,*_):
        self.percent.set(f'{self.speed.get():.1f}%')
        if self.song:
            try:
                events,scale=player.plan(self.song,self.cfg['timing'],self.speed.get()/100)
                end=events[-1]['start']+events[-1]['hold']+self.cfg['timing']['down_seconds']+self.cfg['timing']['up_seconds']
                self.detail.set(f'{len(events)} 个音 · 原曲 {self.song.end_seconds:.1f} 秒\n请求 {self.speed.get():.1f}% · 计划实际 {100/scale:.1f}% · 演奏约 {end:.1f} 秒（另加准备时间）')
            except Exception as e:self.detail.set(str(e))
    def choose(self):
        path=filedialog.askopenfilename(initialdir=str(ROOT),filetypes=[('MIDI','*.mid *.midi')])
        if not path:return
        try:
            song=player.read_song(path)
            player.plan(song,self.cfg['timing'],0.5)
        except Exception as e:
            messagebox.showerror('无法导入',str(e));return
        self.song=song;self.path=Path(path);self.filename.set(self.path.name)
        self.set_speed(50); self.status.set('曲目已导入 · 未连接机械手')
    def lock(self, busy):
        self.busy=busy
        for w in [self.select,self.slider,self.sim,self.play]+self.presets:
            w.configure(state='disabled' if busy else 'normal')
        self.stop.configure(state='normal' if busy else 'disabled')
    def start(self,real):
        if self.busy:return
        if self.song is None:
            messagebox.showinfo('选择曲目','请先选择 MIDI 文件。');return
        events,scale=player.plan(self.song,self.cfg['timing'],self.speed.get()/100)
        self.stop_event.clear();self.lock(True)
        self.status.set('实物演奏中' if real else '模拟预览中 · 不连接机械手')
        threading.Thread(target=self.worker,args=(events,real),daemon=False).start()
    def worker(self,events,real):
        transport=motion=targets=None
        def check():
            if self.stop_event.is_set():raise Cancelled()
        def sleep(seconds):
            if self.stop_event.wait(max(0,seconds)):raise Cancelled()
        def send(pose):
            check()
            if transport:transport.send(pose)
        try:
            targets,constants=player.hand.build_targets(self.cfg)
            current=targets['rest']
            if real:
                transport=player.hand.Transport(self.cfg,constants)
                current=transport.read_pose();check()
            motion=player.hand.Motion(send,current,self.cfg['timing']['update_hz'],sleep=sleep)
            if real:motion.move(targets['rest'],self.cfg['timing']['startup_seconds'])
            origin=motion.clock()
            for i,event in enumerate(events,1):
                check();deadline=origin+event['start']
                if motion.clock()>deadline+0.2:raise RuntimeError('调度迟到，已停止后续动作')
                motion.until(deadline)
                self.bus.put(('progress',f"{i}/{len(events)} · {player.hand.NOTES[event['finger']]}"))
                motion.move(targets[event['finger']],self.cfg['timing']['down_seconds'])
                sleep(event['hold'])
                motion.move(targets['rest'],self.cfg['timing']['up_seconds'])
                sleep(self.cfg['timing']['gap_seconds'])
            self.bus.put(('log','演奏指令完成；请确认实际发声与回弹。' if real else '模拟完成，未连接机械手。'))
        except Cancelled:self.bus.put(('log','已停止后续音符。'))
        except Exception as e:self.bus.put(('log',f'错误：{e}'))
        finally:
            if transport:
                try:
                    if motion:
                        reset=player.hand.Motion(transport.send,motion.pose,self.cfg['timing']['update_hz'])
                        reset.move(targets['rest'],self.cfg['timing']['stop_seconds'])
                        self.bus.put(('log','准备姿态指令已发送，请观察琴键释放。'))
                except Exception as e:self.bus.put(('log',f'复位失败，请人工检查：{e}'))
                finally:
                    try:transport.close()
                    except Exception as e:self.bus.put(('log',f'关闭串口失败：{e}'))
            self.bus.put(('done',''))
    def poll(self):
        try:
            while True:
                kind,text=self.bus.get_nowait()
                if kind=='progress':self.status.set(text)
                elif kind=='log':
                    self.log.configure(state='normal');self.log.insert('end',text+'\n');self.log.see('end');self.log.configure(state='disabled')
                elif kind=='done':
                    self.lock(False);self.status.set('已结束 · 串口已关闭')
                    if self.closing:self.root.destroy();return
        except queue.Empty:pass
        self.root.after(80,self.poll)
    def close(self):
        if self.busy:
            self.closing=True;self.stop_event.set();self.status.set('正在停止并尝试释放，请稍候…')
        else:self.root.destroy()

if __name__=='__main__':
    root=tk.Tk();app=Console(root);root.mainloop()
