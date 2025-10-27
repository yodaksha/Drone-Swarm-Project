"""
command_center.py
Connects to simulation.py on localhost:9999.
Receives JSON messages and renders Tkinter UI.
Sends JSON commands back to the simulation.
"""

import socket
import threading
import json
import time
import tkinter as tk
from tkinter import ttk, messagebox
from queue import Queue, Empty

HOST = '127.0.0.1'
PORT = 9999

# ---------------------------
# Networking helpers
# ---------------------------
def send_json(sock, obj):
    line = json.dumps(obj, separators=(',',':')) + '\n'
    sock.sendall(line.encode('utf-8'))

def recv_loop(sock, out_q, stop_event):
    buf = b''
    sock.settimeout(0.5)
    while not stop_event.is_set():
        try:
            chunk = sock.recv(4096)
            if not chunk:
                stop_event.set()
                break
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n',1)
                if not line:
                    continue
                try:
                    obj = json.loads(line.decode('utf-8'))
                    out_q.put(obj)
                except Exception as e:
                    print("Malformed JSON from sim:", e)
        except socket.timeout:
            continue
        except Exception as e:
            print("Recv exception:", e)
            stop_event.set()
            break

# ---------------------------
# Tkinter UI
# ---------------------------
class CommandCenterApp:
    def __init__(self, root, sock):
        self.root = root
        self.sock = sock
        self.recv_q = Queue()
        self.stop_event = threading.Event()
        self.selected_manual_drone = None
        self.pending_reports = []
        self.last_snapshot = None

        # start recv thread
        self.recv_thread = threading.Thread(target=recv_loop, args=(sock, self.recv_q, self.stop_event), daemon=True)
        self.recv_thread.start()

        # UI
        self.canvas_size = 600
        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg='white')
        self.canvas.grid(row=0, column=0, rowspan=6)

        self.report_listbox = tk.Listbox(root, height=12)
        self.report_listbox.grid(row=0, column=1)
        self.report_listbox.bind('<<ListboxSelect>>', self.on_report_select)

        btn_confirm = ttk.Button(root, text="Confirm", command=self.confirm_report)
        btn_reject = ttk.Button(root, text="Reject", command=self.reject_report)
        btn_confirm.grid(row=1, column=1, sticky='we')
        btn_reject.grid(row=2, column=1, sticky='we')

        ctrl_frame = ttk.LabelFrame(root, text="Manual Controls")
        ctrl_frame.grid(row=3, column=1, sticky='we', padx=4, pady=4)
        self.btn_up = ttk.Button(ctrl_frame, text="↑")
        self.btn_down = ttk.Button(ctrl_frame, text="↓")
        self.btn_left = ttk.Button(ctrl_frame, text="←")
        self.btn_right = ttk.Button(ctrl_frame, text="→")
        self.btn_up.grid(row=0, column=1)
        self.btn_left.grid(row=1, column=0)
        self.btn_down.grid(row=1, column=1)
        self.btn_right.grid(row=1, column=2)

        # bind press/release for hold
        self.btn_up.bind('<ButtonPress-1>', lambda e: self.hold_control('up', True))
        self.btn_up.bind('<ButtonRelease-1>', lambda e: self.hold_control('up', False))
        self.btn_down.bind('<ButtonPress-1>', lambda e: self.hold_control('down', True))
        self.btn_down.bind('<ButtonRelease-1>', lambda e: self.hold_control('down', False))
        self.btn_left.bind('<ButtonPress-1>', lambda e: self.hold_control('left', True))
        self.btn_left.bind('<ButtonRelease-1>', lambda e: self.hold_control('left', False))
        self.btn_right.bind('<ButtonPress-1>', lambda e: self.hold_control('right', True))
        self.btn_right.bind('<ButtonRelease-1>', lambda e: self.hold_control('right', False))

        self.info_label = ttk.Label(root, text="Not connected")
        self.info_label.grid(row=4, column=1)

        root.after(100, self.poll)

    def hold_control(self, key, val):
        if self.selected_manual_drone is None:
            return
        cmd = {'type':'manual_hold','drone_id':self.selected_manual_drone,'key':key,'val':val}
        try: send_json(self.sock, cmd)
        except: pass

    def on_report_select(self, evt):
        sel = self.report_listbox.curselection()
        if not sel:
            return
        idx = sel[0]
        rpt = self.pending_reports[idx]
        self.selected_manual_drone = rpt['drone_id']
        # request manual control
        try:
            send_json(self.sock, {'type':'manual_control','drone_id':self.selected_manual_drone})
        except: pass

    def confirm_report(self):
        sel = self.report_listbox.curselection()
        if not sel:
            messagebox.showinfo("Info","Select a report")
            return
        idx = sel[0]
        rpt = self.pending_reports.pop(idx)
        try:
            send_json(self.sock, {'type':'confirm','drone_id':rpt['drone_id'],'poi_idx':rpt['poi_idx']})
        except: pass
        self.refresh_reports()

    def reject_report(self):
        sel = self.report_listbox.curselection()
        if not sel:
            messagebox.showinfo("Info","Select a report")
            return
        idx = sel[0]
        rpt = self.pending_reports.pop(idx)
        try:
            send_json(self.sock, {'type':'reject','drone_id':rpt['drone_id'],'poi_idx':rpt['poi_idx']})
        except: pass
        self.refresh_reports()

    def refresh_reports(self):
        self.report_listbox.delete(0, tk.END)
        for r in self.pending_reports:
            s = f"Drone {r['drone_id']} -> POI {r['poi_idx']} @ ({r['drone_x']:.1f},{r['drone_y']:.1f})"
            self.report_listbox.insert(tk.END, s)

    def poll(self):
        updated = False
        while True:
            try:
                msg = self.recv_q.get_nowait()
            except Empty:
                break
            if msg.get('type') == 'report':
                self.pending_reports.append(msg)
                self.refresh_reports()
            elif msg.get('type') == 'state':
                self.last_snapshot = msg
                updated = True

        if updated and self.last_snapshot:
            self.draw_snapshot(self.last_snapshot)

        self.root.after(100, self.poll)

    def draw_snapshot(self, snap):
        self.canvas.delete('all')
        env_size = snap['env_size']
        scale = self.canvas_size / env_size

        # POIs
        for poi in snap['poi']:
            x = poi['x'] * scale
            y = poi['y'] * scale
            status = poi.get('status','unknown')
            color = 'orange' if status=='unknown' else ('green' if status=='confirmed' else 'red')
            r = 6
            self.canvas.create_oval(x-r,y-r,x+r,y+r, fill=color, outline='black')
            self.canvas.create_text(x+10,y+4, text=f"POI{poi['idx']}", anchor='w', font=('TkDefaultFont',8))

        # Drones
        for d in snap['drones']:
            x = d['x'] * scale
            y = d['y'] * scale
            r = 4
            fill = 'blue'
            if d['status']=='halted_for_manual': fill = 'yellow'
            if d['status']=='manual_control': fill = 'purple'
            self.canvas.create_oval(x-r,y-r,x+r,y+r, fill=fill, outline='black')
            self.canvas.create_text(x+6,y-6, text=str(d['id']), anchor='w', font=('TkDefaultFont',7))

        self.info_label.config(text=f"Drones: {len(snap['drones'])} — Pending: {len(self.pending_reports)}")

    def shutdown(self):
        self.stop_event.set()
        try:
            self.sock.close()
        except:
            pass

# ---------------------------
# Main connect and run
# ---------------------------
def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((HOST, PORT))
    except Exception as e:
        print("Could not connect to simulation:", e)
        return

    root = tk.Tk()
    root.title("Command Center")
    app = CommandCenterApp(root, sock)

    def on_close():
        if messagebox.askokcancel("Quit","Stop and exit?"):
            try:
                send_json(sock, {'type':'stop_manual','drone_id':-1})
            except:
                pass
            app.stop_event.set()
            try:
                sock.close()
            except:
                pass
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == '__main__':
    main()
