"""
simulation.py
Run this first. Listens for one Command Center connection on TCP.
Sends JSON messages (one JSON object per newline) to client:
 - {'type':'state', ...}  periodic snapshots
 - {'type':'report', ...} when a drone halts on detection

Receives JSON commands from client:
 - {'type':'confirm', 'drone_id': int, 'poi_idx': int}
 - {'type':'reject', 'drone_id': int, 'poi_idx': int}
 - {'type':'manual_control', 'drone_id': int}
 - {'type':'stop_manual', 'drone_id': int}
 - {'type':'move_command', 'drone_id': int, 'dx': float, 'dy': float}
 - {'type':'manual_hold', 'drone_id': int, 'key': 'up'|'down'|..., 'val': bool}
"""

import socket
import threading
import json
import time
import math
import random
from queue import Queue, Empty

HOST = '127.0.0.1'
PORT = 9999

# Config
CONFIG = {
    'n_drones': 20,
    'grid_size': 300,
    'poi_count': 3,
    'region_div': 6,
    'detect_radius': 6.5,
    'region_time': 1.6,
    'snapshot_interval': 0.12
}

# -----------------------------
# Simulation classes
# -----------------------------
class Environment:
    def __init__(self, size, poi_count):
        self.size = size
        self.poi = []
        self.poi_found = {}
        for i in range(poi_count):
            x = random.uniform(0, size)
            y = random.uniform(0, size)
            self.poi.append((x, y))
            self.poi_found[i] = 'unknown'
    def getinfo(self, x, y, r):
        for idx, (px, py) in enumerate(self.poi):
            d = math.hypot(px - x, py - y)
            if d <= r:
                return idx
        return None

class Drone:
    def __init__(self, drone_id, x, y):
        self.id = drone_id
        self.x = x
        self.y = y
        self.vx = 0.0
        self.vy = 0.0
        self.speed = 1.5
        self.power = 100.0
        self.status = 'exploring'   # exploring | halted_for_manual | manual_control
        self.assigned_region = None
        self.region_timer = 0.0
        self.reported_poi = None
        self.manual_controls = {'up': False, 'down': False, 'left': False, 'right': False}

    def step_manual(self, env_size, dt):
        ds = 2.5
        if self.manual_controls['up']:
            self.y = max(0, self.y - ds * dt * 30)
        if self.manual_controls['down']:
            self.y = min(env_size, self.y + ds * dt * 30)
        if self.manual_controls['left']:
            self.x = max(0, self.x - ds * dt * 30)
        if self.manual_controls['right']:
            self.x = min(env_size, self.x + ds * dt * 30)

# -----------------------------
# Networking helper (JSON lines)
# -----------------------------
def send_json(sock, obj):
    try:
        line = json.dumps(obj, separators=(',',':')) + '\n'
        sock.sendall(line.encode('utf-8'))
    except Exception as e:
        raise

def recv_loop(conn, in_q, stop_event):
    """Read newline-delimited JSON from conn and put objects in in_q"""
    buf = b''
    conn.settimeout(0.5)
    while not stop_event.is_set():
        try:
            chunk = conn.recv(4096)
            if not chunk:
                stop_event.set()
                break
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                if not line:
                    continue
                try:
                    obj = json.loads(line.decode('utf-8'))
                    in_q.put(obj)
                except Exception as e:
                    print("Malformed JSON from client:", e)
        except socket.timeout:
            continue
        except Exception as e:
            print("Recv loop exception:", e)
            stop_event.set()
            break

# -----------------------------
# Main simulation server
# -----------------------------
def simulation_server(host, port, config):
    random.seed(42)
    env = Environment(config['grid_size'], config['poi_count'])
    drones = []
    for i in range(config['n_drones']):
        x = random.uniform(0, config['grid_size'])
        y = random.uniform(0, config['grid_size'])
        drones.append(Drone(i, x, y))

    region_div = config['region_div']
    sub_w = config['grid_size'] / region_div
    sub_h = config['grid_size'] / region_div
    all_regions = [(i,j) for i in range(region_div) for j in range(region_div)]
    explored_regions = set()

    # initial assignment
    for i,d in enumerate(drones):
        d.assigned_region = all_regions[i % len(all_regions)]

    # Setup socket server (single client allowed)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"[SIM] Listening on {host}:{port} — waiting for Command Center to connect...")
    conn, addr = srv.accept()
    print(f"[SIM] Command Center connected from {addr}")

    in_q = Queue()
    stop_event = threading.Event()
    recv_thread = threading.Thread(target=recv_loop, args=(conn, in_q, stop_event), daemon=True)
    recv_thread.start()

    last_time = time.time()
    acc = 0.0
    try:
        while not stop_event.is_set():
            now = time.time()
            dt = now - last_time
            last_time = now
            acc += dt

            # process incoming commands
            while True:
                try:
                    cmd = in_q.get_nowait()
                except Empty:
                    break
                ctype = cmd.get('type')
                if ctype == 'confirm':
                    pid = cmd.get('poi_idx')
                    if pid is not None:
                        env.poi_found[pid] = 'confirmed'
                    did = cmd.get('drone_id')
                    for d in drones:
                        if d.id == did:
                            d.status = 'exploring'
                            d.reported_poi = None
                elif ctype == 'reject':
                    pid = cmd.get('poi_idx')
                    if pid is not None:
                        env.poi_found[pid] = 'rejected'
                    did = cmd.get('drone_id')
                    for d in drones:
                        if d.id == did:
                            d.status = 'exploring'
                            d.reported_poi = None
                elif ctype == 'manual_control':
                    did = cmd.get('drone_id')
                    for d in drones:
                        if d.id == did and d.status == 'halted_for_manual':
                            d.status = 'manual_control'
                elif ctype == 'stop_manual':
                    did = cmd.get('drone_id')
                    for d in drones:
                        if d.id == did:
                            d.status = 'exploring'
                            d.reported_poi = None
                            d.manual_controls = {'up':False,'down':False,'left':False,'right':False}
                elif ctype == 'move_command':
                    did = cmd.get('drone_id')
                    dx = float(cmd.get('dx',0))
                    dy = float(cmd.get('dy',0))
                    for d in drones:
                        if d.id == did:
                            d.x = min(max(0, d.x + dx), env.size)
                            d.y = min(max(0, d.y + dy), env.size)
                elif ctype == 'manual_hold':
                    did = cmd.get('drone_id')
                    key = cmd.get('key')
                    val = bool(cmd.get('val', False))
                    for d in drones:
                        if d.id == did:
                            d.manual_controls[key] = val

            # update drones
            for d in drones:
                if d.status == 'manual_control':
                    d.step_manual(env.size, dt)
                    d.power = max(0.0, d.power - 0.02)
                elif d.status == 'halted_for_manual':
                    pass
                elif d.status == 'exploring':
                    rx, ry = d.assigned_region
                    rx0 = rx * sub_w
                    ry0 = ry * sub_h
                    rx1 = rx0 + sub_w
                    ry1 = ry0 + sub_h
                    target_x = random.uniform(rx0 + 0.1*sub_w, rx1 - 0.1*sub_w)
                    target_y = random.uniform(ry0 + 0.1*sub_h, ry1 - 0.1*sub_h)
                    angle = math.atan2(target_y - d.y, target_x - d.x)
                    d.vx = math.cos(angle) * d.speed * (0.6 + random.random()*0.8)
                    d.vy = math.sin(angle) * d.speed * (0.6 + random.random()*0.8)
                    d.x += d.vx * dt * 10.0
                    d.y += d.vy * dt * 10.0
                    d.x = min(max(d.x, rx0), rx1)
                    d.y = min(max(d.y, ry0), ry1)
                    d.power = max(0.0, d.power - 0.01)
                    d.region_timer += dt
                    if d.region_timer >= config['region_time']:
                        explored_regions.add(d.assigned_region)
                        remaining = [r for r in all_regions if r not in explored_regions]
                        if remaining:
                            d.assigned_region = random.choice(remaining)
                        else:
                            explored_regions.clear()
                            d.assigned_region = random.choice(all_regions)
                        d.region_timer = 0.0

                    # detection
                    poi_idx = env.getinfo(d.x, d.y, config['detect_radius'])
                    if poi_idx is not None:
                        d.status = 'halted_for_manual'
                        d.reported_poi = poi_idx
                        report = {'type':'report', 'drone_id':d.id, 'poi_idx':poi_idx,
                                  'drone_x':d.x, 'drone_y':d.y, 'time':time.time()}
                        try:
                            send_json(conn, report)
                        except Exception as e:
                            print("Send error (report):", e)
                            stop_event.set()
                            break

            # send periodic snapshot
            if acc >= config['snapshot_interval']:
                acc = 0.0
                snapshot = {
                    'type':'state',
                    'time': time.time(),
                    'env_size': env.size,
                    'poi': [{'x':p[0],'y':p[1],'status':env.poi_found[idx],'idx':idx}
                            for idx,p in enumerate(env.poi)],
                    'drones': [{'id':d.id,'x':d.x,'y':d.y,'status':d.status,'power':d.power,
                                'assigned_region': d.assigned_region,'reported_poi': d.reported_poi}
                               for d in drones],
                    'explored_regions': list(map(list, explored_regions))
                }
                try:
                    send_json(conn, snapshot)
                except Exception as e:
                    print("Send error (snapshot):", e)
                    stop_event.set()
                    break

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("[SIM] KeyboardInterrupt, shutting down.")
    finally:
        stop_event.set()
        try:
            conn.close()
        except:
            pass
        srv.close()
        print("[SIM] Stopped.")

if __name__ == '__main__':
    simulation_server(HOST, PORT, CONFIG)
