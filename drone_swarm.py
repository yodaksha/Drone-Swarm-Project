import numpy as np
import time
import math
import random
from PIL import Image, ImageDraw, ImageFont
import tkinter as tk
from tkinter import ttk, scrolledtext
from PIL import ImageTk
import io
from datetime import datetime
import threading
import queue
import logging

# Logging 
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('drone_simulation.log'),
        logging.StreamHandler()
    ]
)

# ISP messages

MSG_TARGET_FOUND   = 'target_found'
MSG_STATUS_UPDATE  = 'status_update'
MSG_ACCEPT_TARGET  = 'accept_target'
MSG_DISCARD_TARGET = 'discard_target'
MSG_MANUAL_CONTROL = 'manual_control'
MSG_RETURN_EXPLORE = 'return_explore'


class Config:
    ENV_SIZE          = 60         
    NUM_TARGETS       = 4

    NUM_DRONES        = 20
    INITIAL_POWER     = 1500
    POWER_RATE        = 0.15        
    DETECTION_RADIUS  = 2.5
    DRONE_SPEED       = 0.6
    MANUAL_SPEED      = 1.2
    LOW_POWER_THRESH  = 250

    REGION_SIZE       = 5           
    REGION_DWELL      = 40          

    MIN_DIST          = 1.8
    AVOID_FORCE       = 0.35

    CELL_SIZE         = 9
    SIM_UPDATE_EVERY  = 4         
    UI_POLL_MS        = 50        
    SIM_DELAY         = 0.08       


# ENVIRONMENT
class Environment:
    def __init__(self):
        self.size = Config.ENV_SIZE
        self.grid = np.zeros((self.size, self.size))
        self.targets = []

        min_sep = self.size // (Config.NUM_TARGETS + 1)
        attempts = 0
        while len(self.targets) < Config.NUM_TARGETS and attempts < 500:
            x = random.randint(5, self.size - 5)
            y = random.randint(5, self.size - 5)
            too_close = any(math.dist((x, y), t) < min_sep for t in self.targets)
            if not too_close:
                self.targets.append((x, y))
                self.grid[y, x] = 1
            attempts += 1

        logging.info(f"Environment {self.size}x{self.size}, targets: {self.targets}")

    def getinfo(self, x, y, r):
        for tx, ty in self.targets:
            if math.sqrt((x - tx) ** 2 + (y - ty) ** 2) <= r:
                return True, (tx, ty)
        return False, None

    def render(self, drones, explored_regions, confirmed_targets):
        cs = Config.CELL_SIZE
        sz = self.size * cs
        img = Image.new('RGB', (sz, sz), color='#1a1a2e')
        draw = ImageDraw.Draw(img)

        for (rx, ry) in explored_regions:
            x1, y1 = rx * cs, ry * cs
            draw.rectangle([x1, y1, x1 + cs, y1 + cs], fill='#16213e')

        for i in range(0, self.size + 1, Config.REGION_SIZE):
            px = i * cs
            draw.line([(px, 0), (px, sz)], fill='#0f3460', width=1)
            draw.line([(0, px), (sz, px)], fill='#0f3460', width=1)

        for tx, ty in self.targets:
            cx, cy = tx * cs, ty * cs
            draw.ellipse([cx - 6, cy - 6, cx + 6, cy + 6], fill='#e94560', outline='#ff6b6b', width=2)

        for ct in confirmed_targets:
            tx, ty = ct['target_position']
            cx, cy = int(tx) * cs, int(ty) * cs
            draw.ellipse([cx - 9, cy - 9, cx + 9, cy + 9], outline='#00ff88', width=2)

        STATUS_COLORS = {
            'exploring': '#4fc3f7',
            'halted':    '#ffa726',
            'manual':    '#ce93d8',
            'low_power': '#ef5350',
            'dead':      '#546e7a',
        }
        for drone in drones:
            cx = int(drone.x * cs)
            cy = int(drone.y * cs)
            color = STATUS_COLORS.get(drone.display_status(), '#4fc3f7')
            r = 4
            draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=color, outline='white', width=1)

            if drone.status == 'halted':
                dr = int(Config.DETECTION_RADIUS * cs)
                draw.ellipse([cx - dr, cy - dr, cx + dr, cy + dr], outline='#ffa726', width=1)

        return img


# DRONE
class Drone:
    def __init__(self, drone_id, x, y):
        self.id               = drone_id
        self.x                = float(x)
        self.y                = float(y)
        self.vx               = 0.0
        self.vy               = 0.0
        self.power_remaining  = Config.INITIAL_POWER
        self.status           = 'exploring'
        self.found_target     = None
        self.low_power_warned = False

        self.sweep_path       = []   
        self.sweep_idx        = 0    
        self.dwell_timer      = 0    

    # Navigation
    def set_sweep_path(self, regions):
        self.sweep_path  = regions
        self.sweep_idx   = 0
        self.dwell_timer = 0

    def current_region(self):
        if self.sweep_idx < len(self.sweep_path):
            return self.sweep_path[self.sweep_idx]
        return None

    def move_towards_region(self, rx, ry):
        cx = rx + Config.REGION_SIZE / 2
        cy = ry + Config.REGION_SIZE / 2
        dx, dy = cx - self.x, cy - self.y
        dist   = math.sqrt(dx * dx + dy * dy)
        if dist > 0.4:
            self.vx = (dx / dist) * Config.DRONE_SPEED + random.uniform(-0.05, 0.05)
            self.vy = (dy / dist) * Config.DRONE_SPEED + random.uniform(-0.05, 0.05)
        else:
            self.vx = random.uniform(-0.2, 0.2)
            self.vy = random.uniform(-0.2, 0.2)

    def avoid_collision(self, others):
        
        ax, ay = 0.0, 0.0
        for other in others:
            if other.id == self.id:
                continue
            dx, dy = self.x - other.x, self.y - other.y
            d = math.sqrt(dx * dx + dy * dy)
            if 0 < d < Config.MIN_DIST:
                f   = Config.AVOID_FORCE / (d + 0.1)
                ax += (dx / d) * f
                ay += (dy / d) * f
        self.vx += ax
        self.vy += ay

    def update(self, env_size):
        if self.power_remaining <= 0:
            if self.status != 'dead':
                self.status = 'dead'
                logging.error(f"Drone {self.id} dead at ({self.x:.1f},{self.y:.1f})")
            return

        self.x += self.vx
        self.y += self.vy
        self.x = max(0.0, min(float(env_size - 1), self.x))
        self.y = max(0.0, min(float(env_size - 1), self.y))
        self.power_remaining -= Config.POWER_RATE

        if (self.power_remaining < Config.LOW_POWER_THRESH
                and not self.low_power_warned):
            self.low_power_warned = True
            logging.warning(f"Drone {self.id} low power: {self.power_remaining:.0f}")

    def display_status(self):
        if self.status == 'dead':
            return 'dead'
        if self.power_remaining < Config.LOW_POWER_THRESH and self.status == 'exploring':
            return 'low_power'
        return self.status


# BOUSTROPHEDON SWEEP ALGORITHM
def build_boustrophedon_path(regions_in_slice):
    if not regions_in_slice:
        return []

    col_map = {}
    for (rx, ry) in regions_in_slice:
        col_map.setdefault(rx, []).append(ry)

    path = []
    for col_idx, rx in enumerate(sorted(col_map)):
        ry_list = sorted(col_map[rx])
        if col_idx % 2 == 1:
            ry_list = list(reversed(ry_list))  
        for ry in ry_list:
            path.append((rx, ry))

    return path


def partition_regions_to_drones(all_regions, drones):
    active = [d for d in drones if d.status not in ('dead',)]
    if not active:
        return {}

    sorted_regions = sorted(all_regions, key=lambda r: (r[0], r[1]))
    n = len(sorted_regions)
    k = len(active)

    assignments = {}
    for i, drone in enumerate(active):
        start = (i * n) // k
        end   = ((i + 1) * n) // k
        assignments[drone.id] = sorted_regions[start:end]

    return assignments


# SIMULATION  
class DroneSimulation:
    def __init__(self):
        self.env      = Environment()
        self.drones   = [
            Drone(i,
                  random.uniform(0, self.env.size),
                  random.uniform(0, self.env.size))
            for i in range(Config.NUM_DRONES)
        ]

        rs = Config.REGION_SIZE
        self.all_regions      = [(rx, ry)
                                  for rx in range(0, self.env.size, rs)
                                  for ry in range(0, self.env.size, rs)]
        self.explored_regions = set()

        self.to_simulation = queue.Queue()
        self.to_ui         = queue.Queue()

        self.iteration         = 0
        self.running           = True
        self.start_time        = time.time()
        self.confirmed_targets = []  

        self._initial_assignment()

        logging.info(f"Simulation ready – {Config.NUM_DRONES} drones, "
                     f"{len(self.env.targets)} targets, "
                     f"{len(self.all_regions)} sub-regions")

    def _initial_assignment(self):
        assignments = partition_regions_to_drones(self.all_regions, self.drones)
        for drone in self.drones:
            slice_regions = assignments.get(drone.id, [])
            path = build_boustrophedon_path(slice_regions)
            drone.set_sweep_path(path)
            logging.debug(f"Drone {drone.id} assigned {len(path)} waypoints")

    def _reassign_orphan_regions(self):
        active = [d for d in self.drones
                  if d.status == 'exploring' and d.power_remaining > 0]
        if not active:
            return

        covered = set()
        for d in active:
            for r in d.sweep_path[d.sweep_idx:]:
                covered.add(r)

        orphans = [r for r in self.all_regions
                   if r not in self.explored_regions and r not in covered]

        if not orphans:
            return

        for region in orphans:
            best = min(active,
                       key=lambda d: math.dist((d.x, d.y),
                                               (region[0] + Config.REGION_SIZE / 2,
                                                region[1] + Config.REGION_SIZE / 2)))
            best.sweep_path.append(region)

    # Main Loop
    def run(self):
        reassign_counter = 0

        while self.running:
            self.iteration += 1
            reassign_counter += 1

            for drone in self.drones:
                if drone.status == 'exploring' and drone.power_remaining > 0:
                    found, tpos = self.env.getinfo(
                        drone.x, drone.y, Config.DETECTION_RADIUS)
                    if found and drone.found_target is None:
                        drone.status      = 'halted'
                        drone.found_target = tpos
                        self.to_ui.put({
                            'type':            MSG_TARGET_FOUND,
                            'drone_id':        drone.id,
                            'position':        (drone.x, drone.y),
                            'target_position': tpos,
                            'timestamp':       time.time()
                        })
                        logging.info(f"Drone {drone.id} found target {tpos}")

            try:
                while not self.to_simulation.empty():
                    msg = self.to_simulation.get_nowait()

                    if msg['type'] in (MSG_ACCEPT_TARGET, MSG_DISCARD_TARGET):
                        d = self.drones[msg['drone_id']]
                        d.status      = 'exploring'
                        d.found_target = None
                        if msg['type'] == MSG_ACCEPT_TARGET:
                            self.confirmed_targets.append(msg.get('report'))
                        logging.info(f"Drone {msg['drone_id']} released "
                                     f"({'accepted' if msg['type']==MSG_ACCEPT_TARGET else 'discarded'})")

                    elif msg['type'] == MSG_MANUAL_CONTROL:
                        d = self.drones[msg['drone_id']]
                        if d.power_remaining <= 0:
                            continue
                        d.status = 'manual'
                        spd = Config.MANUAL_SPEED
                        direction = msg['direction']
                        if direction == 'up':    d.y -= spd
                        elif direction == 'down':  d.y += spd
                        elif direction == 'left':  d.x -= spd
                        elif direction == 'right': d.x += spd
                        d.x = max(0, min(self.env.size - 1, d.x))
                        d.y = max(0, min(self.env.size - 1, d.y))

            except queue.Empty:
                pass

            if reassign_counter % 100 == 0:
                self._reassign_orphan_regions()

            for drone in self.drones:
                if drone.status != 'exploring' or drone.power_remaining <= 0:
                    continue

                region = drone.current_region()

                if region is None:
                    drone.vx = random.uniform(-0.1, 0.1)
                    drone.vy = random.uniform(-0.1, 0.1)
                else:
                    rx, ry = region
                    drone.move_towards_region(rx, ry)
                    drone.dwell_timer += 1

                    cx = rx + Config.REGION_SIZE / 2
                    cy = ry + Config.REGION_SIZE / 2
                    if math.dist((drone.x, drone.y), (cx, cy)) < 1.0:
                        drone.dwell_timer += 2  

                    if drone.dwell_timer >= Config.REGION_DWELL:
                        self.explored_regions.add(region)
                        drone.sweep_idx   += 1
                        drone.dwell_timer  = 0
                        logging.debug(f"Drone {drone.id} completed region {region}")

                drone.avoid_collision(self.drones)
                drone.update(self.env.size)

            if self.iteration % Config.SIM_UPDATE_EVERY == 0:
                img      = self.env.render(self.drones,
                                           self.explored_regions,
                                           self.confirmed_targets)
                buf = io.BytesIO()
                img.save(buf, format='PNG')

                active = sum(1 for d in self.drones if d.power_remaining > 0)
                self.to_ui.put({
                    'type':             MSG_STATUS_UPDATE,
                    'iteration':        self.iteration,
                    'image':            buf.getvalue(),
                    'drones':           [{'id': d.id, 'x': d.x, 'y': d.y,
                                          'status': d.display_status(),
                                          'power': d.power_remaining}
                                         for d in self.drones],
                    'explored_regions': len(self.explored_regions),
                    'total_regions':    len(self.all_regions),
                    'active_drones':    active,
                    'elapsed_time':     time.time() - self.start_time
                })

            time.sleep(Config.SIM_DELAY)

    def stop(self):
        self.running = False


# COMMAND CENTER  (Tkinter UI – main thread)
class CommandCenter:
    def __init__(self, simulation):
        self.sim = simulation

        self.root = tk.Tk()
        self.root.title("Drone Swarm – Command Center")
        self.root.configure(bg='#0d1117')
        self.root.geometry("1280x820")

        self.reports           = []
        self.confirmed_targets = []
        self.manual_queue      = []          
        self.current_drone_id  = None
        self.current_image     = None
        self.drone_statuses    = []
        self.explored_count    = 0
        self.total_regions     = 0

        self._build_ui()
        self._poll()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # UI Construction 
    def _build_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Dark.TFrame',       background='#0d1117')
        style.configure('Panel.TLabelframe', background='#161b22',
                         foreground='#58a6ff', font=('Consolas', 9, 'bold'),
                         bordercolor='#30363d')
        style.configure('Panel.TLabelframe.Label', background='#161b22',
                         foreground='#58a6ff')
        style.configure('Dark.TLabel',       background='#161b22',
                         foreground='#c9d1d9', font=('Consolas', 9))
        style.configure('Title.TLabel',      background='#0d1117',
                         foreground='#58a6ff', font=('Consolas', 11, 'bold'))
        style.configure('Action.TButton',    background='#21262d',
                         foreground='#c9d1d9', font=('Consolas', 9, 'bold'),
                         relief='flat', borderwidth=1)
        style.map('Action.TButton',
                  background=[('active', '#30363d')])

        # Root layout 
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        # LEFT: map canvas
        left = tk.Frame(self.root, bg='#161b22', bd=2, relief='flat')
        left.grid(row=0, column=0, sticky='nsew', padx=(10, 5), pady=10)

        tk.Label(left, text='ENVIRONMENT VIEW',
                 bg='#161b22', fg='#58a6ff',
                 font=('Consolas', 10, 'bold')).pack(pady=(8, 4))

        canvas_size = Config.ENV_SIZE * Config.CELL_SIZE
        self.canvas = tk.Canvas(left, width=canvas_size, height=canvas_size,
                                bg='#1a1a2e', highlightthickness=1,
                                highlightbackground='#30363d')
        self.canvas.pack(padx=10, pady=(0, 6))

        self.stats_var = tk.StringVar(value='Initialising…')
        tk.Label(left, textvariable=self.stats_var,
                 bg='#161b22', fg='#8b949e',
                 font=('Consolas', 8)).pack(pady=(0, 8))

        # RIGHT: controls + reports
        right = tk.Frame(self.root, bg='#0d1117')
        right.grid(row=0, column=1, sticky='nsew', padx=(5, 10), pady=10)
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)
        right.rowconfigure(3, weight=1)

        # Manual control panel
        ctrl = tk.LabelFrame(right, text=' MANUAL CONTROL ',
                             bg='#161b22', fg='#58a6ff',
                             font=('Consolas', 9, 'bold'),
                             bd=1, relief='flat')
        ctrl.grid(row=0, column=0, sticky='ew', pady=(0, 6))
        ctrl.columnconfigure(0, weight=1)

        self.ctrl_drone_var = tk.StringVar(value='No drone selected')
        tk.Label(ctrl, textvariable=self.ctrl_drone_var,
                 bg='#161b22', fg='#ffa657',
                 font=('Consolas', 10, 'bold')).pack(pady=(6, 4))

        btn_grid = tk.Frame(ctrl, bg='#161b22')
        btn_grid.pack(pady=4)

        BSTYLE = dict(bg='#21262d', fg='#c9d1d9',
                      activebackground='#30363d', activeforeground='white',
                      font=('Consolas', 10, 'bold'), width=5,
                      relief='flat', bd=1)

        tk.Button(btn_grid, text='▲',
                  command=lambda: self._send_manual('up'), **BSTYLE
                  ).grid(row=0, column=1, padx=3, pady=3)
        tk.Button(btn_grid, text='◀',
                  command=lambda: self._send_manual('left'), **BSTYLE
                  ).grid(row=1, column=0, padx=3, pady=3)
        tk.Button(btn_grid, text='▼',
                  command=lambda: self._send_manual('down'), **BSTYLE
                  ).grid(row=1, column=1, padx=3, pady=3)
        tk.Button(btn_grid, text='▶',
                  command=lambda: self._send_manual('right'), **BSTYLE
                  ).grid(row=1, column=2, padx=3, pady=3)

        # Keyboard bindings
        self.root.bind('<Up>',    lambda e: self._send_manual('up'))
        self.root.bind('<Down>',  lambda e: self._send_manual('down'))
        self.root.bind('<Left>',  lambda e: self._send_manual('left'))
        self.root.bind('<Right>', lambda e: self._send_manual('right'))

        act_frame = tk.Frame(ctrl, bg='#161b22')
        act_frame.pack(pady=(2, 8))

        ACCEPT_STYLE = dict(bg='#1f6feb', fg='white',
                            activebackground='#388bfd', activeforeground='white',
                            font=('Consolas', 9, 'bold'), relief='flat',
                            bd=0, padx=12, pady=4)
        DISCARD_STYLE = dict(bg='#da3633', fg='white',
                             activebackground='#f85149', activeforeground='white',
                             font=('Consolas', 9, 'bold'), relief='flat',
                             bd=0, padx=12, pady=4)

        tk.Button(act_frame, text='✔ ACCEPT',
                  command=self._accept_target, **ACCEPT_STYLE).pack(
                      side=tk.LEFT, padx=8)
        tk.Button(act_frame, text='✘ DISCARD',
                  command=self._discard_target, **DISCARD_STYLE).pack(
                      side=tk.LEFT, padx=8)

        # Investigation queue
        qframe = tk.LabelFrame(right, text=' INVESTIGATION QUEUE ',
                               bg='#161b22', fg='#58a6ff',
                               font=('Consolas', 9, 'bold'),
                               bd=1, relief='flat')
        qframe.grid(row=1, column=0, sticky='ew', pady=(0, 6))

        self.queue_listbox = tk.Listbox(qframe, height=5,
                                        bg='#0d1117', fg='#c9d1d9',
                                        selectbackground='#1f6feb',
                                        selectforeground='white',
                                        font=('Consolas', 9),
                                        bd=0, highlightthickness=0)
        self.queue_listbox.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        self.queue_listbox.bind('<<ListboxSelect>>', self._on_queue_select)

        # Target reports log
        rframe = tk.LabelFrame(right, text=' TARGET REPORTS ',
                               bg='#161b22', fg='#58a6ff',
                               font=('Consolas', 9, 'bold'),
                               bd=1, relief='flat')
        rframe.grid(row=2, column=0, sticky='nsew', pady=(0, 6))

        self.reports_text = scrolledtext.ScrolledText(
            rframe, height=12, width=46,
            bg='#0d1117', fg='#8b949e',
            insertbackground='#c9d1d9',
            font=('Consolas', 8),
            bd=0, relief='flat')
        self.reports_text.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        # Confirmed targets
        cframe = tk.LabelFrame(right, text=' CONFIRMED TARGETS ',
                               bg='#161b22', fg='#3fb950',
                               font=('Consolas', 9, 'bold'),
                               bd=1, relief='flat')
        cframe.grid(row=3, column=0, sticky='ew', pady=(0, 6))

        self.confirmed_var = tk.StringVar(value='No confirmed targets yet')
        tk.Label(cframe, textvariable=self.confirmed_var,
                 bg='#161b22', fg='#3fb950',
                 font=('Consolas', 9)).pack(pady=6)

    def _poll(self):
        try:
            while not self.sim.to_ui.empty():
                msg = self.sim.to_ui.get_nowait()
                if msg['type'] == MSG_TARGET_FOUND:
                    self._on_target_found(msg)
                elif msg['type'] == MSG_STATUS_UPDATE:
                    self._on_status_update(msg)
        except queue.Empty:
            pass
        except Exception as e:
            logging.error(f"UI poll error: {e}")
        self.root.after(Config.UI_POLL_MS, self._poll)

    # Message Handlers 
    def _on_target_found(self, msg):
        self.reports.append(msg)
        did = msg['drone_id']

        if did not in self.manual_queue:
            self.manual_queue.append(did)
            tx, ty = msg['target_position']
            self.queue_listbox.insert(
                tk.END, f"  Drone {did:02d}  →  target ({tx},{ty})")
            self.queue_listbox.itemconfig(
                self.queue_listbox.size() - 1, fg='#ffa657')

        ts = datetime.fromtimestamp(msg['timestamp']).strftime('%H:%M:%S')
        tx, ty = msg['target_position']
        self._log(f"[{ts}] Drone {did:02d} detected target at ({tx},{ty})\n", '#ffa657')

        if self.current_drone_id is None and self.manual_queue:
            self._select_drone(self.manual_queue[0])

    def _on_status_update(self, status):
        try:
            img = Image.open(io.BytesIO(status['image']))
            cs = Config.ENV_SIZE * Config.CELL_SIZE
            img = img.resize((cs, cs), Image.LANCZOS)
            self.current_image = ImageTk.PhotoImage(img)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.current_image)
        except Exception as e:
            logging.error(f"Image render error: {e}")

        exp  = status['explored_regions']
        tot  = status['total_regions']
        act  = status['active_drones']
        elap = status['elapsed_time']
        pct  = exp / tot * 100 if tot else 0

        self.stats_var.set(
            f"Explored: {exp}/{tot}  ({pct:.1f}%)  │  "
            f"Active Drones: {act}/{Config.NUM_DRONES}  │  "
            f"Time: {elap:.0f}s  │  Iter: {status['iteration']}")

    # Controls 
    def _send_manual(self, direction):
        if self.current_drone_id is not None:
            self.sim.to_simulation.put({
                'type':      MSG_MANUAL_CONTROL,
                'drone_id':  self.current_drone_id,
                'direction': direction
            })

    def _accept_target(self):
        if self.current_drone_id is None:
            return
        report = next(
            (r for r in self.reports if r['drone_id'] == self.current_drone_id), None)
        if report:
            self.confirmed_targets.append(report)
            self.sim.to_simulation.put({
                'type':     MSG_ACCEPT_TARGET,
                'drone_id': self.current_drone_id,
                'report':   report
            })
            n = len(self.confirmed_targets)
            self.confirmed_var.set(
                f"{n} target(s) confirmed: " +
                ", ".join(f"({r['target_position'][0]},{r['target_position'][1]})"
                          for r in self.confirmed_targets))
            self._log(f"  ✔ Target ACCEPTED from Drone {self.current_drone_id}\n",
                      '#3fb950')
            self._advance_queue()

    def _discard_target(self):
        if self.current_drone_id is None:
            return
        self.sim.to_simulation.put({
            'type':     MSG_DISCARD_TARGET,
            'drone_id': self.current_drone_id
        })
        self._log(f"  ✘ Target DISCARDED from Drone {self.current_drone_id}\n",
                  '#f85149')
        self._advance_queue()

    def _advance_queue(self):
        if self.current_drone_id in self.manual_queue:
            idx = self.manual_queue.index(self.current_drone_id)
            self.manual_queue.pop(idx)
            self.queue_listbox.delete(idx)
        if self.manual_queue:
            self._select_drone(self.manual_queue[0])
            self.queue_listbox.selection_set(0)
        else:
            self.current_drone_id = None
            self.ctrl_drone_var.set('No drone selected')

    def _select_drone(self, drone_id):
        self.current_drone_id = drone_id
        self.ctrl_drone_var.set(f'Controlling  Drone {drone_id:02d}')

    def _on_queue_select(self, event):
        sel = self.queue_listbox.curselection()
        if sel:
            self._select_drone(self.manual_queue[sel[0]])

    def _log(self, text, color='#8b949e'):
        self.reports_text.insert(tk.END, text)

        end_idx = self.reports_text.index(tk.END)
        line_no = int(end_idx.split('.')[0]) - 2
        tag = f'color_{color.replace("#","")}'
        self.reports_text.tag_configure(tag, foreground=color)
        self.reports_text.tag_add(tag, f'{line_no}.0', f'{line_no}.end')
        self.reports_text.see(tk.END)

    def _on_close(self):
        self.sim.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


# ENTRY POINT

if __name__ == '__main__':
    print("=" * 62)
    print("  DRONE SWARM SYSTEM  –  Boustrophedon Sweep Algorithm")
    print("=" * 62)

    sim = DroneSimulation()

    sim_thread = threading.Thread(target=sim.run, daemon=True)
    sim_thread.start()
    logging.info("Simulation thread started")

    cc = CommandCenter(sim)
    cc.run()

    print("System shut down.")