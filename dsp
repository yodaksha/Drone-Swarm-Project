
# DRONE SWARM EXPLORATION SYSTEM 


import numpy as np
import time
import math
import random
from PIL import Image, ImageDraw
import tkinter as tk
from tkinter import ttk, scrolledtext
from PIL import ImageTk
import io
from datetime import datetime
import threading
import queue

class Environment:
    """2D grid environment with points of interest"""

    def __init__(self, size=50, num_targets=3):
        self.size = size
        self.grid = np.zeros((size, size))
        self.targets = []

        # Place random targets
        for _ in range(num_targets):
            x = random.randint(0, size - 1)
            y = random.randint(0, size - 1)
            self.targets.append((x, y))
            self.grid[y, x] = 1

    def getinfo(self, x, y, r):
        """Check if any target exists within radius r of point (x, y)"""
        for tx, ty in self.targets:
            distance = math.sqrt((x - tx)**2 + (y - ty)**2)
            if distance <= r:
                return True, (tx, ty)
        return False, None

    def render(self, drones, explored_regions):
        """Render the current state as an image"""
        cell_size = 10
        img_size = self.size * cell_size
        img = Image.new('RGB', (img_size, img_size), color='white')
        draw = ImageDraw.Draw(img)

        # Draw explored regions
        for region in explored_regions:
            rx, ry = region
            x1 = rx * cell_size
            y1 = ry * cell_size
            x2 = (rx + 1) * cell_size
            y2 = (ry + 1) * cell_size
            draw.rectangle([x1, y1, x2, y2], fill='lightgray')

        # Draw targets
        for tx, ty in self.targets:
            x = tx * cell_size
            y = ty * cell_size
            draw.ellipse([x-5, y-5, x+5, y+5], fill='red', outline='darkred')

        # Draw drones
        for drone in drones:
            x = int(drone.x * cell_size)
            y = int(drone.y * cell_size)

            if drone.status == 'halted':
                color = 'orange'
            elif drone.status == 'manual':
                color = 'purple'
            else:
                color = 'blue'

            draw.ellipse([x-3, y-3, x+3, y+3], fill=color, outline='black')

        return img



# DRONE CLASS
class Drone:
    """Drone with position, velocity, and status"""

    def __init__(self, drone_id, x, y):
        self.id = drone_id
        self.x = x
        self.y = y
        self.vx = 0
        self.vy = 0
        self.power_remaining = 1000
        self.status = 'exploring'
        self.assigned_region = None
        self.region_explore_time = 0
        self.detection_radius = 2.0
        self.found_target = None

    def move_to_region(self, region_x, region_y, region_size):
        """Move drone towards assigned region center"""
        center_x = region_x + region_size / 2
        center_y = region_y + region_size / 2

        dx = center_x - self.x
        dy = center_y - self.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance > 0.5:
            self.vx = dx / distance * 0.5 + random.uniform(-0.1, 0.1)
            self.vy = dy / distance * 0.5 + random.uniform(-0.1, 0.1)
        else:
            self.vx = random.uniform(-0.3, 0.3)
            self.vy = random.uniform(-0.3, 0.3)

    def update(self, env_size):
        """Update drone position"""
        self.x += self.vx
        self.y += self.vy
        self.power_remaining -= 0.1

        self.x = max(0, min(env_size - 1, self.x))
        self.y = max(0, min(env_size - 1, self.y))



# SIMULATION CLASS 

class DroneSimulation:
    """Simulation that runs in a background thread"""

    def __init__(self):
        self.env = Environment(size=50, num_targets=3)
        self.drones = [Drone(i, random.uniform(0, self.env.size), random.uniform(0, self.env.size)) 
                      for i in range(20)]

        self.region_size = 5
        self.explored_regions = set()
        self.all_regions = [(rx, ry) for rx in range(0, self.env.size, self.region_size) 
                           for ry in range(0, self.env.size, self.region_size)]
        self.explore_time_threshold = 50
        self.iteration = 0
        self.running = True

        # Queues for communication
        self.to_simulation = queue.Queue()
        self.to_ui = queue.Queue()

        print(f"Simulation initialized - Targets at: {self.env.targets}")

    def run(self):
        """Main simulation loop"""
        while self.running:
            self.iteration += 1

            # Check for targets
            for drone in self.drones:
                if drone.status == 'exploring':
                    found, target_pos = self.env.getinfo(drone.x, drone.y, drone.detection_radius)
                    if found and drone.found_target is None:
                        drone.status = 'halted'
                        drone.found_target = target_pos
                        self.to_ui.put({
                            'type': 'target_found',
                            'drone_id': drone.id,
                            'position': (drone.x, drone.y),
                            'target_position': target_pos,
                            'timestamp': time.time()
                        })
                        print(f"Drone {drone.id} found target at {target_pos}")

            # Check for commands from UI
            try:
                while not self.to_simulation.empty():
                    message = self.to_simulation.get_nowait()

                    if message['type'] in ['accept_target', 'discard_target']:
                        drone_id = message['drone_id']
                        self.drones[drone_id].status = 'exploring'
                        self.drones[drone_id].found_target = None

                    elif message['type'] == 'manual_control':
                        drone_id = message['drone_id']
                        direction = message['direction']
                        drone = self.drones[drone_id]
                        drone.status = 'manual'

                        if direction == 'up':
                            drone.y -= 1
                        elif direction == 'down':
                            drone.y += 1
                        elif direction == 'left':
                            drone.x -= 1
                        elif direction == 'right':
                            drone.x += 1

                        drone.x = max(0, min(self.env.size - 1, drone.x))
                        drone.y = max(0, min(self.env.size - 1, drone.y))
            except queue.Empty:
                pass

            # Update drones using algorithm
            for drone in self.drones:
                if drone.status == 'exploring':
                    if drone.assigned_region is None:
                        for region in self.all_regions:
                            if region not in self.explored_regions:
                                drone.assigned_region = region
                                drone.region_explore_time = 0
                                break

                    if drone.assigned_region is not None:
                        rx, ry = drone.assigned_region
                        drone.move_to_region(rx, ry, self.region_size)
                        drone.region_explore_time += 1

                        if drone.region_explore_time >= self.explore_time_threshold:
                            self.explored_regions.add(drone.assigned_region)
                            drone.assigned_region = None

                    drone.update(self.env.size)

            # Send status update every 5 iterations
            if self.iteration % 5 == 0:
                img = self.env.render(self.drones, self.explored_regions)
                img_bytes = io.BytesIO()
                img.save(img_bytes, format='PNG')

                self.to_ui.put({
                    'type': 'status_update',
                    'iteration': self.iteration,
                    'image': img_bytes.getvalue(),
                    'drones': [{'id': d.id, 'x': d.x, 'y': d.y, 'status': d.status, 
                               'power': d.power_remaining} for d in self.drones],
                    'explored_regions': len(self.explored_regions),
                    'total_regions': len(self.all_regions)
                })

            time.sleep(0.1)

    def stop(self):
        """Stop the simulation"""
        self.running = False



# COMMAND CENTER CLASS

class CommandCenter:
    """Command center UI using tkinter"""

    def __init__(self, simulation):
        self.simulation = simulation
        self.root = tk.Tk()
        self.root.title("Drone Swarm Command Center")
        self.root.geometry("1200x800")

        self.reports = []
        self.confirmed_targets = []
        self.manual_queue = []
        self.current_manual_drone = None
        self.current_image = None
        self.drones_status = []
        self.explored_count = 0
        self.total_regions = 0

        self.setup_ui()
        self.update_loop()

        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_ui(self):
        """Setup the UI layout"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)

        # Left panel
        left_frame = ttk.LabelFrame(main_frame, text="Environment View", padding="10")
        left_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)

        self.canvas = tk.Canvas(left_frame, width=500, height=500, bg='white')
        self.canvas.pack()

        stats_frame = ttk.Frame(left_frame)
        stats_frame.pack(fill=tk.X, pady=5)
        self.stats_label = ttk.Label(stats_frame, text="Exploration: 0/0 regions", font=('Arial', 10))
        self.stats_label.pack()

        # Right panel
        right_frame = ttk.Frame(main_frame)
        right_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        right_frame.rowconfigure(1, weight=1)
        right_frame.columnconfigure(0, weight=1)

        control_frame = ttk.LabelFrame(right_frame, text="Manual Control", padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=5)

        self.manual_drone_label = ttk.Label(control_frame, text="No drone selected", 
                                           font=('Arial', 10, 'bold'))
        self.manual_drone_label.pack()

        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(pady=10)
        ttk.Button(btn_frame, text="UP", command=lambda: self.send_manual_command('up')).grid(
            row=0, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="LEFT", command=lambda: self.send_manual_command('left')).grid(
            row=1, column=0, padx=2, pady=2)
        ttk.Button(btn_frame, text="DOWN", command=lambda: self.send_manual_command('down')).grid(
            row=1, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="RIGHT", command=lambda: self.send_manual_command('right')).grid(
            row=1, column=2, padx=2, pady=2)

        decision_frame = ttk.Frame(control_frame)
        decision_frame.pack(pady=5)
        ttk.Button(decision_frame, text="Accept Target", command=self.accept_target).pack(
            side=tk.LEFT, padx=5)
        ttk.Button(decision_frame, text="Discard Target", command=self.discard_target).pack(
            side=tk.LEFT, padx=5)

        queue_frame = ttk.LabelFrame(right_frame, text="Investigation Queue", padding="10")
        queue_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        self.queue_listbox = tk.Listbox(queue_frame, height=6)
        self.queue_listbox.pack(fill=tk.BOTH, expand=True)
        self.queue_listbox.bind('<<ListboxSelect>>', self.on_queue_select)

        reports_frame = ttk.LabelFrame(right_frame, text="Target Reports", padding="10")
        reports_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        self.reports_text = scrolledtext.ScrolledText(reports_frame, height=10, width=50)
        self.reports_text.pack(fill=tk.BOTH, expand=True)

        confirmed_frame = ttk.LabelFrame(right_frame, text="Confirmed Targets", padding="10")
        confirmed_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=5)
        self.confirmed_label = ttk.Label(confirmed_frame, text="No confirmed targets yet", 
                                        font=('Arial', 9))
        self.confirmed_label.pack()

    def update_loop(self):
        """Main update loop - checks for messages from simulation"""
        try:
            while not self.simulation.to_ui.empty():
                message = self.simulation.to_ui.get_nowait()

                if message['type'] == 'target_found':
                    self.handle_target_report(message)
                elif message['type'] == 'status_update':
                    self.handle_status_update(message)
        except queue.Empty:
            pass

        # Schedule next update
        self.root.after(50, self.update_loop)

    def handle_target_report(self, report):
        """Handle a new target report from a drone"""
        self.reports.append(report)
        drone_id = report['drone_id']

        if drone_id not in self.manual_queue:
            self.manual_queue.append(drone_id)
            self.queue_listbox.insert(tk.END, 
                f"Drone {drone_id} - Target at ({report['target_position'][0]:.1f}, {report['target_position'][1]:.1f})")

        timestamp = datetime.fromtimestamp(report['timestamp']).strftime('%H:%M:%S')
        log_msg = f"[{timestamp}] Drone {drone_id} detected target\n"
        self.reports_text.insert(tk.END, log_msg)
        self.reports_text.see(tk.END)

        if self.current_manual_drone is None and len(self.manual_queue) > 0:
            self.current_manual_drone = self.manual_queue[0]
            self.manual_drone_label.config(text=f"Controlling Drone {self.current_manual_drone}")

    def handle_status_update(self, status):
        """Handle status update with environment image"""
        try:
            img_data = status['image']
            img = Image.open(io.BytesIO(img_data))
            img = img.resize((500, 500), Image.LANCZOS)
            self.current_image = ImageTk.PhotoImage(img)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.current_image)
        except Exception as e:
            print(f"Error: {e}")

        self.drones_status = status['drones']
        self.explored_count = status['explored_regions']
        self.total_regions = status['total_regions']
        progress = (self.explored_count / self.total_regions * 100) if self.total_regions > 0 else 0
        self.stats_label.config(
            text=f"Exploration: {self.explored_count}/{self.total_regions} regions ({progress:.1f}%)")

    def send_manual_command(self, direction):
        """Send manual control command to simulation"""
        if self.current_manual_drone is not None:
            self.simulation.to_simulation.put({
                'type': 'manual_control',
                'drone_id': self.current_manual_drone,
                'direction': direction
            })

    def accept_target(self):
        """Accept the current target and move to next in queue"""
        if self.current_manual_drone is not None:
            report = next((r for r in self.reports if r['drone_id'] == self.current_manual_drone), None)

            if report:
                self.confirmed_targets.append(report)
                self.simulation.to_simulation.put({
                    'type': 'accept_target',
                    'drone_id': self.current_manual_drone
                })
                self.confirmed_label.config(text=f"Confirmed: {len(self.confirmed_targets)} targets")
                self.reports_text.insert(tk.END, 
                    f"Target ACCEPTED from Drone {self.current_manual_drone}\n")
                self.reports_text.see(tk.END)
                self.move_to_next_in_queue()

    def discard_target(self):
        """Discard the current target and move to next in queue"""
        if self.current_manual_drone is not None:
            self.simulation.to_simulation.put({
                'type': 'discard_target',
                'drone_id': self.current_manual_drone
            })
            self.reports_text.insert(tk.END, 
                f"Target DISCARDED from Drone {self.current_manual_drone}\n")
            self.reports_text.see(tk.END)
            self.move_to_next_in_queue()

    def move_to_next_in_queue(self):
        """Move to the next drone in manual queue"""
        if self.current_manual_drone in self.manual_queue:
            idx = self.manual_queue.index(self.current_manual_drone)
            self.manual_queue.pop(idx)
            self.queue_listbox.delete(idx)

        if len(self.manual_queue) > 0:
            self.current_manual_drone = self.manual_queue[0]
            self.manual_drone_label.config(text=f"Controlling Drone {self.current_manual_drone}")
            self.queue_listbox.selection_clear(0, tk.END)
            self.queue_listbox.selection_set(0)
        else:
            self.current_manual_drone = None
            self.manual_drone_label.config(text="No drone selected")

    def on_queue_select(self, event):
        """Handle selection from queue listbox"""
        selection = self.queue_listbox.curselection()
        if selection:
            idx = selection[0]
            self.current_manual_drone = self.manual_queue[idx]
            self.manual_drone_label.config(text=f"Controlling Drone {self.current_manual_drone}")

    def on_closing(self):
        """Handle window close event"""
        self.simulation.stop()
        self.root.destroy()

    def run(self):
        """Start the UI main loop"""
        self.root.mainloop()

# MAIN LAUNCHER

print("="*60)
print("DRONE SWARM SYSTEM - Starting...")
print("="*60)

# Create simulation
simulation = DroneSimulation()

# Start simulation in background thread
sim_thread = threading.Thread(target=simulation.run, daemon=True)
sim_thread.start()

# Create and run command center UI
cc = CommandCenter(simulation)
cc.run()

print("System shut down successfully")
