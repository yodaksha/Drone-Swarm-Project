#!/usr/bin/env python
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
import logging
from scipy.spatial import Voronoi, distance

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('drone_simulation.log'),
        logging.StreamHandler()
    ]
)

class Config:
    # Environment settings
    ENV_SIZE = 50
    NUM_TARGETS = 3
    
    # Drone settings
    NUM_DRONES = 20
    INITIAL_POWER = 1000
    POWER_CONSUMPTION_RATE = 0.1
    DETECTION_RADIUS = 2.0
    DRONE_SPEED = 0.5
    MANUAL_SPEED = 1.0
    LOW_POWER_THRESHOLD = 200
    
    # Region exploration settings
    REGION_SIZE = 5
    EXPLORE_TIME_THRESHOLD = 50
    
    # Rendering settings
    CELL_SIZE = 10
    UPDATE_INTERVAL = 5  # iterations
    UI_UPDATE_MS = 50  # milliseconds
    SIMULATION_DELAY = 0.1  # seconds
    
    # Collision avoidance
    MIN_DRONE_DISTANCE = 1.5
    AVOIDANCE_FORCE = 0.3
    
    # Voronoi-based exploration
    VORONOI_UPDATE_INTERVAL = 20  # Recompute Voronoi every N iterations
    USE_VORONOI = True  # Set to False to use greedy algorithm

class Environment:

    def __init__(self, size=None, num_targets=None):
        self.size = size or Config.ENV_SIZE
        num_targets = num_targets or Config.NUM_TARGETS
        
        if self.size < 10:
            raise ValueError("Environment size must be at least 10")
        if num_targets < 1:
            raise ValueError("Must have at least 1 target")
            
        self.grid = np.zeros((self.size, self.size))
        self.targets = []

        # Place random targets ensuring they don't overlap
        attempts = 0
        while len(self.targets) < num_targets and attempts < num_targets * 10:
            x = random.randint(0, self.size - 1)
            y = random.randint(0, self.size - 1)
            if (x, y) not in self.targets:
                self.targets.append((x, y))
                self.grid[y, x] = 1
            attempts += 1
        
        logging.info(f"Environment created: {self.size}x{self.size} with {len(self.targets)} targets")

    def getinfo(self, x, y, r):
        for tx, ty in self.targets:
            distance = math.sqrt((x - tx)**2 + (y - ty)**2)
            if distance <= r:
                return True, (tx, ty)
        return False, None

    def render(self, drones, explored_regions):
        cell_size = Config.CELL_SIZE
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

    def __init__(self, drone_id, x, y):
        self.id = drone_id
        self.x = x
        self.y = y
        self.vx = 0
        self.vy = 0
        self.power_remaining = Config.INITIAL_POWER
        self.status = 'exploring'
        self.assigned_region = None
        self.region_explore_time = 0
        self.detection_radius = Config.DETECTION_RADIUS
        self.found_target = None
        self.low_power_warning_sent = False

    def move_to_region(self, region_x, region_y, region_size):
        center_x = region_x + region_size / 2
        center_y = region_y + region_size / 2

        dx = center_x - self.x
        dy = center_y - self.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance > 0.5:
            self.vx = dx / distance * Config.DRONE_SPEED + random.uniform(-0.1, 0.1)
            self.vy = dy / distance * Config.DRONE_SPEED + random.uniform(-0.1, 0.1)
        else:
            # Random movement within region
            self.vx = random.uniform(-0.3, 0.3)
            self.vy = random.uniform(-0.3, 0.3)
    
    def avoid_collision(self, other_drones):
        avoidance_x = 0
        avoidance_y = 0
        
        for other in other_drones:
            if other.id == self.id:
                continue
            
            dx = self.x - other.x
            dy = self.y - other.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if 0 < distance < Config.MIN_DRONE_DISTANCE:
                # Apply repulsive force
                force = Config.AVOIDANCE_FORCE / (distance + 0.1)
                avoidance_x += (dx / distance) * force
                avoidance_y += (dy / distance) * force
        
        self.vx += avoidance_x
        self.vy += avoidance_y

    def update(self, env_size):
        
        if self.power_remaining > 0:
            self.x += self.vx
            self.y += self.vy
            self.power_remaining -= Config.POWER_CONSUMPTION_RATE

            # Keep within bounds
            self.x = max(0, min(env_size - 1, self.x))
            self.y = max(0, min(env_size - 1, self.y))
            
            # Check for low power
            if self.power_remaining < Config.LOW_POWER_THRESHOLD and not self.low_power_warning_sent:
                self.low_power_warning_sent = True
                logging.warning(f"Drone {self.id} low on power: {self.power_remaining:.1f}")
        else:
            # Drone out of power
            if self.status != 'dead':
                self.status = 'dead'
                logging.error(f"Drone {self.id} ran out of power at ({self.x:.1f}, {self.y:.1f})")



# SIMULATION CLASS 

class DroneSimulation:
    

    def __init__(self):
        self.env = Environment()
        self.drones = [Drone(i, random.uniform(0, self.env.size), random.uniform(0, self.env.size)) 
                      for i in range(Config.NUM_DRONES)]

        self.region_size = Config.REGION_SIZE
        self.explored_regions = set()
        self.all_regions = [(rx, ry) for rx in range(0, self.env.size, self.region_size) 
                           for ry in range(0, self.env.size, self.region_size)]
        self.explore_time_threshold = Config.EXPLORE_TIME_THRESHOLD
        self.iteration = 0
        self.running = True
        self.start_time = time.time()

        # Queues for communication
        self.to_simulation = queue.Queue()
        self.to_ui = queue.Queue()
        
        # Voronoi-based exploration state
        self.voronoi_assignments = {}
        self.last_voronoi_update = -Config.VORONOI_UPDATE_INTERVAL  # Force first update
        self.last_drone_positions = None

        logging.info(f"Simulation initialized - {Config.NUM_DRONES} drones, {len(self.env.targets)} targets")
        logging.info(f"Targets at: {self.env.targets}")
        logging.info(f"Using {'Voronoi' if Config.USE_VORONOI else 'Greedy'} exploration algorithm")
    
    def assign_regions_voronoi(self, unexplored_regions):
       
        # Get active exploring drones
        active_drones = [d for d in self.drones 
                        if d.power_remaining > 0 and d.status == 'exploring']
        
        if not active_drones or not unexplored_regions:
            return {}
        
        # Need at least 4 points for Voronoi in 2D
        if len(active_drones) < 4:
            # Fall back to greedy for small number of drones
            return self.assign_regions_greedy(active_drones, unexplored_regions)
        
        try:
            # Extract drone positions
            drone_positions = np.array([[d.x, d.y] for d in active_drones])
            
            # Create Voronoi diagram
            vor = Voronoi(drone_positions)
            
            # Assign each region to nearest drone (Voronoi cell)
            assignments = {d.id: [] for d in active_drones}
            
            for region in unexplored_regions:
                # Calculate region center
                region_center = np.array([region[0] + self.region_size / 2, 
                                         region[1] + self.region_size / 2])
                
                # Find which drone's Voronoi cell this region belongs to
                distances = distance.cdist([region_center], drone_positions)[0]
                closest_drone_idx = np.argmin(distances)
                
                drone = active_drones[closest_drone_idx]
                assignments[drone.id].append(region)
            
            # Log assignment statistics
            non_empty = sum(1 for regions in assignments.values() if regions)
            total_assigned = sum(len(regions) for regions in assignments.values())
            logging.debug(f"Voronoi assignment: {non_empty} drones with regions, {total_assigned} total regions")
            
            return assignments
            
        except Exception as e:
            logging.warning(f"Voronoi assignment failed: {e}, falling back to greedy")
            return self.assign_regions_greedy(active_drones, unexplored_regions)
    
    def assign_regions_greedy(self, active_drones, unexplored_regions):
        
        assignments = {d.id: [] for d in active_drones}
        
        # Simple partitioning: each drone gets nearby regions
        for drone in active_drones:
            # Find regions within reasonable distance
            nearby_regions = sorted(unexplored_regions, 
                key=lambda r: math.sqrt((drone.x - r[0])**2 + (drone.y - r[1])**2))[:5]
            assignments[drone.id] = nearby_regions
        
        return assignments
    
    def should_update_voronoi(self):
        
        # Check iteration interval
        if self.iteration - self.last_voronoi_update < Config.VORONOI_UPDATE_INTERVAL:
            return False
        
        # Check if drone positions changed significantly
        if self.last_drone_positions is not None:
            current_positions = [(d.x, d.y) for d in self.drones 
                               if d.power_remaining > 0 and d.status == 'exploring']
            
            if len(current_positions) == len(self.last_drone_positions):
                max_movement = 0
                for curr, last in zip(current_positions, self.last_drone_positions):
                    movement = math.sqrt((curr[0] - last[0])**2 + (curr[1] - last[1])**2)
                    max_movement = max(max_movement, movement)
                
                # Only update if significant movement occurred
                if max_movement < 3.0:
                    return False
        
        return True

    def run(self):
        while self.running:
            self.iteration += 1

            # Check for targets
            for drone in self.drones:
                if drone.status == 'exploring' and drone.power_remaining > 0:
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
                        logging.info(f"Drone {drone.id} found target at {target_pos}, power: {drone.power_remaining:.1f}")

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
                        
                        if drone.power_remaining <= 0:
                            logging.warning(f"Cannot control Drone {drone_id} - out of power")
                            continue
                            
                        drone.status = 'manual'
                        speed = Config.MANUAL_SPEED

                        if direction == 'up':
                            drone.y -= speed
                        elif direction == 'down':
                            drone.y += speed
                        elif direction == 'left':
                            drone.x -= speed
                        elif direction == 'right':
                            drone.x += speed

                        drone.x = max(0, min(self.env.size - 1, drone.x))
                        drone.y = max(0, min(self.env.size - 1, drone.y))
            except queue.Empty:
                pass

            # Update Voronoi assignments periodically
            unexplored = [r for r in self.all_regions if r not in self.explored_regions]
            
            if Config.USE_VORONOI and self.should_update_voronoi() and unexplored:
                self.voronoi_assignments = self.assign_regions_voronoi(unexplored)
                self.last_voronoi_update = self.iteration
                self.last_drone_positions = [(d.x, d.y) for d in self.drones 
                                            if d.power_remaining > 0 and d.status == 'exploring']
                logging.info(f"Voronoi assignments updated at iteration {self.iteration}")

            # Update drones using algorithm
            for drone in self.drones:
                if drone.status == 'exploring' and drone.power_remaining > 0:
                    # Assign region if needed
                    if drone.assigned_region is None:
                        if Config.USE_VORONOI:
                            # Voronoi-based assignment
                            my_regions = self.voronoi_assignments.get(drone.id, [])
                            if my_regions:
                                # Pick closest from assigned regions
                                closest_region = min(my_regions,
                                    key=lambda r: math.sqrt((drone.x - r[0])**2 + (drone.y - r[1])**2))
                                drone.assigned_region = closest_region
                                drone.region_explore_time = 0
                        else:
                            # Greedy assignment (original algorithm)
                            if unexplored:
                                closest_region = min(unexplored, 
                                    key=lambda r: math.sqrt((drone.x - r[0])**2 + (drone.y - r[1])**2))
                                drone.assigned_region = closest_region
                                drone.region_explore_time = 0

                    if drone.assigned_region is not None:
                        rx, ry = drone.assigned_region
                        drone.move_to_region(rx, ry, self.region_size)
                        drone.region_explore_time += 1

                        if drone.region_explore_time >= self.explore_time_threshold:
                            self.explored_regions.add(drone.assigned_region)
                            drone.assigned_region = None
                            logging.debug(f"Drone {drone.id} completed region {drone.assigned_region}")

                    # Apply collision avoidance
                    drone.avoid_collision(self.drones)
                    drone.update(self.env.size)

            # Send status update periodically
            if self.iteration % Config.UPDATE_INTERVAL == 0:
                img = self.env.render(self.drones, self.explored_regions)
                img_bytes = io.BytesIO()
                img.save(img_bytes, format='PNG')

                active_drones = sum(1 for d in self.drones if d.power_remaining > 0)
                
                self.to_ui.put({
                    'type': 'status_update',
                    'iteration': self.iteration,
                    'image': img_bytes.getvalue(),
                    'drones': [{'id': d.id, 'x': d.x, 'y': d.y, 'status': d.status, 
                               'power': d.power_remaining} for d in self.drones],
                    'explored_regions': len(self.explored_regions),
                    'total_regions': len(self.all_regions),
                    'active_drones': active_drones,
                    'elapsed_time': time.time() - self.start_time
                })

            time.sleep(Config.SIMULATION_DELAY)

    def stop(self):
        
        self.running = False



# COMMAND CENTER CLASS

class CommandCenter:
   

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
        try:
            while not self.simulation.to_ui.empty():
                message = self.simulation.to_ui.get_nowait()

                if message['type'] == 'target_found':
                    self.handle_target_report(message)
                elif message['type'] == 'status_update':
                    self.handle_status_update(message)
        except queue.Empty:
            pass
        except Exception as e:
            logging.error(f"Error in UI update loop: {e}")

        # Schedule next update
        self.root.after(Config.UI_UPDATE_MS, self.update_loop)

    def handle_target_report(self, report):
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
        try:
            img_data = status['image']
            img = Image.open(io.BytesIO(img_data))
            img = img.resize((500, 500), Image.LANCZOS)
            self.current_image = ImageTk.PhotoImage(img)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.current_image)
        except Exception as e:
            logging.error(f"Error updating image: {e}")

        self.drones_status = status['drones']
        self.explored_count = status['explored_regions']
        self.total_regions = status['total_regions']
        active_drones = status.get('active_drones', 0)
        elapsed = status.get('elapsed_time', 0)
        
        progress = (self.explored_count / self.total_regions * 100) if self.total_regions > 0 else 0
        self.stats_label.config(
            text=f"Exploration: {self.explored_count}/{self.total_regions} regions ({progress:.1f}%) | "
                 f"Active Drones: {active_drones}/{Config.NUM_DRONES} | Time: {elapsed:.1f}s")

    def send_manual_command(self, direction):
        if self.current_manual_drone is not None:
            self.simulation.to_simulation.put({
                'type': 'manual_control',
                'drone_id': self.current_manual_drone,
                'direction': direction
            })

    def accept_target(self):
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
        selection = self.queue_listbox.curselection()
        if selection:
            idx = selection[0]
            self.current_manual_drone = self.manual_queue[idx]
            self.manual_drone_label.config(text=f"Controlling Drone {self.current_manual_drone}")

    def on_closing(self):
        self.simulation.stop()
        self.root.destroy()

    def run(self):
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