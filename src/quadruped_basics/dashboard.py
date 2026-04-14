#!/usr/bin/env python3
import os
import glob
import subprocess
import threading
import queue
import json
import signal
import tkinter as tk
from tkinter import ttk, simpledialog, messagebox
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Twist
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Configuration files
POSES_FILE = "quadruped_poses.json"
OFFSETS_FILE = "quadruped_offsets.json"

# --- 1. ROS 2 NODE ---
class DashboardNode(Node):
    def __init__(self):
        super().__init__('quadruped_dashboard')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/joint_group_position_controller/commands', 10)
        self.override_pub = self.create_publisher(Bool, '/dashboard_override', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.history_length = 50
        self.time_data = list(range(self.history_length))
        self.leg_data = [[0.0]*self.history_length for _ in range(8)]

    def send_angles(self, angles):
        msg = Float64MultiArray()
        msg.data = [float(a) for a in angles]
        self.publisher_.publish(msg)
        for i in range(8):
            self.leg_data[i].pop(0)
            self.leg_data[i].append(angles[i])

    def set_override(self, state):
        msg = Bool()
        msg.data = state
        self.override_pub.publish(msg)

    def send_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)


# --- 2. GRAPHICAL USER INTERFACE ---
class QuadrupedDashboard:
    def __init__(self, root, ros_node):
        self.root = root
        self.root.title("Quadruped Command Center Pro")
        self.root.geometry("1200x750")
        self.ros_node = ros_node
        
        self.running_processes = {} 
        self.terminal_windows = {}
        
        self.saved_poses = self.load_json(POSES_FILE, {})
        self.offsets = self.load_json(OFFSETS_FILE, [0.0] * 8)
        self.servo_angles = [tk.DoubleVar(value=0.0) for _ in range(8)]
        
        # Variables for Teleop speeds
        self.teleop_speed = tk.DoubleVar(value=0.5)
        self.teleop_turn = tk.DoubleVar(value=1.0)

        self.setup_ui()
        self.update_graph()
        self.refresh_files()

    def load_json(self, filename, default_val):
        if os.path.exists(filename):
            try:
                with open(filename, 'r') as f:
                    return json.load(f)
            except: pass
        return default_val

    def save_json(self, filename, data):
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)

    def setup_ui(self):
        # --- LEFT PANEL: DIRECT CONTROL & CALIBRATION ---
        left_frame = ttk.LabelFrame(self.root, text="Direct Control (Angles)")
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        servo_names = ["FL Shoulder", "FL Knee", "BR Shoulder", "BR Knee", 
                       "FR Shoulder", "FR Knee", "BL Shoulder", "BL Knee"]
        
        for i in range(8):
            row = ttk.Frame(left_frame)
            row.pack(fill=tk.X, pady=5, padx=5)
            ttk.Label(row, text=servo_names[i], width=12).pack(side=tk.LEFT)
            slider = ttk.Scale(row, from_=-1.57, to=1.57, variable=self.servo_angles[i], command=self.on_slider_change)
            slider.pack(side=tk.LEFT, expand=True, fill=tk.X)
            ttk.Label(row, textvariable=self.servo_angles[i], width=5).pack(side=tk.LEFT)

        self.manual_mode = tk.BooleanVar(value=False)
        self.chk_override = ttk.Checkbutton(
            left_frame, text="Force Manual Mode\n(Disables auto-walk)", 
            variable=self.manual_mode, command=self.toggle_override
        )
        self.chk_override.pack(pady=10)

        ttk.Button(left_frame, text="Reset to Zero", command=self.reset_servos).pack(pady=5)
        ttk.Button(left_frame, text="Set as new Zero", command=self.set_zero).pack(pady=5)
        ttk.Button(left_frame, text="Clear calibration", command=self.clear_zero).pack(pady=5)

        # --- BOTTOM LEFT PANEL: POSES ---
        pose_frame = ttk.LabelFrame(left_frame, text="Saved Poses")
        pose_frame.pack(fill=tk.BOTH, expand=True, pady=10, padx=5)

        self.pose_listbox = tk.Listbox(pose_frame, height=5)
        self.pose_listbox.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        for pose in self.saved_poses.keys():
            self.pose_listbox.insert(tk.END, pose)

        btn_row1 = ttk.Frame(pose_frame)
        btn_row1.pack(fill=tk.X)
        ttk.Button(btn_row1, text="Save", command=self.save_pose).pack(side=tk.LEFT, expand=True, padx=2)
        ttk.Button(btn_row1, text="Apply", command=self.apply_pose).pack(side=tk.LEFT, expand=True, padx=2)
        ttk.Button(pose_frame, text="Delete", command=self.delete_pose).pack(fill=tk.X, padx=2, pady=2)

        # --- CENTER PANEL: LAUNCH MANAGER & TELEOP ---
        mid_frame = ttk.Frame(self.root)
        mid_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        launch_frame = ttk.LabelFrame(mid_frame, text="Launch Manager")
        launch_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 5))

        self.file_listbox = tk.Listbox(launch_frame, height=7)
        self.file_listbox.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        btn_frame = ttk.Frame(launch_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="Refresh", command=self.refresh_files).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Launch", command=self.launch_file).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Kill", command=self.stop_file).pack(side=tk.LEFT, padx=5)

        ttk.Label(launch_frame, text="Running processes:").pack(pady=(5,0))
        self.running_listbox = tk.Listbox(launch_frame, height=3, fg="green")
        self.running_listbox.pack(fill=tk.X, padx=5, pady=5)

        # --- INTEGRATED TELEOP ZONE ---
        teleop_frame = ttk.LabelFrame(mid_frame, text="Teleop (Keyboard Control)")
        teleop_frame.pack(fill=tk.X, pady=(5, 0))

        # Speed adjustment bar
        speed_frame = ttk.Frame(teleop_frame)
        speed_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(speed_frame, text="Speed (m/s):").pack(side=tk.LEFT)
        ttk.Spinbox(speed_frame, from_=0.1, to=2.0, increment=0.1, textvariable=self.teleop_speed, width=5).pack(side=tk.LEFT, padx=5)
        
        ttk.Label(speed_frame, text="Rotation (rad/s):").pack(side=tk.LEFT, padx=(10, 0))
        ttk.Spinbox(speed_frame, from_=0.1, to=3.0, increment=0.1, textvariable=self.teleop_turn, width=5).pack(side=tk.LEFT, padx=5)

        self.teleop_lbl = tk.Label(
            teleop_frame, 
            text="DEACTIVATED\nClick here to take control\n\nArrows or I,J,K,L\nSpace = Stop", 
            bg="#2d2d2d", fg="gray", font=("Arial", 11, "bold"), cursor="hand2", height=5
        )
        self.teleop_lbl.pack(fill=tk.BOTH, expand=True, pady=5, padx=5)
        
        self.teleop_status_var = tk.StringVar(value="Current Speed: 0.0 | Turn: 0.0")
        tk.Label(teleop_frame, textvariable=self.teleop_status_var, fg="blue", font=("Arial", 10, "bold")).pack(pady=2)

        # Bindings to capture the keyboard
        self.teleop_lbl.bind("<Button-1>", self.activate_teleop)
        self.teleop_lbl.bind("<KeyPress>", self.key_pressed)
        self.teleop_lbl.bind("<FocusOut>", self.deactivate_teleop)

        # --- RIGHT PANEL: LIVE TELEMETRY ---
        right_frame = ttk.LabelFrame(self.root, text="Live Telemetry")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.fig = Figure(figsize=(4, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # --- LOGIC: INTEGRATED TELEOP ---
    def activate_teleop(self, event):
        self.teleop_lbl.focus_set() 
        self.teleop_lbl.config(
            bg="darkgreen", fg="white", 
            text="ACTIVE\nUse keyboard to drive.\n(Click elsewhere to deactivate)"
        )
        
        # Activate manual mode automatically
        if not self.manual_mode.get():
            self.manual_mode.set(True)
            self.toggle_override()

    def deactivate_teleop(self, event):
        self.teleop_lbl.config(
            bg="#2d2d2d", fg="gray", 
            text="DEACTIVATED\nClick here to take control\n\nArrows or I,J,K,L\nSpace = Stop"
        )
        self.ros_node.send_cmd_vel(0.0, 0.0)
        self.teleop_status_var.set("Current Speed: 0.0 | Turn: 0.0")

    def key_pressed(self, event):
        key = event.keysym.lower()
        
        # Retrieve user-defined values
        speed = self.teleop_speed.get()
        turn = self.teleop_turn.get()
        linear, angular = 0.0, 0.0

        if key in ['up', 'i']:
            linear = speed
        elif key in ['down', 'k']:
            linear = -speed
        elif key in ['left', 'j']:
            angular = turn
        elif key in ['right', 'l']:
            angular = -turn
        elif key == 'space':
            linear, angular = 0.0, 0.0
        else:
            return

        self.teleop_status_var.set(f"Current Speed: {linear} | Turn: {angular}")
        self.ros_node.send_cmd_vel(linear, angular)

    # --- LOGIC: CONTROL AND CALIBRATION ---
    def toggle_override(self):
        self.ros_node.set_override(self.manual_mode.get())

    def on_slider_change(self, event=None):
        if not self.manual_mode.get():
            self.manual_mode.set(True)
            self.toggle_override()
        angles = [round(var.get() + offset, 4) for var, offset in zip(self.servo_angles, self.offsets)]
        self.ros_node.send_angles(angles)

    def set_zero(self):
        for i in range(8):
            self.offsets[i] += self.servo_angles[i].get()
            self.servo_angles[i].set(0.0)
        self.save_json(OFFSETS_FILE, self.offsets)
        self.on_slider_change()

    def clear_zero(self):
        self.offsets = [0.0] * 8
        self.save_json(OFFSETS_FILE, self.offsets)
        self.on_slider_change()

    def reset_servos(self):
        for var in self.servo_angles:
            var.set(0.0)
        self.on_slider_change()

    # --- LOGIC: POSES ---
    def save_pose(self):
        name = simpledialog.askstring("New Pose", "Name:", parent=self.root)
        if name:
            self.saved_poses[name] = [var.get() for var in self.servo_angles]
            self.save_json(POSES_FILE, self.saved_poses)
            self.pose_listbox.insert(tk.END, name)

    def apply_pose(self):
        selection = self.pose_listbox.get(tk.ACTIVE)
        if selection in self.saved_poses:
            if not self.manual_mode.get():
                self.manual_mode.set(True)
                self.toggle_override()
            for i, val in enumerate(self.saved_poses[selection]):
                self.servo_angles[i].set(val)
            self.on_slider_change()

    def delete_pose(self):
        selection = self.pose_listbox.get(tk.ACTIVE)
        if selection and selection in self.saved_poses:
            del self.saved_poses[selection]
            self.save_json(POSES_FILE, self.saved_poses)
            self.pose_listbox.delete(tk.ACTIVE)

    # --- LOGIC: PROCESS MANAGEMENT ---
    def refresh_files(self):
        self.file_listbox.delete(0, tk.END)
        package_path = os.path.expanduser("~/ros2_ws/src/quadruped_basics")
        for f in glob.glob(os.path.join(package_path, "launch", "*.launch.py")):
            self.file_listbox.insert(tk.END, f"LAUNCH: {os.path.basename(f)}")
        for f in glob.glob(os.path.join(package_path, "scripts", "*.py")):
            if "dashboard" not in f:
                self.file_listbox.insert(tk.END, f"RUN: {os.path.basename(f)}")

    def update_running_list(self):
        self.running_listbox.delete(0, tk.END)
        for name in self.running_processes.keys():
            self.running_listbox.insert(tk.END, name)

    def launch_file(self):
        selection = self.file_listbox.get(tk.ACTIVE)
        if not selection: return
        cmd_type, filename = selection.split(": ")
        
        cmd = ["ros2", "launch" if cmd_type == "LAUNCH" else "run", "quadruped_basics", filename]
        process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, 
            text=True, bufsize=1, preexec_fn=os.setsid
        )
        self.running_processes[filename] = process
        self.update_running_list()
        self.create_terminal_window(filename, process)

    def stop_file(self):
        selection = self.running_listbox.get(tk.ACTIVE)
        if not selection and self.running_processes: return
        filename = selection or list(self.running_processes.keys())[0]
        self.kill_process_and_window(filename)

    def kill_process_and_window(self, filename):
        if filename in self.running_processes:
            process = self.running_processes[filename]
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except ProcessLookupError: pass
            del self.running_processes[filename]
            self.update_running_list()

        if filename in self.terminal_windows:
            if self.terminal_windows[filename].winfo_exists():
                self.terminal_windows[filename].destroy()
            del self.terminal_windows[filename]

    def create_terminal_window(self, name, process):
        top = tk.Toplevel(self.root)
        top.title(f"Terminal: {name}")
        top.geometry("600x300")
        top.protocol("WM_DELETE_WINDOW", lambda: self.kill_process_and_window(name))
        self.terminal_windows[name] = top
        
        text_area = tk.Text(top, bg="black", fg="lime green", font=("Consolas", 10))
        text_area.pack(fill=tk.BOTH, expand=True)

        output_queue = queue.Queue()
        def read_output():
            for line in iter(process.stdout.readline, ''): output_queue.put(line)
            process.stdout.close()
        threading.Thread(target=read_output, daemon=True).start()

        def update_text():
            try:
                while True: 
                    text_area.insert(tk.END, output_queue.get_nowait())
                    text_area.see(tk.END)
            except queue.Empty: pass
            
            if top.winfo_exists() and process.poll() is None:
                top.after(100, update_text)
                
        top.after(100, update_text)

    # --- LOGIC: GRAPHS ---
    def update_graph(self):
        self.ax.clear()
        self.ax.set_ylim(-2.0, 2.0)
        self.ax.plot(self.ros_node.time_data, self.ros_node.leg_data[1], label="FL Knee")
        self.ax.plot(self.ros_node.time_data, self.ros_node.leg_data[3], label="BR Knee")
        self.ax.legend(loc="upper right")
        self.canvas.draw()
        self.root.after(100, self.update_graph)

# --- 3. STARTUP ---
def main():
    rclpy.init()
    ros_node = DashboardNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    app = QuadrupedDashboard(root, ros_node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        for process in app.running_processes.values():
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except ProcessLookupError: pass
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
