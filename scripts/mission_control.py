#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.image as mpimg
import numpy as np
import random
import math

# ==========================================
# YAW TO QUATERNION CONVERTER
# ==========================================
def get_quaternion_from_euler(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw

# ==========================================
# GENETIC ALGORITHM (GA) - OPEN TSP
# ==========================================
class GeneticAlgorithmTSP:
    def __init__(self, distance_matrix, pop_size=400, generations=2000, mutation_rate=0.15):
        self.distance_matrix = distance_matrix
        self.num_points = len(distance_matrix)
        self.pop_size = pop_size
        self.generations = generations
        self.mutation_rate = mutation_rate

    def create_individual(self):
        route = list(range(1, self.num_points))
        random.shuffle(route)
        return [0] + route

    def calculate_fitness(self, route):
        total_dist = 0
        for i in range(len(route) - 1):
            total_dist += self.distance_matrix[route[i]][route[i+1]]
        return 1.0 / float(total_dist) if total_dist > 0 else 0

    def crossover(self, parent1, parent2):
        child = [0] + [-1] * (self.num_points - 1)
        start, end = sorted(random.sample(range(1, self.num_points), 2))
        child[start:end] = parent1[start:end]
        p2_idx = 1
        for i in range(1, self.num_points):
            if child[i] == -1:
                while parent2[p2_idx] in child:
                    p2_idx += 1
                child[i] = parent2[p2_idx]
        return child

    def mutate(self, route):
        if random.random() < self.mutation_rate:
            idx1, idx2 = random.sample(range(1, self.num_points), 2)
            route[idx1], route[idx2] = route[idx2], route[idx1]
        return route

    def run(self):
        population = [self.create_individual() for _ in range(self.pop_size)]
        best_route = population[0]
        best_fitness = 0
        for _ in range(self.generations):
            fitnesses = [self.calculate_fitness(ind) for ind in population]
            max_fit_idx = np.argmax(fitnesses)
            if fitnesses[max_fit_idx] > best_fitness:
                best_fitness = fitnesses[max_fit_idx]
                best_route = population[max_fit_idx]
            new_population = [best_route.copy()] 
            for _ in range(self.pop_size - 1):
                p1 = population[random.choices(range(self.pop_size), weights=fitnesses)[0]]
                p2 = population[random.choices(range(self.pop_size), weights=fitnesses)[0]]
                child = self.crossover(p1, p2)
                child = self.mutate(child)
                new_population.append(child)
            population = new_population
        return best_route

# ==========================================
# AMR FLEET GUI
# ==========================================
class FleetGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("AMR Fleet Management System")
        self.root.geometry("1400x900")
        self.root.configure(bg="white")
        
        rclpy.init()
        self.navigator = BasicNavigator()

        # AMCL Pose Subscriber (Always updating current position)
        self.current_robot_pose = None
        self.pose_sub = self.navigator.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            10
        )
# "P4":  [3.042, -9.42, -1.503]
        self.all_waypoints = {
            "P1 (Spawn)": [-0.042, -0.046, 0.045], "P2":  [-4.3, -4.85, 3.116],
            "P3":  [0.154, -7.562, 3.067], "P4":  [4.3, -10.8, -1.503],
            "P5":  [9.5, -10.4, -1.555], "P6":  [14.088, -9.902, 3.052],
            "P7":  [17.555, -10.146, 3.077], "P8":  [28.005, -10.899, 3.055],
            "P9":  [31.289, -11.202, 3.087], "P10": [34.960, -11.425, 3.099],
            "P11": [42.856, -11.224, 0.055], "P12": [43.219, 3.704, 1.463],
            "P13": [37.529, 5.806, -3.087], "P14": [33.794, 6.156, 3.052],
            "P15": [30.160, 6.789, 3.110], "P16": [23.403, 6.974, -3.101],
            "P17": [10.753, 9.0, 1.546], "P18": [4.338, 9.617, 1.768],
            "P19": [1.241, 7.094, 2.930], "P20": [-2.877, 5.049, -3.005],
            "P21": [17.804, -0.218, -0.055], "P22": [17.806, -2.531, 0.053],
            "P23": [25.462, -4.600, -0.116], "P24": [25.654, 0.919, 0.148],
            "P25": [33.232, -0.358, -0.131], "P26": [31.764, -5.195, -0.147]
        }
        
        self.mission_list = []  
        self.optimal_poses = [] 
        self.map_img = None
        
        self.is_mission_active = False
        self.current_exec_index = 0
        self.retry_count = 0
        self.max_retries = 15  

        self.setup_ui()
        # HARDCODED DEFAULT MAP PATH
        self.default_map_path = "/home/pham-van-de/ros2_ws/src/robot_omni/maps/hospital_map.pgm"
        self.load_default_map()
        self.spin_ros()

    def _amcl_pose_callback(self, msg):
        # Liên tục cập nhật tọa độ mới nhất của xe
        self.current_robot_pose = msg.pose.pose

    def spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.navigator, timeout_sec=0.01)
            self.root.after(100, self.spin_ros)

    def setup_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        
        control_frame = tk.Frame(self.root, width=350, bg="white", highlightbackground="black", highlightthickness=1)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        control_frame.pack_propagate(False) 
        
        # --- WAYPOINT SELECTION (SCROLLABLE TABLE) ---
        tk.Label(control_frame, text="AVAILABLE WAYPOINTS", bg="white", font=("Arial", 11, "bold")).pack(pady=(20, 5))
        
        tree_frame = tk.Frame(control_frame, bg="white")
        tree_frame.pack(fill=tk.X, padx=15)
        
        # 2-Column Treeview
        self.tree = ttk.Treeview(tree_frame, columns=('id', 'coords'), show='headings', height=10)
        self.tree.heading('id', text='Point ID')
        self.tree.heading('coords', text='Coordinates (X, Y)')
        self.tree.column('id', width=90, anchor=tk.CENTER)
        self.tree.column('coords', width=160, anchor=tk.CENTER)
        
        # Scrollbar cho bảng
        scrollbar = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscroll=scrollbar.set)
        
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Nạp dữ liệu vào bảng
        for name, coords in self.all_waypoints.items():
            coord_str = f"({coords[0]:.1f}, {coords[1]:.1f})"
            self.tree.insert('', tk.END, values=(name, coord_str))
            
        tk.Button(control_frame, text="Add Selected [+]", command=self.add_waypoint, bg="#e0e0e0", relief=tk.SOLID, bd=1, font=("Arial", 10)).pack(pady=10, fill=tk.X, padx=15)
        
        tk.Label(control_frame, text="-"*40, bg="white", fg="gray").pack(pady=5)
        
        # --- MISSION QUEUE ---
        tk.Label(control_frame, text="MISSION QUEUE", bg="white", font=("Arial", 11, "bold")).pack(pady=5)
        self.listbox_mission = tk.Listbox(control_frame, font=("Arial", 11), height=10, relief=tk.SOLID, bd=1)
        self.listbox_mission.pack(pady=5, padx=15, fill=tk.BOTH, expand=True)
        
        # Nút Remove và Clear All
        btn_frame = tk.Frame(control_frame, bg="white")
        btn_frame.pack(fill=tk.X, padx=15, pady=5)
        tk.Button(btn_frame, text="Remove [-]", command=self.remove_waypoint, bg="#ffeaa7", relief=tk.SOLID, bd=1).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        tk.Button(btn_frame, text="Clear All [x]", command=self.clear_all_waypoints, bg="#ff7675", relief=tk.SOLID, bd=1).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(5, 0))
        
        tk.Label(control_frame, text="-"*40, bg="white", fg="gray").pack(pady=10)
        
        # --- EXECUTION BUTTONS ---
        self.btn_ga = tk.Button(control_frame, text="⚙️ OPTIMIZE ROUTE (GA)", command=self.run_ga_optimization, bg="#55efc4", relief=tk.SOLID, bd=2, font=("Arial", 11, "bold"))
        self.btn_ga.pack(pady=5, fill=tk.X, padx=15)
        
        self.btn_run = tk.Button(control_frame, text="🚀 EXECUTE MISSION", command=self.start_robot, bg="#74b9ff", relief=tk.SOLID, bd=2, font=("Arial", 11, "bold"), state=tk.DISABLED)
        self.btn_run.pack(pady=5, fill=tk.X, padx=15)

        self.btn_stop = tk.Button(control_frame, text="🛑 EMERGENCY STOP", command=self.emergency_stop, bg="#ff4d4d", relief=tk.SOLID, bd=2, font=("Arial", 11, "bold"), state=tk.DISABLED)
        self.btn_stop.pack(pady=(5, 20), fill=tk.X, padx=15)

        # --- MAP DISPLAY ---
        self.map_frame = tk.Frame(self.root, bg="white", highlightbackground="black", highlightthickness=1)
        self.map_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(0, 10), pady=10)
        self.fig, self.ax = plt.subplots(figsize=(10, 10)) 
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.map_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def load_default_map(self):
        try:
            self.map_img = mpimg.imread(self.default_map_path)
        except Exception as e: 
            print(f"Lỗi load map: {e}")
        self.draw_map_and_points()

    def draw_map_and_points(self):
        self.ax.clear()
        if self.map_img is not None:
            res, ox, oy = 0.05, -17.573, -32.236
            h, w = self.map_img.shape
            self.ax.imshow(self.map_img, cmap='gray', origin='upper', extent=[ox, ox + w*res, oy, oy + h*res])
        
        for name, coords in self.all_waypoints.items():
            self.ax.plot(coords[0], coords[1], 'ko', markersize=4, alpha=0.5)
            self.ax.text(coords[0]+0.3, coords[1]+0.3, name.split()[0], fontsize=8)
            
        self.ax.set_xlim([-10, 60]); self.ax.set_ylim([-20, 20])
        self.canvas.draw()

    def add_waypoint(self):
        if self.is_mission_active: return
        selected = self.tree.selection()
        if not selected: return
        for item in selected:
            pt = self.tree.item(item, 'values')[0]
            if pt not in self.mission_list:
                self.mission_list.append(pt)
                self.listbox_mission.insert(tk.END, pt)

    def remove_waypoint(self):
        if self.is_mission_active: return
        sel = self.listbox_mission.curselection()
        if sel:
            self.mission_list.pop(sel[0])
            self.listbox_mission.delete(sel[0])

    def clear_all_waypoints(self):
        if self.is_mission_active: return
        self.mission_list.clear()
        self.listbox_mission.delete(0, tk.END)
        self.optimal_poses.clear()
        self.btn_run.config(state=tk.DISABLED)
        self.draw_map_and_points()

    def get_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = get_quaternion_from_euler(yaw)
        return pose

    def run_ga_optimization(self):
        if len(self.mission_list) < 1:
            messagebox.showwarning("Warning", "Please add at least 1 point to the mission!")
            return
            
        self.btn_ga.config(text="⏳ CALCULATING A*...", state=tk.DISABLED)
        self.root.config(cursor="watch"); self.root.update()

        try:
            # 1. LẤY VỊ TRÍ XE THỰC TẾ (Làm điểm Start cho mọi chu kỳ mới)
            current_pose = PoseStamped()
            current_pose.header.frame_id = 'map'
            current_pose.header.stamp = self.navigator.get_clock().now().to_msg()

            if self.current_robot_pose is not None:
                # AMCL đã cung cấp vị trí
                current_pose.pose = self.current_robot_pose
                print(f"📍 Start Point (Current AMCL): X={current_pose.pose.position.x:.2f}, Y={current_pose.pose.position.y:.2f}")
            else:
                # Fallback an toàn nếu hệ thống khởi động chậm
                coords = self.all_waypoints["P1 (Spawn)"]
                current_pose.pose.position.x, current_pose.pose.position.y = coords[0], coords[1]
                current_pose.pose.orientation.z, current_pose.pose.orientation.w = math.sin(coords[2]/2), math.cos(coords[2]/2)
                print("⚠️ Warning: AMCL pose not ready. Using Default P1 as start.")

            # 2. TẠO DANH SÁCH NODES
            nodes = [current_pose]
            for name in self.mission_list:
                c = self.all_waypoints[name]
                nodes.append(self.get_pose_stamped(c[0], c[1], c[2]))
                
            n = len(nodes)
            dist_matrix = np.zeros((n, n))
            
            # 3. MA TRẬN A* KHOẢNG CÁCH
            for i in range(n):
                for j in range(n):
                    if i != j:
                        path = self.navigator.getPath(nodes[i], nodes[j], planner_id='GridBased', use_start=True)
                        if path and path.poses:
                            dist_matrix[i][j] = len(path.poses) * 0.05
                        else:
                            dist_matrix[i][j] = math.sqrt((nodes[i].pose.position.x - nodes[j].pose.position.x)**2 + 
                                                         (nodes[i].pose.position.y - nodes[j].pose.position.y)**2) * 2.0
            
            # 4. CHẠY GA
            ga = GeneticAlgorithmTSP(dist_matrix)
            best_order = ga.run()
            
            # 5. LOGIC CLUSTER & QUAY ĐẦU (LOOK-AHEAD)
            self.optimal_poses = []
            clusters = [
                ["P6", "P7"],
                ["P8", "P9", "P10"],
                ["P13", "P14", "P15"]
            ]

            def get_name(idx):
                return "Start" if idx == 0 else self.mission_list[idx-1]

            def is_same_cluster(name1, name2):
                n1_base = name1.split()[0]
                n2_base = name2.split()[0]
                for c in clusters:
                    if n1_base in c and n2_base in c:
                        return True
                return False

            for i in range(len(best_order)):
                curr_idx = best_order[i]
                curr_name = get_name(curr_idx)
                pose = nodes[curr_idx]
                
                if i < len(best_order) - 1:
                    next_idx = best_order[i+1]
                    next_name = get_name(next_idx)
                    
                    if is_same_cluster(curr_name, next_name):
                        # Cùng cụm -> Trượt ngang (Giữ nguyên Yaw cũ)
                        if len(self.optimal_poses) > 0:
                            pose.pose.orientation.x = self.optimal_poses[-1].pose.orientation.x
                            pose.pose.orientation.y = self.optimal_poses[-1].pose.orientation.y
                            pose.pose.orientation.z = self.optimal_poses[-1].pose.orientation.z
                            pose.pose.orientation.w = self.optimal_poses[-1].pose.orientation.w
                    else:
                        # Khác cụm -> Tính góc quay đầu nhìn về điểm tiếp theo
                        next_pose = nodes[next_idx]
                        dx = next_pose.pose.position.x - pose.pose.position.x
                        dy = next_pose.pose.position.y - pose.pose.position.y
                        yaw = math.atan2(dy, dx)
                        
                        qx, qy, qz, qw = get_quaternion_from_euler(yaw)
                        pose.pose.orientation.x = qx
                        pose.pose.orientation.y = qy
                        pose.pose.orientation.z = qz
                        pose.pose.orientation.w = qw
                else:
                    if len(self.optimal_poses) > 0:
                        pose.pose.orientation.x = self.optimal_poses[-1].pose.orientation.x
                        pose.pose.orientation.y = self.optimal_poses[-1].pose.orientation.y
                        pose.pose.orientation.z = self.optimal_poses[-1].pose.orientation.z
                        pose.pose.orientation.w = self.optimal_poses[-1].pose.orientation.w
                        
                self.optimal_poses.append(pose)

            # 6. CẬP NHẬT UI
            self.listbox_mission.delete(0, tk.END)
            new_mission_list = []
            for i, idx in enumerate(best_order):
                if idx == 0: 
                    self.listbox_mission.insert(tk.END, "-> [Current Position]")
                else: 
                    real_name = self.mission_list[idx-1]
                    self.listbox_mission.insert(tk.END, f"-> {real_name}")
                    new_mission_list.append(real_name)
            
            self.mission_list = new_mission_list

            self.draw_optimized_path()
            self.btn_run.config(state=tk.NORMAL)

        finally:
            self.btn_ga.config(text="⚙️ OPTIMIZE ROUTE (GA)", state=tk.NORMAL)
            self.root.config(cursor="arrow")

    def draw_optimized_path(self):
        self.draw_map_and_points()
        if len(self.optimal_poses) == 0: return
        
        sp = self.optimal_poses[0].pose.position
        self.ax.plot(sp.x, sp.y, 'g*', markersize=12)
        
        for i in range(len(self.optimal_poses) - 1):
            p1, p2 = self.optimal_poses[i], self.optimal_poses[i+1]
            path = self.navigator.getPath(p1, p2, use_start=True)
            
            if path and path.poses:
                px = [p.pose.position.x for p in path.poses]
                py = [p.pose.position.y for p in path.poses]
                self.ax.plot(px, py, color='red', linewidth=1.5)
                m = len(px)//2
                self.ax.annotate('', xy=(px[m+1], py[m+1]), xytext=(px[m], py[m]),
                                 arrowprops=dict(arrowstyle="->", color='red'))
            
            self.ax.plot(p2.pose.position.x, p2.pose.position.y, 'bs', markersize=6)
            self.ax.text(p2.pose.position.x, p2.pose.position.y + 1, f"[{i+1}]", color='blue', weight='bold')

        self.canvas.draw()

    # ==========================================
    # CƠ CHẾ GỬI LỆNH TỪNG ĐIỂM + ĐIỂM BÌNH THƯỜNG + ESTOP
    # ==========================================
    def start_robot(self):
        if len(self.optimal_poses) <= 1: return
        
        # 1. TẠO HÀNG ĐỢI LỆNH MỚI (CHỈ GỬI ĐIỂM BÌNH THƯỜNG, ĐÃ BỎ ĐIỂM ẢO)
        self.execution_queue = []
        for i in range(1, len(self.optimal_poses)):
            curr_name = self.mission_list[i-1]
            self.execution_queue.append({
                'name': curr_name,
                'pose': self.optimal_poses[i]
            })

        # 2. BẮT ĐẦU CHẠY HÀNG ĐỢI
        self.is_mission_active = True
        self.current_exec_index = 0
        self.retry_count = 0
        
        self.btn_run.config(state=tk.DISABLED, text="⏳ EXECUTING MISSION...")
        self.btn_ga.config(state=tk.DISABLED) 
        self.btn_stop.config(state=tk.NORMAL)
        
        self.send_next_goal()

    def send_next_goal(self):
        if not self.is_mission_active: return
        
        if self.current_exec_index < len(self.execution_queue):
            task = self.execution_queue[self.current_exec_index]
            
            # Lấy mục tiêu và làm mới timestamp
            target = task['pose']
            target.header.stamp = self.navigator.get_clock().now().to_msg()
            
            print(f"🚀 Gửi lệnh đến: {task['name']} (Lần thử: {self.retry_count + 1})")
            
            # Gửi từng điểm một bằng goToPose thay vì followWaypoints
            self.navigator.goToPose(target)
            self.root.after(100, self.monitor_single_goal)
        else:
            self.finish_mission(success=True)

    def monitor_single_goal(self):
        if not self.is_mission_active: return

        if not self.navigator.isTaskComplete():
            self.root.after(100, self.monitor_single_goal)
        else:
            result = self.navigator.getResult()
            task = self.execution_queue[self.current_exec_index]
            
            if result == TaskResult.SUCCEEDED:
                print(f"✅ Đã đến {task['name']} thành công!")
                
                # Cập nhật GUI Listbox
                self.listbox_mission.delete(1)
                if len(self.mission_list) > 0:
                    self.mission_list.pop(0)
                    
                # Tiến tới điểm tiếp theo
                self.current_exec_index += 1
                self.retry_count = 0
                self.send_next_goal()
                
            else:
                # Nếu bị kẹt thì cho phép thử lại
                self.retry_count += 1
                if self.retry_count <= self.max_retries:
                    print(f"⚠️ Thất bại tại {task['name']}. Đang thử lại ({self.retry_count}/{self.max_retries})...")
                    self.send_next_goal()
                else:
                    self.finish_mission(success=False, msg=f"Xe không thể hoàn thành lệnh tại {task['name']} sau {self.max_retries} lần thử lại.")

    def emergency_stop(self):
        self.is_mission_active = False
        self.navigator.cancelTask()
        
        self.btn_run.config(state=tk.NORMAL, text="🚀 EXECUTE REMAINING")
        self.btn_ga.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        print("🛑 KÍCH HOẠT DỪNG KHẨN CẤP!")
        messagebox.showwarning("Emergency Stop", "Đã dừng robot khẩn cấp!\n\nCác điểm CHƯA ĐI vẫn được giữ lại trong Queue. Bạn có thể chèn thêm điểm rồi bấm tính (GA) lại từ vị trí hiện tại, hoặc nhấn 'Execute' để đi nốt.")

    def finish_mission(self, success, msg=""):
        self.is_mission_active = False
        self.btn_run.config(state=tk.NORMAL, text="🚀 EXECUTE MISSION")
        self.btn_ga.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)

        if success:
            messagebox.showinfo("Mission Complete", "Robot đã hoàn thành toàn bộ hành trình!")
            self.clear_all_waypoints()
        else:
            messagebox.showerror("Mission Failed", msg)

if __name__ == "__main__":
    root = tk.Tk()
    app = FleetGUI(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (rclpy.shutdown(), root.destroy()))
    root.mainloop()