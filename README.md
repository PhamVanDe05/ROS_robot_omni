# 🚀 Điều Hướng Robot Đa Hướng (Mecanum) trong Bệnh Viện

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-FF7F00?logo=gazebo)
![Nav2](https://img.shields.io/badge/Nav2-Navigation-blue)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python)

Dự án này cung cấp một hệ thống mô phỏng toàn diện cho robot di chuyển đa hướng (Mecanum) hoạt động trong môi trường bệnh viện. Tích hợp đầy đủ hệ sinh thái **ROS 2 Jazzy**, **Gazebo Harmonic**, và **Nav2** để thực hiện các tác vụ lập bản đồ (mapping), định vị (localization) và điều hướng tự động (autonomous navigation) đi kèm với giao diện điều khiển trực quan.

## 👥 Đội Ngũ Phát Triển
* **Nguyễn Phương Duy** - 23134010
* **Đoàn Hải Đăng** - 23134010
* **Phạm Văn Để** - 23134012
* **Phạm Nguyễn Văn Đông** - 23134013

---

## ✨ Tính Năng Nổi Bật

* 🔄 **Robot Đa Hướng (Mecanum):** Sử dụng `mecanum_drive_controller` để điều khiển 4 bánh độc lập, cho phép robot di chuyển ngang, chéo và mượt mà trong các không gian hẹp của bệnh viện.
* 🏥 **Môi Trường Bệnh Viện Thực Tế:** Mô phỏng chi tiết với thế giới `hospital_full.world`, bao gồm đầy đủ các chướng ngại vật phức tạp như giường bệnh, hành lang, và vật thể động.
* 📡 **Nav2 & Cảm Biến Tiên Tiến:** * Sử dụng thuật toán **AMCL** để định vị chính xác. 
  * Bản đồ chi phí (Costmap) được xây dựng dựa trên sự kết hợp dữ liệu từ **Lidar** (`/scan_front`, `/scan_rear`) và **RGBD Camera** (chuyển đổi ảnh Depth sang mây điểm `PointCloud2`).
* 🧠 **Điều Hướng Thông Minh (Smart Navigation):** * Tích hợp `velocity_smoother` giúp xe tăng/giảm tốc mượt mà.
  * Tùy chỉnh **Behavior Tree C++** (`smart_recovery_bt.xml` & `is_space_clear_plugin`) giúp robot tự động phân tích không gian và tính toán góc thoát hiểm thông minh khi bị kẹt.
* 🖥️ **Giao Diện Điều Khiển (Fleet GUI):** Phần mềm viết bằng Python/Tkinter (`mission_control.py`) tích hợp **Thuật toán Di Truyền (Genetic Algorithm - GA)** để tự động tối ưu hóa lộ trình ngắn nhất (TSP) đi qua nhiều điểm.
* 🧹 **Tối Ưu Hóa Dữ Liệu:** Sử dụng bộ lọc Laser (`laser_filters`) để làm sạch nhiễu cảm biến trước khi đưa dữ liệu vào hệ thống Nav2.

---

## 🛠️ Chi Tiết Công Nghệ Lõi Đã Thực Hiện
* **Đồng bộ Dữ Liệu:** Thiết lập `bridge_config.yaml` kết nối mượt mà Lidar, Camera 3D, IMU giữa Gazebo và ROS 2.
* **Tuning Costmap & DWB Local Planner:** Tinh chỉnh `global_costmap` và `local_costmap` xử lý mây điểm 3D, loại bỏ lỗi "bóng ma" vật cản. Tối ưu ma trận gia tốc, vận tốc để xe chạy bốc nhưng vẫn giữ an toàn, không rớt frame.
* **Can thiệp Behavior Tree (C++):** Phát triển plugin C++ tùy chỉnh có khả năng đọc bản đồ quét và tự động tính toán góc xoay né vật cản động, vượt qua giới hạn của các recovery tiêu chuẩn.

---

## 💻 Yêu Cầu Hệ Thống (Prerequisites)
* Hệ điều hành: Ubuntu 24.04 (Noble Numbat)
* ROS 2: Jazzy Jalisco
* Gazebo: Harmonic
* Các package phụ thuộc bắt buộc: `nav2_bringup`, `robot_localization`, `laser_filters`, `ros_gz_bridge`, `behaviortree_cpp`.

---

## 🚀 Hướng Dẫn Cài Đặt 

**1. Clone kho lưu trữ về workspace của bạn:**
```bash
cd ~/ros2_ws/src
git clone <link-github-cua-ban>
2. Cài đặt các thư viện phụ thuộc (rosdep):

Bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
3. Biên dịch dự án:

Bash
colcon build --packages-select robot_omni --symlink-install
source install/setup.bash
📖 Hướng Dẫn Sử Dụng (User Guide)
Để vận hành hệ thống, bạn cần mở 3 Terminal khác nhau và chạy lần lượt các bước sau:

Bước 1: Khởi động Môi trường Mô Phỏng (Gazebo)
Mở Terminal 1 và gõ:

Bash
ros2 launch robot_omni gazebo_control.launch.py
Hệ thống sẽ tải bệnh viện 3D, spawn robot Mecanum và kích hoạt bộ điều khiển động cơ cùng bộ lọc nhiễu EKF.

<img width="1003" height="886" alt="Screenshot from 2026-04-21 23-53-49" src="https://github.com/user-attachments/assets/0c606f8b-6ba6-4ebd-994e-be241456ff9e" />


Bước 2: Khởi động Hệ thống Điều Hướng (Nav2) & RViz
Mở Terminal 2 và gõ:

Bash
ros2 launch robot_omni localization.launch.py
RViz sẽ hiện lên với bản đồ 2D (hospital_map). Hệ thống AMCL đã được thiết lập để tự động nhận diện vị trí khởi tạo ban đầu, các cảm biến Lidar và chùm PointCloud màu xanh lá cây từ Camera 3D sẽ xuất hiện để quét vật cản xung quanh.

<img width="1615" height="927" alt="Screenshot from 2026-04-21 23-52-38" src="https://github.com/user-attachments/assets/9d5ed4df-7518-4d6e-9ecf-4c40ad689100" />


Bước 3: Khởi động Giao diện Điều khiển (Mission Control GUI)
Mở Terminal 3 và gõ:

Bash
python3 src/robot_omni/scripts/mission_control.py
Giao diện AMR Fleet Management System sẽ hiện lên.
Tại đây, hệ thống sẽ tự động nạp sẵn một danh sách điểm mặc định (VD: P16 -> P7 -> P12) vào hàng đợi nhiệm vụ.

<img width="1407" height="936" alt="Screenshot from 2026-04-21 23-54-29" src="https://github.com/user-attachments/assets/80f60ce1-2172-4e10-a887-4c6886f5576a" />


Bước 4: Lập Lịch & Tối Ưu Hóa Lộ Trình (Genetic Algorithm)
Bạn có thể chọn thêm các điểm đến từ bảng "AVAILABLE WAYPOINTS" và nhấn "Add Selected [+]".

Sau khi đã có danh sách các trạm cần đến ở "MISSION QUEUE", nhấn nút xanh lá "⚙️ OPTIMIZE ROUTE (GA)".

Hệ thống sẽ chạy thuật toán Di truyền (GA) kết hợp với A* Costmap để tính toán ra đường đi ngắn nhất qua tất cả các điểm. Quỹ đạo màu đỏ sẽ được vẽ ngay trên bản đồ 2D của GUI.

<img width="1407" height="936" alt="Screenshot from 2026-04-21 23-59-16" src="https://github.com/user-attachments/assets/44b1907f-eed2-4252-b8bc-ef7a2577616a" />


Bước 5: Thực Thi Nhiệm Vụ & Xử Lý Sự Cố
Nhấn nút xanh dương "🚀 EXECUTE MISSION" để robot bắt đầu chạy.

Bạn có thể quan sát robot luồn lách qua các hành lang trên RViz. Nếu gặp chướng ngại vật động ngáng đường, Behavior Tree C++ sẽ tự động kích hoạt tiến trình Recovery (lùi lại, xoay đầu tìm góc thoáng) để vượt qua.

Nhấn nút đỏ "🛑 EMERGENCY STOP" bất cứ lúc nào để dừng robot ngay lập tức; các điểm chưa đi sẽ được giữ nguyên trong hàng đợi để tiếp tục sau.

📸 [CHÈN HÌNH ẢNH SỐ 5: Chụp màn hình RViz cho thấy robot đang vẽ vạch bám theo đường đi (mũi tên xanh/đỏ) trong lúc di chuyển]
