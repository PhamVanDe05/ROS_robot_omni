
# python3 ~/ros2_ws/src/robot_omni/scripts/waypoint_recorder.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import threading
import os

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        # Chỉ lắng nghe vị trí THỰC TẾ của xe trên bản đồ
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        
        self.current_pose = None
        self.point_count = 1
        
        # File lưu trữ
        self.file_path = os.path.expanduser('~/dead_toa_do_cua_toi.txt')
        
        with open(self.file_path, 'w') as f:
            f.write("--- DANH SÁCH WAYPOINTS [x, y, yaw] ---\n")
            
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🟢 CÔNG CỤ CHỤP TỌA ĐỘ ĐÃ SẴN SÀNG!')
        self.get_logger().info('1. Mở RViz, dùng "Nav2 Goal" để sai xe chạy tới điểm cần đến.')
        self.get_logger().info('2. Đợi xe dừng hẳn, qua Terminal này nhấn ENTER để chụp tọa độ.')
        self.get_logger().info('='*60 + '\n')

        # Bật luồng chờ phím Enter
        self.input_thread = threading.Thread(target=self.wait_for_keypress)
        self.input_thread.daemon = True
        self.input_thread.start()

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def amcl_callback(self, msg):
        # Liên tục cập nhật vị trí xe
        self.current_pose = msg.pose.pose

    def wait_for_keypress(self):
        while True:
            input("👉 NHẤN ENTER để LƯU TỌA ĐỘ HIỆN TẠI của xe...\n")
            
            if self.current_pose:
                x = self.current_pose.position.x
                y = self.current_pose.position.y
                yaw = self.get_yaw(self.current_pose.orientation)
                
                # Lưu vào file
                with open(self.file_path, 'a') as f:
                    f.write(f"[{x:.3f}, {y:.3f}, {yaw:.3f}],\n")
                    
                self.get_logger().info(f'✅ ĐÃ LƯU THÀNH CÔNG -> Điểm {self.point_count}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')
                self.point_count += 1
            else:
                self.get_logger().warning('⚠️ Chưa có tọa độ. Nhớ cấp "2D Pose Estimate" trên RViz trước nhé!')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n🛑 Đã tắt công cụ.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()