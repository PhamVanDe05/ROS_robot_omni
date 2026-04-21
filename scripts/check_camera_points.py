#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sensor_msgs_py.point_cloud2 as pc2
import math
import random

class PointCloudReader(Node):
    def __init__(self):
        super().__init__('pointcloud_reader_node')
        
        # ĐẶC BIỆT QUAN TRỌNG: Lấy dữ liệu từ Gazebo Bridge phải dùng Best Effort
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Lắng nghe Topic của Camera 3D
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.listener_callback,
            qos_profile
        )
        self.get_logger().info("✅ Đã kết nối với Mắt thần 3D! Đang chờ quét vật cản...")

    def listener_callback(self, msg):
        # Đọc dữ liệu giải nén thành danh sách các tọa độ X, Y, Z
        # skip_nans=True giúp bỏ qua các tia chiếu vào không khí (vô tận)
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        obstacle_points = []
        
        # Duyệt qua toàn bộ hàng trăm ngàn điểm
        for p in points:
            x, y, z = p[0], p[1], p[2]
            
            # LỌC VẬT CẢN (Giống cách Nav2 đang làm)
            # Chỉ lấy vật cao từ 15cm đến 1.8m, và nằm trong bán kính 8m
            if 0.15 < z < 1.8 and math.sqrt(x**2 + y**2) < 8.0:
                obstacle_points.append((x, y))

        total_points = len(obstacle_points)
        
        # Nếu có quét trúng vật cản thì in ra
        if total_points > 0:
            print(f"\n🚀 PHÁT HIỆN VẬT CẢN! Cấu tạo từ {total_points} điểm ảnh.")
            print("📍 Tọa độ (X, Y) của 10 điểm ngẫu nhiên trên vật cản:")
            
            # Trộn ngẫu nhiên để xem tọa độ các góc khác nhau của cái ghế
            sample_points = random.sample(obstacle_points, min(10, total_points))
            
            for i, (px, py) in enumerate(sample_points):
                print(f"   - Điểm {i+1}: X = {px:.3f} (m) , Y = {py:.3f} (m)")
            print("-" * 50)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()