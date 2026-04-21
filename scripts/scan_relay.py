# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan

# class ScanRelay(Node):
#     def __init__(self):
#         super().__init__('scan_relay_node')
#         # Lắng nghe dữ liệu Lidar phía trước
#         self.subscription = self.create_subscription(
#             LaserScan,
#             '/scan_front_raw',
#             self.scan_callback,
#             10)
        
#         # Phát lại ra topic /scan cho AMCL
#         self.publisher = self.create_publisher(LaserScan, '/scan', 10)
#         self.get_logger().info("Đã khởi động Scan Relay: Forwarding /scan_front_raw -> /scan")

#     def scan_callback(self, msg):
#         # Chuyển tiếp y nguyên message, TF sẽ tự động lo việc tính toán vị trí frame
#         self.publisher.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ScanRelay()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()