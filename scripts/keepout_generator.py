#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
import cv2
import numpy as np
import yaml
import os

class KeepoutGenerator(Node):
    def __init__(self):
        super().__init__('keepout_generator')
        
        # Chỉ lắng nghe vị trí THỰC TẾ của xe trên bản đồ
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        
        self.current_pose = None
        
        # Đường dẫn bản đồ (Đọc map.yaml gốc để xuất ra keepout_mask.pgm)
        self.map_yaml_path = os.path.expanduser('~/ros2_ws/src/robot_omni/maps/hospital_map.yaml')
        self.keepout_image_path = os.path.expanduser('~/ros2_ws/src/robot_omni/maps/keepout_mask.pgm')
        self.keepout_yaml_path = os.path.expanduser('~/ros2_ws/src/robot_omni/maps/keepout_mask.yaml')
        
        # Lưu trữ điểm
        self.zones = []         # Chứa tất cả các vùng cấm đã chốt
        self.current_zone = []  # Chứa các điểm của vùng cấm đang chạy (VD: 4 điểm)
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🟢 CÔNG CỤ TẠO VÙNG CẤM ĐÃ SẴN SÀNG!')
        self.get_logger().info('👉 Lái xe đến góc vật cản -> Nhấn [ENTER] để lưu điểm.')
        self.get_logger().info('👉 Xong 1 vật (ít nhất 3 điểm) -> Nhập [N] + ENTER để sang vật khác.')
        self.get_logger().info('👉 Xong tất cả -> Nhập [S] + ENTER để xuất File Bản Đồ Vùng Cấm.')
        self.get_logger().info('='*60 + '\n')

        # Bật luồng chờ phím Enter (Giữ nguyên cấu trúc xịn của bạn)
        self.input_thread = threading.Thread(target=self.wait_for_keypress)
        self.input_thread.daemon = True
        self.input_thread.start()

    def amcl_callback(self, msg):
        # Liên tục cập nhật vị trí xe
        self.current_pose = msg.pose.pose

    def world_to_map(self, x, y, origin_x, origin_y, resolution, map_height):
        """Thuật toán chuyển đổi hệ tọa độ thực tế (Mét) sang tọa độ Ảnh (Pixel)"""
        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)
        # Odom hệ trục Oxy ở dưới trái, OpenCV ảnh hệ trục ở trên trái nên phải đảo ngược Y
        map_y = map_height - map_y 
        return [map_x, map_y]

    def generate_mask(self):
        """Hàm xuất ảnh khi nhấn phím S"""
        if not self.zones and not self.current_zone:
            self.get_logger().warning('⚠️ Chưa có vùng nào được lưu! Bạn phải chạy xe và nhấn Enter trước.')
            return

        self.get_logger().info('⏳ Đang xử lý hình ảnh và tô màu Vùng Cấm...')
        
        # 1. Đọc thông số bản đồ gốc
        try:
            with open(self.map_yaml_path, 'r') as file:
                map_data = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f'❌ Lỗi đọc file map.yaml: {e}')
            return
            
        resolution = map_data['resolution']
        origin_x = map_data['origin'][0]
        origin_y = map_data['origin'][1]
        
        # 2. Lấy kích thước ảnh gốc để tạo ảnh mask có kích thước y hệt
        img_path = os.path.join(os.path.dirname(self.map_yaml_path), map_data['image'])
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error('❌ Không tìm thấy ảnh map gốc. Hãy kiểm tra lại đường dẫn!')
            return
        map_height, map_width = img.shape
        
        # 3. Tạo một bức ảnh TRẮNG tinh hoàn toàn
        mask_img = np.full((map_height, map_width), 255, dtype=np.uint8)

        # 4. Gom nốt vùng đang chạy dở vào danh sách (nếu có)
        all_zones = self.zones.copy()
        if len(self.current_zone) >= 3:
            all_zones.append(self.current_zone)
            
        # 5. Lấy tọa độ và tô ĐEN các vùng
        for i, zone in enumerate(all_zones):
            pts = np.array([self.world_to_map(p[0], p[1], origin_x, origin_y, resolution, map_height) for p in zone], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.fillPoly(mask_img, [pts], 0) # Lệnh 0 nghĩa là tô màu đen
            self.get_logger().info(f'  + Đã xử lý Vùng {i+1} (Gồm {len(zone)} điểm)')

        # 6. Lưu file ảnh .pgm
        cv2.imwrite(self.keepout_image_path, mask_img)
        
        # 7. Tạo file .yaml tương ứng cho mask
        map_data['image'] = os.path.basename(self.keepout_image_path)
        map_data['mode'] = 'trinary'
        with open(self.keepout_yaml_path, 'w') as file:
            yaml.dump(map_data, file)
            
        self.get_logger().info(f'✅ XONG! Đã xuất file thành công tại:\n  - {self.keepout_image_path}\n  - {self.keepout_yaml_path}')
        self.get_logger().info('Khởi động lại Nav2 để hệ thống áp dụng Tường Ảo!')

    def wait_for_keypress(self):
        """Luồng xử lý phím bấm liên tục"""
        while True:
            cmd = input("👉 Lệnh [ENTER: Lưu điểm] | [N: Chốt Vùng] | [S: Xuất File] -> ").strip().upper()
            
            if cmd == '':
                # Nhấn Enter -> Lưu 1 điểm
                if self.current_pose:
                    x = self.current_pose.position.x
                    y = self.current_pose.position.y
                    self.current_zone.append((x, y))
                    self.get_logger().info(f'📍 Đã lưu ĐIỂM {len(self.current_zone)} của VÙNG {len(self.zones)+1}: x={x:.2f}, y={y:.2f}')
                else:
                    self.get_logger().warning('⚠️ Chưa có tọa độ. Nhớ cấp "2D Pose Estimate" trên RViz trước nhé!')
            
            elif cmd == 'N':
                # Nhấn N -> Sang đống ghế khác
                if len(self.current_zone) >= 3:
                    self.zones.append(self.current_zone)
                    self.get_logger().info(f'➡️ Đã CHỐT Vùng {len(self.zones)}. Sẵn sàng vẽ khu vực MỚI.')
                    self.current_zone = [] # Làm rỗng để nhận tọa độ đống ghế tiếp theo
                else:
                    self.get_logger().warning(f'⚠️ Vùng hiện tại mới có {len(self.current_zone)} điểm. Xe cần ít nhất 3 điểm (tạo thành 1 hình) mới hiểu được!')
                    
            elif cmd == 'S':
                # Nhấn S -> Lưu ảnh đen trắng
                self.generate_mask()

def main(args=None):
    rclpy.init(args=args)
    node = KeepoutGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n🛑 Đã tắt công cụ.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()