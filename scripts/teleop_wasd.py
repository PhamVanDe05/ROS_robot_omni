# python3 ~/ros2_ws/src/robot_omni/scripts/teleop_wasd.py



#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty

msg = """
MÁY ĐIỀU KHIỂN XE MECANUM PHẠM VĂN ĐỆ
---------------------------
Di chuyển:        Trượt ngang:
    W                 Q (Trái)
A   S   D             E (Phải)
---------------------------
W/S : Tiến/Lùi
A/D : Xoay Trái/Phải
Q/E : Đi ngang Trái/Phải (Mecanum power!)
Space: Dừng khẩn cấp
CTRL-C để thoát
"""

# Cấu hình các phím bấm
move_bindings = {
    'w': (1.0, 0.0, 0.0),  's': (-1.0, 0.0, 0.0),
    'a': (0.0, 0.0, 1.5),  'd': (0.0, 0.0, -1.5),
    'q': (0.0, 1.0, 0.0),  'e': (0.0, -1.0, 0.0),
    ' ': (0.0, 0.0, 0.0),
}

class TeleopWASD(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        # Đảm bảo topic đúng với topic bạn đang dùng trên rqt
        self.pub = self.create_publisher(TwistStamped, '/mobile_base_controller/reference', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        # SỬA LỖI TẠI ĐÂY: Dùng fileno() thay vì readline
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(msg)
        try:
            while True:
                key = self.get_key()
                if key in move_bindings:
                    x, y, th = move_bindings[key]
                    
                    # Tạo tin nhắn TwistStamped (Header + Twist)
                    t_msg = TwistStamped()
                    t_msg.header.stamp = self.get_clock().now().to_msg()
                    t_msg.header.frame_id = 'base_link'
                    
                    # Cài đặt tốc độ (Bạn có thể tăng giảm số nhân ở đây)
                    t_msg.twist.linear.x = x * 1.5  # Tốc độ tiến/lùi
                    t_msg.twist.linear.y = y * 1.0  # Tốc độ trượt ngang (Q/E)
                    t_msg.twist.angular.z = th      # Tốc độ xoay
                    
                    self.pub.publish(t_msg)
                elif key == '\x03': # Nhấn CTRL-C để thoát
                    break
        except Exception as e:
            print(e)
        finally:
            # Trả lại trạng thái bàn phím bình thường khi thoát
            t_msg = TwistStamped()
            self.pub.publish(t_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = TeleopWASD()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()