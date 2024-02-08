import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from ackermann_msgs.msg import AckermannDrive
import termios
import sys
import tty
import select

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(AckermannDrive, 'ackermann_drive', 10)
        self.timer = self.create_timer(0.1, self.publish_control)

        self.speed = 0.0
        self.steering_angle = 0.0

        self.acceleration = 0.1
        self.deceleration = 0.1
        self.speed_slew_rate = 0.02
        self.steering_increment = 0.1
        self.steering_slew_rate = 0.02
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Keyboard Control Node is running. Use W/A/S/D to control the car. Press Ctrl-C to exit.')

    def get_key(self):
        tty.setcbreak(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            return key
        else:
            return ''

    def publish_control(self):
        key = self.get_key()
        if key:
            if key == 'w' or key == 's':
                if key == 'w':
                    self.speed += self.acceleration
                else:
                    self.speed -= self.deceleration
            else:
                self.speed -= self.speed_slew_rate * self.speed
            if key == 'a' or key == 'd':
                if key == 'a':
                    self.steering_angle += self.steering_increment
                else:
                    self.steering_angle -= self.steering_increment
            else:
                self.steering_angle -= self.steering_slew_rate * self.steering_angle
            if key == '\x03': # ctrl-c
                self.get_logger().info('Keyboard Control Node is shutting down.')
                rclpy.shutdown()
                sys.exit(0)
        else:
            self.speed -= self.speed_slew_rate * self.speed
            self.steering_angle -= self.steering_slew_rate * self.steering_angle
        self.steering_angle = max(min(self.steering_angle, 0.52), -0.52) # +-30deg
        msg = AckermannDrive()
        msg.speed = self.speed
        msg.steering_angle = self.steering_angle
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    try:
        rclpy.spin(keyboard_control)
    except:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, keyboard_control.settings)
        keyboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
