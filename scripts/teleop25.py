#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

# Movement instructions
INSTRUCTIONS = """
AUV Teleop Control
---------------------------
Moving around:
   w/s : forward/backward
   a/d : left/right (strafe)
   q/e : yaw left/right
   r/f : up/down

   i/k : increase/decrease linear speed
   j/l : increase/decrease angular speed

SPACE : stop all movement
CTRL-C : quit

Current speeds will be displayed
"""

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('auv_teleop_keyboard')
        
        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Speed settings
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s
        self.speed_step = 0.1
        
        # Current velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.wz = 0.0
        
        # Key bindings
        self.move_bindings = {
            'w': (1, 0, 0, 0),   # forward
            's': (-1, 0, 0, 0),  # backward
            'a': (0, 1, 0, 0),   # left
            'd': (0, -1, 0, 0),  # right
            'r': (0, 0, 1, 0),   # up
            'f': (0, 0, -1, 0),  # down
            'q': (0, 0, 0, 1),   # yaw left
            'e': (0, 0, 0, -1),  # yaw right
        }
        
        self.speed_bindings = {
            'i': (1, 0),   # increase linear
            'k': (-1, 0),  # decrease linear
            'j': (0, 1),   # increase angular
            'l': (0, -1),  # decrease angular
        }
        
        self.get_logger().info('AUV Teleop Keyboard initialized')
        print(INSTRUCTIONS)
        self.print_current_speeds()
    
    def get_key(self, timeout=0.1):
        """Get keyboard input with timeout"""
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            return key
        return None
    
    def print_current_speeds(self):
        """Print current speed settings"""
        print(f"\nCurrent speeds - Linear: {self.linear_speed:.2f} m/s, Angular: {self.angular_speed:.2f} rad/s")
    
    def publish_velocity(self):
        """Publish the current velocity command"""
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.linear.z = self.vz
        twist.angular.z = self.wz
        self.publisher.publish(twist)
    
    def run(self):
        """Main control loop"""
        # Save terminal settings
        settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                key = self.get_key()
                
                if key:
                    # Check for quit
                    if key == '\x03':  # Ctrl-C
                        break
                    
                    # Check for stop
                    elif key == ' ':
                        self.vx = 0.0
                        self.vy = 0.0
                        self.vz = 0.0
                        self.wz = 0.0
                        print("\nStopped")
                    
                    # Check for movement
                    elif key in self.move_bindings:
                        dx, dy, dz, dw = self.move_bindings[key]
                        self.vx = dx * self.linear_speed
                        self.vy = dy * self.linear_speed
                        self.vz = dz * self.linear_speed
                        self.wz = dw * self.angular_speed
                        print(f"\rVel: x={self.vx:.2f} y={self.vy:.2f} z={self.vz:.2f} yaw={self.wz:.2f}", end='')
                    
                    # Check for speed adjustment
                    elif key in self.speed_bindings:
                        dl, da = self.speed_bindings[key]
                        self.linear_speed = max(0.1, min(2.0, self.linear_speed + dl * self.speed_step))
                        self.angular_speed = max(0.1, min(2.0, self.angular_speed + da * self.speed_step))
                        self.print_current_speeds()
                    
                    # Publish velocity
                    self.publish_velocity()
                
                # Spin once to process callbacks
                rclpy.spin_once(self, timeout_sec=0)
        
        except Exception as e:
            print(f"\nError: {e}")
        
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            
            # Stop the AUV
            twist = Twist()
            self.publisher.publish(twist)
            print("\nShutting down teleop")


def main(args=None):
    rclpy.init(args=args)
    
    teleop = TeleopKeyboard()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()