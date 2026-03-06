import sys
import tty
import termios
import select

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.keyboard_control_pub_ = self.create_publisher(String, 'keyboard_node/key', 10)
    

    def send_key(self, key):
        msg = String()
        msg.data = key
        self.keyboard_control_pub_.publish(msg)
        
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return None

def main(args=None):
    rclpy.init(args=args)
    keyboard_node = KeyboardNode()
    while rclpy.ok():
        key = get_key()
        if key:
            if key == 'b':
                print("Exiting keyboard node.")
                break
            if key == 'r':
                print("Resetting detector.")
           
            keyboard_node.send_key(key)
        time.sleep(0.001)
        
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()