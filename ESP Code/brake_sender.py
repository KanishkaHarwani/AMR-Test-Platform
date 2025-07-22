import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

def main():
    rclpy.init()
    node = Node('brake_burst_publisher')
    pub = node.create_publisher(Bool, '/brake_input', 10)

    # Publish True
    msg = Bool()
    msg.data = True
    pub.publish(msg)
    node.get_logger().info('Published: True')

    # Wait 1 second
    time.sleep(0.20)

    # Publish False
    msg.data = False
    pub.publish(msg)
    node.get_logger().info('Published: False')

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

