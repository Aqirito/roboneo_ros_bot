import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class LedStatePublisher(Node):
    def __init__(self):
        super().__init__('led_state_publisher')
        self.publisher_ = self.create_publisher(Int32, 'led_state', 10)

    def send_led_state(self, state: int):
        msg = Int32()
        msg.data = state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published LED state: {state}')


def main(args=None):
    rclpy.init(args=args)
    node = LedStatePublisher()

    try:
        while rclpy.ok():
            cmd = input("Enter LED state (1=ON, 0=OFF, q=quit): ").strip()
            if cmd.lower() == 'q':
                break
            elif cmd in ('0', '1'):
                node.send_led_state(int(cmd))
            else:
                print("Invalid input! Please enter 0, 1, or q.")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
