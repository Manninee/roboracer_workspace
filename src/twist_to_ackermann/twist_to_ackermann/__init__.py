import rclpy

from .twist_to_ackermann_node import TwistToAckermann


def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermann()

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(str(e))
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()