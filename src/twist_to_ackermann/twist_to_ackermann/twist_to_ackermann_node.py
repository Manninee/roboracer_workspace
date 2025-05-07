from typing import Union
from math import atan, isclose

from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped


class TwistToAckermann(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')

        self._twist_stamped = self.declare_parameter("twist_stamped", False).value
        self._ackermann_stamped = self.declare_parameter("ackermann_stamped", True).value

        self._twist_topic = self.declare_parameter("twist_topic", "/cmd_vel").value
        self._ackermann_topic = self.declare_parameter("ackermann_topic", "/drive").value

        self._wheelbase = self.declare_parameter("wheelbase", 1.0).value

        self._input_type = TwistStamped if self._twist_stamped else Twist
        self._output_type = AckermannDriveStamped if self._ackermann_stamped else AckermannDrive

        self._subscription = self.create_subscription(self._input_type, self._twist_topic, self.cb, 1)
        self._publisher = self.create_publisher(self._output_type, self._ackermann_topic, 1)

        self.get_logger().info(f"{self.__class__.__name__} started with parameters:\n"
                               f"    Stamped: Twist={str(self._twist_stamped)}, Ackermann={str(self._ackermann_stamped)}\n"
                               f"    Topics: Twist={self._twist_topic}, Ackermann={self._ackermann_topic}\n"
                               f"    Wheelbase: {self._wheelbase}m")

    @staticmethod
    def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
        if isclose(omega, 0.0) or isclose(v, 0.0):
            return 0.0

        return atan((wheelbase * omega) / v)

    def cb(self, data: Union[Twist, TwistStamped]):
        if self._twist_stamped:
            v = data.twist.linear.x
            omega = data.twist.angular.z
            header = data.header
        else:
            v = data.linear.x
            omega = data.angular.z
            header = None


        steering_angle = TwistToAckermann.convert_trans_rot_vel_to_steering_angle(v, omega, self._wheelbase)

        msg = self._output_type()
        if self._ackermann_stamped:
            if header is not None:
                msg.header.stamp = header.stamp
                msg.header.frame_id = header.frame_id
            msg.drive.steering_angle = steering_angle
            msg.drive.speed = v
        else:
            msg.steering_angle = steering_angle
            msg.speed = v

        self._publisher.publish(msg)
