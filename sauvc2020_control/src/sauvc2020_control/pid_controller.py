import rospy
from sauvc2020_msgs.msg import MotionData
from geometry_msgs.msg import QuaternionStamped
import math


class PIDController(object):
    """A class that represent the PID Controller."""

    def __init__(self, stabilised_speed_publisher):
        """Initialise the controller
        Args:
            stabilised_speed_publisher (rospy.Publisher): a ROS publisher for motor stabilised speed
        """
        rospy.Subscriber("/motor/motion", MotionData, self._update_motion_data)
        rospy.Subscriber("/filter/quaternion", QuaternionStamped, self._get_quaternion_data)
        self._stabilised_speed_publisher = stabilised_speed_publisher  # type: type(rospy.Publisher)
        self._robot_motion = "stop"  # type: str
        self._P = 25  # type: float
        motor_initial_speed = 1500  # type: int
        self._motor_stabilised_speed = {
            "1": motor_initial_speed,
            "2": motor_initial_speed,
            "3": motor_initial_speed,
            "4": motor_initial_speed,
            "5": motor_initial_speed,
            "6": motor_initial_speed,
            "7": motor_initial_speed,
            "8": motor_initial_speed,
        }  # type: dict
        self._motor_actual_speed = {
            "1": motor_initial_speed,
            "2": motor_initial_speed,
            "3": motor_initial_speed,
            "4": motor_initial_speed,
            "5": motor_initial_speed,
            "6": motor_initial_speed,
            "7": motor_initial_speed,
            "8": motor_initial_speed,
        }  # type: dict
        self._actual_euler = {"alpha": float, "beta": float, "gamma": float}  # type: dict
        self._target_euler = {"alpha": float, "beta": float, "gamma": float}  # type: dict

    def _update_motion_data(self, msg):
        """Update the motion data."""
        if self._robot_motion != msg.motion:
            self._target_euler = self._target_euler
            self._robot_motion = msg.motion
        self._motor_actual_speed["1"] = msg.motors_speed.motor_id1_speed
        self._motor_actual_speed["2"] = msg.motors_speed.motor_id2_speed
        self._motor_actual_speed["3"] = msg.motors_speed.motor_id3_speed
        self._motor_actual_speed["4"] = msg.motors_speed.motor_id4_speed
        self._motor_actual_speed["5"] = msg.motors_speed.motor_id5_speed
        self._motor_actual_speed["6"] = msg.motors_speed.motor_id6_speed
        self._motor_actual_speed["7"] = msg.motors_speed.motor_id7_speed
        self._motor_actual_speed["8"] = msg.motors_speed.motor_id8_speed

    def _get_quaternion_data(self, msg):
        """Get IMU quaternion data."""
        alpha, beta, gamma = PIDController.get_euler_angle_from_quat(msg.quaternion.w, msg.quaternion.x,
                                                                     msg.quaternion.y, msg.quaternion.z)
        self._actual_euler["alpha"], self._actual_euler["beta"], self._actual_euler["gamma"] \
            = alpha, beta, gamma

    @staticmethod
    def get_euler_angle_from_quat(w, x, y, z):
        """Get euler angle from quaternion data."""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        alpha = math.atan2(t0, t1) * 180 / math.pi
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        beta = math.asin(t2) * 180 / math.pi
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        gamma = math.atan2(t3, t4) * 180 / math.pi
        return alpha, beta, gamma

    def _compute_forward_movement_error(self):
        """Compute the forward movement error's magnitude and direction."""
        if (self._actual_euler["gamma"] >= 0 and self._target_euler["gamma"] >= 0) \
                or (self._actual_euler["gamma"] < 0 and self._target_euler["gamma"] < 0):
            error = math.fabs(self._target_euler["gamma"] - self._actual_euler["gamma"])
            if self._target_euler["gamma"] > self._actual_euler["gamma"]:
                direction_to_compensate = "CCW"
            else:
                direction_to_compensate = "CW"
        else:
            if math.fabs(self._actual_euler["gamma"]) > 90 and math.fabs(self._target_euler["gamma"]) > 90:
                error = math.fabs(180 - math.fabs(self._target_euler["gamma"])) + \
                        math.fabs(180 - math.fabs(self._actual_euler["gamma"]))
                if self._target_euler["gamma"] < self._actual_euler["gamma"]:
                    direction_to_compensate = "CCW"
                else:
                    direction_to_compensate = "CW"
            else:
                error = math.fabs(self._target_euler["gamma"]) + math.fabs(self._actual_euler["gamma"])
                if self._target_euler["gamma"] > self._actual_euler["gamma"]:
                    direction_to_compensate = "CCW"
                else:
                    direction_to_compensate = "CW"
        return direction_to_compensate, error

    def _compute_stabilised_speed(self, motor_id, error, direction):
        """Compute the stabilised speed from the controller."""
        if (motor_id == "1" and direction == "CCW") or (motor_id == "2" and direction == "CW"):
            error = -1*error
        return int(self._motor_actual_speed[motor_id] + self._P * error)

    def _update_stabilised_speed(self):
        """Update the stabilised speed."""
        if self._robot_motion == "forward":
            try:
                direction, error = self._compute_forward_movement_error()
            except TypeError:
                pass
            else:
                self._motor_stabilised_speed["1"] = self._compute_stabilised_speed("1", error, direction)
                self._motor_stabilised_speed["2"] = self._compute_stabilised_speed("2", error, direction)
                self._motor_stabilised_speed["3"] = self._motor_actual_speed["3"]
                self._motor_stabilised_speed["4"] = self._motor_actual_speed["4"]
                self._motor_stabilised_speed["5"] = self._motor_actual_speed["5"]
                self._motor_stabilised_speed["6"] = self._motor_actual_speed["6"]
                self._motor_stabilised_speed["7"] = self._motor_actual_speed["7"]
                self._motor_stabilised_speed["8"] = self._motor_actual_speed["8"]

    def publish_stabilised_speed(self):
        """Publish the stabilised motor speed."""
        self._update_stabilised_speed()
        self._stabilised_speed_publisher.publish(
            self._motor_stabilised_speed["1"],
            self._motor_stabilised_speed["2"],
            self._motor_stabilised_speed["3"],
            self._motor_stabilised_speed["4"],
            self._motor_stabilised_speed["5"],
            self._motor_stabilised_speed["6"],
            self._motor_stabilised_speed["7"],
            self._motor_stabilised_speed["8"]
        )
