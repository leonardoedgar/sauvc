import rospy
from sauvc2020_msgs.msg import MotionData
from geometry_msgs.msg import QuaternionStamped


class PIDController(object):
    """A class that represent the PID Controller."""
    def __init__(self, stabilised_speed_publisher):
        """Initialise the controller
        Args:
            stabilised_speed_publisher (rospy.Publisher): a ROS publisher for motor stabilised speed
        """
        rospy.Subscriber("/motor/motion", MotionData, self.update_motion_data)
        rospy.Subscriber("/filter/quaternion", QuaternionStamped, self.get_quaternion_data)
        self._stabilised_speed_publisher = stabilised_speed_publisher  # type: type(rospy.Publisher)
        self._robot_motion = "stop"  # type: str
        self._P = 100  # type: float
        self._motor_stabilised_speed = {
            "1": int,
            "2": int,
            "3": int,
            "4": int,
            "5": int,
            "6": int,
            "7": int,
            "8": int,
        }  # type: dict
        self._motor_actual_speed = {
            "1": int,
            "2": int,
            "3": int,
            "4": int,
            "5": int,
            "6": int,
            "7": int,
            "8": int,
        }  # type: dict
        self._actual_quaternion = {"x": float, "y": float, "z": float, "w": float}  # type: dict
        self._target_quaternion = {"x": float, "y": float, "z": float, "w": float}  # type: dict

    def update_motion_data(self, msg):
        """Update the motion data."""
        if self._robot_motion != msg.motion:
            self._target_quaternion = self._actual_quaternion
            self._robot_motion = msg.motion
        self._motor_actual_speed["1"] = msg.motors_speed.motor_id1_speed
        self._motor_actual_speed["2"] = msg.motors_speed.motor_id2_speed
        self._motor_actual_speed["3"] = msg.motors_speed.motor_id3_speed
        self._motor_actual_speed["4"] = msg.motors_speed.motor_id4_speed
        self._motor_actual_speed["5"] = msg.motors_speed.motor_id5_speed
        self._motor_actual_speed["6"] = msg.motors_speed.motor_id6_speed
        self._motor_actual_speed["7"] = msg.motors_speed.motor_id7_speed
        self._motor_actual_speed["8"] = msg.motors_speed.motor_id8_speed

    def get_quaternion_data(self, msg):
        """Get IMU quaternion data."""
        self._actual_quaternion["x"] = msg.quaternion.x
        self._actual_quaternion["y"] = msg.quaternion.y
        self._actual_quaternion["z"] = msg.quaternion.z
        self._actual_quaternion["w"] = msg.quaternion.w

    def compute_forward_movement_error(self):
        """Compute the forward movement error's magnitude and direction."""
        direction_to_compensate = ""
        error = float
        if self._robot_motion == "forward":
            if self._target_quaternion["z"] > 0 and self._actual_quaternion["z"] > 0:
                error = self._target_quaternion["z"] - self._actual_quaternion["z"]
                if error > 0:
                    direction_to_compensate = "CW"
                else:
                    direction_to_compensate = "CCW"
            elif self._target_quaternion["z"] > 0 and self._actual_quaternion["z"] < 0:
                if self._target_quaternion["z"] > 0.5:
                    direction_to_compensate = "CW"
                    error = abs(1 - self._target_quaternion["z"] + self._actual_quaternion["z"])
                else:
                    direction_to_compensate = "CCW"
                    error = self._target_quaternion["z"] - self._actual_quaternion["z"]

            elif self._target_quaternion["z"] < 0 and self._actual_quaternion["z"] > 0:
                if self._target_quaternion["z"] > -0.5:
                    direction_to_compensate = "CCW"
                    error = abs(1 + self._target_quaternion["z"] - self._actual_quaternion["z"])
                else:
                    direction_to_compensate = "CW"
                    error = abs(self._target_quaternion["z"] - self._actual_quaternion["z"])
            else:
                error = self._target_quaternion["z"] - self._actual_quaternion["z"]
                if error > 0:
                    direction_to_compensate = "CCW"
                else:
                    direction_to_compensate = "CW"
                error = abs(error)
        return direction_to_compensate, error

    def compute_stabilised_speed(self, motor_id, error, direction):
        """Compute the stabilised speed from the controller."""
        if direction == "CCW":
            return int(self._motor_actual_speed[motor_id] + self._P*error)
        else:
            return int(self._motor_actual_speed[motor_id] + self._P*error)

    def update_stabilised_speed(self):
        """Update the stabilised speed."""
        if self._robot_motion == "forward":
            direction, error = self.compute_forward_movement_error()
            self._motor_stabilised_speed["1"] = self._motor_actual_speed["1"]
            self._motor_stabilised_speed["2"] = self.compute_stabilised_speed(2, error, direction)
            self._motor_stabilised_speed["3"] = self._motor_actual_speed["3"]
            self._motor_stabilised_speed["4"] = self.compute_stabilised_speed(4, error, direction)
            self._motor_stabilised_speed["5"] = self._motor_actual_speed["5"]
            self._motor_stabilised_speed["6"] = self._motor_actual_speed["6"]
            self._motor_stabilised_speed["7"] = self._motor_actual_speed["7"]
            self._motor_stabilised_speed["8"] = self._motor_actual_speed["8"]

    def publish_stabilised_speed(self):
        """Publish the stabilised motor speed."""
        self.update_stabilised_speed()
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
