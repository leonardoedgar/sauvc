#!/usr/bin/env python
import rospy
from sauvc2020_control import PIDController
from sauvc2020_msgs.msg import MotorSpeed
import pytest


def test_publish_stabilised_speed(mocker):
    """Test that PID Controller is able to publish the stabilised speed."""
    pid_controller, stabilised_speed = get_pid_controller_with_mocked_stabilised_speed(mocker)
    pid_controller.publish_stabilised_speed()
    assert pid_controller._stabilised_speed_publisher.publish.call_args[0] == (
        stabilised_speed["1"],
        stabilised_speed["2"],
        stabilised_speed["3"],
        stabilised_speed["4"],
        stabilised_speed["5"],
        stabilised_speed["6"],
        stabilised_speed["7"],
        stabilised_speed["8"]
    )


def test_get_quarternion_data(mocker):
    """Test that PiD Controller is able to get quaternion data."""
    pid_controller = get_sample_pid_controller()
    msg = mocker.MagicMock()
    w, x, y, z = 0.4, 0.7, 0.6, 0.5
    msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z = w, x, y, z
    pid_controller._get_quaternion_data(msg)
    assert pid_controller._actual_quaternion == {"w": w, "x": x, "y": y, "z": z}


def test_update_motion_data(mocker):
    """Test that PID Controller is able to update its motion data."""
    pid_controller = get_sample_pid_controller()
    msg, motion_data = mocked_motion_data(mocker)
    pid_controller._update_motion_data(msg)
    assert pid_controller._robot_motion == motion_data["motion"]
    assert pid_controller._motor_actual_speed == motion_data["motors_speed"]


def test_compute_forward_movement_error():
    """Test that PID Controller is able to compute forward movement error."""
    assert True


@pytest.mark.parametrize("motor_id, error, direction, stabilised_speed", [
    ("1", 0.2, "CCW", 1520),
    ("5", 0.3, "CW", 1530)
])
def test_compute_stabilised_speed(motor_id, error, direction, stabilised_speed):
    """Test that PID Controller is able to compute stabilised speed correctly."""
    pid_controller = get_sample_pid_controller()
    assert pid_controller._compute_stabilised_speed(motor_id, error, direction) == stabilised_speed


def test_update_stabilised_speed(mocker):
    """Test that PID Controller is able to update its stabilised speed."""
    pid_controller, motors_speed = get_pid_controller_with_mocked_actual_motors_speed()
    pid_controller._compute_forward_movement_error = mocker.MagicMock()
    pid_controller._compute_forward_movement_error.return_value = "CW", 0.1
    pid_controller._compute_stabilised_speed = mocker.MagicMock()
    pid_controller._compute_stabilised_speed.side_effect = [motors_speed["2"], motors_speed["4"]]
    pid_controller._robot_motion = "forward"
    pid_controller._update_stabilised_speed()
    assert pid_controller._motor_stabilised_speed == motors_speed


def get_sample_pid_controller():
    """Setup function to create PID Controller"""
    return PIDController(rospy.Publisher('/motor/stabilised_speed', MotorSpeed, queue_size=10))


def mocked_motion_data(mocker):
    """A function to get mocked motion data"""
    msg = mocker.MagicMock()
    motion_data = {
        "motion": "forward",
        "motors_speed": get_mocked_motors_speed()
    }
    msg.motion, \
        msg.motors_speed.motor_id1_speed, \
        msg.motors_speed.motor_id2_speed, \
        msg.motors_speed.motor_id3_speed, \
        msg.motors_speed.motor_id4_speed, \
        msg.motors_speed.motor_id5_speed, \
        msg.motors_speed.motor_id6_speed, \
        msg.motors_speed.motor_id7_speed, \
        msg.motors_speed.motor_id8_speed = motion_data["motion"], \
                                           motion_data["motors_speed"]["1"], \
                                           motion_data["motors_speed"]["2"], \
                                           motion_data["motors_speed"]["3"], \
                                           motion_data["motors_speed"]["4"], \
                                           motion_data["motors_speed"]["5"], \
                                           motion_data["motors_speed"]["6"], \
                                           motion_data["motors_speed"]["7"], \
                                           motion_data["motors_speed"]["8"]
    return msg, motion_data


def get_pid_controller_with_mocked_actual_motors_speed():
    """Setup PID Controller with mocked actual motors speed"""
    pid_controller = get_sample_pid_controller()
    motors_speed = get_mocked_motors_speed()
    pid_controller._motor_actual_speed = motors_speed
    return pid_controller, motors_speed


def get_mocked_motors_speed():
    """A function to get mocked motors speed."""
    return {
        "1": 1400,
        "2": 1500,
        "3": 1600,
        "4": 1650,
        "5": 1500,
        "6": 1800,
        "7": 1750,
        "8": 1550
    }


def get_pid_controller_with_mocked_stabilised_speed(mocker):
    """A function to get PID Controller with mocked stabilised speed."""
    mocker.patch('rospy.Publisher')
    pid_controller = get_sample_pid_controller()
    sample_stabilised_speed = {
        "1": 1500,
        "2": 1600,
        "3": 1700,
        "4": 1750,
        "5": 1650,
        "6": 1600,
        "7": 1750,
        "8": 1600

    }
    pid_controller._motor_stabilised_speed = sample_stabilised_speed
    pid_controller._update_stabilised_speed = mocker.MagicMock()
    return pid_controller, sample_stabilised_speed
