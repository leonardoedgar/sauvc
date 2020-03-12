#!/usr/bin/env python
import rospy
from sauvc_control import PIDController
from sauvc_msgs.msg import ThrustersSpeed
import pytest
import math


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
    w, x, y, z = 0, 0.1, 0.3, 0.5
    alpha, beta, gamma = 0, 45, 90
    mocked_get_euler_angle_from_quat = \
        mocker.patch('sauvc_control.pid_controller.PIDController.get_euler_angle_from_quat')
    PIDController.get_euler_angle_from_quat.return_value = alpha, beta, gamma
    msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z = w, x, y, z
    pid_controller._get_quaternion_data(msg)
    assert mocked_get_euler_angle_from_quat.call_args[0] == (w, x, y, z)
    assert pid_controller._actual_euler == {"alpha": alpha, "beta": beta, "gamma": gamma}


def test_get_euler_angle_from_quaternion():
    """Test that PID Controller is able to get euler angle from quaternion."""
    w, x, y, z = 0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)
    desired_alpha, desired_beta, desired_gamma = 90, 0, 180
    tolerable_error = 0.1
    actual_alpha, actual_beta, actual_gamma = PIDController.get_euler_angle_from_quat(w, x, y, z)
    assert (math.fabs(actual_alpha - desired_alpha), math.fabs(actual_beta - desired_beta),
            math.fabs(actual_gamma - desired_gamma)) <= (tolerable_error, tolerable_error, tolerable_error)


def test_update_motion_data(mocker):
    """Test that PID Controller is able to update its motion data."""
    pid_controller = get_sample_pid_controller()
    msg, motion_data = get_mocked_motion_data(mocker)
    pid_controller._update_motion_data(msg)
    assert pid_controller._auv_motion == motion_data["motion"]
    assert pid_controller._thrusters_actual_speed == motion_data["thrusters_speed"]


@pytest.mark.parametrize("actual_euler, target_euler, error, direction_to_compensate", [
    ({"alpha": 0, "beta": 0, "gamma": 10}, {"alpha": 0, "beta": 0, "gamma": 0}, 10, "CW"),
    ({"alpha": 0, "beta": 0, "gamma": -10}, {"alpha": 0, "beta": 0, "gamma": 0}, 10, "CCW"),
    ({"alpha": 0, "beta": 0, "gamma": 100}, {"alpha": 0, "beta": 0, "gamma": 90}, 10, "CW"),
    ({"alpha": 0, "beta": 0, "gamma": 80}, {"alpha": 0, "beta": 0, "gamma": 90}, 10, "CCW"),
    ({"alpha": 0, "beta": 0, "gamma": 170}, {"alpha": 0, "beta": 0, "gamma": 180}, 10, "CCW"),
    ({"alpha": 0, "beta": 0, "gamma": -170}, {"alpha": 0, "beta": 0, "gamma": 180}, 10, "CW"),
    ({"alpha": 0, "beta": 0, "gamma": -100}, {"alpha": 0, "beta": 0, "gamma": -90}, 10, "CCW"),
    ({"alpha": 0, "beta": 0, "gamma": -80}, {"alpha": 0, "beta": 0, "gamma": -90}, 10, "CW"),
])
def test_compute_forward_movement_error(actual_euler, target_euler, error, direction_to_compensate):
    """Test that PID Controller is able to compute forward movement error."""
    pid_controller = get_sample_pid_controller()
    pid_controller._actual_euler, pid_controller._target_euler = actual_euler, target_euler
    allowable_tolerance = 0.1
    actual_dir_to_compensate, actual_error = pid_controller._compute_forward_movement_error()
    assert actual_dir_to_compensate == direction_to_compensate
    print("actual error: " + str(actual_error))
    assert math.fabs(actual_error - error) <= allowable_tolerance


@pytest.mark.parametrize("thruster_id, error, direction, stabilising_speed", [
    ("1", 3, "CCW", 75),
    ("2", 2, "CW", 50)
])
def test_compute_stabilised_speed(thruster_id, error, direction, stabilising_speed):
    """Test that PID Controller is able to compute stabilised speed correctly."""
    pid_controller, thrusters_speed = get_pid_controller_with_mocked_actual_thrusters_speed()
    assert pid_controller._compute_stabilised_speed(thruster_id, error, direction) == int(thrusters_speed[thruster_id])\
        + stabilising_speed


def test_update_stabilised_speed(mocker):
    """Test that PID Controller is able to update its stabilised speed."""
    pid_controller, thrusters_speed = get_pid_controller_with_mocked_actual_thrusters_speed()
    pid_controller._compute_forward_movement_error = mocker.MagicMock()
    pid_controller._compute_forward_movement_error.return_value = "CW", 0.01
    pid_controller._compute_stabilised_speed = mocker.MagicMock()
    pid_controller._compute_stabilised_speed.side_effect = [thrusters_speed["1"], thrusters_speed["2"]]
    pid_controller._auv_motion = "forward"
    pid_controller._update_stabilised_speed()
    assert pid_controller._thrusters_stabilised_speed == thrusters_speed


def get_sample_pid_controller():
    """Setup function to create PID Controller"""
    return PIDController(rospy.Publisher('/thrusters/stabilised_speed', ThrustersSpeed, queue_size=10))


def get_mocked_motion_data(mocker):
    """A function to get mocked motion data"""
    msg = mocker.MagicMock()
    motion_data = {
        "motion": "forward",
        "thrusters_speed": get_mocked_thrusters_speed()
    }
    msg.motion, \
    msg.thrusters_speed.thruster_id1_speed, \
    msg.thrusters_speed.thruster_id2_speed, \
    msg.thrusters_speed.thruster_id3_speed, \
    msg.thrusters_speed.thruster_id4_speed, \
    msg.thrusters_speed.thruster_id5_speed, \
    msg.thrusters_speed.thruster_id6_speed, \
    msg.thrusters_speed.thruster_id7_speed, \
    msg.thrusters_speed.thruster_id8_speed = motion_data["motion"], \
                                             motion_data["thrusters_speed"]["1"], \
                                             motion_data["thrusters_speed"]["2"], \
                                             motion_data["thrusters_speed"]["3"], \
                                             motion_data["thrusters_speed"]["4"], \
                                             motion_data["thrusters_speed"]["5"], \
                                             motion_data["thrusters_speed"]["6"], \
                                             motion_data["thrusters_speed"]["7"], \
                                             motion_data["thrusters_speed"]["8"]
    return msg, motion_data


def get_pid_controller_with_mocked_actual_thrusters_speed():
    """Setup PID Controller with mocked actual thrusters speed"""
    pid_controller = get_sample_pid_controller()
    thrusters_speed = get_mocked_thrusters_speed()
    pid_controller._thrusters_actual_speed = thrusters_speed
    return pid_controller, thrusters_speed


def get_mocked_thrusters_speed():
    """A function to get mocked thrusters speed."""
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
    pid_controller._thrusters_stabilised_speed = sample_stabilised_speed
    pid_controller._update_stabilised_speed = mocker.MagicMock()
    return pid_controller, sample_stabilised_speed
