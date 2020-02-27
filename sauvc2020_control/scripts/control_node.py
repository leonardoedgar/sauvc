#!/usr/bin/env python
import rospy
from sauvc2020_control.pid_controller import PIDController
from sauvc2020_msgs.msg import MotorSpeed

if __name__ == '__main__':
    rospy.init_node("pid_controller", anonymous=True)
    pid_controller = PIDController(rospy.Publisher("/motor/stabilised_speed", MotorSpeed, queue_size=10))
    while not rospy.is_shutdown():
        pid_controller.publish_stabilised_speed()
        rospy.sleep(1)
