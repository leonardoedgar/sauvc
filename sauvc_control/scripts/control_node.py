#!/usr/bin/env python
import rospy
from sauvc_control.pid_controller import PIDController
from sauvc_msgs.msg import ThrustersSpeed

if __name__ == '__main__':
    rospy.init_node("pid_controller", anonymous=True)
    pid_controller = PIDController(rospy.Publisher("/thrusters/stabilised_speed", ThrustersSpeed, queue_size=10))
    while not rospy.is_shutdown():
        pid_controller.publish_stabilised_speed()
        print(pid_controller._thrusters_actual_speed)
        rospy.sleep(1)
