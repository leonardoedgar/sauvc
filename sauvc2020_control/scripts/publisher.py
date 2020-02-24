import rospy
from sauvc2020_msgs.msg import MotorSpeed


def talker():
    pub = rospy.Publisher('chatter', MotorSpeed, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pub.publish(10, 20, 30, 40, 50, 60, 70, 80)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
