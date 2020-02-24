import rospy
from sauvc2020_msgs.msg import MotorSpeed


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ": I heard \n%s", data)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter_resp", MotorSpeed, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
