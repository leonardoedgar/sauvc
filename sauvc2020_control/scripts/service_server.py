import rospy
from sauvc2020_msgs.srv import RobotMotion


def handle_add_two_ints(req, resp):
    resp.answer = True
    print "Robot move direction: [%s]"%req.motion
    return resp


def add_two_ints_server():
    rospy.init_node('sample_server')
    rospy.Service('/direction', RobotMotion, handle_add_two_ints)
    print "Ready to receive robot motion."
    rospy.spin()


if __name__ == "__main__":
    add_two_ints_server()
