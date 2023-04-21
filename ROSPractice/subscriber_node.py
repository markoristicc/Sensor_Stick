#!/usr/bin/env
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("RECEIVED DATA: %s", data.data)

def listener():
    rospy.init_node("Subscriber_Node", anonymous = True)
    rospy.Subscriber('talking_topic', String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass