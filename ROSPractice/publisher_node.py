#!/usr/bin/env
import rospy
from std_msgs.msg import String
import serial

#ports = serial.tools.list_ports.comports()
#serialInst = serial.Serial()

#portList = []

def talk_to_me():
    pub = rospy.Publisher('talking_topic', String, queue_size=10)
    rospy.init_node('publisher_node', anonymous = True)
    rate = rospy.Rate(1)
    rospy.loginfo("Publisher Node Started, now publishing messages")
    ser = serial.Serial('/dev/ttyACM0', 115200)
    while not rospy.is_shutdown():
        #packet = serialInst.readline()
        #msg = packet.decode('utf')
        s = ser.readline()
        msg = s.decode('utf')
        #msg = "Hello World - %s" % rospy.get_time()
        pub.publish(msg)
        rate.sleep()
    ser.close()

if __name__ == '__main__':
    try:
        talk_to_me()
    except rospy.ROSInterruptException:
        pass