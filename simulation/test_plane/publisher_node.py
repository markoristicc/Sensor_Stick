#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Temperature, Imu
import serial
import struct

def publish_imu(timer_event):
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    stick = serial.Serial('/dev/ttyACM0', 115200)
    # stick.open()
    dataList = []
    USBpacket = bytearray(stick.read(72))
    # print(((USBpacket)))
    # for s in USBpacket:
    #     try:
    #         print(hex(s))
    #     except:
    #         print("not a byte")
    #         # print(s)
    #         pass
        
    if(hex(USBpacket[0]) == '0x45'):
        # print("Here")
        if(USBpacket[1] & (1<<5)): 
            for i in range(8, 20, 4):
                    dataList.append(struct.unpack('<f', USBpacket[i:i+4])[0])
            imu_msg.linear_acceleration.x = dataList[0]
            imu_msg.linear_acceleration.y = dataList[1]
            imu_msg.linear_acceleration.z = dataList[2]
        if(USBpacket[1] & (1<<4)): 
            for i in range(20, 32, 4):
                    dataList.append(struct.unpack('<f', USBpacket[i:i+4])[0])
            imu_msg.angular_velocity.x = dataList[3]
            imu_msg.angular_velocity.y = dataList[4]
            imu_msg.angular_velocity.z = dataList[5]
        # print(dataList)
    # print("Packet time")

    imu_msg.header.stamp = rospy.Time.now()

    imu_pub.publish(imu_msg)
    

if __name__ == '__main__':
    try:
        rospy.loginfo("Publisher Node Started, now publishing messages")
        rospy.init_node('imu_node')
        IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

        imu_pub = rospy.Publisher('imu/data_raw', Imu,queue_size=100)
        imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
