import rclpy
import struct
from rclpy.node import Node
from array import array

from stick_message.msg import StickData
import serial

class TalkerNode(Node):
	def __init__(self):
		super().__init__("talker_node")
		self.publisher_ = self.create_publisher(StickData, 'topic', 100)
		timer_period = 5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.count = 0
		
	def timer_callback(self):
		self.ser = serial.Serial('/dev/ttyACM0', 115200)
		msg = StickData()
		datalist = [] #i think list then conversion is inefficient
		USBpacket = self.ser.read(72) #size of the USB Packet
		for i in USBpacket:
			print (type (i),i)
		print (USBpacket[0])
		print (hex(USBpacket[0]))
		if hex(USBpacket[0]) == '0x45': 
			#BASE SET UP FOR MESSAGE SUBCOMPONENTS
			#imu 
			timestamp = int.from_bytes(USBpacket[4:8], byteorder = 'little')
			msg.imu.header.stamp.sec = timestamp>>3
			'''msg.imu.header.stamp.nsec = (timestamp%1000)<<6
			msg.imu.orientation = 0
			msg.imu.orientation_covariance[0] = -1 #addressing might be incorrect
			msg.imu.linear_acceleration_covariance = [0,0,0,0,0,0,0,0,0] #this might be unnecessary
			msg.imu.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]
			#magnetometer
			msg.magfield.header.stamp.sec = msg.imu.header.stamp.sec
			msg.magfield.header.stamp.nsec = msg.imu.header.stamp.nsec
			msg.magfield.magnetic_field_covariance = [0,0,0,0,0,0,0,0,0]
			#gps
			msg.gps.header.stamp.sec = msg.imu.header.stamp.sec
			msg.gps.header.stamp.nsec = msg.imu.header.stamp.nsec'''
			if USBpacket[1] & (1<<5): #accelerometer
				for i in range(8,20,4):
					print (USBpacket[i: i + 4])
					datalist.append(struct.unpack('<f',USBpacket[i: i + 4])[0])
				#datalist[0:3] now Accelerometer data, compose packet
				msg.imu.linear_acceleration.x = float(datalist[0])
				msg.imu.linear_acceleration.y = float(datalist[1])
				msg.imu.linear_acceleration.z = float(datalist[2])
			else: 
				temp = 0
				#need to null populate message/not send out message, possibly for all
			if USBpacket[1] & (1<<4): #gyroscope
				for i in range(20,32,4):
					print (USBpacket[i: i + 4])
					datalist.append(struct.unpack('<f',USBpacket[i: i + 4])[0])
				#datalist[3:6] 
				msg.imu.angular_velocity.x = float(datalist[3])
				msg.imu.angular_velocity.y = float(datalist[4])
				msg.imu.angular_velocity.z = float(datalist[5])
				
			if USBpacket[1] & (1<<3): #magnetometer
				for i in range(32,44,4):
					print (USBpacket[i: i + 4])
					datalist.append(struct.unpack('<f',USBpacket[i: i + 4])[0])
				#datalist[6:9] now magnemoter data, compose packet
				msg.magfield.magnetic_field.x = float(datalist[6])
				msg.magfield.magnetic_field.y = float(datalist[7])
				msg.magfield.magnetic_field.z = float(datalist[8])
				
			if USBpacket[1] & (1<<2): #airspeed
				for i in range(44, 52, 4):
					datalist.append(struct.unpack('<f',USBpacket[i: i + 4])[0])
				# datalist[10:12] now airspeed data
				
			if USBpacket[1] & (1<<1): #barometer
				for i in range(52,64,4):
					datalist.append(int.from_bytes(USBpacket[i: i + 4], byteorder = 'little'))
				#datalist[12:15] now barometer data
				
			if USBpacket[1] & 1: #gps
				for i in range(64,72,4):
					datalist.append(int.from_bytes(USBpacket[i: i + 4], byteorder = 'little'))
				#datalist[15:17] now GPs data
				msg.gps.latitude = float(datalist[15])
				msg.gps.longitude = float(datalist[16])
		print (datalist)
		self.publisher_.publish(msg)
		self.count += 1
		self.get_logger().info(f"Publishing packet {self.count}")

def main(args = None):
	rclpy.init(args=args)
	
	# create node
	talkerNode = TalkerNode()
	
	# use node
	rclpy.spin(talkerNode)
	# destroy node
	talkerNode.destroy_node()
	
	rclpy.shutdown()
	

if __name__ == "__main__":
	main()
