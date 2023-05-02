import rclpy
from rclpy.node import Node
from array import array

from std_msgs.msg import Int32MultiArray
import serial

class TalkerNode(Node):
	def __init__(self):
		super().__init__("talker_node")
		self.publisher_ = self.create_publisher(Int32MultiArray, 'topic', 100)
		timer_period = 5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.count = 0
		
	def timer_callback(self):
		self.ser = serial.Serial('/dev/ttyACM0', 115200)
		msg = Int32MultiArray()
		# msg.data = s.decode('utf')
		# self.publisher_.publish(msg)
		#ok so instead it should be
		msg = Int32MultiArray()
		datalist = [] #i think list then conversion is inefficient
		USBpacket = self.ser.read(336) #size of the USB Packet
		if hex(USBpacket[0]) != 0x45: #or '8A?
			#invalid packet, don't publish anything
			self.publisher_.publish("") #idk anything would work rlly
		else:
			datalist.append(int.from_bytes(USBpacket[2:6]))
			#datalist[0] now contains timestamp
			if USBpacket[1] & (1<<5): #accelerometer
				for i in range(6,12,2):
					datalist.append(int.from_bytes(USBpacket[i: i + 2]))
				#datalist[1:4] now Accelerometer data
			if USBpacket[1] & (1<<4): #gyroscope
				for i in range(12,18,2):
					datalist.append(int.from_bytes(USBpacket[i: i + 2]))
				#datalist[4:7] now gyroscope data
			if USBpacket[1] & (1<<3): #magnetometer
				for i in range(18,24,2):
					datalist.append(int.from_bytes(USBpacket[i: i + 2]))
				#datalist[7:10] now magnemoter data
			if USBpacket[1] & (1<<2): #airpeed
				datalist.append(int.from_bytes(USBpacket[24:26]))
				#datalist[10] now DP data
			if USBpacket[1] & (1<<1): #barometer
				for i in range(26,34,4):
					datalist.append(int.from_bytes(USBpacket[i: i + 4]))
				#datalist[11:13] now barometer data
			if USBpacket[1] & 1: #gps
				for i in range(34,42,4):
					datalist.append(int.from_bytes(USBpacket[i: i + 4]))
				#datalist[13:15] now GPs data
		msg.data = array("i", datalist) #don't know if this is correct really
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

