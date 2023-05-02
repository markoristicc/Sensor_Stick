import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial

class TalkerNode(Node):
	def __init__(self):
		super().__init__("talker_node")
		self.publisher_ = self.create_publisher(String, 'topic', 100)
		timer_period = 5
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.count = 0
		
	def timer_callback(self):
		ser = serial.Serial('/dev/ttyACM0', 115200)
		s = ser.readline()
		msg = String()
		#msgd = s.decode('utf')
		msg.data = s.decode('utf')
		self.publisher_.publish(msg)
		self.count += 1
		self.get_logger().info(f"Publishing {msg.data}")
		

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

