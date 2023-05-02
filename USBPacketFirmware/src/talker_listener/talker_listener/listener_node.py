import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray


class ListenerNode(Node):
	def __init__(self):
		super().__init__("talker_node")
		self.subscription = self.create_subscription(
				Int32MultiArray,
				'topic',
				self.listener_callback,
				10
		)
		
	def listener_callback(self, msg):
		self.get_logger().info(f"Received {msg.data}")
		

def main(args = None):
	rclpy.init(args=args)
	
	# create node
	listenerNode = ListenerNode()
	
	# use node
	rclpy.spin(listenerNode)
	
	# destroy node
	listenerNode.destroy_node()
	rclpy.shutdown()
	

if __name__ == "__main__":
	main()

