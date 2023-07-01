import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from ros_map_server_interfaces.msg import Example
from map_server import example_function

class MapServer(Node):
    def __init__(self):
        super().__init__("ros_map_server")
        self.get_logger().info('Starting')
        
        #Params
        self.declare_parameter('rate', 1)
        rate = self.get_parameter('rate').get_parameter_value().integer_value

        #Publisher
        #self.pub = self.create_publisher(MSG, TOPIC, QUEUE_SIZE)

        #Subscriber
        #self.subscription = self.create_subscription(
        #    MSG,
        #    TOPIC,
        #    CALLBACK,
        #    QUEUE_SIZE)
        #self.subscription  # prevent unused variable warning


        #Timer
        self.timer = self.create_timer(1/rate, self.loop)

    def loop(self):
        '''
        main loop
        '''
        self.get_logger().info(str(Example()))
        example_function()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MapServer()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()