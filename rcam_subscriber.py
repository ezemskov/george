import logging            
import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image

class RcamSubscriberNode(Node):

    def __init__(self, topicName):
        super().__init__('rcam_subscriber')

        self.sub = self.create_subscription(Image, topicName, self.callback, 1)
        logging.debug('RcamSubscriberNode subscribed')

    def callback(self, msg):
      ts_ = msg.header.stamp;
      ts = ts_.sec + 10E-9 * ts_.nanosec;
      logging.debug('Received Image: "%s" ts [%.9f] dims [%dx%d] step %d'.format( 
         msg.header.frame_id, ts,
         msg.width, msg.height, msg.step
      ));


def main():
    loggerHandlers = [
        logging.StreamHandler(stream=sys.stdout)
    ] 
    logging.basicConfig(handlers=loggerHandlers, level=logging.DEBUG)

    rclpy.init()
    node = RcamSubscriberNode("/depth/image_rect_raw")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
