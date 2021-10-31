# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images

from functools import partial #bind argument value

from threading import Thread

import sys

class ImageSubscriber(Node):
  def __init__(self):
    # Initiate the Node class's constructor and give it a name
    super().__init__('ImageSubscriber')

    self._callbacks = {}

  def subscribe(self, ros_topic_name, callback):
      self._callbacks[ros_topic_name] = callback
      subscriber_callback_bound = partial(self.subscriber_callback, ros_topic_name_=ros_topic_name)
      
      # Create the subscriber. This subscriber will receive an Image from the topic. The queue size is 10 messages.
      self.create_subscription(
            Image, 
            ros_topic_name, 
            subscriber_callback_bound, 
            qos_profile_sensor_data)

      self.get_logger().info('Subscribed on ' + ros_topic_name)


  def subscriber_callback(self, data, ros_topic_name_): 
    try:
      # Convert ROS Image message to OpenCV/numpy image
      imageNumpy = CvBridge().imgmsg_to_cv2(data)
      self.get_logger().info('Received video frame dimensions {0}x{1}'.format(imageNumpy.shape[1], imageNumpy.shape[0]))
      
      callback = self._callbacks.get(ros_topic_name_)
      if (callback is not None):
        callback(imageNumpy)
    except CvBridgeError as exc:
          self.get_logger().error(exc)

class ImageSubscriberWrapper:

    def __init__(self):
        rclpy.init()
        self.node = ImageSubscriber()
        self.thread = Thread(target=self._threadFunc)
        self.thread.start()

    def subscribe(self, ros_topic_name, callback = None):
        self.node.subscribe(ros_topic_name, callback)

    def _threadFunc(self):
        rclpy.spin(self.node)
        rclpy.shutdown()


def callback(self, data):
    # Display the message on the console
    self.get_logger().info('Receiving video frame')

def main():
    args = sys.argv

    if (len(args) < 2):
        print("Commandline : {0} ros_topic_name".format(args[0]))
        return

    sub = ImageSubscriberWrapper()
    sub.subscribe(args[1], callback)

if __name__ == '__main__':
    main()
