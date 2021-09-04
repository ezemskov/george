# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images

from threading import Thread

class ImageSubscriber(Node):
  def __init__(self, ros_topic_name, callback_):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    self.callback = callback_
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.sub = self.create_subscription(
      Image, 
      ros_topic_name, 
      self.subscriber_callback, 
      qos_profile_sensor_data)
    self.get_logger().info('Subscribed on ' + ros_topic_name)

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def subscriber_callback(self, data): 
    try:
      # Convert ROS Image message to OpenCV/numpy image
      imageNumpy = CvBridge().imgmsg_to_cv2(data)
      self.get_logger().info('Received video frame dimensions {0}x{1}'.format(imageNumpy.shape[1], imageNumpy.shape[0]))
      if (self.callback is not None):
        self.callback(imageNumpy)
    except CvBridgeError as exc:
          self.get_logger().error(exc)

class ImageSubscriberWrapper:
    def __init__(self, ros_topic_name, callback = None):
        rclpy.init()
        self.node = ImageSubscriber(ros_topic_name, callback)
        self.thread = Thread(target=self._threadFunc)
        self.thread.start()

    def _threadFunc(self):
        rclpy.spin(self.node)
        rclpy.shutdown()

