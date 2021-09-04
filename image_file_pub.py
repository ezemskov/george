# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import sys

class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self, file_path, ros_topic_name):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')
         
    self.img = cv2.imread(file_path)
    if (self.img is None): 
      self.get_logger().error("Unable to open '{}'".format(file_path))
      return

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, ros_topic_name, 10)
      
    # We will publish a message every 0.1 seconds
    timer_period = 1.0  # seconds
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    # Publish the image.
    # The 'cv2_to_imgmsg' method converts an OpenCV
    # image to a ROS 2 image message
    self.publisher_.publish(self.br.cv2_to_imgmsg(self.img))
 
    # Display the message on the console
    self.get_logger().info('Publishing image')
  

def main():
  args = sys.argv

  if (len(args) < 3):
      print("Commandline : {0} file_path ros_topic_name".format(args[0]))
      return

  # Initialize the rclpy library
  rclpy.init()
  
  # Create the node
  image_publisher = ImagePublisher(args[1], args[2])
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()