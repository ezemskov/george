// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class RcamSubscriber : public rclcpp::Node
{
public:
  RcamSubscriber(const std::string& topicName)
  : Node("rcam_subscriber")
  {
    subscriptions_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
          topicName, 10, std::bind(&RcamSubscriber::ImageCallback, this, _1)
    ));

    subscriptions_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
          topicName, 10, std::bind(&RcamSubscriber::PointCloud2Callback, this, _1)
    ));
  }

private:
  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    const auto& ts_ = msg->header.stamp;
    const double ts = ts_.sec + 10E-9 * ts_.nanosec;
    RCLCPP_INFO(this->get_logger(), "Received Image: '%s' ts [%.9f] dims [%dx%d] step %d %s", 
      msg->header.frame_id.c_str(), ts,
      msg->width, msg->height,
      msg->step,
      (msg->is_bigendian ? "BE":"LE")
    );
  }

  void PointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    const auto& ts_ = msg->header.stamp;
    const double ts = ts_.sec + 10E-9 * ts_.nanosec;
    RCLCPP_INFO(this->get_logger(), "Received PointCloud2: '%s' ts [%.9f] dims [%dx%d] step %d/%d %s dense %d", 
      msg->header.frame_id.c_str(), ts,
      msg->width, msg->height,
      msg->point_step, msg->row_step,
      (msg->is_bigendian ? "BE":"LE"),
      int(msg->is_dense)
    );
  }

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

int main(int argc, char * argv[])
{
  const std::string topicName((argc > 1) ? argv[1] : "");

  try
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RcamSubscriber>(topicName));
    rclcpp::shutdown();
  }
  catch (std::exception& ex)
  {
      std::cerr << ex.what() << std::endl;
  }
  return 0;
}
