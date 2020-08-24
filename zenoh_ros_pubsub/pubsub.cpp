// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

class PubSub : public rclcpp::Node
{
public:
  size_t tx_count;

  PubSub()
  : Node("zenoh_ros_pubsub",
      rclcpp::NodeOptions()
        .start_parameter_services(false)
        .enable_rosout(false)),
    tx_count(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&PubSub::topic_callback, this, _1));
  }

  void publish()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(tx_count++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    fflush(stdout);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "ROS INIT DONE" << std::endl;

  auto pubsub_node = std::make_shared<PubSub>();

  rclcpp::Time t_last_publish = pubsub_node->get_clock()->now();

  while (rclcpp::ok() && pubsub_node->tx_count < 2)
  {
    rclcpp::spin_some(pubsub_node);
    rclcpp::sleep_for(std::chrono::milliseconds(250));

    const rclcpp::Time t = pubsub_node->get_clock()->now();
    if ((t - t_last_publish).seconds() > 1.0)
    {
      t_last_publish = t;
      pubsub_node->publish();
    }
  }
  rclcpp::shutdown();
  return 0;
}
