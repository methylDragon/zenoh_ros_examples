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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("wow"), count_(0)
  {
    std::cout << "STARTING NODE" << std::endl;

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    std::cout << "PUBLISHER CREATED" << std::endl;

    rclcpp::sleep_for(std::chrono::seconds(1));

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));

    while (count_ < 3)
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);

      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    // timer_ = this->create_wall_timer(
    //   500ms, std::bind(&MinimalPublisher::timer_callback, this));

  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    fflush(stdout);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "ROS INIT DONE" << std::endl;

  rclcpp::spin_some(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();

  return 0;
}
