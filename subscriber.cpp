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

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber()
  : Node("subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "rttest_sample", std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    struct timespec time_start={0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);
    static double last_sec = 0;
    static double last_nsec = 0;
    double current_sec = time_start.tv_sec;
    double current_nsec = time_start.tv_nsec;

    double interval = 0;
    static int count = 0;

    if(last_sec == 0)
    {
       last_sec = current_sec;
       last_nsec = current_nsec;
       return;
    }

    count++;
    interval = (current_sec - last_sec) + ((current_nsec - last_nsec)/1000000000);
    if(interval >= 1.0)
    {
      double fps = count/interval;
      last_sec = current_sec;
      last_nsec = current_nsec;
      RCLCPP_INFO(this->get_logger(), "[%d.%d] fps[%.3f] size[%d]", time_start.tv_sec, time_start.tv_nsec, fps, msg->data.size())
      count = 0;
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}
