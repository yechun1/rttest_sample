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
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>
#include <random>



using namespace std::chrono_literals;


class Publisher : public rclcpp::Node
{
public:
  Publisher()
  : Node("publisher"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("rttest_sample");
    timer_ = this->create_wall_timer(
      50ms, std::bind(&Publisher::timer_callback, this));
  }

private:

  struct Point
  {
    float x, y, z;
    Point(float x, float y, float z)
      : x(x), y(y), z(z) {}
  };


  void timer_callback()
  {
    struct timespec time={0, 0};
    clock_gettime(CLOCK_REALTIME, &time);
//    RCLCPP_INFO(this->get_logger(), "A:[%d.%d]", time.tv_sec, time.tv_nsec)

    auto message = createPointCloud2WithPoints();
//    RCLCPP_INFO(this->get_logger(), "B:[%d.%d]", time.tv_sec, time.tv_nsec)

    publisher_->publish(message);
//    RCLCPP_INFO(this->get_logger(), "C:[%d.%d]", time.tv_sec, time.tv_nsec)

    clock_gettime(CLOCK_REALTIME, &time);
    RCLCPP_INFO(this->get_logger(), "[%d.%d] Publishing:(%d,%d), size(%d)", time.tv_sec, time.tv_nsec,
                message->width, message->height, message->data.size())
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  size_t count_;


  sensor_msgs::msg::PointCloud2::SharedPtr createPointCloud2WithPoints()
  {
    static std::vector<Point> points = {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}};

    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header = std_msgs::msg::Header();
    cloud->header.stamp = rclcpp::Clock().now();

    cloud->is_bigendian = false;
    cloud->is_dense = true;

    cloud->width = 640;
    cloud->height = 48;

    cloud->fields.resize(3);
    cloud->fields[0].name = "x";
    cloud->fields[1].name = "y";
    cloud->fields[2].name = "z";

    sensor_msgs::msg::PointField::_offset_type offset = 0;
    for (uint32_t i = 0; i < cloud->fields.size(); ++i, offset += sizeof(float)) {
      cloud->fields[i].count = 1;
      cloud->fields[i].offset = offset;
      cloud->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud->point_step = offset;
    cloud->row_step = cloud->point_step * cloud->width;
    cloud->data.resize(cloud->row_step * cloud->height);

    auto floatData = reinterpret_cast<float *>(cloud->data.data());
      std::random_device seeder;
      std::mt19937 gen(seeder());
      std::uniform_real_distribution<float> distr(1, 1024);
    double temp = distr(gen);
    for (uint32_t i = 0; i < cloud->height * cloud->width; ++i) {
  //    printf("dis=%f\n", temp);
      floatData[i * (cloud->point_step / sizeof(float)) + 0] = temp;
      floatData[i * (cloud->point_step / sizeof(float)) + 1] = temp;
      floatData[i * (cloud->point_step / sizeof(float)) + 2] = temp;
    }


    return cloud;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
