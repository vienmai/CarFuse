#ifndef NDT_MAP_PUBLISHER_
#define NDT_MAP_PUBLISHER_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MapPublisher : public rclcpp::Node {
 public:
  MapPublisher();
  void publish_map(const std::string &path);

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
};

#endif
