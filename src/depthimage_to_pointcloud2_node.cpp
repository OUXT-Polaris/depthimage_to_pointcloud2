#include "depthimage_to_pointcloud2/depthimage_to_pointcloud2_component.hpp"
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<depthimage_to_pointcloud2::Depthimage2Pointcloud2>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}