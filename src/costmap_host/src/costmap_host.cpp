// Humble's nav2_costmap_2d executable hard-codes its node to /costmap/costmap,
// so two standalone costmaps cannot coexist. This host takes the costmap name
// as its first non-ROS argument and places the node at /<name>/<name>; the
// costmap is then published on /<name>/costmap.

#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  const std::string name = args.size() > 1 ? args[1] : "costmap";

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(name, "/", name);
  rclcpp::spin(costmap->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
