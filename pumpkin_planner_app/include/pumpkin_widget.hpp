#pragma once

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/node.hpp"
#include <vector>
#include <geometry_msgs/msg/pose_array.hpp>
#include "QWidget"
#include "rviz_selection_3d/msg/selection_region.hpp"



namespace Ui {
  class RVizDrawGUI;
}
namespace pumpkin_widget 
{

class RVizDrawGUI : public QWidget
{

  Q_OBJECT

  public:
    explicit RVizDrawGUI(rclcpp::Node::SharedPtr ros_node, QWidget *parent = nullptr);

    ~RVizDrawGUI();

  private:
    rclcpp::Node::SharedPtr ros_node_;
    Ui::RVizDrawGUI* ui_;
    std::vector<geometry_msgs::msg::PoseArray> saved_poses_arrays_;
    rclcpp::Subscription
};

} // namespace pumpkin_widget