#include "pumpkin_widget.hpp"

#include <QAbstractButton>
#include <QTimer>
#include <QFileDialog>
#include "ui_rviz_draw_gui.h" // will always be "ui_" + <ui file name> + ".h"

#include "yaml_utils.h"
#include <iostream>
#include <fstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pumpkin_widget
{

static const double pumpkin_radius = 0.5; // example radius

RVizDrawGUI::RVizDrawGUI(rclcpp::Node::SharedPtr ros_node, QWidget *parent)
    : QWidget(parent), ros_node_(ros_node), ui_(new Ui::RVizDrawGUI)
{
  // initialize the UI for the GUI
  ui_->setupUi(this);

  // connect signals and slots
  connect(ui_->save_poses_to_yaml_push_button, &QAbstractButton::clicked, this, [this]() {
    // Save Poses to YAML file
    int count = 0;
    YAML::Node node = YAML::convert<std::vector<geometry_msgs::msg::PoseArray>>::encode(saved_poses_arrays_);
    // build filename safely: convert count to string and start with a std::string
    std::string filename = std::string("poses_array_") + ".yaml";
    std::ofstream fout(filename, std::ios::trunc);
    fout << node;
    fout.close();
  });

  connect(ui_->clear_pose_arrays_push_button, &QAbstractButton::clicked, this, [this]() {
    // Clear saved poses arrays
    saved_poses_arrays_.clear();
  });

  connect(ui_->load_pose_array_from_yaml_push_button, &QAbstractButton::clicked, this, [this]() {
    // open a file dialog to select a YAML file
    QString file_name = QFileDialog::getOpenFileName(this, tr("Open YAML File"), "", tr("YAML Files (*.yaml *.yml)"));
    if (file_name.isEmpty()) {
      return; // user cancelled the dialog
    }
    // Load PoseArray from YAML file
    std::ifstream fin(file_name.toStdString());
    YAML::Node node = YAML::Load(fin);
    try {
      // Use node.as<T>() which will call YAML::convert<T>::decode under the hood
      geometry_msgs::msg::PoseArray poses_array = node.as<geometry_msgs::msg::PoseArray>();
      saved_poses_arrays_.push_back(poses_array);
    } catch (const YAML::BadConversion& e) {
      // Handle YAML parsing / conversion error
      std::cerr << "Error converting YAML to PoseArray: " << e.what() << std::endl;
    }
  });

  connect(ui_->send_planning_req_push_button, &QAbstractButton::clicked, this, [this]() {
    // publish the vector of PoseArrays to a ROS2 topic

  });

  pose_array_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseArray>("selected_poses", 10);

  // set up selection sub topic 
  selection_region_sub_ = ros_node_->create_subscription<rviz_selection_3d::msg::SelectionRegion>(
    "/select_3d_tool/region_points", 10,
    [this](const rviz_selection_3d::msg::SelectionRegion::SharedPtr msg) {
      RCLCPP_INFO(ros_node_->get_logger(), "Received selection region with %zu points", msg->points.size());

      // convert the points to the pumpkin face frame from world using tf2
      auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
      tf2_ros::Buffer tf_buffer(clock);
      tf2_ros::TransformListener tf_listener(tf_buffer, ros_node_);

      std::string target_frame = "pumpkin_face";
      std::string source_frame = "world";

      for (auto& point : msg->points_3d)
      {
          geometry_msgs::msg::PointStamped p_in, p_out;
          p_in.header.frame_id = source_frame;
          p_in.header.stamp = ros_node_->get_clock()->now();  // use current ROS time
          p_in.point = point;

          try
          {
              tf_buffer.transform(p_in, p_out, target_frame, tf2::durationFromSec(0.1));
              point = p_out.point;  // overwrite the original point
          }
          catch (tf2::TransformException &ex)
          {
            std::cerr << "Transform error: " << ex.what() << std::endl;
          }
      }
      // Process the received selection region message

      geometry_msgs::msg::PoseArray poses_array;
      poses_array.header.frame_id = "pumpkin_face";
      for (const auto& point : msg->points_3d) {
        // RCLCPP_INFO(ros_node_->get_logger(), "Point: x=%f, y=%f, z=%f", point.x, point.y, point.z);
        geometry_msgs::msg::Pose pose;
        pose.position = point;
        // get the rotation based on the normal of the pumpkin marker
        // compute normal from pumpkin center to the selected point
        geometry_msgs::msg::Point center = pumpkin_marker_.pose.position;
        double vx = point.x - center.x;
        double vy = point.y - center.y;
        double vz = point.z - center.z;
        double norm = std::sqrt(vx*vx + vy*vy + vz*vz);

        // default normal = +Z if point coincides with center
        double nx = 0.0, ny = 0.0, nz = 1.0;
        if (norm > 1e-8) {
          nx = vx / norm;
          ny = vy / norm;
          nz = vz / norm;
        }

        // build quaternion that rotates the +Z axis to the normal vector
        // axis = z x n = (-ny, nx, 0)
        double dot = nz; // dot((0,0,1), n) == n.z
        if (dot > 1.0) dot = 1.0;
        if (dot < -1.0) dot = -1.0;
        double angle = std::acos(dot);

        geometry_msgs::msg::Quaternion q;
        if (std::fabs(angle) < 1e-8) {
          // no rotation
          q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;
        } else {
          double ax = -ny;
          double ay = nx;
          double az = 0.0;
          double alen = std::sqrt(ax*ax + ay*ay + az*az);
          if (alen < 1e-8) {
            // normal is opposite of +Z -> 180deg rotation around X (or Y)
            q.x = 1.0; q.y = 0.0; q.z = 0.0; q.w = 0.0;
          } else {
            ax /= alen; ay /= alen; az /= alen;
            double s = std::sin(angle * 0.5);
            q.x = ax * s;
            q.y = ay * s;
            q.z = az * s;
            q.w = std::cos(angle * 0.5);
          }
        }

        pose.orientation = q;
        poses_array.poses.push_back(pose);
        
      }

      saved_poses_arrays_.push_back(poses_array);

    });
  
  // set up publisher for pumpkin markers
  pumpkin_marker_pub_ = ros_node_->create_publisher<visualization_msgs::msg::Marker>("pumpkin_marker", 10);
  pumpkin_marker_ = visualization_msgs::msg::Marker();
  pumpkin_marker_.header.frame_id = "pumpkin_face";
  pumpkin_marker_.header.stamp = ros_node_->now();
  pumpkin_marker_.ns = "pumpkin";
  pumpkin_marker_.id = 0;
  pumpkin_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  pumpkin_marker_.action = visualization_msgs::msg::Marker::ADD;
  pumpkin_marker_.pose.position.x = 0.0;
  pumpkin_marker_.pose.position.y = 0.0;
  pumpkin_marker_.pose.position.z = -pumpkin_radius;
  pumpkin_marker_.scale.x = pumpkin_radius * 2.0;
  pumpkin_marker_.scale.y = pumpkin_radius * 2.0;
  pumpkin_marker_.scale.z = pumpkin_radius * 2.0;
  pumpkin_marker_.color.r = 1.0;
  pumpkin_marker_.color.g = 0.5;
  pumpkin_marker_.color.b = 0.0;
  pumpkin_marker_.color.a = 1.0;

  timer_ = ros_node_->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {
      // periodically publish marker

      pumpkin_marker_pub_->publish(pumpkin_marker_);
      geometry_msgs::msg::PoseArray all_poses;
      all_poses.header.frame_id = "pumpkin_face";
      all_poses.header.stamp = ros_node_->now();
      for (const auto& poses_array : saved_poses_arrays_) {
        all_poses.poses.insert(all_poses.poses.end(), poses_array.poses.begin(), poses_array.poses.end());
      }
      pose_array_pub_->publish(all_poses);
      // RCLCPP_INFO(ros_node_->get_logger(), "Published %zu selected poses", all_poses.poses.size());
    });
}

RVizDrawGUI::~RVizDrawGUI()
{
  delete ui_;
}

} // namespace pumpkin_widget