#include "pumpkin_widget.hpp"

#include <QAbstractButton>
#include <QTimer>
#include "ui_rviz_draw_gui.h" // will always be "ui_" + <ui file name> + ".h"

#include "yaml_utils.hpp"

namespace pumpkin_widget
{

RVizDrawGUI::RVizDrawGUI(rclcpp::Node::SharedPtr ros_node, QWidget *parent)
    : QWidget(parent), ros_node_(ros_node), ui_(new Ui::RVizDrawGUI)
{
  // initialize the UI for the GUI
  ui_->setupUi(this);

  // connect signals and slots
  connect(ui_->save_poses_to_yaml_push_button, &QAbstractButton::clicked, this, [this]() {
    // Save Poses to YAML file
    int count = 0;
    for (const auto& poses_array : saved_poses_arrays_) {
      YAML::Node node = YAML::convert<geometry_msgs::msg::PoseArray>::encode(poses_array);
      std::string filename = "poses_array_" + count + ".yaml";
      std::ofstream fout(filename, std::ios::app);
      fout << node;
      fout.close();
      count++;
    }
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
      geometry_msgs::msg::PoseArray poses_array = YAML::convert<geometry_msgs::msg::PoseArray>::decode(node);
      saved_poses_arrays_.push_back(poses_array);
    } catch (const std::exception& e) {
      // Handle YAML parsing error
      std::cerr << "Error loading YAML file: " << e.what() << std::endl;
    }
  });

  connect(ui_->send_planning_req_push_button, &QAbstractButton::clicked, this, [this]() {
    // publish the vector of PoseArrays to a ROS2 topic

  });

}

RVizDrawGUI::~RVizDrawGUI()
{
  delete ui_;
}

} // namespace pumpkin_widget