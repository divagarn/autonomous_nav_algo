// RViz tools that drop dummy perception objects on the map (Autoware's
// "Add Car / Add Bus / Add Pedestrian" planning-simulator tools).
// Click for position, drag for heading. Sends "add:<type>:x:y:yaw" on
// /dummy/request, handled by buggy_pcd_lanelet_map's dummy_objects node.
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <std_msgs/msg/string.hpp>

namespace buggy_rviz_plugins
{

class DummyObjectTool : public rviz_default_plugins::tools::PoseTool
{
public:
  DummyObjectTool(const char * type, const char * title, char key)
  : type_(type), title_(title) {shortcut_key_ = key;}

  void onInitialize() override
  {
    PoseTool::onInitialize();
    setName(QString::fromStdString(title_));
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    pub_ = node->create_publisher<std_msgs::msg::String>("/dummy/request", 10);
  }

protected:
  void onPoseSet(double x, double y, double theta) override
  {
    std_msgs::msg::String msg;
    msg.data = "add:" + type_ + ":" + std::to_string(x) + ":" + std::to_string(y) + ":" +
      std::to_string(theta);
    pub_->publish(msg);
  }

private:
  std::string type_;
  std::string title_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class AddCarTool : public DummyObjectTool
{
public:
  AddCarTool()
  : DummyObjectTool("car", "Add Car", 'c') {}
};

class AddBusTool : public DummyObjectTool
{
public:
  AddBusTool()
  : DummyObjectTool("bus", "Add Bus", 'b') {}
};

class AddPedestrianTool : public DummyObjectTool
{
public:
  AddPedestrianTool()
  : DummyObjectTool("pedestrian", "Add Pedestrian", 'p') {}
};

}  // namespace buggy_rviz_plugins

PLUGINLIB_EXPORT_CLASS(buggy_rviz_plugins::AddCarTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(buggy_rviz_plugins::AddBusTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(buggy_rviz_plugins::AddPedestrianTool, rviz_common::Tool)
