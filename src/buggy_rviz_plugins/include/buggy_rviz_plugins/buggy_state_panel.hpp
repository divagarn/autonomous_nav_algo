#ifndef BUGGY_RVIZ_PLUGINS__BUGGY_STATE_PANEL_HPP_
#define BUGGY_RVIZ_PLUGINS__BUGGY_STATE_PANEL_HPP_

#include <QButtonGroup>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QMap>
#include <QPushButton>
#include <QString>
#include <QElapsedTimer>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

namespace buggy_rviz_plugins
{

// Autoware-style state panel: status read-outs, operation mode (Stop / Auto /
// Local / Remote), gear, turn signal, velocity limit, emergency and
// clear-route and the mission (Start / Stop / Resume of the stops in <map>/mission.yaml, /mission/request +
// /mission/state of the mission_manager node, shown as "not running" without --mission). Talks to buggy_pcd_lanelet_map's operation_manager via
// /system/request (commands) and /system/state ("k=v;k=v" string).
class BuggyStatePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit BuggyStatePanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void setState(const QString & text);
  void onClearRoute();
  void setMissionState(const QString & text);
  void checkMissionAlive();
  void setGnssState(const QString & text);
  void checkGnssAlive();

private:
  void request(const QString & text);
  void missionRequest(const QString & text);
  QPushButton * addChoice(
    QButtonGroup * group, QLayout * layout, const QString & label, const QString & key,
    const QString & value);

  QLabel * routing_label_;
  QLabel * localization_label_;
  QLabel * motion_label_;
  QLabel * progress_label_;
  QLabel * steering_label_;
  QLabel * drive_label_;
  QLabel * safety_label_;
  QLabel * message_label_;
  QButtonGroup * mode_group_;
  QButtonGroup * gear_group_;
  QButtonGroup * signal_group_;
  QDoubleSpinBox * limit_spin_;
  QPushButton * emergency_button_;
  QMap<QString, QPushButton *> buttons_;  // keyed "<group>:<value>"

  // Mission (named stops, mission_manager node): Start / Stop / Resume + what it is doing
  QLabel * mission_label_;
  QLabel * mission_msg_label_;
  QPushButton * mission_start_;
  QPushButton * mission_stop_;
  QPushButton * mission_resume_;
  // GNSS auto start pose (gnss_pose_initializer, ./buggy live|replay --gnss): fix status + enable / disable button
  QLabel * gnss_dot_;
  QLabel * gnss_fix_label_;
  QLabel * gnss_status_label_;
  QPushButton * gnss_toggle_;
  QTimer * gnss_timer_;
  QElapsedTimer gnss_age_;
  bool gnss_seen_{false};
  QTimer * mission_timer_;
  QElapsedTimer mission_age_;
  bool mission_seen_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_, mission_sub_, gnss_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clear_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dummy_pub_, mission_pub_, gnss_pub_;
};

}  // namespace buggy_rviz_plugins

#endif  // BUGGY_RVIZ_PLUGINS__BUGGY_STATE_PANEL_HPP_
