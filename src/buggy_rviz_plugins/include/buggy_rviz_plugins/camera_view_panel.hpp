#ifndef BUGGY_RVIZ_PLUGINS__CAMERA_VIEW_PANEL_HPP_
#define BUGGY_RVIZ_PLUGINS__CAMERA_VIEW_PANEL_HPP_

#include <QElapsedTimer>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QString>
#include <QTimer>
#include <QVector>
#include <QWidget>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace buggy_rviz_plugins
{

// Top view of the camera lane path (x forward, y left) with distance rings.
class PathPlot : public QWidget
{
public:
  explicit PathPlot(QWidget * parent = nullptr);
  void setPath(const QVector<QPointF> & pts);
  QSize sizeHint() const override {return QSize(260, 300);}

protected:
  void paintEvent(QPaintEvent *) override;

private:
  QVector<QPointF> pts_;
};

// Separate panel for the camera views of sensor_fusion. NOTHING is shown (and nothing subscribed) until a
// button is pressed; pressing the active button again hides the view:
//   Camera feed          /object_detection/image_rect          plain rectified detector image
//   Camera overlay feed  /fusion_visualization/overlay         boxes, lane mask and path drawn on it (--overlay)
//   Lane path curvature  /sensor_fusion/lane_path              curvature / radius / steering from the tracked lane path
class CameraViewPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CameraViewPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void showImage(const QImage & img);
  void showPath(const QVector<QPointF> & pts);
  void checkAlive();

private:
  enum View {None = 0, Feed = 1, Overlay = 2, Curvature = 3};
  void select(View v);
  void subscribeImage(const std::string & topic);
  QString describe(const QVector<QPointF> & pts) const;

  View view_{None};
  QPushButton * buttons_[4]{};
  QStackedWidget * stack_;
  QLabel * none_label_, * image_label_, * status_label_, * curv_label_;
  PathPlot * plot_;
  QPixmap last_pixmap_;
  QTimer * alive_timer_;
  QElapsedTimer age_, throttle_;
  bool got_data_{false};
  std::string topic_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
};

}  // namespace buggy_rviz_plugins

#endif  // BUGGY_RVIZ_PLUGINS__CAMERA_VIEW_PANEL_HPP_
