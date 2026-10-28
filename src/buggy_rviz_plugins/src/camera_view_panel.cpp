#include "buggy_rviz_plugins/camera_view_panel.hpp"

#include <algorithm>
#include <cmath>

#include <QMetaObject>
#include <QMainWindow>
#include <QDockWidget>
#include <QMetaType>
#include <QScrollArea>
#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace buggy_rviz_plugins
{
namespace
{
const char * kFeedTopic = "/object_detection/image_rect";
const char * kOverlayTopic = "/fusion_visualization/overlay";
const char * kPathTopic = "/sensor_fusion/lane_path";
constexpr double kWheelbase = 1.6;       // m (config/buggy_params.yaml wheelbase)
constexpr double kMinRadius = 4.4;       // m: wheelbase / tan(20 deg), the steering limit of the buggy

const char * kButtonStyle =
  "QPushButton { padding: 8px; }"
  "QPushButton:checked { background-color: #2e7d32; color: white; font-weight: bold; }";

// signed curvature [1/m], + left, of three points (Menger)
double menger(const QPointF & a, const QPointF & b, const QPointF & c)
{
  const double abx = b.x() - a.x(), aby = b.y() - a.y(), bcx = c.x() - b.x(), bcy = c.y() - b.y();
  const double cross = abx * bcy - aby * bcx;
  const double la = std::hypot(abx, aby), lb = std::hypot(bcx, bcy), lc = std::hypot(c.x() - a.x(), c.y() - a.y());
  const double d = la * lb * lc;
  return d < 1e-9 ? 0.0 : 2.0 * cross / d;
}

// mean curvature of the path between arc lengths s0 and s1 from the vehicle
double mean_curvature(const QVector<QPointF> & p, double s0, double s1, double * worst = nullptr)
{
  double s = 0.0, sum = 0.0;
  int n = 0;
  for (int i = 1; i + 1 < p.size(); ++i) {
    s += std::hypot(p[i].x() - p[i - 1].x(), p[i].y() - p[i - 1].y());
    if (s >= s0 && s <= s1) {
      const double k = menger(p[i - 1], p[i], p[i + 1]);
      sum += k;
      ++n;
      if (worst && std::abs(k) > std::abs(*worst)) {*worst = k;}
    }
  }
  return n ? sum / n : 0.0;
}

QString curv_text(double k)
{
  if (std::abs(k) < 0.004) {return "straight";}
  return QString("%1 %2 1/m   R = %3 m").arg(k > 0 ? "LEFT " : "RIGHT").arg(std::abs(k), 0, 'f', 3).arg(1.0 / std::abs(k), 0, 'f', 1);
}
}  // namespace

PathPlot::PathPlot(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(120, 100);
}

void PathPlot::setPath(const QVector<QPointF> & pts)
{
  pts_ = pts;
  update();
}

void PathPlot::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.fillRect(rect(), QColor(16, 26, 64));
  const double range = 22.0;                       // m shown ahead
  const double sc = (height() - 30) / range;
  const double cx = width() / 2.0, by = height() - 14.0;
  auto map = [&](double x, double y) {return QPointF(cx - y * sc, by - x * sc);};
  p.setPen(QPen(QColor(80, 100, 170), 1, Qt::DashLine));
  for (int r = 5; r <= 20; r += 5) {
    p.drawLine(map(r, -8), map(r, 8));
    p.setPen(QColor(150, 170, 230));
    p.drawText(map(r, 8) + QPointF(-34, -2), QString("%1 m").arg(r));
    p.setPen(QPen(QColor(80, 100, 170), 1, Qt::DashLine));
  }
  p.drawLine(map(0, 0), map(range, 0));
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(255, 255, 255));
  p.drawRect(QRectF(cx - 0.65 * sc, by, 1.3 * sc, 6));                                  // vehicle at the origin
  if (pts_.size() > 1) {
    p.setPen(QPen(QColor(0, 220, 120), 3));
    for (int i = 1; i < pts_.size(); ++i) {p.drawLine(map(pts_[i - 1].x(), pts_[i - 1].y()), map(pts_[i].x(), pts_[i].y()));}
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(255, 255, 255));
    for (const auto & q : pts_) {p.drawEllipse(map(q.x(), q.y()), 2.5, 2.5);}
  } else {
    p.setPen(QColor(200, 200, 200));
    p.drawText(rect(), Qt::AlignCenter, "no lane path");
  }
}

CameraViewPanel::CameraViewPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  qRegisterMetaType<QVector<QPointF>>("QVector<QPointF>");
  const char * names[] = {"", "Camera feed", "Camera overlay feed", "Lane path curvature"};
  auto * row = new QHBoxLayout;
  for (int i = 1; i <= 3; ++i) {
    buttons_[i] = new QPushButton(names[i]);
    buttons_[i]->setCheckable(true);
    buttons_[i]->setStyleSheet(kButtonStyle);
    row->addWidget(buttons_[i]);
    connect(buttons_[i], &QPushButton::clicked, this, [this, i](bool on) {select(on ? static_cast<View>(i) : None);});
  }
  buttons_[Feed]->setToolTip(QString("Plain rectified camera image (%1)").arg(kFeedTopic));
  buttons_[Overlay]->setToolTip(QString("Camera image with boxes, lane and path drawn (%1, needs --overlay)").arg(kOverlayTopic));
  buttons_[Curvature]->setToolTip(QString("Curvature of the tracked lane path (%1)").arg(kPathTopic));

  none_label_ = new QLabel("No view selected.\nPress a button to show it; press it again to hide.");
  none_label_->setAlignment(Qt::AlignCenter);
  none_label_->setStyleSheet("color: #9fb0e0;");
  image_label_ = new QLabel;
  image_label_->setAlignment(Qt::AlignCenter);
  image_label_->setMinimumSize(120, 68);
  image_label_->setStyleSheet("background-color: #101a40;");
  image_label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  plot_ = new PathPlot;
  curv_label_ = new QLabel;
  curv_label_->setTextFormat(Qt::RichText);
  curv_label_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  curv_label_->setWordWrap(true);
  auto * curv_page = new QWidget;
  auto * curv_layout = new QHBoxLayout(curv_page);
  curv_layout->setContentsMargins(0, 0, 0, 0);
  auto * curv_scroll = new QScrollArea;                      // the text may be taller than the panel: scroll it
  curv_scroll->setWidget(curv_label_);
  curv_scroll->setWidgetResizable(true);
  curv_scroll->setFrameShape(QFrame::NoFrame);
  curv_layout->addWidget(plot_, 1);
  curv_layout->addWidget(curv_scroll, 1);

  stack_ = new QStackedWidget;
  stack_->addWidget(none_label_);
  stack_->addWidget(image_label_);
  stack_->addWidget(curv_page);
  status_label_ = new QLabel;
  status_label_->setStyleSheet("color: #ffd166;");
  status_label_->setWordWrap(true);

  auto * col = new QVBoxLayout;
  col->addLayout(row);
  col->addWidget(stack_, 1);
  col->addWidget(status_label_);
  setLayout(col);

  alive_timer_ = new QTimer(this);
  alive_timer_->setInterval(1000);
  connect(alive_timer_, &QTimer::timeout, this, &CameraViewPanel::checkAlive);
  alive_timer_->start();
}

void CameraViewPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  // Dock this panel as a TAB next to "Buggy State" instead of below it: stacked docks add up their minimum
  // heights, and RViz then cannot be maximised on a laptop screen.
  QTimer::singleShot(600, this, [this]() {
      auto * mine = qobject_cast<QDockWidget *>(parentWidget());
      auto * main = mine ? qobject_cast<QMainWindow *>(mine->parentWidget()) : nullptr;
      if (!main) {return;}
      for (auto * d : main->findChildren<QDockWidget *>()) {
        if (d != mine && d->windowTitle() == "Buggy State") {
          main->tabifyDockWidget(d, mine);
          d->raise();
          break;
        }
      }
    });
}

void CameraViewPanel::select(View v)
{
  view_ = v;
  for (int i = 1; i <= 3; ++i) {buttons_[i]->setChecked(i == static_cast<int>(v));}
  image_sub_.reset();           // nothing is subscribed unless a view is shown
  path_sub_.reset();
  got_data_ = false;
  last_pixmap_ = QPixmap();
  image_label_->clear();
  stack_->setCurrentIndex(v == None ? 0 : (v == Curvature ? 2 : 1));
  if (v == None) {
    status_label_->setText("");
    return;
  }
  if (!node_) {
    status_label_->setText("ROS node not ready");
    return;
  }
  age_.invalidate();
  if (v == Curvature) {
    topic_ = kPathTopic;
    plot_->setPath({});
    curv_label_->setText("");
    path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Path::ConstSharedPtr m) {
        QVector<QPointF> pts;
        for (const auto & ps : m->poses) {pts.push_back(QPointF(ps.pose.position.x, ps.pose.position.y));}
        QMetaObject::invokeMethod(this, "showPath", Qt::QueuedConnection, Q_ARG(QVector<QPointF>, pts));
      });
  } else {
    subscribeImage(v == Feed ? kFeedTopic : kOverlayTopic);
  }
  status_label_->setText(QString("waiting for %1 ...").arg(QString::fromStdString(topic_)));
}

void CameraViewPanel::subscribeImage(const std::string & topic)
{
  topic_ = topic;
  throttle_.invalidate();
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
      if (throttle_.isValid() && throttle_.elapsed() < 66) {return;}          // at most ~15 images/s
      throttle_.restart();
      const auto & e = m->encoding;
      QImage::Format fmt;
      bool swap = false;
      if (e == "rgb8") {fmt = QImage::Format_RGB888;}
      else if (e == "bgr8") {fmt = QImage::Format_RGB888; swap = true;}
      else if (e == "mono8" || e == "8UC1") {fmt = QImage::Format_Grayscale8;}
      else {return;}
      QImage img(m->data.data(), static_cast<int>(m->width), static_cast<int>(m->height),
        static_cast<int>(m->step), fmt);
      QImage copy = swap ? img.rgbSwapped() : img.copy();
      QMetaObject::invokeMethod(this, "showImage", Qt::QueuedConnection, Q_ARG(QImage, copy));
    });
}

void CameraViewPanel::showImage(const QImage & img)
{
  if (view_ != Feed && view_ != Overlay) {return;}
  last_pixmap_ = QPixmap::fromImage(img);
  image_label_->setPixmap(last_pixmap_.scaled(image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  got_data_ = true;
  age_.restart();
  status_label_->setText(QString("%1   %2 x %3").arg(QString::fromStdString(topic_)).arg(img.width()).arg(img.height()));
}

QString CameraViewPanel::describe(const QVector<QPointF> & p) const
{
  double worst = 0.0;
  const double near_k = mean_curvature(p, 0.0, 6.0), mid_k = mean_curvature(p, 6.0, 12.0),
    far_k = mean_curvature(p, 12.0, 25.0);
  mean_curvature(p, 0.0, 25.0, &worst);
  double len = 0.0;
  for (int i = 1; i < p.size(); ++i) {len += std::hypot(p[i].x() - p[i - 1].x(), p[i].y() - p[i - 1].y());}
  // steering that follows the near curvature, + right like /steering_angle (left curve -> negative)
  const double steer = -std::atan(kWheelbase * near_k) * 180.0 / M_PI;
  QString tight;
  if (std::abs(worst) > 1e-6 && 1.0 / std::abs(worst) < kMinRadius) {
    tight = QString(" <span style='color:#ff8a80'>(tighter than the buggy can turn, R &lt; %1 m)</span>").arg(kMinRadius);
  }
  return QString(
    "<b>Near 0-6 m</b><br>%1<br><br><b>Mid 6-12 m</b><br>%2<br><br><b>Far 12-25 m</b><br>%3<br><br>"
    "<b>Steering that follows the near curve</b><br>%4 deg (+right)<br><br>"
    "<b>Tightest point</b><br>%5%6<br><br><span style='color:#9fb0e0'>%7 points, %8 m</span>")
         .arg(curv_text(near_k), curv_text(mid_k), curv_text(far_k))
         .arg(steer, 0, 'f', 1).arg(curv_text(worst), tight).arg(p.size()).arg(len, 0, 'f', 1);
}

void CameraViewPanel::showPath(const QVector<QPointF> & pts)
{
  if (view_ != Curvature) {return;}
  plot_->setPath(pts);
  curv_label_->setText(pts.size() > 2 ? describe(pts) : "lane path too short");
  got_data_ = true;
  age_.restart();
  status_label_->setText(QString::fromStdString(topic_));
}

void CameraViewPanel::checkAlive()
{
  if (view_ == None) {return;}
  const bool stale = !age_.isValid() || age_.elapsed() > 2500;
  if (!stale) {return;}
  QString hint = view_ == Overlay ? " (start with --camera --overlay)" :
    view_ == Feed ? " (start with --camera)" : " (start with --camera; the lane path needs lane segmentation)";
  status_label_->setText(QString("no data on %1%2").arg(QString::fromStdString(topic_), hint));
  if (view_ == Curvature && got_data_) {plot_->setPath({}); curv_label_->setText("");}
  got_data_ = false;
}

}  // namespace buggy_rviz_plugins

PLUGINLIB_EXPORT_CLASS(buggy_rviz_plugins::CameraViewPanel, rviz_common::Panel)
