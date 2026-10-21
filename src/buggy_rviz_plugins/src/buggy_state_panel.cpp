#include "buggy_rviz_plugins/buggy_state_panel.hpp"

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QStringList>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace buggy_rviz_plugins
{

namespace
{
const char * kButtonStyle =
  "QPushButton { padding: 6px; }"
  "QPushButton:checked { background-color: #2e7d32; color: white; font-weight: bold; }"
  "QPushButton:disabled { color: #777; }";
}  // namespace

BuggyStatePanel::BuggyStatePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  routing_label_ = new QLabel("unset");
  localization_label_ = new QLabel("uninitialized");
  motion_label_ = new QLabel("stopped");
  progress_label_ = new QLabel("-");
  steering_label_ = new QLabel("-");
  drive_label_ = new QLabel("-");
  safety_label_ = new QLabel("-");
  message_label_ = new QLabel("");
  message_label_->setStyleSheet("color: #c62828;");
  message_label_->setWordWrap(true);

  auto * status_form = new QFormLayout;
  status_form->addRow("Routing", routing_label_);
  status_form->addRow("Localization", localization_label_);
  status_form->addRow("Motion", motion_label_);
  status_form->addRow("Speed / Progress", progress_label_);
  status_form->addRow("Steering / CTE", steering_label_);
  status_form->addRow("Drive output", drive_label_);
  status_form->addRow("Sensors", safety_label_);
  auto * status_box = new QGroupBox("Status");
  status_box->setLayout(status_form);

  // Operation mode
  mode_group_ = new QButtonGroup(this);
  auto * mode_row = new QHBoxLayout;
  addChoice(mode_group_, mode_row, "STOP", "mode", "stop");
  addChoice(mode_group_, mode_row, "AUTO", "mode", "auto");
  addChoice(mode_group_, mode_row, "LOCAL", "mode", "local");
  addChoice(mode_group_, mode_row, "REMOTE", "mode", "remote");
  auto * mode_box = new QGroupBox("Operation mode");
  mode_box->setLayout(mode_row);

  // Vehicle options
  gear_group_ = new QButtonGroup(this);
  auto * gear_row = new QHBoxLayout;
  addChoice(gear_group_, gear_row, "P", "gear", "park");
  addChoice(gear_group_, gear_row, "R", "gear", "reverse");
  addChoice(gear_group_, gear_row, "N", "gear", "neutral");
  addChoice(gear_group_, gear_row, "D", "gear", "drive");

  signal_group_ = new QButtonGroup(this);
  auto * signal_row = new QHBoxLayout;
  addChoice(signal_group_, signal_row, "Off", "turn_signal", "none");
  addChoice(signal_group_, signal_row, "Left", "turn_signal", "left");
  addChoice(signal_group_, signal_row, "Right", "turn_signal", "right");
  addChoice(signal_group_, signal_row, "Hazard", "turn_signal", "hazard");

  limit_spin_ = new QDoubleSpinBox;
  limit_spin_->setRange(0.0, 30.0);
  limit_spin_->setSingleStep(0.5);
  limit_spin_->setSuffix(" km/h");
  limit_spin_->setKeyboardTracking(false);

  auto * vehicle_form = new QFormLayout;
  vehicle_form->addRow("Gear", gear_row);
  vehicle_form->addRow("Turn signal", signal_row);
  vehicle_form->addRow("Velocity limit", limit_spin_);
  auto * vehicle_box = new QGroupBox("Vehicle");
  vehicle_box->setLayout(vehicle_form);

  emergency_button_ = new QPushButton("EMERGENCY STOP");
  emergency_button_->setCheckable(true);
  emergency_button_->setStyleSheet(
    "QPushButton { padding: 8px; font-weight: bold; }"
    "QPushButton:checked { background-color: #c62828; color: white; }");
  auto * clear_button = new QPushButton("Clear route");
  auto * dummy_button = new QPushButton("Delete all dummies");
  clear_button->setStyleSheet(kButtonStyle);

  auto * layout = new QVBoxLayout;
  layout->addWidget(status_box);
  layout->addWidget(mode_box);
  layout->addWidget(vehicle_box);
  layout->addWidget(emergency_button_);
  layout->addWidget(clear_button);
  layout->addWidget(dummy_button);
  layout->addWidget(message_label_);
  layout->addStretch();
  setLayout(layout);

  connect(clear_button, &QPushButton::clicked, this, &BuggyStatePanel::onClearRoute);
  connect(dummy_button, &QPushButton::clicked, this, [this]() {
      if (dummy_pub_) {
        std_msgs::msg::String m;
        m.data = "delete_all";
        dummy_pub_->publish(m);
      }
    });
  connect(emergency_button_, &QPushButton::clicked, this, [this](bool on) {
      request(QString("emergency:%1").arg(on ? "true" : "false"));
    });
  connect(limit_spin_, &QDoubleSpinBox::editingFinished, this, [this]() {
      request(QString("velocity_limit:%1").arg(limit_spin_->value()));
    });

  // AUTO needs a route + pose: disabled until the manager says it is available.
  buttons_["mode:auto"]->setEnabled(false);
}

QPushButton * BuggyStatePanel::addChoice(
  QButtonGroup * group, QLayout * layout, const QString & label, const QString & key,
  const QString & value)
{
  auto * button = new QPushButton(label);
  button->setCheckable(true);
  button->setStyleSheet(kButtonStyle);
  group->addButton(button);
  layout->addWidget(button);
  buttons_[key + ":" + value] = button;
  connect(button, &QPushButton::clicked, this, [this, key, value]() {
      request(key + ":" + value);
    });
  return button;
}

void BuggyStatePanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  clear_pub_ = node->create_publisher<std_msgs::msg::Empty>("/planning/clear_route", 10);
  request_pub_ = node->create_publisher<std_msgs::msg::String>("/system/request", 10);
  dummy_pub_ = node->create_publisher<std_msgs::msg::String>("/dummy/request", 10);

  state_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/system/state", 10,
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      QMetaObject::invokeMethod(
        this, "setState", Qt::QueuedConnection,
        Q_ARG(QString, QString::fromStdString(msg->data)));
    });
}

void BuggyStatePanel::setState(const QString & text)
{
  QMap<QString, QString> kv;
  for (const QString & item : text.split(';')) {
    const int eq = item.indexOf('=');
    if (eq > 0) {
      kv[item.left(eq)] = item.mid(eq + 1);
    }
  }

  routing_label_->setText(kv.value("routing"));
  localization_label_->setText(
    QString("%1  (NDT score %2, age %3 s)")
      .arg(kv.value("localization"), kv.value("ndt_score", "-"), kv.value("ndt_age", "-")));
  motion_label_->setText(kv.value("motion"));
  progress_label_->setText(
    QString("%1 km/h  /  %2 %").arg(kv.value("speed"), kv.value("progress")));
  steering_label_->setText(
    QString("act %1 / cmd %2 deg (+right)  /  CTE %3 m")
    .arg(kv.value("steering"), kv.value("cmd_steer"), kv.value("cte")));
  drive_label_->setText(kv.value("drive"));
  safety_label_->setText(
    kv.contains("sensors") ?
    QString("%1  =>  %2 obstacle(s)").arg(kv.value("sensors"), kv.value("objects")) :
    QString("%1, %2 object(s)").arg(kv.value("detector"), kv.value("objects")));
  safety_label_->setToolTip("Stop model: " + kv.value("stop_model"));
  safety_label_->setStyleSheet((kv.value("detector") == "STALE" || kv.value("sensors").contains("STALE")) ? "color: #c62828; font-weight: bold;" : "");
  drive_label_->setStyleSheet(
    kv.value("drive").startsWith("ARMED") ? "color: #c62828; font-weight: bold;" : "");
  message_label_->setText(kv.value("msg"));

  for (const QString key : {"mode", "gear", "turn_signal"}) {
    const QString id = key + ":" + kv.value(key);
    if (buttons_.contains(id)) {
      buttons_[id]->setChecked(true);
    }
  }
  buttons_["mode:auto"]->setEnabled(
    kv.value("auto_available") == "true" || kv.value("mode") == "auto");
  emergency_button_->setChecked(kv.value("emergency") == "true");
  if (!limit_spin_->hasFocus()) {
    limit_spin_->setValue(kv.value("velocity_limit").toDouble());
  }
}

void BuggyStatePanel::request(const QString & text)
{
  if (request_pub_) {
    std_msgs::msg::String msg;
    msg.data = text.toStdString();
    request_pub_->publish(msg);
  }
}

void BuggyStatePanel::onClearRoute()
{
  if (clear_pub_) {
    clear_pub_->publish(std_msgs::msg::Empty());
  }
}

}  // namespace buggy_rviz_plugins

PLUGINLIB_EXPORT_CLASS(buggy_rviz_plugins::BuggyStatePanel, rviz_common::Panel)
