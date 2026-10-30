#include "buggy_rviz_plugins/buggy_state_panel.hpp"

#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QSpinBox>
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
  message_label_->setStyleSheet("color: #ffd166;");
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
  // Auto goal / closed loop: handled by route_planner via /system/request
  auto * loop_spin = new QSpinBox;
  loop_spin->setRange(1, 20);
  loop_spin->setPrefix("laps: ");
  auto * loop_button = new QPushButton("Loop back to HOME");
  auto * home_button = new QPushButton("Return HOME");
  auto * set_home_button = new QPushButton("Set HOME here");
  auto * loop_row = new QHBoxLayout;
  loop_row->addWidget(loop_spin);
  loop_row->addWidget(loop_button);
  auto * home_row = new QHBoxLayout;
  home_row->addWidget(home_button);
  home_row->addWidget(set_home_button);
  auto * loop_layout = new QVBoxLayout;
  loop_layout->addLayout(loop_row);
  loop_layout->addLayout(home_row);
  auto * loop_box = new QGroupBox("Auto goal (closed loop)");
  loop_box->setLayout(loop_layout);
  loop_box->setToolTip(
    "HOME = where AUTO was first engaged (or 'Set HOME here').\n"
    "Loop: plans the route back to HOME, plus laps-1 more laps; then press AUTO.");
  // Mission: the named stops (A, B ... home) of the map, with a wait at each - needs the stack started with --mission
  mission_label_ = new QLabel("mission node not running (start with --mission)");
  mission_label_->setWordWrap(true);
  mission_msg_label_ = new QLabel("");
  mission_msg_label_->setWordWrap(true);
  mission_msg_label_->setStyleSheet("color: #ffd166;");
  mission_start_ = new QPushButton("Start mission");
  mission_stop_ = new QPushButton("Stop mission");
  mission_resume_ = new QPushButton("Resume");
  mission_start_->setToolTip("Drive to the first stop, wait there, drive on to the next ... (needs --drive for the buggy to move)");
  mission_stop_->setToolTip("Stop the buggy and end the mission");
  mission_resume_->setToolTip("Go on after the wait (only with auto_restart: false)");
  mission_stop_->setStyleSheet("QPushButton { padding: 6px; } QPushButton:enabled { background-color: #b23b3b; color: white; }");
  auto * mission_row = new QHBoxLayout;
  mission_row->addWidget(mission_start_);
  mission_row->addWidget(mission_stop_);
  mission_row->addWidget(mission_resume_);
  auto * mission_layout = new QVBoxLayout;
  mission_layout->addWidget(mission_label_);
  mission_layout->addWidget(mission_msg_label_);
  mission_layout->addLayout(mission_row);
  auto * mission_box = new QGroupBox("Mission (stops A, B ...)");
  mission_box->setLayout(mission_layout);
  for (auto * b : {mission_start_, mission_stop_, mission_resume_}) {b->setEnabled(false);}
  mission_timer_ = new QTimer(this);
  mission_timer_->setInterval(1000);
  connect(mission_timer_, &QTimer::timeout, this, &BuggyStatePanel::checkMissionAlive);
  mission_timer_->start();

  // GNSS auto start pose: fix status + an on/off switch (needs --gnss and a georeferenced map)
  gnss_dot_ = new QLabel(QString::fromUtf8("\u25CF"));
  gnss_dot_->setStyleSheet("color: #777; font-size: 22px;");
  gnss_fix_label_ = new QLabel("GNSS auto start pose not running (start with --gnss)");
  gnss_fix_label_->setWordWrap(true);
  gnss_status_label_ = new QLabel("");
  gnss_status_label_->setWordWrap(true);
  gnss_status_label_->setStyleSheet("color: #ffd166;");
  gnss_toggle_ = new QPushButton("GNSS auto-localize: OFF");
  gnss_toggle_->setCheckable(true);
  gnss_toggle_->setEnabled(false);
  gnss_toggle_->setStyleSheet(
    "QPushButton { padding: 6px; }"
    "QPushButton:checked { background-color: #2e7d32; color: white; font-weight: bold; }"
    "QPushButton:disabled { color: #777; }");
  gnss_toggle_->setToolTip("ON: the buggy sets its own start pose from GNSS (RTK fixed + heading) when localization is not running.\n"
                           "OFF: it never does; use 2D Pose Estimate.");
  auto * gnss_top = new QHBoxLayout;
  gnss_top->addWidget(gnss_dot_);
  gnss_top->addWidget(gnss_fix_label_, 1);
  auto * gnss_layout = new QVBoxLayout;
  gnss_layout->addLayout(gnss_top);
  gnss_layout->addWidget(gnss_status_label_);
  gnss_layout->addWidget(gnss_toggle_);
  auto * gnss_box = new QGroupBox("GNSS auto start pose");
  gnss_box->setLayout(gnss_layout);
  gnss_timer_ = new QTimer(this);
  gnss_timer_->setInterval(1000);
  connect(gnss_timer_, &QTimer::timeout, this, &BuggyStatePanel::checkGnssAlive);
  gnss_timer_->start();

  auto * clear_button = new QPushButton("Clear route");
  auto * dummy_button = new QPushButton("Delete all dummies");
  clear_button->setStyleSheet(kButtonStyle);

  // Two columns instead of one tall column: the panel stays short enough for a laptop screen without a
  // scroll area (left: status + operation mode; right: vehicle, auto goal; buttons and message below).
  auto * bottom_row = new QHBoxLayout;
  bottom_row->addWidget(emergency_button_);
  bottom_row->addWidget(clear_button);
  bottom_row->addWidget(dummy_button);
  auto * grid = new QGridLayout;
  grid->addWidget(status_box, 0, 0);
  grid->addWidget(mode_box, 1, 0);
  grid->addWidget(vehicle_box, 0, 1);
  grid->addWidget(loop_box, 1, 1);
  grid->setAlignment(vehicle_box, Qt::AlignTop);
  grid->setAlignment(loop_box, Qt::AlignTop);
  grid->addWidget(mission_box, 2, 0);
  grid->addWidget(gnss_box, 2, 1);
  grid->setAlignment(mission_box, Qt::AlignTop);
  grid->setAlignment(gnss_box, Qt::AlignTop);
  grid->addLayout(bottom_row, 3, 0, 1, 2);
  grid->addWidget(message_label_, 4, 0, 1, 2);
  grid->setRowStretch(5, 1);
  grid->setColumnStretch(0, 1);
  grid->setColumnStretch(1, 1);
  setLayout(grid);

  connect(loop_button, &QPushButton::clicked, this, [this, loop_spin]() {
      request(QString("loop:%1").arg(loop_spin->value()));
    });
  connect(home_button, &QPushButton::clicked, this, [this]() {request("home:go");});
  connect(set_home_button, &QPushButton::clicked, this, [this]() {request("home:set");});
  connect(mission_start_, &QPushButton::clicked, this, [this]() {missionRequest("start");});
  connect(mission_stop_, &QPushButton::clicked, this, [this]() {missionRequest("stop");});
  connect(mission_resume_, &QPushButton::clicked, this, [this]() {missionRequest("resume");});
  connect(gnss_toggle_, &QPushButton::clicked, this, [this](bool on) {
      if (gnss_pub_) {
        std_msgs::msg::String m;
        m.data = on ? "enable" : "disable";
        gnss_pub_->publish(m);
      }
    });
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
  mission_pub_ = node->create_publisher<std_msgs::msg::String>("/mission/request", 10);
  gnss_pub_ = node->create_publisher<std_msgs::msg::String>("/gnss_init/request", 10);
  gnss_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/gnss_init/state", 10,
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      QMetaObject::invokeMethod(
        this, "setGnssState", Qt::QueuedConnection,
        Q_ARG(QString, QString::fromStdString(msg->data)));
    });
  mission_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/mission/state", 10,
    [this](std_msgs::msg::String::ConstSharedPtr msg) {
      QMetaObject::invokeMethod(
        this, "setMissionState", Qt::QueuedConnection,
        Q_ARG(QString, QString::fromStdString(msg->data)));
    });

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
  safety_label_->setStyleSheet((kv.value("detector") == "STALE" || kv.value("sensors").contains("STALE")) ? "color: #ff8a80; font-weight: bold;" : "");
  drive_label_->setStyleSheet(
    kv.value("drive").startsWith("ARMED") ? "color: #ff8a80; font-weight: bold;" : "");
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

namespace
{
QString gnss_state_text(int s)
{
  switch (s) {
    case 0: return "no fix";
    case 1: return "single point";
    case 2: return "DGPS";
    case 3: return "dead reckoning";
    case 4: return "RTK fixed";
    case 5: return "RTK float";
    case 6: return "single, no heading";
    case 7: return "DGPS, no heading";
    case 8: return "RTK fixed, no heading";
    case 9: return "RTK float, no heading";
    default: return "no GNSS data";
  }
}
}  // namespace

void BuggyStatePanel::setGnssState(const QString & text)
{
  QMap<QString, QString> kv;
  for (const QString & item : text.split(';')) {
    const int eq = item.indexOf('=');
    if (eq > 0) {
      kv[item.left(eq)] = item.mid(eq + 1);
    }
  }
  gnss_seen_ = true;
  gnss_age_.restart();
  const bool enabled = kv.value("enabled") == "1";
  const bool usable = kv.value("usable") == "1";
  const bool fix_ok = kv.value("fix_ok") == "1";
  const int st = kv.value("gnss_state", "-1").toInt();
  const double fix_age = kv.value("fix_age", "-1").toDouble();
  const bool has_fix = st >= 0 && fix_age >= 0.0 && fix_age < 2.0;
  if (has_fix) {
    gnss_fix_label_->setText(QString("%1 (%2)   std %3 m   heading %4 deg")
      .arg(gnss_state_text(st)).arg(st).arg(kv.value("std_m"), kv.value("heading")));
  } else {
    gnss_fix_label_->setText("no GNSS message (is the CHCNAV device sending?)");
  }
  // green = would start now, yellow = GNSS present but not good enough / off, red = no GNSS, grey = switched off
  QString color = "#777";
  if (usable && enabled) {
    color = !has_fix ? "#e53935" : (fix_ok ? "#43a047" : "#fbc02d");
  } else if (usable && !enabled) {
    color = has_fix && fix_ok ? "#9e9e9e" : "#616161";
  }
  gnss_dot_->setStyleSheet(QString("color: %1; font-size: 22px;").arg(color));
  gnss_status_label_->setText(kv.value("status"));
  gnss_toggle_->setEnabled(usable);
  gnss_toggle_->setChecked(enabled && usable);
  gnss_toggle_->setText(QString("GNSS auto-localize: %1").arg(enabled && usable ? "ON" : "OFF"));
}

void BuggyStatePanel::checkGnssAlive()
{
  if (gnss_seen_ && gnss_age_.isValid() && gnss_age_.elapsed() > 3000) {
    gnss_seen_ = false;
    gnss_dot_->setStyleSheet("color: #777; font-size: 22px;");
    gnss_fix_label_->setText("GNSS auto start pose not running (start with --gnss)");
    gnss_status_label_->setText("");
    gnss_toggle_->setEnabled(false);
    gnss_toggle_->setChecked(false);
    gnss_toggle_->setText("GNSS auto-localize: OFF");
  }
}

void BuggyStatePanel::missionRequest(const QString & text)
{
  if (mission_pub_) {
    std_msgs::msg::String msg;
    msg.data = text.toStdString();
    mission_pub_->publish(msg);
  }
}

void BuggyStatePanel::setMissionState(const QString & text)
{
  QMap<QString, QString> kv;
  for (const QString & item : text.split(';')) {
    const int eq = item.indexOf('=');
    if (eq > 0) {
      kv[item.left(eq)] = item.mid(eq + 1);
    }
  }
  mission_seen_ = true;
  mission_age_.restart();
  const QString phase = kv.value("phase");
  QString line = QString("%1: stop %2 of %3 \"%4\"")
    .arg(phase.toUpper(), QString::number(kv.value("index").toInt() + 1), kv.value("stops"), kv.value("target"));
  if (phase == "waiting") {
    line += QString("  -  %1 s left").arg(kv.value("wait_left"));
  }
  line += QString("   (loop %1)").arg(kv.value("loop") == "true" ? "on" : "off");
  mission_label_->setText(line);
  mission_label_->setStyleSheet(
    phase == "aborted" ? "color: #ff8a80; font-weight: bold;" :
    (phase == "driving" || phase == "waiting") ? "color: #a5d6a7; font-weight: bold;" : "");
  mission_msg_label_->setText(kv.value("msg"));
  const bool running = phase == "planning" || phase == "driving" || phase == "waiting" || phase == "paused";
  mission_start_->setEnabled(!running);
  mission_stop_->setEnabled(running);
  mission_resume_->setEnabled(phase == "paused");
}

void BuggyStatePanel::checkMissionAlive()
{
  if (mission_seen_ && mission_age_.isValid() && mission_age_.elapsed() > 3000) {
    mission_seen_ = false;
    mission_label_->setText("mission node not running (start with --mission)");
    mission_label_->setStyleSheet("");
    mission_msg_label_->setText("");
    for (auto * b : {mission_start_, mission_stop_, mission_resume_}) {b->setEnabled(false);}
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
