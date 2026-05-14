// Copyright (c) 2026 BYU FROST Lab
// Licensed under the Apache License, Version 2.0

#include <pluginlib/class_list_macros.hpp>
#include "cougars_mapviz/cougars_startup_plugin.hpp"

#include <QComboBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QFormLayout>
#include <QPushButton>
#include <QPainter>
#include <QEvent>

#include <iostream>
#include <algorithm>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <cougars_interfaces/msg/system_control.hpp>

PLUGINLIB_EXPORT_CLASS(cougars_mapviz::CougarsStartupPlugin, mapviz::MapvizPlugin)

namespace cougars_mapviz {

CougarsStartupPlugin::CougarsStartupPlugin()
    : MapvizPlugin(), config_widget_(new QWidget()), ns_combo_(nullptr), namespace_discovery_timer_(nullptr) {
}

CougarsStartupPlugin::~CougarsStartupPlugin() {
  if (namespace_discovery_timer_) {
    namespace_discovery_timer_->stop();
    delete namespace_discovery_timer_;
  }
  delete config_widget_;
}

bool CougarsStartupPlugin::Initialize(QGLWidget* canvas) {
  (void)canvas;

  // Build UI
  auto layout = new QFormLayout(config_widget_);

  ns_combo_ = new QComboBox(config_widget_);
  layout->addRow(QString("Publish Topic"), ns_combo_);

  auto start_chk = new QCheckBox(config_widget_);
  layout->addRow(QString("Start"), start_chk);

  auto rosbag_chk = new QCheckBox(config_widget_);
  layout->addRow(QString("Record Rosbag"), rosbag_chk);

  auto rosbag_prefix = new QLineEdit(config_widget_);
  layout->addRow(QString("Rosbag Prefix"), rosbag_prefix);

  auto thruster_chk = new QCheckBox(config_widget_);
  layout->addRow(QString("Thruster Arm"), thruster_chk);

  auto dvl_chk = new QCheckBox(config_widget_);
  layout->addRow(QString("DVL Acoustics"), dvl_chk);

  auto publish_btn = new QPushButton("Publish", config_widget_);
  layout->addRow(publish_btn);

  QObject::connect(publish_btn, &QPushButton::clicked, [this, start_chk, rosbag_chk, rosbag_prefix, thruster_chk, dvl_chk]() {
    auto msg = cougars_interfaces::msg::SystemControl();
    msg.header.stamp = node_->now();

    msg.start.data = start_chk->isChecked();
    msg.rosbag_flag.data = rosbag_chk->isChecked();
    msg.rosbag_prefix = rosbag_prefix->text().toStdString();
    msg.thruster_arm.data = thruster_chk->isChecked();
    msg.dvl_acoustics.data = dvl_chk->isChecked();

    std::string t = "/" + ns_combo_->currentText().toStdString();  // Full topic name from combo box
    auto pub = node_->create_publisher<cougars_interfaces::msg::SystemControl>(t, rclcpp::QoS(10));
    pub->publish(msg);

    PrintInfo("Published SystemControl to " + t);
  });

  // Set up discovery timer (same pattern as waypoint plugin)
  namespace_discovery_timer_ = new QTimer(this);
  QObject::connect(namespace_discovery_timer_, SIGNAL(timeout()), this, SLOT(DiscoverNamespaces()));
  namespace_discovery_timer_->start(1000);
  DiscoverNamespaces();

  return true;
}

QWidget* CougarsStartupPlugin::GetConfigWidget(QWidget* parent) {
  (void)parent;
  return config_widget_;
}

void CougarsStartupPlugin::Shutdown() {
  if (namespace_discovery_timer_) {
    namespace_discovery_timer_->stop();
  }
}

void CougarsStartupPlugin::Draw(double x, double y, double scale) {
  (void)x; (void)y; (void)scale;
}

void CougarsStartupPlugin::Paint(QPainter* painter, double x, double y, double scale) {
  (void)painter; (void)x; (void)y; (void)scale;
}

void CougarsStartupPlugin::Transform() {
  // nothing
}

void CougarsStartupPlugin::LoadConfig(const YAML::Node& node, const std::string& path) {
  (void)node; (void)path;
}

void CougarsStartupPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path) {
  (void)emitter; (void)path;
}

bool CougarsStartupPlugin::SupportsPainting() {
  return false;
}

void CougarsStartupPlugin::PrintError(const std::string& message) {
  std::cerr << "[CougarsStartupPlugin] ERROR: " << message << std::endl;
}

void CougarsStartupPlugin::PrintInfo(const std::string& message) {
  std::cout << "[CougarsStartupPlugin] INFO: " << message << std::endl;
}

void CougarsStartupPlugin::PrintWarning(const std::string& message) {
  std::cerr << "[CougarsStartupPlugin] WARN: " << message << std::endl;
}

bool CougarsStartupPlugin::eventFilter(QObject* object, QEvent* event) {
  (void)object; (void)event;
  return false;
}

void CougarsStartupPlugin::DiscoverNamespaces() {
  auto topics_and_types = node_->get_topic_names_and_types();
  std::vector<std::string> system_control_topics;
  
  for (const auto& [topic, types] : topics_and_types) {
    // Look for /system/control topics
    if (topic.find("/system/control") != std::string::npos && 
        topic.substr(topic.find("/system/control")) == "/system/control") {
      // Remove leading '/' if present
      std::string clean_topic = topic;
      if (!clean_topic.empty() && clean_topic[0] == '/') {
        clean_topic = clean_topic.substr(1);
      }
      system_control_topics.push_back(clean_topic);
    }
  }
  
  // Sort topics
  std::sort(system_control_topics.begin(), system_control_topics.end());
  
  // Only update UI if topics have changed
  if (system_control_topics != detected_namespaces_) {
    detected_namespaces_ = system_control_topics;
    
    // Store current selection
    QString current_selection = ns_combo_->currentText();
    
    // Clear and repopulate combo box
    ns_combo_->clear();
    for (const auto& topic : system_control_topics) {
      ns_combo_->addItem(QString::fromStdString(topic));
    }
    
    // Restore selection if still available
    int index = ns_combo_->findText(current_selection);
    if (index >= 0) {
      ns_combo_->setCurrentIndex(index);
    }
  }
}

}  // namespace cougars_mapviz

