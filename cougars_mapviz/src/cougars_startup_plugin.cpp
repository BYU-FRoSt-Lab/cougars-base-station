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
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <cougars_interfaces/msg/system_control.hpp>

PLUGINLIB_EXPORT_CLASS(cougars_mapviz::CougarsStartupPlugin, mapviz::MapvizPlugin)

namespace cougars_mapviz {

CougarsStartupPlugin::CougarsStartupPlugin()
    : MapvizPlugin(), config_widget_(new QWidget()) {
}

CougarsStartupPlugin::~CougarsStartupPlugin() {
  delete config_widget_;
}

bool CougarsStartupPlugin::Initialize(QGLWidget* canvas) {
  (void)canvas;

  // Build UI
  auto layout = new QFormLayout(config_widget_);

  auto ns_combo = new QComboBox(config_widget_);
  ns_combo->addItems({"coug0", "coug1", "coug2", "coug3"});
  layout->addRow(QString("Vehicle Namespace"), ns_combo);

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

  QObject::connect(publish_btn, &QPushButton::clicked, [this, ns_combo, start_chk, rosbag_chk, rosbag_prefix, thruster_chk, dvl_chk]() {
    auto msg = cougars_interfaces::msg::SystemControl();
    msg.header.stamp = node_->now();

    msg.start.data = start_chk->isChecked();
    msg.rosbag_flag.data = rosbag_chk->isChecked();
    msg.rosbag_prefix = rosbag_prefix->text().toStdString();
    msg.thruster_arm.data = thruster_chk->isChecked();
    msg.dvl_acoustics.data = dvl_chk->isChecked();

    std::string ns = ns_combo->currentText().toStdString();
    std::string t = ns + "/system/control";
    auto pub = node_->create_publisher<cougars_interfaces::msg::SystemControl>(t, rclcpp::QoS(10));
    pub->publish(msg);

    PrintInfo("Published SystemControl to " + t);
  });

  return true;
}

QWidget* CougarsStartupPlugin::GetConfigWidget(QWidget* parent) {
  (void)parent;
  return config_widget_;
}

void CougarsStartupPlugin::Shutdown() {
  // no-op
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

}  // namespace cougars_mapviz

