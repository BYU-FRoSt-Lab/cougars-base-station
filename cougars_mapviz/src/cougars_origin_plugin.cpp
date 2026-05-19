// Copyright (c) 2026 BYU FROST Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file cougars_origin_plugin.cpp
 * @brief Plugin that displays and edits the origin location with an X marker.
 */

#include <cougars_mapviz/cougars_origin_plugin.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <string>

PLUGINLIB_EXPORT_CLASS(cougars_mapviz::CougarsOrigin, mapviz::MapvizPlugin)

namespace cougars_mapviz {

static rclcpp::QoS originPublisherQoS() {
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  qos.transient_local();
  return qos;
}

CougarsOrigin::CougarsOrigin()
    : MapvizPlugin(), config_widget_(new QWidget()), map_canvas_(nullptr),
      origin_(0, 0, 0) {
  QVBoxLayout* layout = new QVBoxLayout(config_widget_);

  // Latitude
  QHBoxLayout* lat_layout = new QHBoxLayout();
  lat_layout->addWidget(new QLabel("Latitude:"));
  lat_spinbox_ = new QDoubleSpinBox();
  lat_spinbox_->setRange(-90, 90);
  lat_spinbox_->setDecimals(6);
  lat_layout->addWidget(lat_spinbox_);
  layout->addLayout(lat_layout);
  
  // Longitude
  QHBoxLayout* lon_layout = new QHBoxLayout();
  lon_layout->addWidget(new QLabel("Longitude:"));
  lon_spinbox_ = new QDoubleSpinBox();
  lon_spinbox_->setRange(-180, 180);
  lon_spinbox_->setDecimals(6);
  lon_layout->addWidget(lon_spinbox_);
  layout->addLayout(lon_layout);
  
  // Altitude
  QHBoxLayout* alt_layout = new QHBoxLayout();
  alt_layout->addWidget(new QLabel("Altitude:"));
  alt_spinbox_ = new QDoubleSpinBox();
  alt_spinbox_->setRange(-1000, 10000);
  alt_spinbox_->setDecimals(2);
  alt_layout->addWidget(alt_spinbox_);
  layout->addLayout(alt_layout);
  
  // Publish button
  publish_button_ = new QPushButton("Publish Origin");
  QObject::connect(publish_button_, SIGNAL(clicked()), this, SLOT(PublishOrigin()));
  layout->addWidget(publish_button_);
  
  layout->addStretch();
}

CougarsOrigin::~CougarsOrigin() {}

bool CougarsOrigin::Initialize(QGLWidget* canvas) {
  map_canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
  
    rclcpp::QoS origin_qos(rclcpp::KeepLast(1));
    origin_qos.transient_local();
    origin_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/local_xy_origin", origin_qos,
      std::bind(&CougarsOrigin::OriginCallback, this, std::placeholders::_1));

    publisher_ = node_->create_publisher<geographic_msgs::msg::GeoPoint>(
        "/origin", originPublisherQoS());

  initialized_ = true;
  return true;
}

void CougarsOrigin::Draw(double x, double y, double scale) {
  (void)x;
  (void)y;
  
  swri_transform_util::Transform transform;
  if (!tf_manager_->GetTransform(target_frame_, swri_transform_util::_wgs84_frame, transform)) {
    return;
  }
  
  tf2::Vector3 origin_transformed = transform * origin_;
  
  // Draw an X at the origin
  double arm_length = 12.5 * scale;
  
  // OpenGL marker (red)
  glColor4f(1.0f, 0.0f, 0.0f, 1.0f);  // Red
  glLineWidth(3.0f);
  glBegin(GL_LINES);
  
  // First diagonal: top-left to bottom-right
  glVertex2d(origin_transformed.x() - arm_length, origin_transformed.y() - arm_length);
  glVertex2d(origin_transformed.x() + arm_length, origin_transformed.y() + arm_length);
  
  // Second diagonal: top-right to bottom-left
  glVertex2d(origin_transformed.x() + arm_length, origin_transformed.y() - arm_length);
  glVertex2d(origin_transformed.x() - arm_length, origin_transformed.y() + arm_length);
  
  glEnd();
}

void CougarsOrigin::Paint(QPainter* painter, double x, double y, double scale) {
  // Drawing is handled in Draw() (OpenGL) which provides the correct placement.
  (void)painter;
  (void)x;
  (void)y;
  (void)scale;
  return;
}

void CougarsOrigin::LoadConfig(const YAML::Node& node, const std::string& path) {
  (void)node;
  (void)path;
}

void CougarsOrigin::SaveConfig(YAML::Emitter& emitter, const std::string& path) {
  (void)emitter;
  (void)path;
}

QWidget* CougarsOrigin::GetConfigWidget(QWidget* parent) {
  config_widget_->setParent(parent);
  return config_widget_;
}

void CougarsOrigin::PrintError(const std::string& message) {
  (void)message;
  // Stub implementation
}

void CougarsOrigin::PrintInfo(const std::string& message) {
  (void)message;
  // Stub implementation
}

void CougarsOrigin::PrintWarning(const std::string& message) {
  (void)message;
  // Stub implementation
}

void CougarsOrigin::OriginCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  origin_.setX(msg->pose.position.x);
  origin_.setY(msg->pose.position.y);
  origin_.setZ(msg->pose.position.z);
  UpdateOriginDisplay();
}

void CougarsOrigin::UpdateOriginDisplay() {
  lat_spinbox_->blockSignals(true);
  lon_spinbox_->blockSignals(true);
  alt_spinbox_->blockSignals(true);
  
  lat_spinbox_->setValue(origin_.y());
  lon_spinbox_->setValue(origin_.x());
  alt_spinbox_->setValue(origin_.z());
  
  lat_spinbox_->blockSignals(false);
  lon_spinbox_->blockSignals(false);
  alt_spinbox_->blockSignals(false);
}

void CougarsOrigin::PublishOrigin() {
  geographic_msgs::msg::GeoPoint geo;
  geo.latitude = lat_spinbox_->value();
  geo.longitude = lon_spinbox_->value();
  geo.altitude = alt_spinbox_->value();

  publisher_->publish(geo);

  origin_.setX(geo.longitude);
  origin_.setY(geo.latitude);
  origin_.setZ(geo.altitude);
  UpdateOriginDisplay();
  if (map_canvas_) {
    map_canvas_->update();
  }
}

}  // namespace cougars_mapviz
