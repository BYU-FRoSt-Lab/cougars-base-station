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
 * @file cougars_origin_plugin.hpp
 * @brief Plugin that displays and edits the origin location with an X marker.
 */

#pragma once

#include <mapviz/map_canvas.h>
#include <mapviz/mapviz_plugin.h>
#include <swri_transform_util/frames.h>
#include <swri_transform_util/transform.h>

#include <QGLWidget>
#include <QObject>
#include <QPainter>
#include <QWidget>
#include <QLabel>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <string>
#include <vector>

namespace cougars_mapviz {

struct OriginPreset {
  std::string name;
  double latitude;
  double longitude;
  double altitude;
};

class CougarsOrigin : public mapviz::MapvizPlugin {
  Q_OBJECT

 public:
  CougarsOrigin();
  ~CougarsOrigin() override;

  bool Initialize(QGLWidget* canvas) override;
  void Shutdown() override {}
  void Draw(double x, double y, double scale) override;
  void Paint(QPainter* painter, double x, double y, double scale) override;
  void Transform() override {}
  void LoadConfig(const YAML::Node& node, const std::string& path) override;
  void SaveConfig(YAML::Emitter& emitter, const std::string& path) override;
  QWidget* GetConfigWidget(QWidget* parent) override;
  bool SupportsPainting() override { return true; }

 protected:
  void PrintError(const std::string& message) override;
  void PrintInfo(const std::string& message) override;
  void PrintWarning(const std::string& message) override;

 protected Q_SLOTS:
  void OriginCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void UpdateOriginDisplay();
  void PublishOrigin();
  void OnPresetSelected(int index);

 private:
  void LoadOriginPresets();

  QWidget* config_widget_;
  QComboBox* preset_combo_;
  QDoubleSpinBox* lat_spinbox_;
  QDoubleSpinBox* lon_spinbox_;
  QDoubleSpinBox* alt_spinbox_;
  QPushButton* publish_button_;
  mapviz::MapCanvas* map_canvas_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr origin_sub_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPoint>::SharedPtr publisher_;
  tf2::Vector3 origin_;
  std::vector<OriginPreset> presets_;
};

}  // namespace cougars_mapviz
