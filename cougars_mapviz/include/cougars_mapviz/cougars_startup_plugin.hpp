// Copyright (c) 2026 BYU FROST Lab
// Licensed under the Apache License, Version 2.0

#pragma once

#include <mapviz/mapviz_plugin.h>
#include <QWidget>
#include <QComboBox>
#include <QTimer>
#include <memory>
#include <vector>
#include <string>

namespace cougars_mapviz {

class CougarsStartupPlugin : public mapviz::MapvizPlugin {
  Q_OBJECT

 public:
  CougarsStartupPlugin();
  ~CougarsStartupPlugin() override;

  bool Initialize(QGLWidget* canvas) override;
  void Shutdown() override;
  void Draw(double x, double y, double scale) override;
  void Paint(QPainter* painter, double x, double y, double scale) override;
  void Transform() override;
  void LoadConfig(const YAML::Node& node, const std::string& path) override;
  void SaveConfig(YAML::Emitter& emitter, const std::string& path) override;
  QWidget* GetConfigWidget(QWidget* parent) override;
  bool SupportsPainting() override;

 protected:
  void PrintError(const std::string& message) override;
  void PrintInfo(const std::string& message) override;
  void PrintWarning(const std::string& message) override;
  bool eventFilter(QObject* object, QEvent* event) override;

 private slots:
  void DiscoverNamespaces();

 private:
  QWidget* config_widget_;
  QComboBox* ns_combo_;
  QTimer* namespace_discovery_timer_;
  std::vector<std::string> detected_namespaces_;
  // UI elements are created in the cpp file to avoid Qt header clutter here.

  void PublishClicked();
};

}  // namespace cougars_mapviz
