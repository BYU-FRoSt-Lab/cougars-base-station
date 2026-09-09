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
 * @file cougars_waypoint_manager.hpp
 * @brief MapViz plugin helper, manages waypoint storage and file I/O.
 */

#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace cougars_mapviz {

// ---------------------------------------------------------------------------
// Origin
// ---------------------------------------------------------------------------

struct GeoOrigin {
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  bool valid = false;
};

// ---------------------------------------------------------------------------
// Per-waypoint data
// ---------------------------------------------------------------------------

struct CougarsWaypoint {
  geometry_msgs::msg::Pose pose;  // position: x=lon, y=lat, z=depth_value

  // Required per waypoint
  std::string depth_ref = "surface";  // "surface" or "bottom"
  bool park = false;

  // Optional per-waypoint overrides (absent = use MissionDefaults)
  std::optional<double> speed;
  std::optional<double> slip_radius;
  std::optional<double> capture_radius;
};

// ---------------------------------------------------------------------------
// Mission-level defaults (apply to all waypoints without per-wp overrides)
// ---------------------------------------------------------------------------

struct MissionDefaults {
  int32_t mission_id = 0;
  double speed = 50.0;
  double slip_radius = 10.0;
  double capture_radius = 4.0;
  std::string agent_ns;  // used as YAML key on save; empty = use full topic name
};

// ---------------------------------------------------------------------------
// Manager
// ---------------------------------------------------------------------------

class CougarsWaypointManager {
 public:
  CougarsWaypointManager() = default;
  ~CougarsWaypointManager() = default;

  // Waypoint CRUD
  void addWaypoint(const std::string& topic, const CougarsWaypoint& waypoint);
  void setWaypoints(const std::string& topic, const std::vector<CougarsWaypoint>& waypoints);
  std::vector<CougarsWaypoint> getWaypoints(const std::string& topic) const;
  const std::map<std::string, std::vector<CougarsWaypoint>>& getAllWaypoints() const;
  void removeTopic(const std::string& topic);
  void clearWaypoints(const std::string& topic);
  void clearAllWaypoints();

  // Mission defaults
  MissionDefaults getDefaults(const std::string& topic) const;
  void setDefaults(const std::string& topic, const MissionDefaults& defaults);

  // Origin
  void setOrigin(const GeoOrigin& origin);
  GeoOrigin getOrigin() const;

  bool saveToFile(const std::string& filename, const std::string& topic = "") const;
  bool loadFromFile(const std::string& filename, const std::string& topic = "");

 private:
  std::map<std::string, std::vector<CougarsWaypoint>> waypoint_map_;
  std::map<std::string, MissionDefaults> defaults_map_;
  GeoOrigin origin_;
};

}  // namespace cougars_mapviz
