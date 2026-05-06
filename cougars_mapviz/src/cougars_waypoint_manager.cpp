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
 * @file cougars_waypoint_manager.cpp
 * @brief Implementation of the CougarsWaypointManager.
 * @date Jan 2026
 */

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <cougars_mapviz/cougars_waypoint_manager.hpp>

namespace cougars_mapviz {

// ---------------------------------------------------------------------------
// Waypoint CRUD
// ---------------------------------------------------------------------------

void CougarsWaypointManager::addWaypoint(const std::string& topic, const CougarsWaypoint& waypoint) {
  waypoint_map_[topic].push_back(waypoint);
}

void CougarsWaypointManager::setWaypoints(const std::string& topic,
                                       const std::vector<CougarsWaypoint>& waypoints) {
  waypoint_map_[topic] = waypoints;
}

std::vector<CougarsWaypoint> CougarsWaypointManager::getWaypoints(const std::string& topic) const {
  auto it = waypoint_map_.find(topic);
  return (it != waypoint_map_.end()) ? it->second : std::vector<CougarsWaypoint>{};
}

const std::map<std::string, std::vector<CougarsWaypoint>>& CougarsWaypointManager::getAllWaypoints()
    const {
  return waypoint_map_;
}

void CougarsWaypointManager::clearWaypoints(const std::string& topic) {
  waypoint_map_[topic].clear();
}

void CougarsWaypointManager::clearAllWaypoints() { waypoint_map_.clear(); }

void CougarsWaypointManager::removeTopic(const std::string& topic) {
  waypoint_map_.erase(topic);
  defaults_map_.erase(topic);
}

// ---------------------------------------------------------------------------
// Mission defaults
// ---------------------------------------------------------------------------

MissionDefaults CougarsWaypointManager::getDefaults(const std::string& topic) const {
  auto it = defaults_map_.find(topic);
  return (it != defaults_map_.end()) ? it->second : MissionDefaults{};
}

void CougarsWaypointManager::setDefaults(const std::string& topic, const MissionDefaults& defaults) {
  defaults_map_[topic] = defaults;
}

void CougarsWaypointManager::setOrigin(const GeoOrigin& origin) { origin_ = origin; }
GeoOrigin CougarsWaypointManager::getOrigin() const { return origin_; }

// ---------------------------------------------------------------------------
// YAML helpers
// ---------------------------------------------------------------------------

static MissionDefaults defaultsFromYaml(const YAML::Node& node) {
  MissionDefaults d;
  if (node["mission_id"]) d.mission_id = node["mission_id"].as<int>();
  if (node["speed"]) d.speed = node["speed"].as<double>();
  if (node["slip_radius"]) d.slip_radius = node["slip_radius"].as<double>();
  if (node["capture_radius"]) d.capture_radius = node["capture_radius"].as<double>();
  return d;
}

static std::vector<CougarsWaypoint> waypointsFromYaml(const YAML::Node& seq) {
  std::vector<CougarsWaypoint> wps;
  for (const auto& node : seq) {
    if (!node["lat"] || !node["lon"]) continue;
    CougarsWaypoint wp;
    wp.pose.position.x = node["lon"].as<double>();
    wp.pose.position.y = node["lat"].as<double>();
    wp.pose.position.z = node["z"] ? node["z"].as<double>() : 0.0;
    wp.depth_ref = node["depth_ref"] ? node["depth_ref"].as<std::string>() : "surface";
    wp.park = node["park"] ? node["park"].as<bool>() : false;
    if (node["speed"]) wp.speed = node["speed"].as<double>();
    if (node["slip_radius"]) wp.slip_radius = node["slip_radius"].as<double>();
    if (node["capture_radius"]) wp.capture_radius = node["capture_radius"].as<double>();
    wps.push_back(wp);
  }
  return wps;
}

// ---------------------------------------------------------------------------
// File I/O
// ---------------------------------------------------------------------------

bool CougarsWaypointManager::saveToFile(const std::string& filename,
                                     const std::string& specific_topic) const {
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "mission_type" << YAML::Value << "waypoints";

  if (origin_.valid) {
    out << YAML::Key << "origin" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "altitude" << YAML::Value << origin_.altitude;
    out << YAML::Key << "latitude" << YAML::Value << origin_.latitude;
    out << YAML::Key << "longitude" << YAML::Value << origin_.longitude;
    out << YAML::EndMap;
  }

  for (const auto& [topic, wps] : waypoint_map_) {
    if (!specific_topic.empty() && topic != specific_topic) continue;

    MissionDefaults defs;
    auto it = defaults_map_.find(topic);
    if (it != defaults_map_.end()) defs = it->second;

    std::string key = defs.agent_ns.empty() ? topic : defs.agent_ns;
    out << YAML::Key << key << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "topic" << YAML::Value << topic;

    out << YAML::Key << "defaults" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "capture_radius" << YAML::Value << defs.capture_radius;
    out << YAML::Key << "mission_id" << YAML::Value << defs.mission_id;
    out << YAML::Key << "slip_radius" << YAML::Value << defs.slip_radius;
    out << YAML::Key << "speed" << YAML::Value << defs.speed;
    out << YAML::EndMap;

    out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;
    for (const auto& wp : wps) {
      out << YAML::BeginMap;
      out << YAML::Key << "depth_ref" << YAML::Value << wp.depth_ref;
      out << YAML::Key << "lat" << YAML::Value << wp.pose.position.y;
      out << YAML::Key << "lon" << YAML::Value << wp.pose.position.x;
      out << YAML::Key << "park" << YAML::Value << wp.park;
      out << YAML::Key << "z" << YAML::Value << wp.pose.position.z;
      if (wp.speed.has_value()) out << YAML::Key << "speed" << YAML::Value << *wp.speed;
      if (wp.slip_radius.has_value())
        out << YAML::Key << "slip_radius" << YAML::Value << *wp.slip_radius;
      if (wp.capture_radius.has_value())
        out << YAML::Key << "capture_radius" << YAML::Value << *wp.capture_radius;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
  }

  out << YAML::EndMap;

  std::ofstream file(filename);
  if (!file.is_open()) return false;
  file << out.c_str() << '\n';
  return true;
}

bool CougarsWaypointManager::loadFromFile(const std::string& filename,
                                       const std::string& specific_topic) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(filename);
  } catch (const YAML::Exception&) {
    return false;
  }
  if (!root.IsMap()) return false;

  if (root["origin"]) {
    YAML::Node orig = root["origin"];
    GeoOrigin o;
    o.latitude = orig["latitude"].as<double>(0.0);
    o.longitude = orig["longitude"].as<double>(0.0);
    o.altitude = orig["altitude"].as<double>(0.0);
    o.valid = true;
    origin_ = o;
  }

  int loaded = 0;

  for (auto it = root.begin(); it != root.end(); ++it) {
    std::string key = it->first.as<std::string>();
    if (key == "origin" || key == "mission_type") continue;
    if (!specific_topic.empty() && key != specific_topic) continue;

    YAML::Node val = it->second;

    if (val.IsSequence()) {
      // Legacy format: bare array of waypoints
      waypoint_map_[key] = waypointsFromYaml(val);
      defaults_map_[key] = MissionDefaults{};
    } else if (val.IsMap()) {
      // If the entry has a "topic" field the key is an agent_ns, not the topic
      std::string actual_topic = val["topic"] ? val["topic"].as<std::string>() : key;

      if (val["waypoints"]) {
        waypoint_map_[actual_topic] = waypointsFromYaml(val["waypoints"]);
      }

      MissionDefaults defs = val["defaults"] ? defaultsFromYaml(val["defaults"]) : MissionDefaults{};
      if (val["topic"]) defs.agent_ns = key;
      defaults_map_[actual_topic] = defs;
    }
    loaded++;
  }

  return loaded > 0;
}

}  // namespace cougars_mapviz
