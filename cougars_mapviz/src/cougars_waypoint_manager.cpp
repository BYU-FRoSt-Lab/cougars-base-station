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

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>
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
// JSON helpers
// ---------------------------------------------------------------------------

static QJsonObject defaultsToJson(const MissionDefaults& d) {
  QJsonObject obj;
  obj["mission_id"] = d.mission_id;
  obj["speed"] = d.speed;
  obj["slip_radius"] = d.slip_radius;
  obj["capture_radius"] = d.capture_radius;
  return obj;
}

static MissionDefaults defaultsFromJson(const QJsonObject& obj) {
  MissionDefaults d;
  if (obj.contains("mission_id")) d.mission_id = obj["mission_id"].toInt();
  if (obj.contains("speed")) d.speed = obj["speed"].toDouble();
  if (obj.contains("slip_radius")) d.slip_radius = obj["slip_radius"].toDouble();
  if (obj.contains("capture_radius")) d.capture_radius = obj["capture_radius"].toDouble();
  return d;
}

static QJsonArray waypointsToJson(const std::vector<CougarsWaypoint>& wps) {
  QJsonArray arr;
  for (const auto& wp : wps) {
    QJsonObject obj;
    obj["lon"] = wp.pose.position.x;
    obj["lat"] = wp.pose.position.y;
    obj["z"] = wp.pose.position.z;
    obj["depth_ref"] = QString::fromStdString(wp.depth_ref);
    obj["park"] = wp.park;
    if (wp.speed.has_value()) obj["speed"] = *wp.speed;
    if (wp.slip_radius.has_value()) obj["slip_radius"] = *wp.slip_radius;
    if (wp.capture_radius.has_value()) obj["capture_radius"] = *wp.capture_radius;
    arr.append(obj);
  }
  return arr;
}

static std::vector<CougarsWaypoint> waypointsFromJson(const QJsonArray& arr) {
  std::vector<CougarsWaypoint> wps;
  for (const auto& val : arr) {
    QJsonObject obj = val.toObject();
    if (!obj.contains("lat") || !obj.contains("lon")) continue;

    CougarsWaypoint wp;
    wp.pose.position.x = obj["lon"].toDouble();
    wp.pose.position.y = obj["lat"].toDouble();
    wp.pose.position.z = obj["z"].toDouble();
    wp.depth_ref =
        obj.contains("depth_ref") ? obj["depth_ref"].toString().toStdString() : "surface";
    wp.park = obj.contains("park") ? obj["park"].toBool() : false;
    if (obj.contains("speed")) wp.speed = obj["speed"].toDouble();
    if (obj.contains("slip_radius")) wp.slip_radius = obj["slip_radius"].toDouble();
    if (obj.contains("capture_radius")) wp.capture_radius = obj["capture_radius"].toDouble();
    wps.push_back(wp);
  }
  return wps;
}

// ---------------------------------------------------------------------------
// File I/O
// ---------------------------------------------------------------------------

bool CougarsWaypointManager::saveToFile(const std::string& filename,
                                     const std::string& specific_topic) const {
  QJsonObject root;

  if (origin_.valid) {
    QJsonObject orig;
    orig["latitude"] = origin_.latitude;
    orig["longitude"] = origin_.longitude;
    orig["altitude"] = origin_.altitude;
    root["origin"] = orig;
  }

  for (const auto& [topic, wps] : waypoint_map_) {
    if (!specific_topic.empty() && topic != specific_topic) continue;

    MissionDefaults defs;
    auto it = defaults_map_.find(topic);
    if (it != defaults_map_.end()) defs = it->second;

    QJsonObject entry;
    entry["topic"] = QString::fromStdString(topic);
    entry["defaults"] = defaultsToJson(defs);
    entry["waypoints"] = waypointsToJson(wps);

    QString key = defs.agent_ns.empty() ? QString::fromStdString(topic)
                                        : QString::fromStdString(defs.agent_ns);
    root[key] = entry;
  }

  QFile file(QString::fromStdString(filename));
  if (!file.open(QIODevice::WriteOnly)) return false;
  file.write(QJsonDocument(root).toJson());
  file.close();
  return true;
}

bool CougarsWaypointManager::loadFromFile(const std::string& filename,
                                       const std::string& specific_topic) {
  QFile file(QString::fromStdString(filename));
  if (!file.open(QIODevice::ReadOnly)) return false;

  QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
  file.close();
  if (!doc.isObject()) return false;

  QJsonObject root = doc.object();

  if (root.contains("origin")) {
    QJsonObject orig = root["origin"].toObject();
    GeoOrigin o;
    o.latitude = orig["latitude"].toDouble();
    o.longitude = orig["longitude"].toDouble();
    o.altitude = orig["altitude"].toDouble();
    o.valid = true;
    origin_ = o;
  }

  int loaded = 0;

  for (const QString& key : root.keys()) {
    if (key == "origin") continue;
    std::string topic = key.toStdString();
    if (!specific_topic.empty() && topic != specific_topic) continue;

    QJsonValue val = root[key];

    if (val.isArray()) {
      // Legacy format: array of waypoints, no defaults stored
      waypoint_map_[topic] = waypointsFromJson(val.toArray());
      defaults_map_[topic] = MissionDefaults{};
    } else if (val.isObject()) {
      QJsonObject entry = val.toObject();

      // If the entry has a "topic" field the key is an agent_ns, not the topic
      std::string actual_topic = entry.contains("topic")
                                     ? entry["topic"].toString().toStdString()
                                     : topic;

      if (entry.contains("waypoints")) {
        waypoint_map_[actual_topic] = waypointsFromJson(entry["waypoints"].toArray());
      }

      MissionDefaults defs = entry.contains("defaults")
                                 ? defaultsFromJson(entry["defaults"].toObject())
                                 : MissionDefaults{};
      if (entry.contains("topic")) defs.agent_ns = topic;  // key was the ns
      defaults_map_[actual_topic] = defs;
    }
    loaded++;
  }

  return loaded > 0;
}

}  // namespace cougars_mapviz
