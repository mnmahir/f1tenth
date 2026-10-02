// Maps and paths on disk: occupancy maps (map_server YAML + PGM) and racelines (CSV rows of x,y,speed).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace f1tenth_tools
{
namespace fs = std::filesystem;

// Where paths are looked up: the writable paths_dir first, then the racelines that ship with ugv_race
inline std::vector<std::string> path_dirs(const std::string & paths_dir)
{
  std::vector<std::string> dirs{paths_dir};
  try {
    dirs.push_back(ament_index_cpp::get_package_share_directory("ugv_race") + "/racelines");
  } catch (...) {
  }
  return dirs;
}

// File for a path name, '' if not found
inline std::string find_path_file(const std::string & name, const std::string & paths_dir)
{
  for (const auto & dir : path_dirs(paths_dir)) {
    std::string file = dir + "/" + name + ".csv";
    if (fs::exists(file)) {
      return file;
    }
  }
  return "";
}

inline std::string expand_user(const std::string & path)
{
  if (!path.empty() && path[0] == '~') {
    const char * home = std::getenv("HOME");
    return std::string(home ? home : "") + path.substr(1);
  }
  return path;
}

// Names become file stems, so keep them simple
inline bool valid_name(const std::string & name)
{
  return !name.empty() && name.size() <= 64 &&
         std::all_of(name.begin(), name.end(), [](char c) {
           return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
         });
}

// Sorted file stems with the given extension in a directory
inline std::vector<std::string> list_stems(const std::string & dir, const std::string & ext)
{
  std::vector<std::string> names;
  std::error_code ec;
  if (!fs::is_directory(dir, ec)) {
    return names;
  }
  for (const auto & entry : fs::directory_iterator(dir, ec)) {
    if (entry.is_regular_file() && entry.path().extension() == ext) {
      names.push_back(entry.path().stem().string());
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

struct PathPoint
{
  double x, y, v;
};

inline bool load_path_csv(const std::string & file, std::vector<PathPoint> & points, std::string & error)
{
  std::ifstream in(file);
  if (!in) {
    error = "cannot open " + file;
    return false;
  }
  points.clear();
  std::string line;
  while (std::getline(in, line)) {
    std::stringstream ss(line);
    std::string cell;
    std::vector<double> values;
    while (std::getline(ss, cell, ',')) {
      try {
        values.push_back(std::stod(cell));
      } catch (...) {
        break;  // header or comment line
      }
    }
    if (values.size() >= 2) {
      points.push_back({values[0], values[1], values.size() >= 3 ? values[2] : 0.0});
    }
  }
  if (points.size() < 2) {
    error = file + " has fewer than 2 points";
    return false;
  }
  return true;
}

inline bool save_path_csv(const std::string & file, const std::vector<PathPoint> & points, std::string & error)
{
  std::ofstream out(file);
  if (!out) {
    error = "cannot write " + file;
    return false;
  }
  out.setf(std::ios::fixed);
  out.precision(7);
  for (const auto & p : points) {
    out << p.x << ',' << p.y << ',' << p.v << '\n';
  }
  return true;
}

// A path's coordinates only make sense on the map it was made on: <name>.yaml next to <name>.csv says which
// (the CSV itself stays plain x,y,v for the path follower)
struct PathInfo
{
  std::string map;      // empty if unknown (e.g. paths from before this existed)
  std::string kind;     // recorded | raceline
  std::string source;   // racelines: the path they were optimized from
  std::string created;
};

inline std::string path_info_file(const std::string & csv_file)
{
  return fs::path(csv_file).replace_extension(".yaml").string();
}

inline PathInfo load_path_info(const std::string & csv_file)
{
  PathInfo info;
  try {
    YAML::Node node = YAML::LoadFile(path_info_file(csv_file));
    if (node["map"]) {info.map = node["map"].as<std::string>();}
    if (node["kind"]) {info.kind = node["kind"].as<std::string>();}
    if (node["source"]) {info.source = node["source"].as<std::string>();}
    if (node["created"]) {info.created = node["created"].as<std::string>();}
  } catch (const std::exception &) {
    // no metadata: map unknown
  }
  return info;
}

inline bool save_path_info(const std::string & csv_file, PathInfo info)
{
  if (info.created.empty()) {
    char text[32];
    std::time_t t = std::time(nullptr);
    std::strftime(text, sizeof(text), "%Y-%m-%d %H:%M", std::localtime(&t));
    info.created = text;
  }
  YAML::Emitter yaml;
  yaml << YAML::BeginMap << YAML::Key << "map" << YAML::Value << info.map << YAML::Key << "kind" << YAML::Value <<
    info.kind << YAML::Key << "source" << YAML::Value << info.source << YAML::Key << "created" << YAML::Value <<
    info.created << YAML::EndMap;
  std::ofstream out(path_info_file(csv_file));
  out << "# Which map this path belongs to (written by f1tenth_tools)\n" << yaml.c_str() << "\n";
  return static_cast<bool>(out);
}

struct OccupancyMap
{
  int width = 0, height = 0;
  double resolution = 0.05;
  double origin_x = 0.0, origin_y = 0.0, origin_yaw = 0.0;
  // Row 0 is the top of the image (as stored in the PGM); 0 free, 100 occupied, -1 unknown
  std::vector<int8_t> cells;

  int8_t at_world(double x, double y) const
  {
    // map_server places the bottom-left pixel at the origin (yaw assumed 0)
    int col = static_cast<int>(std::floor((x - origin_x) / resolution));
    int row_from_bottom = static_cast<int>(std::floor((y - origin_y) / resolution));
    if (col < 0 || col >= width || row_from_bottom < 0 || row_from_bottom >= height) {
      return -1;
    }
    return cells[static_cast<size_t>(height - 1 - row_from_bottom) * width + col];
  }
};

// Reads a map_server YAML and its PGM image (P5 binary or P2 ASCII)
inline bool load_map(const std::string & yaml_file, OccupancyMap & map, std::string & error)
{
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(yaml_file);
  } catch (const std::exception & e) {
    error = "cannot read " + yaml_file + ": " + e.what();
    return false;
  }
  fs::path image = doc["image"].as<std::string>("");
  if (image.is_relative()) {
    image = fs::path(yaml_file).parent_path() / image;
  }
  map.resolution = doc["resolution"].as<double>(0.05);
  auto origin = doc["origin"];
  if (origin && origin.size() >= 2) {
    map.origin_x = origin[0].as<double>();
    map.origin_y = origin[1].as<double>();
    map.origin_yaw = origin.size() >= 3 ? origin[2].as<double>() : 0.0;
  }
  bool negate = doc["negate"].as<int>(0) != 0;
  double occupied_thresh = doc["occupied_thresh"].as<double>(0.65);
  double free_thresh = doc["free_thresh"].as<double>(0.196);

  std::ifstream in(image, std::ios::binary);
  if (!in) {
    error = "cannot open " + image.string();
    return false;
  }
  std::string magic;
  in >> magic;
  auto next_int = [&in]() {
    int value = 0;
    while (in >> std::ws && in.peek() == '#') {
      std::string comment;
      std::getline(in, comment);
    }
    in >> value;
    return value;
  };
  if (magic != "P5" && magic != "P2") {
    error = image.string() + " is not a PGM image";
    return false;
  }
  map.width = next_int();
  map.height = next_int();
  int max_value = next_int();
  if (map.width <= 0 || map.height <= 0 || max_value <= 0 || max_value > 255) {
    error = image.string() + " has an unsupported PGM header";
    return false;
  }
  in.get();  // single whitespace before the data
  std::vector<uint8_t> pixels(static_cast<size_t>(map.width) * map.height);
  if (magic == "P5") {
    in.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  } else {
    for (auto & p : pixels) {
      p = static_cast<uint8_t>(next_int());
    }
  }
  if (!in) {
    error = image.string() + " is truncated";
    return false;
  }
  map.cells.resize(pixels.size());
  for (size_t i = 0; i < pixels.size(); ++i) {
    double value = static_cast<double>(pixels[i]) / max_value;
    double occupancy = negate ? value : 1.0 - value;
    map.cells[i] = occupancy > occupied_thresh ? 100 : (occupancy < free_thresh ? 0 : -1);
  }
  return true;
}

}  // namespace f1tenth_tools
