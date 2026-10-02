// RViz displays built into the UI (no plugin needed): the occupancy map in track colours and the car's trail.
#pragma once

#include <QColor>

#include <deque>
#include <memory>

#include "rviz_common/display.hpp"
#include "rviz_default_plugins/displays/map/map_display.hpp"

namespace rviz_rendering {class BillboardLine;}

namespace f1ui
{
// The standard map display with the "map" colour scheme swapped for asphalt free space, white walls and
// transparent unknown space
class TrackMapDisplay : public rviz_default_plugins::displays::MapDisplay
{
public:
  static QColor freeColor() {return QColor(36, 39, 48);}
  static QColor wallColor() {return QColor(236, 238, 242);}

  const nav_msgs::msg::OccupancyGrid & map() const {return current_map_;}
  bool hasMap() const {return loaded_;}

protected:
  void onInitialize() override;
};

// The path the car drove, coloured by speed (blue slow ... red fast)
class TrailDisplay : public rviz_common::Display
{
public:
  TrailDisplay();
  ~TrailDisplay() override;

  void setSpeed(double speed) {speed_ = speed;}
  void clear();
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

protected:
  void onInitialize() override;
  void onDisable() override;
  void fixedFrameChanged() override;

private:
  void rebuild();

  struct Point
  {
    float x, y, z;
    float speed;
  };
  std::unique_ptr<rviz_rendering::BillboardLine> line_;
  std::deque<Point> points_;
  double speed_ = 0.0;
  double max_seen_speed_ = 3.0;
  int frames_since_rebuild_ = 0;
  bool dirty_ = false;
};
}  // namespace f1ui
