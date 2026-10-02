#include "scene_displays.hpp"

#include <OgreDataStream.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_rendering/objects/billboard_line.hpp"
#include "theme.hpp"

namespace f1ui
{
namespace
{
Ogre::TexturePtr paletteTexture(std::vector<unsigned char> bytes)
{
  static int count = 0;
  Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(bytes.data(), bytes.size()));
  return Ogre::TextureManager::getSingleton().loadRawData(
    "F1TrackPalette" + std::to_string(count++), "rviz_rendering", stream, 256, 1,
    Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_1D, 0);
}

// Occupancy 0..100 blends free -> wall; anything else (unknown = -1 = 255) is transparent
std::vector<unsigned char> trackPalette()
{
  std::vector<unsigned char> bytes(256 * 4, 0);
  QColor free = TrackMapDisplay::freeColor(), wall = TrackMapDisplay::wallColor();
  for (int v = 0; v <= 100; ++v) {
    double t = v / 100.0;
    bytes[v * 4 + 0] = static_cast<unsigned char>(free.red() + (wall.red() - free.red()) * t);
    bytes[v * 4 + 1] = static_cast<unsigned char>(free.green() + (wall.green() - free.green()) * t);
    bytes[v * 4 + 2] = static_cast<unsigned char>(free.blue() + (wall.blue() - free.blue()) * t);
    bytes[v * 4 + 3] = 255;
  }
  return bytes;
}
}  // namespace

void TrackMapDisplay::onInitialize()
{
  MapDisplay::onInitialize();
  // Index 0 is the "map" colour scheme (the default)
  palette_textures_[0] = paletteTexture(trackPalette());
  palette_textures_binary_[0] = palette_textures_[0];
  color_scheme_transparency_[0] = true;
}

TrailDisplay::TrailDisplay() = default;

TrailDisplay::~TrailDisplay()
{
  line_.reset();
}

void TrailDisplay::onInitialize()
{
  line_ = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
  line_->setLineWidth(0.05f);
  line_->setNumLines(1);
  line_->setColor(1.0f, 1.0f, 1.0f, 0.99f);  // alpha < 1 turns on blending, so old points can fade
}

void TrailDisplay::onDisable()
{
  clear();
}

void TrailDisplay::fixedFrameChanged()
{
  clear();
}

void TrailDisplay::reset()
{
  Display::reset();
  clear();
}

void TrailDisplay::clear()
{
  points_.clear();
  if (line_) {
    line_->clear();
  }
  dirty_ = false;
}

void TrailDisplay::update(float, float)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!line_ || !context_->getFrameManager()->getTransform(std::string("base_footprint"), position, orientation)) {
    return;
  }
  if (!points_.empty()) {
    const Point & last = points_.back();
    double d = std::hypot(position.x - last.x, position.y - last.y);
    if (d > 2.0) {
      clear();  // relocalized or teleported: don't draw a line across the map
    } else if (d < 0.04) {
      position = Ogre::Vector3(last.x, last.y, last.z);
    }
  }
  if (points_.empty() || position.x != points_.back().x || position.y != points_.back().y) {
    points_.push_back({position.x, position.y, position.z + 0.02f, static_cast<float>(std::abs(speed_))});
    max_seen_speed_ = std::max(max_seen_speed_, std::abs(speed_));
    if (points_.size() > 2500) {
      points_.pop_front();
    }
    dirty_ = true;
  }
  if (dirty_ && ++frames_since_rebuild_ >= 3) {
    rebuild();
  }
}

void TrailDisplay::rebuild()
{
  frames_since_rebuild_ = 0;
  dirty_ = false;
  line_->clear();
  line_->setMaxPointsPerLine(static_cast<uint32_t>(points_.size()));
  size_t i = 0;
  for (const auto & p : points_) {
    QColor c = theme::heat(p.speed / max_seen_speed_);
    // Older points fade out
    float age = static_cast<float>(i++) / std::max<size_t>(1, points_.size());
    line_->addPoint(Ogre::Vector3(p.x, p.y, p.z),
      Ogre::ColourValue(c.redF(), c.greenF(), c.blueF(), 0.25f + 0.75f * age));
  }
}
}  // namespace f1ui
