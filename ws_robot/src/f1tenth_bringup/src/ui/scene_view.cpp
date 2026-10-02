#include "scene_view.hpp"

#include <QTimer>
#include <QVBoxLayout>

#include <OgreQuaternion.h>
#include <OgreVector.h>

#include <algorithm>
#include <cmath>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_group.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/tool_manager.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/render_window.hpp"
#include "scene_displays.hpp"
#include "theme.hpp"

namespace f1ui
{
namespace
{
using rviz_common::properties::Property;

Property * findProperty(Property * root, const QString & path)
{
  Property * p = root;
  for (const QString & name : path.split('/')) {
    Property * next = nullptr;
    for (int i = 0; p && i < p->numChildren(); ++i) {
      if (p->childAt(i)->getName() == name) {
        next = p->childAt(i);
        break;
      }
    }
    if (!next) {
      return nullptr;
    }
    p = next;
  }
  return p;
}

QVariant color(const QColor & c) {return QVariant::fromValue(c);}

constexpr char kTransientLocal[] = "Transient Local";
constexpr char kBestEffort[] = "Best Effort";
}  // namespace

SceneView::SceneView(const std::string & node_name, QWidget * parent)
: QWidget(parent), node_name_(node_name)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  panel_ = new rviz_common::RenderPanel(this);
  layout->addWidget(panel_);
  setMinimumSize(320, 200);

  for (Layer layer : layers()) {
    visible_[layer] = true;
  }
  visible_[Layer::Raceline] = false;
  visible_[Layer::Recording] = false;

  retry_timer_ = new QTimer(this);
  connect(retry_timer_, &QTimer::timeout, this, &SceneView::retryPending);
}

SceneView::~SceneView()
{
  if (manager_) {
    manager_->stopUpdate();
    for (auto & [view, controller] : saved_views_) {
      delete controller;
    }
    saved_views_.clear();
    delete manager_;
    manager_ = nullptr;
  }
  delete panel_;
  panel_ = nullptr;
}

void SceneView::initialize()
{
  if (manager_) {
    return;
  }
  rviz_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(node_name_);
  panel_->getRenderWindow()->initialize();
  auto clock = rviz_node_->get_raw_node()->get_clock();
  manager_ = new rviz_common::VisualizationManager(panel_, rviz_node_, this, clock);
  panel_->initialize(manager_);
  manager_->setFixedFrame(fixed_frame_);
  manager_->initialize();
  connect(manager_, &rviz_common::VisualizationManager::statusUpdate, this, &SceneView::statusMessage);

  setProperties(manager_->getRootDisplayGroup(), {
    {"Global Options/Background Color", color(QColor(13, 14, 19))},
    {"Global Options/Frame Rate", 30},
  });

  auto * tools = manager_->getToolManager();
  interact_tool_ = tools->addTool("rviz_default_plugins/Interact");
  pose_tool_ = tools->addTool("rviz_default_plugins/SetInitialPose");
  measure_tool_ = tools->addTool("rviz_default_plugins/Measure");
  tools->setDefaultTool(interact_tool_);
  tools->setCurrentTool(interact_tool_);
  connect(tools, &rviz_common::ToolManager::toolChanged, this, [this](rviz_common::Tool * tool) {
      if (tool == interact_tool_) {
        Q_EMIT toolFinished();
      }
    });

  createDisplays();
  applyPreset(view_);
  manager_->startUpdate();
  retry_timer_->start(500);
  try {
    ament_index_cpp::get_package_share_directory("ugv_description");
  } catch (const std::exception &) {
    QTimer::singleShot(3000, this, [this]() {
        Q_EMIT statusMessage("The 3D car needs ugv_description built on this computer (see the README)");
      });
  }
}

rviz_common::Display * SceneView::addDisplay(Layer layer, const QString & class_id, const QString & name,
  const std::vector<std::pair<QString, QVariant>> & properties)
{
  auto * display = manager_->createDisplay(class_id, name, visible_[layer]);
  if (display) {
    setProperties(display, properties);
    layers_[layer].push_back(display);
  }
  return display;
}

void SceneView::setProperties(Property * root, const std::vector<std::pair<QString, QVariant>> & properties)
{
  for (const auto & [path, value] : properties) {
    if (auto * p = findProperty(root, path)) {
      p->setValue(value);
    } else {
      pending_.push_back({root, path, value});
    }
  }
}

void SceneView::retryPending()
{
  // Some properties (e.g. a point cloud's colour) only exist once the display has received data
  std::vector<Pending> still;
  for (const auto & item : pending_) {
    if (auto * p = findProperty(item.root, item.path)) {
      p->setValue(item.value);
    } else {
      still.push_back(item);
    }
  }
  pending_.swap(still);
}

void SceneView::createDisplays()
{
  addDisplay(Layer::Grid, "rviz_default_plugins/Grid", "Grid", {
    {"Cell Size", 1.0}, {"Plane Cell Count", 80}, {"Color", color(QColor(44, 48, 60))}, {"Alpha", 0.55},
    {"Offset/Z", -0.02}});

  track_map_ = new TrackMapDisplay();
  manager_->addDisplay(track_map_, visible_[Layer::Map]);
  track_map_->setName("Track");
  setProperties(track_map_, {
    {"Topic/Durability Policy", kTransientLocal}, {"Topic", "/map"}, {"Alpha", 0.99}});
  layers_[Layer::Map].push_back(track_map_);
  connect(track_map_, &rviz_default_plugins::displays::MapDisplay::mapUpdated, this, &SceneView::onMapUpdated);

  addDisplay(Layer::Lidar, "rviz_default_plugins/LaserScan", "Lidar", {
    {"Topic/Reliability Policy", kBestEffort}, {"Topic", "/scan"}, {"Style", "Flat Squares"},
    {"Size (m)", 0.035}, {"Color Transformer", "FlatColor"}, {"Color", color(theme::redBright)}});

  addDisplay(Layer::Car, "rviz_default_plugins/RobotModel", "Car", {
    {"Description Source", "Topic"}, {"Description Topic/Durability Policy", kTransientLocal},
    {"Description Topic", "/robot_description"}});

  addDisplay(Layer::Path, "rviz_default_plugins/Path", "Selected path", {
    {"Topic/Durability Policy", kTransientLocal}, {"Topic", "/f1tenth/selected_path"},
    {"Line Style", "Billboards"}, {"Line Width", 0.04}, {"Color", color(theme::purple)}, {"Alpha", 0.9}});
  addDisplay(Layer::Follower, "rviz_default_plugins/Marker", "Lookahead", {{"Topic", "/lookahead_waypoint"}});
  addDisplay(Layer::Follower, "rviz_default_plugins/Marker", "Current waypoint", {{"Topic", "/current_waypoint"}});

  addDisplay(Layer::Recording, "rviz_default_plugins/Path", "Recording", {
    {"Topic/Durability Policy", kTransientLocal}, {"Topic", "/f1tenth/recorded_path"},
    {"Line Style", "Billboards"}, {"Line Width", 0.05}, {"Color", color(theme::green)}});
  addDisplay(Layer::Raceline, "rviz_default_plugins/MarkerArray", "Raceline", {
    {"Topic/Durability Policy", kTransientLocal}, {"Topic", "/f1tenth/raceline/markers"}});

  addDisplay(Layer::Safety, "rviz_default_plugins/Marker", "Predicted trajectory", {{"Topic", "/trajectory_marker"}});
  addDisplay(Layer::Safety, "rviz_default_plugins/Polygon", "Force-stop zone", {
    {"Topic", "/safety/force_stop_boundary"}, {"Color", color(theme::redBright)}});
  addDisplay(Layer::Safety, "rviz_default_plugins/Polygon", "Collision-check zone", {
    {"Topic", "/safety/ittc_stop_boundary"}, {"Color", color(theme::yellow)}});
  addDisplay(Layer::Safety, "rviz_default_plugins/Polygon", "Avoidance zone", {
    {"Topic", "/avoidance_control_boundary"}, {"Color", color(theme::orange)}});
  addDisplay(Layer::Safety, "rviz_default_plugins/Marker", "Avoidance target", {{"Topic", "/best_point_marker"}});

  trail_ = new TrailDisplay();
  manager_->addDisplay(trail_, visible_[Layer::Trail]);
  trail_->setName("Trail");
  layers_[Layer::Trail].push_back(trail_);
}

std::vector<SceneView::Layer> SceneView::layers()
{
  return {Layer::Grid, Layer::Map, Layer::Lidar, Layer::Car, Layer::Path, Layer::Follower, Layer::Recording,
    Layer::Raceline, Layer::Safety, Layer::Trail};
}

QString SceneView::layerName(Layer layer)
{
  switch (layer) {
    case Layer::Grid: return "Grid";
    case Layer::Map: return "Track map";
    case Layer::Lidar: return "Lidar";
    case Layer::Car: return "Car";
    case Layer::Path: return "Selected path";
    case Layer::Follower: return "Path follower targets";
    case Layer::Recording: return "Recorded path";
    case Layer::Raceline: return "Raceline";
    case Layer::Safety: return "Safety zones & trajectory";
    case Layer::Trail: return "Speed trail";
  }
  return QString();
}

void SceneView::setLayerVisible(Layer layer, bool visible)
{
  visible_[layer] = visible;
  for (auto * display : layers_[layer]) {
    display->setEnabled(visible);
  }
}

bool SceneView::layerVisible(Layer layer) const
{
  auto it = visible_.find(layer);
  return it != visible_.end() && it->second;
}

void SceneView::setFixedFrame(const QString & frame)
{
  if (frame == fixed_frame_) {
    return;
  }
  fixed_frame_ = frame;
  if (manager_) {
    manager_->setFixedFrame(frame);
  }
}

void SceneView::setView(View view)
{
  if (!manager_) {
    view_ = view;
    return;
  }
  auto * vm = manager_->getViewManager();
  if (view == view_ && vm->getCurrent()) {
    return;
  }
  // Remember how the user left this view, so switching back keeps their zoom and angle
  if (vm->getCurrent()) {
    delete saved_views_[view_];
    saved_views_[view_] = vm->copy(vm->getCurrent());
  }
  view_ = view;
  auto saved = saved_views_.find(view);
  if (saved != saved_views_.end() && saved->second) {
    vm->setCurrentFrom(saved->second);
  } else {
    applyPreset(view);
  }
}

void SceneView::resetView()
{
  if (!manager_) {
    return;
  }
  delete saved_views_[view_];
  saved_views_.erase(view_);
  applyPreset(view_);
}

void SceneView::setFollow(bool follow)
{
  follow_ = follow;
  if (manager_ && view_ == View::Bird) {
    if (auto * vc = manager_->getViewManager()->getCurrent()) {
      setProperties(vc, {{"Target Frame", follow ? "base_footprint" : "<Fixed Frame>"}, {"X", 0.0}, {"Y", 0.0}});
    }
  }
}

void SceneView::applyPreset(View view)
{
  auto * vm = manager_->getViewManager();
  switch (view) {
    case View::Bird:
      vm->setCurrentViewControllerType("rviz_default_plugins/TopDownOrtho");
      setProperties(vm->getCurrent(), {
        {"Target Frame", follow_ ? "base_footprint" : "<Fixed Frame>"}, {"Scale", 70.0}, {"Angle", 0.0},
        {"X", 0.0}, {"Y", 0.0}});
      break;
    case View::Chase:
      // Behind and above the car, turning with it
      vm->setCurrentViewControllerType("rviz_default_plugins/ThirdPersonFollower");
      setProperties(vm->getCurrent(), {
        {"Target Frame", "base_footprint"}, {"Distance", 2.6}, {"Yaw", M_PI}, {"Pitch", 0.32},
        {"Focal Point/X", 0.8}, {"Focal Point/Y", 0.0}, {"Focal Point/Z", 0.1}, {"Near Clip Distance", 0.05}});
      break;
    case View::Onboard:
      // Roughly where the camera sits (on top, behind the lidar), looking ahead
      vm->setCurrentViewControllerType("rviz_default_plugins/ThirdPersonFollower");
      setProperties(vm->getCurrent(), {
        {"Target Frame", "base_footprint"}, {"Distance", 1.3}, {"Yaw", M_PI}, {"Pitch", 0.05},
        {"Focal Point/X", 1.5}, {"Focal Point/Y", 0.0}, {"Focal Point/Z", 0.2}, {"Near Clip Distance", 0.05}});
      break;
    case View::Orbit:
      vm->setCurrentViewControllerType("rviz_default_plugins/Orbit");
      setProperties(vm->getCurrent(), {
        {"Target Frame", "base_footprint"}, {"Distance", 6.0}, {"Yaw", 3.9}, {"Pitch", 0.75},
        {"Focal Point/X", 0.0}, {"Focal Point/Y", 0.0}, {"Focal Point/Z", 0.0}});
      break;
  }
}

void SceneView::showWholeMap()
{
  if (!manager_ || map_image_.isNull()) {
    return;
  }
  setView(View::Bird);
  follow_ = false;
  double w = map_image_.width() * map_resolution_, h = map_image_.height() * map_resolution_;
  double scale = 0.92 * std::min(panel_->width() / std::max(w, 0.1), panel_->height() / std::max(h, 0.1));
  if (auto * vc = manager_->getViewManager()->getCurrent()) {
    setProperties(vc, {{"Target Frame", "<Fixed Frame>"}, {"X", map_origin_x_ + w / 2.0},
      {"Y", map_origin_y_ + h / 2.0}, {"Scale", scale}, {"Angle", 0.0}});
  }
}

void SceneView::startPoseTool()
{
  if (manager_) {
    manager_->getToolManager()->setCurrentTool(pose_tool_);
    Q_EMIT statusMessage("Click where the car is and drag in the direction it faces");
  }
}

void SceneView::startMeasureTool()
{
  if (manager_) {
    manager_->getToolManager()->setCurrentTool(measure_tool_);
    Q_EMIT statusMessage("Click two points to measure; right-click when done");
  }
}

void SceneView::cancelTool()
{
  if (manager_) {
    manager_->getToolManager()->setCurrentTool(interact_tool_);
  }
}

void SceneView::setCarSpeed(double speed)
{
  if (trail_) {
    trail_->setSpeed(speed);
  }
}

void SceneView::clearTrail()
{
  if (trail_) {
    trail_->clear();
  }
}

bool SceneView::carPose(double & x, double & y, double & yaw) const
{
  if (!manager_) {
    return false;
  }
  Ogre::Vector3 p;
  Ogre::Quaternion q;
  if (!manager_->getFrameManager()->getTransform(std::string("base_footprint"), p, q)) {
    return false;
  }
  x = p.x;
  y = p.y;
  yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return true;
}

void SceneView::onMapUpdated()
{
  const auto & map = track_map_->map();
  int w = static_cast<int>(map.info.width), h = static_cast<int>(map.info.height);
  if (w <= 0 || h <= 0 || map.data.size() < static_cast<size_t>(w) * h) {
    return;
  }
  QImage image(w, h, QImage::Format_ARGB32);
  QRgb free = TrackMapDisplay::freeColor().rgb(), wall = TrackMapDisplay::wallColor().rgb();
  size_t known = 0;
  for (int row = 0; row < h; ++row) {
    auto * line = reinterpret_cast<QRgb *>(image.scanLine(h - 1 - row));  // grid row 0 is the bottom
    for (int col = 0; col < w; ++col) {
      int8_t v = map.data[static_cast<size_t>(row) * w + col];
      if (v < 0) {
        line[col] = qRgba(0, 0, 0, 0);
      } else {
        known++;
        line[col] = v >= 50 ? wall : free;
      }
    }
  }
  map_image_ = image;
  map_resolution_ = map.info.resolution;
  map_origin_x_ = map.info.origin.position.x;
  map_origin_y_ = map.info.origin.position.y;
  Q_EMIT mapUpdated(w, h, map.info.resolution, static_cast<double>(known) / (static_cast<double>(w) * h));
}

bool SceneView::mapImage(QImage & image, double & resolution, double & origin_x, double & origin_y) const
{
  if (map_image_.isNull()) {
    return false;
  }
  image = map_image_;
  resolution = map_resolution_;
  origin_x = map_origin_x_;
  origin_y = map_origin_y_;
  return true;
}

QWidget * SceneView::getParentWindow()
{
  return window();
}

rviz_common::PanelDockWidget * SceneView::addPane(const QString &, QWidget *, Qt::DockWidgetArea, bool)
{
  return nullptr;
}

void SceneView::setStatus(const QString & message)
{
  Q_EMIT statusMessage(message);
}
}  // namespace f1ui
