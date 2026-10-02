// 3D scene: an embedded RViz render panel with the car's displays (map, lidar, paths, safety boundaries...)
// and camera presets. Mouse controls are RViz's: drag to rotate, middle-drag/shift-drag to pan, wheel to zoom.
#pragma once

#include <QImage>
#include <QMap>
#include <QVariant>
#include <QWidget>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rviz_common/window_manager_interface.hpp"

namespace rviz_common
{
class Display;
class RenderPanel;
class Tool;
class ViewController;
class VisualizationManager;
namespace properties {class Property;}
namespace ros_integration {class RosNodeAbstractionIface;}
}  // namespace rviz_common

class QTimer;

namespace f1ui
{
class TrackMapDisplay;
class TrailDisplay;

class SceneView : public QWidget, public rviz_common::WindowManagerInterface
{
  Q_OBJECT

public:
  enum class View {Bird, Chase, Onboard, Orbit};
  enum class Layer {Grid, Map, Lidar, Car, Path, Follower, Recording, Raceline, Safety, Trail};

  SceneView(const std::string & node_name, QWidget * parent = nullptr);
  ~SceneView() override;

  // Creates the RViz scene; call once the window is shown (the render window needs a native parent)
  void initialize();
  bool initialized() const {return manager_ != nullptr;}

  void setView(View view);
  View view() const {return view_;}
  void resetView();
  void setFollow(bool follow);  // bird view moves with the car
  bool follow() const {return follow_;}
  void setLayerVisible(Layer layer, bool visible);
  bool layerVisible(Layer layer) const;
  static QString layerName(Layer layer);
  static std::vector<Layer> layers();

  // "map" when the car is localized or mapping, "odom" otherwise
  void setFixedFrame(const QString & frame);
  QString fixedFrame() const {return fixed_frame_;}

  void startPoseTool();
  // Bird view over the whole map (fixed, not following the car), e.g. to set the car's pose
  void showWholeMap();
  void startMeasureTool();
  void cancelTool();

  void setCarSpeed(double speed);
  void clearTrail();
  bool carPose(double & x, double & y, double & yaw) const;
  // Latest map seen by the scene, in track colours (row 0 = top), for the 2D track map
  bool mapImage(QImage & image, double & resolution, double & origin_x, double & origin_y) const;

  // rviz_common::WindowManagerInterface
  QWidget * getParentWindow() override;
  rviz_common::PanelDockWidget * addPane(
    const QString & name, QWidget * pane, Qt::DockWidgetArea area, bool floating) override;
  void setStatus(const QString & message) override;

Q_SIGNALS:
  void statusMessage(const QString & message);
  void toolFinished();
  void mapUpdated(int width, int height, double resolution, double known_fraction);

private:
  rviz_common::Display * addDisplay(Layer layer, const QString & class_id, const QString & name,
    const std::vector<std::pair<QString, QVariant>> & properties);
  // Sets "A/B/C" property paths; ones that don't exist yet (created when data arrives) are retried
  void setProperties(rviz_common::properties::Property * root,
    const std::vector<std::pair<QString, QVariant>> & properties);
  void retryPending();
  void applyPreset(View view);
  void createDisplays();
  void onMapUpdated();

  std::string node_name_;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> rviz_node_;
  rviz_common::RenderPanel * panel_ = nullptr;
  rviz_common::VisualizationManager * manager_ = nullptr;
  rviz_common::Tool * interact_tool_ = nullptr;
  rviz_common::Tool * pose_tool_ = nullptr;
  rviz_common::Tool * measure_tool_ = nullptr;
  TrackMapDisplay * track_map_ = nullptr;
  TrailDisplay * trail_ = nullptr;

  std::map<Layer, std::vector<rviz_common::Display *>> layers_;
  std::map<Layer, bool> visible_;
  struct Pending
  {
    rviz_common::properties::Property * root;
    QString path;
    QVariant value;
  };
  std::vector<Pending> pending_;
  QTimer * retry_timer_ = nullptr;

  View view_ = View::Chase;
  bool follow_ = true;
  QString fixed_frame_ = "map";
  std::map<View, rviz_common::ViewController *> saved_views_;

  QImage map_image_;
  double map_resolution_ = 0.05, map_origin_x_ = 0.0, map_origin_y_ = 0.0;
};
}  // namespace f1ui
