#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <algorithm>

#include "pages.hpp"
#include "theme.hpp"
#include "views.hpp"
#include "widgets.hpp"

namespace f1ui
{
namespace
{
constexpr int kThumbWidth = 176, kThumbHeight = 124;

// Map preview in track colours, fitted into a thumbnail
QIcon thumbnail(const QImage & gray)
{
  QPixmap pm(kThumbWidth, kThumbHeight);
  pm.fill(QColor(13, 14, 19));
  QImage colored(gray.size(), QImage::Format_ARGB32);
  for (int row = 0; row < gray.height(); ++row) {
    const uchar * in = gray.constScanLine(row);
    auto * out = reinterpret_cast<QRgb *>(colored.scanLine(row));
    for (int col = 0; col < gray.width(); ++col) {
      out[col] = in[col] < 64 ? qRgb(236, 238, 242) : (in[col] > 192 ? qRgb(36, 39, 48) : qRgba(0, 0, 0, 0));
    }
  }
  // Only the mapped part (maps are mostly unknown canvas around the track)
  int c0 = gray.width(), c1 = -1, r0 = gray.height(), r1 = -1;
  for (int row = 0; row < gray.height(); ++row) {
    const uchar * in = gray.constScanLine(row);
    for (int col = 0; col < gray.width(); ++col) {
      if (in[col] < 64 || in[col] > 192) {
        c0 = std::min(c0, col);
        c1 = std::max(c1, col);
        r0 = std::min(r0, row);
        r1 = std::max(r1, row);
      }
    }
  }
  if (c1 >= 0) {
    int margin = std::max(2, (c1 - c0) / 20);
    colored = colored.copy(QRect(QPoint(c0 - margin, r0 - margin), QPoint(c1 + margin, r1 + margin)));
  }
  QPainter p(&pm);
  p.setRenderHint(QPainter::SmoothPixmapTransform);
  QSize size = colored.size().scaled(QSize(kThumbWidth - 8, kThumbHeight - 8), Qt::KeepAspectRatio);
  p.drawImage(QRect(QPoint((kThumbWidth - size.width()) / 2, (kThumbHeight - size.height()) / 2), size), colored);
  return QIcon(pm);
}

QIcon placeholderThumbnail()
{
  QPixmap pm(kThumbWidth, kThumbHeight);
  pm.fill(QColor(13, 14, 19));
  QPainter p(&pm);
  p.setPen(theme::text3);
  p.setFont(theme::iconFont(36));
  p.drawText(pm.rect(), Qt::AlignCenter, QString(theme::icon::map));
  return QIcon(pm);
}
}  // namespace

GaragePage::GaragePage(RosBridge * ros, QWidget * parent)
: Page(ros, parent)
{
  auto * root = new QHBoxLayout(this);
  root->setContentsMargins(24, 18, 24, 18);
  root->setSpacing(14);

  auto * left = new QVBoxLayout();
  left->setSpacing(14);
  auto * maps_card = new Card("Maps");
  maps_ = new QListWidget();
  maps_->setViewMode(QListView::IconMode);
  maps_->setIconSize(QSize(kThumbWidth, kThumbHeight));
  maps_->setGridSize(QSize(kThumbWidth + 22, kThumbHeight + 40));
  maps_->setResizeMode(QListView::Adjust);
  maps_->setMovement(QListView::Static);
  maps_->setUniformItemSizes(true);
  maps_->setWordWrap(true);
  maps_->setSpacing(4);
  maps_card->body()->addWidget(maps_);
  left->addWidget(maps_card, 3);
  auto * paths_card = new Card("Paths & racelines");
  paths_ = new QListWidget();
  paths_card->body()->addWidget(paths_);
  left->addWidget(paths_card, 2);
  root->addLayout(left, 3);

  auto * detail = new Card("Preview");
  title_ = makeLabel("Select a map or a path", 22, QFont::Black);
  detail->body()->addWidget(title_);
  preview_ = new TrackMap();
  preview_->setPlaceholder("Maps are made in MAPPING sessions; paths are recorded or optimized in PATH sessions.");
  detail->body()->addWidget(preview_, 1);
  info_ = makeLabel("", 13, QFont::Normal, theme::text2);
  info_->setWordWrap(true);
  detail->body()->addWidget(info_);
  auto * actions = new QHBoxLayout();
  use_ = makeButton("USE FOR NEXT SESSION", theme::icon::play, "primary");
  rename_ = makeButton("RENAME", theme::icon::edit);
  delete_ = makeButton("DELETE", theme::icon::trash);
  actions->addWidget(use_, 1);
  actions->addWidget(rename_);
  actions->addWidget(delete_);
  detail->body()->addLayout(actions);
  auto * note = makeLabel("Files live on the car in ~/f1tenth/data. Deleting moves them to a .trash folder there.", 11,
    QFont::Normal, theme::text3);
  note->setWordWrap(true);
  detail->body()->addWidget(note);
  root->addWidget(detail, 2);
  for (auto * b : {use_, rename_, delete_}) {
    b->setEnabled(false);
  }

  connect(maps_, &QListWidget::currentItemChanged, this, [this](QListWidgetItem * item) {
      if (item) {
        paths_->clearSelection();
        showMap(item->text());
      }
    });
  connect(maps_, &QListWidget::itemClicked, this, [this](QListWidgetItem * item) {
      if (selected_ != item->text() || kind_ != "map") {
        paths_->clearSelection();
        showMap(item->text());
      }
    });
  connect(paths_, &QListWidget::currentItemChanged, this, [this](QListWidgetItem * item) {
      if (item) {
        showPath(item->text());
      }
    });
  connect(paths_, &QListWidget::itemClicked, this, [this](QListWidgetItem * item) {
      if (selected_ != item->text() || kind_ != "path") {
        showPath(item->text());
      }
    });
  connect(use_, &QPushButton::clicked, this, [this]() {
      if (kind_ == "map") {
        Q_EMIT useMap(selected_);
      } else if (kind_ == "path") {
        QString made_on = pathMap(state_, selected_);
        if (!made_on.isEmpty()) {
          Q_EMIT useMap(made_on);  // a path comes with its map
        }
        Q_EMIT usePath(selected_);
      }
    });
  connect(rename_, &QPushButton::clicked, this, &GaragePage::rename);
  connect(delete_, &QPushButton::clicked, this, &GaragePage::remove);
  connect(ros_, &RosBridge::supervisorState, this, &GaragePage::onSupervisor);
}

void GaragePage::setActive(bool active)
{
  active_ = active;
  if (active) {
    fetchThumbnails();
  }
}

void GaragePage::onSupervisor(const SupervisorState & state)
{
  state_ = state;
  QStringList maps, paths;
  for (const auto & m : state.maps) {
    maps << QString::fromStdString(m);
  }
  for (const auto & p : state.paths) {
    paths << QString::fromStdString(p);
  }
  if (maps != map_names_) {
    map_names_ = maps;
    QString current = maps_->currentItem() ? maps_->currentItem()->text() : QString();
    if (current == renamed_from_ && !maps.contains(current)) {
      current = renamed_to_;
    }
    QSignalBlocker block(maps_);  // rebuilding isn't the user picking a map
    maps_->clear();
    for (const auto & name : maps) {
      auto * item = new QListWidgetItem(placeholderThumbnail(), name);
      item->setTextAlignment(Qt::AlignHCenter | Qt::AlignTop);
      maps_->addItem(item);
      if (name == current) {
        maps_->setCurrentItem(item);
      }
    }
    thumbnails_.clear();
    if (active_) {
      fetchThumbnails();
    }
  }
  if (paths != path_names_) {
    path_names_ = paths;
    QString current = paths_->currentItem() ? paths_->currentItem()->text() : QString();
    if (current == renamed_from_ && !paths.contains(current)) {
      current = renamed_to_;
    }
    QSignalBlocker block(paths_);
    paths_->clear();
    for (const auto & name : paths) {
      QString made_on = pathMap(state, name);
      auto * item = new QListWidgetItem(glyphIcon(theme::icon::route, made_on.isEmpty() ? theme::text3 : theme::purple,
        18), name);
      item->setToolTip(made_on.isEmpty() ? "Map unknown (made before paths recorded their map)" :
        "Made on " + made_on);
      paths_->addItem(item);
      if (name == current) {
        paths_->setCurrentItem(item);
      }
    }
  }
  // The shown map or path was renamed, or deleted from another Pit Wall
  const QStringList & names = kind_ == "map" ? maps : paths;
  if (!selected_.isEmpty() && !names.contains(selected_)) {
    QString renamed = selected_ == renamed_from_ && names.contains(renamed_to_) ? renamed_to_ : QString();
    if (renamed.isEmpty()) {
      clearDetail();
    } else if (kind_ == "map") {
      showMap(renamed);
    } else {
      showPath(renamed);
    }
  }
}

void GaragePage::fetchThumbnails()
{
  // One request at a time, so a long list doesn't flood the car
  for (int i = 0; i < maps_->count(); ++i) {
    QString name = maps_->item(i)->text();
    if (!thumbnails_.contains(name)) {
      thumbnails_.insert(name);
      ros_->mapPreview(name, 256, [this, name](const MapPreview & preview) {
          if (preview.ok) {
            auto items = maps_->findItems(name, Qt::MatchExactly);
            if (!items.isEmpty()) {
              items.front()->setIcon(thumbnail(preview.image));
            }
          }
          fetchThumbnails();
        });
      return;
    }
  }
}

void GaragePage::showMap(const QString & name)
{
  kind_ = "map";
  selected_ = name;
  shown_map_ = name;
  title_->setText(name);
  info_->setText("Loading...");
  preview_->clearPath();
  ros_->mapPreview(name, 900, [this, name](const MapPreview & preview) {
      if (selected_ != name && shown_map_ != name) {
        return;
      }
      if (!preview.ok) {
        info_->setText(preview.message);
        return;
      }
      preview_->setPreview(preview.image, preview.resolution, preview.origin_x, preview.origin_y);
      if (kind_ == "map") {
        info_->setText(QString("Map  ·  %1 x %2 m  ·  %3 cm cells").arg(preview.image.width() * preview.resolution, 0, 'f', 1)
          .arg(preview.image.height() * preview.resolution, 0, 'f', 1).arg(preview.resolution * 100.0, 0, 'f', 1));
      }
    });
  for (auto * b : {use_, rename_, delete_}) {
    b->setEnabled(true);
  }
}

void GaragePage::showPath(const QString & name)
{
  kind_ = "path";
  selected_ = name;
  title_->setText(name);
  info_->setText("Loading...");
  QString made_on = pathMap(state_, name);
  if (!made_on.isEmpty() && made_on != shown_map_ && map_names_.contains(made_on)) {
    shown_map_ = made_on;
    ros_->mapPreview(made_on, 900, [this, made_on](const MapPreview & preview) {
        if (preview.ok && shown_map_ == made_on) {
          preview_->setPreview(preview.image, preview.resolution, preview.origin_x, preview.origin_y);
        }
      });
  } else if (made_on.isEmpty() || !map_names_.contains(made_on)) {
    shown_map_.clear();
    preview_->clearMap();
  }
  ros_->path(name, [this, name](const PathData & data) {
      if (selected_ != name) {
        return;
      }
      if (!data.ok) {
        info_->setText(data.message);
        return;
      }
      preview_->setPath(data.points, data.speeds);
      double lo = 1e9, hi = 0.0;
      for (float v : data.speeds) {
        lo = std::min(lo, static_cast<double>(v));
        hi = std::max(hi, static_cast<double>(v));
      }
      QString made_on = pathMap(state_, name);
      info_->setText(QString("Path  ·  %1 m  ·  %2 points  ·  %3-%4 m/s  ·  %5").arg(data.length(), 0, 'f', 1)
        .arg(data.points.size()).arg(data.speeds.isEmpty() ? 0.0 : lo, 0, 'f', 1).arg(hi, 0, 'f', 1)
        .arg(made_on.isEmpty() ? QString("map unknown") : (map_names_.contains(made_on) ? "made on " + made_on :
        "made on " + made_on + " (deleted)")));
    });
  for (auto * b : {use_, rename_, delete_}) {
    b->setEnabled(true);
  }
}

void GaragePage::rename()
{
  bool ok = false;
  QString name = QInputDialog::getText(this, "Rename " + kind_, "New name (letters, digits, - and _):",
    QLineEdit::Normal, selected_, &ok).trimmed();
  if (!ok || name.isEmpty() || name == selected_) {
    return;
  }
  renamed_from_ = selected_;
  renamed_to_ = name;
  ros_->fileOp(kind_, "rename", selected_, name, report());
}

void GaragePage::remove()
{
  if (QMessageBox::question(this, "Delete " + kind_ + "?",
    QString("Delete %1 '%2'? It is moved to the trash folder on the car.").arg(kind_, selected_)) != QMessageBox::Yes)
  {
    return;
  }
  ros_->fileOp(kind_, "delete", selected_, QString(), report());  // the detail clears once the car's list drops it
}

void GaragePage::clearDetail()
{
  title_->setText("Select a map or a path");
  info_->clear();
  preview_->clearMap();
  preview_->clearPath();
  maps_->clearSelection();
  paths_->clearSelection();
  maps_->setCurrentItem(nullptr);
  paths_->setCurrentItem(nullptr);
  selected_.clear();
  shown_map_.clear();
  for (auto * b : {use_, rename_, delete_}) {
    b->setEnabled(false);
  }
}
}  // namespace f1ui
