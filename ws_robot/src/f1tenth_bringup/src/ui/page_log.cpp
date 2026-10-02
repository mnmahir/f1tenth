#include <QAbstractTableModel>
#include <QComboBox>
#include <QDateTime>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollBar>
#include <QTabWidget>
#include <QTableView>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <deque>
#include <map>
#include <set>

#include "pages.hpp"
#include "theme.hpp"
#include "widgets.hpp"

namespace f1ui
{
namespace
{
QString levelName(int level)
{
  if (level >= 50) {return "FATAL";}
  if (level >= 40) {return "ERROR";}
  if (level >= 30) {return "WARN";}
  if (level >= 20) {return "INFO";}
  return "DEBUG";
}

QString clock(const builtin_interfaces::msg::Time & stamp)
{
  qint64 ms = static_cast<qint64>(stamp.sec) * 1000 + stamp.nanosec / 1000000;
  return QDateTime::fromMSecsSinceEpoch(ms).toString("HH:mm:ss.zzz");
}

// Terminal colours (ANSI SGR) to HTML spans
QString ansiToHtml(const QString & line)
{
  static const std::map<int, QColor> colors = {
    {31, theme::redBright}, {32, theme::green}, {33, theme::yellow}, {34, theme::blue}, {35, theme::purple},
    {36, theme::cyan}, {37, theme::text}, {91, theme::redBright}, {92, theme::green}, {93, theme::yellow},
    {94, theme::blue}, {95, theme::purple}, {96, theme::cyan}, {97, theme::text}};
  QString out;
  bool open = false;
  int i = 0;
  while (i < line.size()) {
    if (line[i] == QChar(0x1b) && i + 1 < line.size() && line[i + 1] == '[') {
      int end = line.indexOf('m', i + 2);
      if (end < 0) {
        break;
      }
      QColor color;
      bool bold = false;
      for (const auto & code : line.mid(i + 2, end - i - 2).split(';')) {
        int c = code.toInt();
        if (colors.count(c)) {
          color = colors.at(c);
        } else if (c == 1) {
          bold = true;
        }
      }
      if (open) {
        out += "</span>";
        open = false;
      }
      if (color.isValid() || bold) {
        out += QString("<span style=\"%1%2\">").arg(color.isValid() ? "color:" + theme::css(color) + ";" : "")
          .arg(bold ? "font-weight:600;" : "");
        open = true;
      }
      i = end + 1;
      continue;
    }
    int next = line.indexOf(QChar(0x1b), i);
    if (next < 0) {
      next = line.size();
    }
    out += line.mid(i, next - i).toHtmlEscaped();
    i = next;
  }
  if (open) {
    out += "</span>";
  }
  return out;
}

QString stripAnsi(QString line)
{
  static const QRegularExpression ansi("\x1b\\[[0-9;]*m");
  return line.remove(ansi);
}
}  // namespace

// /rosout lines with filtering; keeps the latest 20000
class LogModel : public QAbstractTableModel
{
public:
  using QAbstractTableModel::QAbstractTableModel;

  int rowCount(const QModelIndex & parent = QModelIndex()) const override
  {
    return parent.isValid() ? 0 : static_cast<int>(visible_.size());
  }
  int columnCount(const QModelIndex & = QModelIndex()) const override {return 4;}

  QVariant headerData(int section, Qt::Orientation orientation, int role) const override
  {
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole) {
      static const char * headers[] = {"TIME", "LEVEL", "NODE", "MESSAGE"};
      return headers[section];
    }
    return QVariant();
  }

  QVariant data(const QModelIndex & index, int role) const override
  {
    if (!index.isValid() || index.row() >= static_cast<int>(visible_.size())) {
      return QVariant();
    }
    const Log & log = all_[visible_[index.row()] - first_];
    if (role == Qt::DisplayRole) {
      switch (index.column()) {
        case 0: return clock(log.stamp);
        case 1: return levelName(log.level);
        case 2: return QString::fromStdString(log.name);
        default: return stripAnsi(QString::fromStdString(log.msg));
      }
    }
    if (role == Qt::ForegroundRole) {
      if (index.column() == 1 || (index.column() == 3 && log.level >= 30)) {
        return QBrush(theme::logColor(log.level));
      }
      return QBrush(index.column() == 3 ? theme::text : theme::text3);
    }
    if (role == Qt::FontRole) {
      return index.column() == 1 ? theme::font(11, QFont::Bold) : theme::mono(12);
    }
    if (role == Qt::ToolTipRole && index.column() == 3) {
      return QString::fromStdString(log.file) + ":" + QString::number(log.line);
    }
    return QVariant();
  }

  void append(const std::vector<Log> & batch)
  {
    std::vector<uint64_t> added;
    for (const auto & log : batch) {
      all_.push_back(log);
      nodes_.insert(QString::fromStdString(log.name));
      uint64_t id = first_ + all_.size() - 1;
      if (matches(log)) {
        added.push_back(id);
      }
    }
    if (!added.empty()) {
      beginInsertRows(QModelIndex(), static_cast<int>(visible_.size()), static_cast<int>(visible_.size() + added.size() - 1));
      visible_.insert(visible_.end(), added.begin(), added.end());
      endInsertRows();
    }
    // Drop the oldest
    size_t excess = all_.size() > 20000 ? all_.size() - 20000 : 0;
    if (excess > 0) {
      uint64_t new_first = first_ + excess;
      size_t drop = 0;
      while (drop < visible_.size() && visible_[drop] < new_first) {
        ++drop;
      }
      if (drop > 0) {
        beginRemoveRows(QModelIndex(), 0, static_cast<int>(drop) - 1);
        visible_.erase(visible_.begin(), visible_.begin() + static_cast<std::ptrdiff_t>(drop));
        endRemoveRows();
      }
      all_.erase(all_.begin(), all_.begin() + static_cast<std::ptrdiff_t>(excess));
      first_ = new_first;
    }
  }

  void setFilter(int min_level, const QString & node, const QString & text)
  {
    beginResetModel();
    min_level_ = min_level;
    node_ = node;
    text_ = text;
    visible_.clear();
    for (size_t i = 0; i < all_.size(); ++i) {
      if (matches(all_[i])) {
        visible_.push_back(first_ + i);
      }
    }
    endResetModel();
  }

  void clear()
  {
    beginResetModel();
    first_ += all_.size();
    all_.clear();
    visible_.clear();
    endResetModel();
  }

  // Puts the car's log history first, keeping the lines received since that are newer
  void setHistory(const std::vector<Log> & history)
  {
    std::vector<Log> merged = history;
    if (!history.empty()) {
      const auto & last = history.back().stamp;
      for (const auto & log : all_) {
        if (log.stamp.sec > last.sec || (log.stamp.sec == last.sec && log.stamp.nanosec > last.nanosec)) {
          merged.push_back(log);
        }
      }
    }
    clear();
    append(merged);
  }

  QStringList nodes() const
  {
    QStringList list;
    for (const auto & n : nodes_) {
      list << n;
    }
    return list;
  }

private:
  bool matches(const Log & log) const
  {
    if (log.level < min_level_) {
      return false;
    }
    if (!node_.isEmpty() && QString::fromStdString(log.name) != node_) {
      return false;
    }
    return text_.isEmpty() || QString::fromStdString(log.msg).contains(text_, Qt::CaseInsensitive) ||
           QString::fromStdString(log.name).contains(text_, Qt::CaseInsensitive);
  }

  std::deque<Log> all_;
  std::vector<uint64_t> visible_;  // ids (first_ + index into all_)
  uint64_t first_ = 0;
  std::set<QString> nodes_;
  int min_level_ = 20;
  QString node_, text_;
};

LogPage::LogPage(RosBridge * ros, QWidget * parent)
: Page(ros, parent)
{
  auto * root = new QVBoxLayout(this);
  root->setContentsMargins(24, 18, 24, 18);
  root->setSpacing(10);
  tabs_ = new QTabWidget();
  root->addWidget(tabs_);

  // ROS log
  auto * rosout = new QWidget();
  auto * rv = new QVBoxLayout(rosout);
  rv->setContentsMargins(0, 10, 0, 0);
  auto * filters = new QHBoxLayout();
  level_ = new QComboBox();
  level_->addItem("DEBUG +", 10);
  level_->addItem("INFO +", 20);
  level_->addItem("WARN +", 30);
  level_->addItem("ERROR +", 40);
  level_->setCurrentIndex(1);
  node_ = new QComboBox();
  node_->addItem("All nodes", QString());
  node_->setMinimumWidth(240);
  search_ = new QLineEdit();
  search_->setPlaceholderText("Search");
  search_->setClearButtonEnabled(true);
  pause_ = makeButton("PAUSE", theme::icon::pause);
  pause_->setCheckable(true);
  auto * clear = makeButton("CLEAR", theme::icon::trash);
  filters->addWidget(level_);
  filters->addWidget(node_);
  filters->addWidget(search_, 1);
  filters->addWidget(pause_);
  filters->addWidget(clear);
  rv->addLayout(filters);
  model_ = new LogModel(this);
  table_ = new QTableView();
  table_->setModel(model_);
  table_->verticalHeader()->hide();
  table_->verticalHeader()->setDefaultSectionSize(24);
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Interactive);
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->setColumnWidth(2, 220);
  table_->setShowGrid(false);
  table_->setAlternatingRowColors(true);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setWordWrap(false);
  rv->addWidget(table_, 1);
  tabs_->addTab(rosout, "ROS LOG");

  // Terminal output of the car's components
  auto * terminal = new QWidget();
  auto * tv = new QVBoxLayout(terminal);
  tv->setContentsMargins(0, 10, 0, 0);
  auto * bar = new QHBoxLayout();
  component_ = new QComboBox();
  component_->addItem("All components", QString());
  component_->setMinimumWidth(200);
  auto * reload = makeButton("RELOAD", theme::icon::refresh);
  auto * clear_console = makeButton("CLEAR", theme::icon::trash);
  bar->addWidget(component_);
  bar->addWidget(makeLabel("What the car's launch files print, as in a terminal.", 12, QFont::Normal, theme::text3), 1);
  bar->addWidget(reload);
  bar->addWidget(clear_console);
  tv->addLayout(bar);
  console_ = new QPlainTextEdit();
  console_->setReadOnly(true);
  console_->setMaximumBlockCount(5000);
  console_->setFont(theme::mono(12));
  console_->setLineWrapMode(QPlainTextEdit::WidgetWidth);
  console_->setStyleSheet("QPlainTextEdit { background: #07070a; }");
  tv->addWidget(console_, 1);
  tabs_->addTab(terminal, "TERMINAL");

  auto refilter = [this]() {
      model_->setFilter(level_->currentData().toInt(), node_->currentData().toString(), search_->text());
      table_->scrollToBottom();
    };
  connect(level_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, refilter);
  connect(node_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, refilter);
  connect(search_, &QLineEdit::textChanged, this, refilter);
  connect(clear, &QPushButton::clicked, this, [this]() {model_->clear();});
  connect(component_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &LogPage::rebuildConsole);
  connect(reload, &QPushButton::clicked, this, [this]() {
      history_loaded_ = false;
      loadHistory();
    });
  connect(clear_console, &QPushButton::clicked, this, [this]() {
      console_lines_.clear();
      console_->clear();
    });
  connect(ros_, &RosBridge::rosout, this, &LogPage::onRosout);
  connect(ros_, &RosBridge::console, this, &LogPage::onConsole);
  connect(ros_, &RosBridge::linkChanged, this, [this](bool, bool supervisor) {
      if (supervisor && !history_loaded_) {
        loadHistory();
      }
    });
}

void LogPage::onRosout(const std::vector<Log> & batch)
{
  if (pause_->isChecked()) {
    return;
  }
  auto * scroll = table_->verticalScrollBar();
  bool at_bottom = scroll->value() >= scroll->maximum() - 4;
  model_->append(batch);
  if (node_->count() - 1 != model_->nodes().size()) {
    QString current = node_->currentData().toString();
    node_->blockSignals(true);
    node_->clear();
    node_->addItem("All nodes", QString());
    for (const auto & n : model_->nodes()) {
      node_->addItem(n, n);
    }
    node_->setCurrentIndex(std::max(0, node_->findData(current)));
    node_->blockSignals(false);
  }
  if (at_bottom) {
    table_->scrollToBottom();
  }
}

// The latest lines only: they arrive quickly even over a slow link, and newer ones stream in live
constexpr int kHistoryLines = 500;
// The car always has some history (the supervisor logs its own start), so none means the request failed, e.g.
// while the link was too slow for a moment: ask again a few times, further apart each time
constexpr int kHistoryAttempts = 4;

void LogPage::loadHistory()
{
  history_loaded_ = true;
  loadRosoutHistory(0);
  loadConsoleHistory(0);
}

void LogPage::loadRosoutHistory(int attempt)
{
  ros_->rosoutHistory(kHistoryLines, [this, attempt](const std::vector<Log> & lines) {
      if (lines.empty()) {
        if (attempt + 1 < kHistoryAttempts) {
          QTimer::singleShot(10000 * (attempt + 1), this, [this, attempt]() {loadRosoutHistory(attempt + 1);});
        }
        return;
      }
      model_->setHistory(lines);
      onRosout({});  // refreshes the node list
      table_->scrollToBottom();
    });
}

void LogPage::loadConsoleHistory(int attempt)
{
  ros_->consoleHistory(kHistoryLines, [this, attempt](const std::vector<Log> & lines) {
      if (lines.empty()) {
        if (attempt + 1 < kHistoryAttempts) {
          QTimer::singleShot(10000 * (attempt + 1), this, [this, attempt]() {loadConsoleHistory(attempt + 1);});
        }
        return;
      }
      // Keep live lines that are newer than the history
      auto last = lines.back().stamp;
      std::vector<Log> merged = lines;
      for (const auto & line : console_lines_) {
        if (line.stamp.sec > last.sec || (line.stamp.sec == last.sec && line.stamp.nanosec > last.nanosec)) {
          merged.push_back(line);
        }
      }
      console_lines_ = merged;
      rebuildConsole();
    });
}

void LogPage::onConsole(const std::vector<Log> & batch)
{
  for (const auto & line : batch) {
    console_lines_.push_back(line);
    appendConsole(line);
  }
  if (console_lines_.size() > 6000) {
    console_lines_.erase(console_lines_.begin(), console_lines_.begin() + 1000);
  }
}

void LogPage::appendConsole(const Log & line)
{
  if (stripAnsi(QString::fromStdString(line.msg)).trimmed().isEmpty()) {
    return;  // colour codes only
  }
  QString name = QString::fromStdString(line.name);
  if (component_->findData(name) < 0) {
    component_->addItem(name, name);
  }
  QString filter = component_->currentData().toString();
  if (!filter.isEmpty() && filter != name) {
    return;
  }
  auto * scroll = console_->verticalScrollBar();
  bool at_bottom = scroll->value() >= scroll->maximum() - 4;
  QString html = QString("<span style=\"color:%1\">%2</span> <span style=\"color:%3\">%4</span> %5")
    .arg(theme::css(theme::text3)).arg(clock(line.stamp).left(8))
    .arg(theme::css(line.level >= 40 ? theme::redBright : theme::cyan)).arg(name.leftJustified(12, ' ').toHtmlEscaped()
    .replace(" ", "&nbsp;")).arg(ansiToHtml(QString::fromStdString(line.msg)));
  console_->appendHtml(html);
  if (at_bottom) {
    scroll->setValue(scroll->maximum());
  }
}

void LogPage::rebuildConsole()
{
  console_->clear();
  for (const auto & line : console_lines_) {
    appendConsole(line);
  }
}
}  // namespace f1ui
