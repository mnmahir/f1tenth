#include <QDoubleValidator>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QIntValidator>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <sstream>

#include "pages.hpp"
#include "theme.hpp"
#include "widgets.hpp"

namespace f1ui
{
namespace
{
using rclcpp::ParameterType;

QString typeName(ParameterType type)
{
  switch (type) {
    case ParameterType::PARAMETER_BOOL: return "bool";
    case ParameterType::PARAMETER_INTEGER: return "int";
    case ParameterType::PARAMETER_DOUBLE: return "double";
    case ParameterType::PARAMETER_STRING: return "string";
    case ParameterType::PARAMETER_BYTE_ARRAY: return "byte[]";
    case ParameterType::PARAMETER_BOOL_ARRAY: return "bool[]";
    case ParameterType::PARAMETER_INTEGER_ARRAY: return "int[]";
    case ParameterType::PARAMETER_DOUBLE_ARRAY: return "double[]";
    case ParameterType::PARAMETER_STRING_ARRAY: return "string[]";
    default: return "unset";
  }
}

QString formatDouble(double v)
{
  QString s = QString::number(v, 'g', 10);
  if (!s.contains('.') && !s.contains('e') && !s.contains("inf") && !s.contains("nan")) {
    s += ".0";
  }
  return s;
}

QString toText(const rclcpp::Parameter & p)
{
  QStringList parts;
  switch (p.get_type()) {
    case ParameterType::PARAMETER_BOOL: return p.as_bool() ? "true" : "false";
    case ParameterType::PARAMETER_INTEGER: return QString::number(p.as_int());
    case ParameterType::PARAMETER_DOUBLE: return formatDouble(p.as_double());
    case ParameterType::PARAMETER_STRING: return QString::fromStdString(p.as_string());
    case ParameterType::PARAMETER_BOOL_ARRAY:
      for (bool b : p.as_bool_array()) {parts << (b ? "true" : "false");}
      return parts.join(", ");
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      for (auto v : p.as_integer_array()) {parts << QString::number(v);}
      return parts.join(", ");
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      for (double v : p.as_double_array()) {parts << formatDouble(v);}
      return parts.join(", ");
    case ParameterType::PARAMETER_STRING_ARRAY:
      for (const auto & v : p.as_string_array()) {parts << QString::fromStdString(v);}
      return parts.join(", ");
    case ParameterType::PARAMETER_BYTE_ARRAY:
      return QString("%1 bytes").arg(p.as_byte_array().size());
    default: return "";
  }
}

// Parses text as the parameter's current type; false if it doesn't fit
bool fromText(const QString & name, ParameterType type, const QString & text, rclcpp::Parameter & out)
{
  std::string key = name.toStdString();
  bool ok = true;
  QStringList items;
  for (const auto & s : text.split(',', Qt::SkipEmptyParts)) {
    items << s.trimmed();
  }
  auto as_bool = [&ok](const QString & s) {
      QString v = s.trimmed().toLower();
      if (v == "true" || v == "1") {return true;}
      if (v != "false" && v != "0") {ok = false;}
      return false;
    };
  switch (type) {
    case ParameterType::PARAMETER_BOOL: out = rclcpp::Parameter(key, as_bool(text)); break;
    case ParameterType::PARAMETER_INTEGER: out = rclcpp::Parameter(key, static_cast<int64_t>(text.trimmed().toLongLong(&ok))); break;
    case ParameterType::PARAMETER_DOUBLE: out = rclcpp::Parameter(key, text.trimmed().toDouble(&ok)); break;
    case ParameterType::PARAMETER_STRING: out = rclcpp::Parameter(key, text.toStdString()); break;
    case ParameterType::PARAMETER_BOOL_ARRAY: {
        std::vector<bool> v;
        for (const auto & s : items) {v.push_back(as_bool(s));}
        out = rclcpp::Parameter(key, v);
        break;
      }
    case ParameterType::PARAMETER_INTEGER_ARRAY: {
        std::vector<int64_t> v;
        for (const auto & s : items) {
          bool item_ok = false;
          v.push_back(s.toLongLong(&item_ok));
          ok = ok && item_ok;
        }
        out = rclcpp::Parameter(key, v);
        break;
      }
    case ParameterType::PARAMETER_DOUBLE_ARRAY: {
        std::vector<double> v;
        for (const auto & s : items) {
          bool item_ok = false;
          v.push_back(s.toDouble(&item_ok));
          ok = ok && item_ok;
        }
        out = rclcpp::Parameter(key, v);
        break;
      }
    case ParameterType::PARAMETER_STRING_ARRAY: {
        std::vector<std::string> v;
        for (const auto & s : items) {v.push_back(s.toStdString());}
        out = rclcpp::Parameter(key, v);
        break;
      }
    default: ok = false;
  }
  return ok;
}

QString rangeText(const rcl_interfaces::msg::ParameterDescriptor & d)
{
  QStringList parts;
  if (!d.description.empty()) {
    parts << QString::fromStdString(d.description);
  }
  if (!d.floating_point_range.empty()) {
    parts << QString("Range %1 to %2").arg(d.floating_point_range[0].from_value).arg(d.floating_point_range[0].to_value);
  }
  if (!d.integer_range.empty()) {
    parts << QString("Range %1 to %2").arg(d.integer_range[0].from_value).arg(d.integer_range[0].to_value);
  }
  if (!d.additional_constraints.empty()) {
    parts << QString::fromStdString(d.additional_constraints);
  }
  if (d.read_only) {
    parts << "Read-only: set at start-up (save to the car and restart the component)";
  }
  return parts.join("\n");
}
}  // namespace

ParamsPage::ParamsPage(RosBridge * ros, QWidget * parent)
: Page(ros, parent)
{
  auto * root = new QHBoxLayout(this);
  root->setContentsMargins(24, 18, 24, 18);
  root->setSpacing(14);

  auto * nodes = new Card("Nodes");
  nodes->setFixedWidth(320);
  node_filter_ = new QLineEdit();
  node_filter_->setPlaceholderText("Filter nodes");
  node_filter_->setClearButtonEnabled(true);
  nodes->body()->addWidget(node_filter_);
  node_list_ = new QListWidget();
  node_list_->setFont(theme::mono(12));
  nodes->body()->addWidget(node_list_);
  root->addWidget(nodes);

  auto * params = new Card("Parameters");
  auto * head = new QHBoxLayout();
  node_title_ = makeLabel("Pick a node", 22, QFont::Black);
  head->addWidget(node_title_, 1);
  refresh_ = makeButton("RELOAD", theme::icon::refresh);
  save_ = makeButton("SAVE TO CAR", theme::icon::save, "primary");
  save_->setToolTip("Write this node's current parameters to ~/f1tenth/data/params on the car; they are loaded "
    "every time it starts");
  head->addWidget(refresh_);
  head->addWidget(save_);
  params->body()->addLayout(head);
  param_filter_ = new QLineEdit();
  param_filter_->setPlaceholderText("Filter parameters");
  param_filter_->setClearButtonEnabled(true);
  params->body()->addWidget(param_filter_);
  table_ = new QTableWidget(0, 3);
  table_->setHorizontalHeaderLabels({"PARAMETER", "VALUE", "TYPE"});
  table_->verticalHeader()->hide();
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Interactive);
  table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  table_->setColumnWidth(0, 340);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionMode(QAbstractItemView::NoSelection);
  table_->setShowGrid(false);
  table_->setAlternatingRowColors(true);
  table_->verticalHeader()->setDefaultSectionSize(36);
  params->body()->addWidget(table_, 1);
  state_ = makeLabel("Changes apply to the running node right away. SAVE TO CAR keeps them for next time.", 12,
    QFont::Normal, theme::text3);
  state_->setWordWrap(true);
  params->body()->addWidget(state_);
  root->addWidget(params, 1);
  refresh_->setEnabled(false);
  save_->setEnabled(false);

  connect(node_filter_, &QLineEdit::textChanged, this, [this]() {onNodes(ros_->nodes());});
  connect(node_list_, &QListWidget::currentTextChanged, this, [this](const QString & node) {
      if (!node.isEmpty() && node != node_) {
        load(node);
      }
    });
  connect(refresh_, &QPushButton::clicked, this, [this]() {load(node_);});
  connect(save_, &QPushButton::clicked, this, [this]() {
      save_->setEnabled(false);
      ros_->saveParams(node_, [this](bool ok, const QString & text) {
        save_->setEnabled(true);
        Q_EMIT message(ok ? Level::Ok : Level::Error, text);
      });
    });
  connect(param_filter_, &QLineEdit::textChanged, this, &ParamsPage::filterRows);
  connect(ros_, &RosBridge::nodesChanged, this, &ParamsPage::onNodes);
  connect(ros_, &RosBridge::parameterEvent, this, &ParamsPage::onParameterEvent);
}

void ParamsPage::showNode(const QString & node)
{
  node_filter_->clear();
  auto items = node_list_->findItems(node, Qt::MatchExactly);
  if (!items.isEmpty()) {
    node_list_->setCurrentItem(items.front());
  } else {
    load(node);
  }
}

void ParamsPage::onNodes(const QStringList & nodes)
{
  QString filter = node_filter_->text().trimmed();
  QString current = node_list_->currentItem() ? node_list_->currentItem()->text() : node_;
  node_list_->blockSignals(true);
  node_list_->clear();
  for (const auto & node : nodes) {
    if (node.startsWith("/_") || node.contains("/f1tenth_ui_") || node.contains("/_ros2cli") ||
      node.startsWith("/launch_ros_") || node.startsWith("/transform_listener_impl_") ||
      (!filter.isEmpty() && !node.contains(filter, Qt::CaseInsensitive)))
    {
      continue;
    }
    node_list_->addItem(node);
    if (node == current) {
      node_list_->setCurrentRow(node_list_->count() - 1);
    }
  }
  node_list_->blockSignals(false);
}

void ParamsPage::load(const QString & node)
{
  node_ = node;
  node_title_->setText(node);
  table_->setRowCount(0);
  params_.clear();
  state_->setText("Loading...");
  refresh_->setEnabled(true);
  save_->setEnabled(false);
  ros_->loadParameters(node, [this, node](bool ok, const QString & error, const std::vector<ParamInfo> & params) {
      if (node != node_) {
        return;
      }
      if (!ok) {
        state_->setText(error);
        return;
      }
      fill(params);
      save_->setEnabled(true);
      state_->setText(QString("%1 parameters. Changes apply to the running node right away; SAVE TO CAR keeps "
        "them for next time.").arg(params.size()));
    });
}

void ParamsPage::fill(const std::vector<ParamInfo> & params)
{
  filling_ = true;
  params_ = params;
  table_->setRowCount(static_cast<int>(params.size()));
  for (int row = 0; row < static_cast<int>(params.size()); ++row) {
    const auto & info = params[row];
    QString name = QString::fromStdString(info.value.get_name());
    bool read_only = info.descriptor.read_only || name.startsWith("qos_overrides.");
    auto * name_item = new QTableWidgetItem(name);
    name_item->setFont(theme::mono(12, read_only ? QFont::Normal : QFont::DemiBold));
    name_item->setForeground(read_only ? theme::text3 : theme::text);
    name_item->setToolTip(rangeText(info.descriptor));
    table_->setItem(row, 0, name_item);
    auto * type_item = new QTableWidgetItem(typeName(info.value.get_type()));
    type_item->setForeground(theme::text3);
    table_->setItem(row, 2, type_item);

    if (info.value.get_type() == ParameterType::PARAMETER_BOOL) {
      auto * toggle = new Toggle(info.value.as_bool() ? "true" : "false");
      toggle->setChecked(info.value.as_bool());
      toggle->setEnabled(!read_only);
      connect(toggle, &Toggle::toggled, this, [this, row, toggle](bool on) {
          toggle->setText(on ? "true" : "false");
          if (!filling_) {
            commit(row);
          }
        });
      auto * holder = new QWidget();
      auto * layout = new QHBoxLayout(holder);
      layout->setContentsMargins(6, 0, 6, 0);
      layout->addWidget(toggle);
      layout->addStretch(1);
      table_->setCellWidget(row, 1, holder);
    } else {
      auto * edit = new QLineEdit(toText(info.value));
      edit->setFont(theme::mono(12));
      edit->setReadOnly(read_only || info.value.get_type() == ParameterType::PARAMETER_BYTE_ARRAY ||
        info.value.get_type() == ParameterType::PARAMETER_NOT_SET);
      edit->setToolTip(rangeText(info.descriptor));
      if (info.value.get_type() == ParameterType::PARAMETER_INTEGER) {
        edit->setValidator(new QIntValidator(edit));
      } else if (info.value.get_type() == ParameterType::PARAMETER_DOUBLE) {
        auto * v = new QDoubleValidator(edit);
        v->setLocale(QLocale::c());
        edit->setValidator(v);
      }
      connect(edit, &QLineEdit::editingFinished, this, [this, row, edit]() {
          if (!filling_ && edit->isModified()) {
            edit->setModified(false);
            commit(row);
          }
        });
      table_->setCellWidget(row, 1, edit);
    }
  }
  filling_ = false;
  filterRows();
}

void ParamsPage::commit(int row)
{
  if (row < 0 || row >= static_cast<int>(params_.size())) {
    return;
  }
  const auto & info = params_[row];
  QString name = QString::fromStdString(info.value.get_name());
  QString text;
  QWidget * cell = table_->cellWidget(row, 1);
  if (auto * edit = qobject_cast<QLineEdit *>(cell)) {
    text = edit->text();
  } else if (auto * toggle = cell ? cell->findChild<Toggle *>() : nullptr) {
    text = toggle->isChecked() ? "true" : "false";
  }
  rclcpp::Parameter param;
  if (!fromText(name, info.value.get_type(), text, param)) {
    Q_EMIT message(Level::Error, QString("'%1' is not a valid %2").arg(text, typeName(info.value.get_type())));
    return;
  }
  QString node = node_;
  ros_->setParameter(node, param, [this, node, row, param](bool ok, const QString & text) {
      Q_EMIT message(ok ? Level::Ok : Level::Error, node + ": " + text);
      if (node != node_ || row >= static_cast<int>(params_.size())) {
        return;
      }
      if (ok) {
        params_[row].value = param;
      }
      // Show what the node really has (rejected values snap back)
      filling_ = true;
      QWidget * cell = table_->cellWidget(row, 1);
      const auto & value = params_[row].value;
      if (auto * edit = qobject_cast<QLineEdit *>(cell)) {
        edit->setText(toText(value));
        edit->setStyleSheet(QString("border-color: %1;").arg(theme::css(ok ? theme::green : theme::redBright)));
        QTimer::singleShot(1200, edit, [edit]() {edit->setStyleSheet(QString());});
      } else if (auto * toggle = cell ? cell->findChild<Toggle *>() : nullptr) {
        toggle->setChecked(value.get_type() == ParameterType::PARAMETER_BOOL && value.as_bool());
      }
      filling_ = false;
    });
}

void ParamsPage::onParameterEvent(const ParameterEvent & event)
{
  if (QString::fromStdString(event.node) != node_) {
    return;
  }
  if (!event.new_parameters.empty() || !event.deleted_parameters.empty()) {
    load(node_);
    return;
  }
  filling_ = true;
  for (const auto & changed : event.changed_parameters) {
    for (int row = 0; row < static_cast<int>(params_.size()); ++row) {
      if (params_[row].value.get_name() != changed.name) {
        continue;
      }
      params_[row].value = rclcpp::Parameter::from_parameter_msg(changed);
      QWidget * cell = table_->cellWidget(row, 1);
      if (auto * edit = qobject_cast<QLineEdit *>(cell)) {
        if (!edit->hasFocus()) {
          edit->setText(toText(params_[row].value));
        }
      } else if (auto * toggle = cell ? cell->findChild<Toggle *>() : nullptr) {
        toggle->setChecked(params_[row].value.get_type() == ParameterType::PARAMETER_BOOL &&
          params_[row].value.as_bool());
      }
    }
  }
  filling_ = false;
}

void ParamsPage::filterRows()
{
  QString filter = param_filter_->text().trimmed();
  for (int row = 0; row < table_->rowCount(); ++row) {
    auto * item = table_->item(row, 0);
    table_->setRowHidden(row, item && !filter.isEmpty() && !item->text().contains(filter, Qt::CaseInsensitive));
  }
}
}  // namespace f1ui
