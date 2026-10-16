#include "app/TemplatePointsTab.h"

#include "app/HardpointDelegates.h"
#include "app/HardpointModel.h"
#include "app/TemplateDraft.h"

#include <QCheckBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableView>
#include <QVBoxLayout>

#include <algorithm>

namespace suspkin {

void setUpPointView(QTableView* view, QAbstractItemModel* proxy, const QList<int>& columns)
{
    view->setModel(proxy);
    view->setSelectionBehavior(QAbstractItemView::SelectRows);
    view->setSelectionMode(QAbstractItemView::ExtendedSelection);
    view->setAlternatingRowColors(true);
    view->setWordWrap(false);
    view->verticalHeader()->setVisible(false);
    view->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    view->verticalHeader()->setDefaultSectionSize(26);
    view->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked
                          | QAbstractItemView::EditKeyPressed | QAbstractItemView::AnyKeyPressed);

    auto* coordinates = new CoordinateDelegate(view);
    for (int column = HardpointModel::XColumn; column <= HardpointModel::ZColumn; ++column)
        view->setItemDelegateForColumn(column, coordinates);
    view->setItemDelegateForColumn(HardpointModel::NameColumn, new PointNameDelegate(view));
    view->setItemDelegateForColumn(HardpointModel::TypeColumn, new PointTypeDelegate(view));
    auto* bodies = new BodyDelegate(view);
    view->setItemDelegateForColumn(HardpointModel::Part1Column, bodies);
    view->setItemDelegateForColumn(HardpointModel::Part2Column, bodies);
    view->setItemDelegateForColumn(HardpointModel::BushingColumn, new BushingDelegate(view));

    for (int column = 0; column < HardpointModel::ColumnCount; ++column)
        view->setColumnHidden(column, !columns.contains(column));
    view->setColumnWidth(HardpointModel::IndexColumn, 40);
    view->setColumnWidth(HardpointModel::NameColumn, 200);
    for (int column = HardpointModel::XColumn; column <= HardpointModel::ZColumn; ++column)
        view->setColumnWidth(column, 110);
    view->setColumnWidth(HardpointModel::TypeColumn, 150);
    view->setColumnWidth(HardpointModel::Part1Column, 170);
    view->setColumnWidth(HardpointModel::Part2Column, 170);
}

TemplatePointsTab::TemplatePointsTab(TemplateDraft* draft, QWidget* parent)
    : QWidget(parent), m_draft(draft)
{
    auto* intro = new QLabel(
        tr("The points of the near side, in millimetres: x forward, y left, z up. The far side "
           "is their mirror image, made by Hardpoints > Mirror and kept up with every edit."),
        this);
    intro->setWordWrap(true);

    m_proxy = new NearSideProxy(draft->model(), this);
    m_view = new QTableView(this);
    setUpPointView(m_view, m_proxy,
                   { HardpointModel::IndexColumn, HardpointModel::NameColumn,
                     HardpointModel::XColumn, HardpointModel::YColumn, HardpointModel::ZColumn });

    auto* add = new QPushButton(tr("+1"), this);
    add->setToolTip(tr("Add a point at the origin; give it its name and coordinates in the "
                       "table."));
    auto* remove = new QPushButton(tr("−1"), this);
    remove->setToolTip(tr("Remove the selected points, and their mirror images with them."));
    m_farSide = new QCheckBox(tr("Show the far side too"), this);
    m_farSide->setToolTip(tr("Points on the right of the car that are not the mirror of one on "
                             "the left -- measured separately -- are hidden unless this is on."));
    auto* buttons = new QHBoxLayout;
    buttons->addWidget(add);
    buttons->addWidget(remove);
    buttons->addStretch(1);
    buttons->addWidget(m_farSide);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(intro);
    layout->addLayout(buttons);
    layout->addWidget(m_view, 1);

    connect(add, &QPushButton::clicked, this, &TemplatePointsTab::addPoint);
    connect(remove, &QPushButton::clicked, this, &TemplatePointsTab::removeSelectedPoints);
    connect(m_farSide, &QCheckBox::toggled, this, [this](bool on) {
        m_proxy->setFarSideShown(on);
        emit farSideShownChanged(on);
    });
}

bool TemplatePointsTab::farSideShown() const { return m_farSide->isChecked(); }

void TemplatePointsTab::setFarSideShown(bool shown) { m_farSide->setChecked(shown); }

void TemplatePointsTab::addPoint()
{
    HardpointModel* model = m_draft->model();
    QString name = tr("NewPoint");
    for (int n = 2; model->table().indexOf(name) >= 0; ++n) name = tr("NewPoint%1").arg(n);

    Hardpoint point;
    point.name = name;
    const int row = model->rowCount();
    if (!model->insertPoint(row, point)) return;
    const QModelIndex at = m_proxy->mapFromSource(model->index(row, HardpointModel::NameColumn));
    m_view->setCurrentIndex(at);
    m_view->edit(at);
}

void TemplatePointsTab::removeSelectedPoints()
{
    HardpointModel* model = m_draft->model();
    std::vector<int> rows;
    for (const QModelIndex& index : m_view->selectionModel()->selectedRows())
        rows.push_back(m_proxy->mapToSource(index).row());
    // The model takes each point's mirror image with it.
    if (rows.empty()) return;
    model->removePoints(rows);
}

} // namespace suspkin
