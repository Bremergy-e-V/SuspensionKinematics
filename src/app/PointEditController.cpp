#include "app/PointEditController.h"

#include "app/ChassisDistanceEntry.h"
#include "app/CoordinateEntry.h"
#include "app/HardpointModel.h"
#include "geom/MeshQuery.h"
#include "model/ChassisOffset.h"
#include "render/MoveGizmo.h"
#include "render/ViewportWidget.h"

#include <QLocale>

#include <utility>

namespace suspkin {

PointEditController::PointEditController(ViewportWidget* viewport, HardpointModel* model,
                                         StatusLine status, QObject* parent)
    : QObject(parent), m_viewport(viewport), m_model(model), m_status(std::move(status))
{
    // A child of the viewport: it opens over the marker it is about, and it is
    // gone as soon as the viewport is.
    m_entry = new CoordinateEntry(m_viewport);

    connect(m_viewport, &ViewportWidget::coordinateEntryRequested, this,
            &PointEditController::openCoordinateEntry);
    connect(m_entry, &CoordinateEntry::committed, this,
            [this](int axis, double value) { moveCoordinate(m_entryRow, axis, value); });
    connect(m_entry, &CoordinateEntry::closed, this, [this] {
        m_entryRow = -1;
        // The viewport takes the keyboard back, so the next X, Y or Z lands
        // where the first one did rather than nowhere.
        m_viewport->setFocus(Qt::OtherFocusReason);
    });

    // The distance field stays open while the arrows are dragged, so it is not
    // closed by the viewport taking the click.
    m_distanceEntry = new ChassisDistanceEntry(m_viewport);
    connect(m_viewport, &ViewportWidget::chassisDistanceRequested, this,
            &PointEditController::openChassisDistance);
    connect(m_distanceEntry, &ChassisDistanceEntry::distanceEdited, this, [this] {
        if (m_distanceRow >= 0)
            m_distanceBase = previewChassisDistance(tablePosition(m_distanceRow));
    });
    connect(m_distanceEntry, &ChassisDistanceEntry::finished, this,
            &PointEditController::finishChassisDistance);
    connect(m_distanceEntry, &ChassisDistanceEntry::closed, this,
            &PointEditController::chassisDistanceClosed);

    // Nothing on the ribbon says that a selected point can be dragged or typed
    // at, so picking one in the viewport says it. The table's own selection
    // does not: there the coordinate columns are already in front of the user.
    // Picking another point lets go of the distance being held.
    connect(m_viewport, &ViewportWidget::hardpointSelectionEdited, this,
            [this](const QList<int>&, int current) {
                if (m_distanceRow >= 0 && current != m_distanceRow) m_distanceEntry->dismiss();
                announceEditing(current);
            });

    // A drag says where it has got to the whole way along and what it came to
    // at the end. Only the end is an edit; the rest is a readout -- or, while a
    // distance is held, the marker shown where that distance puts it.
    connect(m_viewport, &ViewportWidget::hardpointDragging, this,
            [this](int row, int axis, double distance) {
                if (row >= 0 && row == m_distanceRow) {
                    Vec3 along;
                    (axis == 0 ? along.x : axis == 1 ? along.y : along.z) = distance;
                    previewChassisDistance(m_distanceBase + along);
                    return;
                }
                showDragPosition(row, axis, distance);
            });
    connect(m_viewport, &ViewportWidget::hardpointMoved, this,
            [this](int row, int axis, double distance) {
                const HardpointTable& table = m_model->table();
                if (row < 0 || row >= static_cast<int>(table.points.size())) return;
                if (row == m_distanceRow) {
                    dropAtChassisDistance(row, axis, distance);
                    return;
                }
                // Added to the table's own double, not read back off the
                // marker: the marker is a float, and the two coordinates the
                // drag did not touch have to come through it unchanged.
                moveCoordinate(row, axis,
                               table.points[static_cast<std::size_t>(row)].coord[axis] + distance);
            });
}

void PointEditController::setChassisSource(ChassisSource chassis)
{
    m_chassis = std::move(chassis);
}

void PointEditController::dismiss()
{
    m_entry->dismiss();
    m_distanceEntry->dismiss();
}

void PointEditController::announceEditing(int current)
{
    if (current < 0 || !m_viewport->pointEditingEnabled()) return;
    const HardpointTable& table = m_model->table();
    if (current >= static_cast<int>(table.points.size())) return;
    const Hardpoint& point = table.points[static_cast<std::size_t>(current)];
    if (m_model->isGhost(current)) {
        m_status(tr("%1 is the mirror of %2 — move %2 and it follows").arg(point.name, point.mirrorOf),
                 6000);
        return;
    }
    m_status(tr("%1 — drag an arrow to move it, or press X, Y or Z to type a coordinate")
                 .arg(point.name),
             6000);
}

void PointEditController::openCoordinateEntry(int row, int axis)
{
    const HardpointTable& table = m_model->table();
    if (row < 0 || row >= static_cast<int>(table.points.size())) return;

    QPointF anchor;
    if (!m_viewport->markerPosition(row, &anchor)) return; // off screen: nothing to open beside

    const Hardpoint& point = table.points[static_cast<std::size_t>(row)];
    m_entryRow = row;
    m_entry->openAt(anchor, point.name, axis, point.coord[axis]);
}

void PointEditController::moveCoordinate(int row, int axis, double value)
{
    if (row < 0 || row >= m_model->rowCount()) return;
    if (axis < 0 || axis >= 3) return;

    // Through the model, exactly as the table's own cell does it: what follows
    // -- the parts, the solve, the wheels, the edits file -- hangs off the
    // coordinateChanged() that this produces.
    const QModelIndex index = m_model->index(row, HardpointModel::XColumn + axis);
    m_model->setData(index, value, Qt::EditRole);
}

Vec3 PointEditController::tablePosition(int row) const
{
    const Hardpoint& point = m_model->table().points[static_cast<std::size_t>(row)];
    return Vec3(point.coord[0], point.coord[1], point.coord[2]);
}

void PointEditController::openChassisDistance(int row)
{
    const HardpointTable& table = m_model->table();
    if (row < 0 || row >= static_cast<int>(table.points.size()) || m_model->isGhost(row)) return;
    const MeshQuery* chassis = m_chassis ? m_chassis() : nullptr;
    if (!chassis || chassis->isEmpty()) {
        m_status(tr("There is no chassis to keep a distance to: import one first"), 6000);
        return;
    }
    QPointF anchor;
    if (!m_viewport->markerPosition(row, &anchor)) return; // off screen: nothing to open beside

    m_entry->dismiss();
    m_distanceRow = row;
    m_distanceBase = tablePosition(row);
    const QString& name = table.points[static_cast<std::size_t>(row)].name;
    m_distanceEntry->openAt(anchor, name, chassis->distanceTo(tablePosition(row)));
    m_status(tr("%1 — drag an arrow and it keeps this distance to the chassis; Enter to finish")
                 .arg(name),
             0);
}

std::optional<Vec3> PointEditController::atChassisDistance(const Vec3& from, double distance) const
{
    const MeshQuery* chassis = m_chassis ? m_chassis() : nullptr;
    if (!chassis) return std::nullopt;
    const ChassisOffset offset = offsetFromChassis(*chassis, from, distance);
    if (!offset.position) m_status(offset.error, 6000);
    return offset.position;
}

Vec3 PointEditController::previewChassisDistance(const Vec3& from)
{
    const std::optional<double> distance = m_distanceEntry->distance();
    const std::optional<Vec3> at = distance ? atChassisDistance(from, *distance) : std::nullopt;
    if (!at) return from;
    m_viewport->moveHardpoint(m_distanceRow, at->toVector());
    return *at;
}

void PointEditController::dropAtChassisDistance(int row, int axis, double distance)
{
    // From where the marker was when the arrow was taken: the drag's own origin.
    Vec3 dropped = m_distanceBase;
    (axis == 0 ? dropped.x : axis == 1 ? dropped.y : dropped.z) += distance;
    const std::optional<double> held = m_distanceEntry->distance();
    const std::optional<Vec3> at = held ? atChassisDistance(dropped, *held) : std::nullopt;
    // Through movePoint(): one edit, one step, and a rigid part and the
    // point's ghosts come with it.
    m_model->movePoint(row, at.value_or(dropped));
    m_distanceBase = tablePosition(row);

    QPointF anchor;
    if (m_viewport->markerPosition(row, &anchor)) m_distanceEntry->followAnchor(anchor);
    // The drag took the keyboard; Enter belongs to the field.
    m_distanceEntry->focusField();
}

void PointEditController::finishChassisDistance(double distance)
{
    const int row = m_distanceRow;
    if (row < 0 || row >= m_model->rowCount()) return;
    const MeshQuery* chassis = m_chassis ? m_chassis() : nullptr;
    if (!chassis) return;
    const Vec3 now = tablePosition(row);
    // Already there -- the distance it opened on, or a drag just put it there
    // -- is not another edit.
    if (std::abs(chassis->distanceTo(now) - distance) < 1e-6) return;
    if (const std::optional<Vec3> at = atChassisDistance(now, distance)) m_model->movePoint(row, *at);
}

void PointEditController::chassisDistanceClosed()
{
    const int row = m_distanceRow;
    m_distanceRow = -1;
    // Whatever the field was previewing, the marker shows the table again.
    if (row >= 0 && row < m_model->rowCount()) m_viewport->moveHardpoint(row, tablePosition(row).toVector());
    m_status(QString(), 0);
    m_viewport->setFocus(Qt::OtherFocusReason);
}

void PointEditController::showDragPosition(int row, int axis, double distance)
{
    const HardpointTable& table = m_model->table();
    if (row < 0 || row >= static_cast<int>(table.points.size())) return;

    const Hardpoint& point = table.points[static_cast<std::size_t>(row)];
    const QLocale locale;
    m_status(tr("%1  %2 %3 mm  (%4 mm)")
                 .arg(point.name, MoveGizmo::axisLabel(axis),
                      locale.toString(point.coord[axis] + distance, 'f', 3),
                      locale.toString(distance, 'f', 3)),
             0);
}

} // namespace suspkin
