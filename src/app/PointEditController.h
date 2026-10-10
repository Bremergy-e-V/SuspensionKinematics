#pragma once

#include "geom/Vec3.h"

#include <QObject>
#include <QString>

#include <functional>
#include <optional>

namespace suspkin {

class ChassisDistanceEntry;
class CoordinateEntry;
class HardpointModel;
class MeshQuery;
class ViewportWidget;

/// The ways a point is moved in the viewport itself: the arrows on the selected
/// marker, the field X, Y or Z opens over it, and the distance to the chassis
/// C opens -- which the arrows then keep while they drag.
///
/// Both end in HardpointModel::setData() on a coordinate column, exactly as the
/// table's own cell does, so a point dragged and a point typed are the same
/// edit -- the parts, the solve, the wheels and the project all follow from the
/// model's own signal, and nothing here knows about any of them.
class PointEditController : public QObject {
    Q_OBJECT

public:
    /// Says something in the status line for @p milliseconds; 0 until replaced.
    using StatusLine = std::function<void(const QString& text, int milliseconds)>;
    /// The chassis a distance is measured to, or nothing when there is none.
    /// Asked when it is needed, because building one over a big model takes a
    /// moment.
    using ChassisSource = std::function<const MeshQuery*()>;

    PointEditController(ViewportWidget* viewport, HardpointModel* model, StatusLine status,
                        QObject* parent);

    void setChassisSource(ChassisSource chassis);

    /// Close the coordinate field and the distance field without taking what
    /// was typed: the point they were opened on may be about to move or go.
    void dismiss();

private:
    /// Open the coordinate field on @p axis of the point at @p row, holding
    /// what the table has for it.
    void openCoordinateEntry(int row, int axis);
    /// Write one coordinate of one point through the table.
    void moveCoordinate(int row, int axis, double value);
    /// Say where a marker being dragged has got to. A readout, not an edit --
    /// the table is untouched until the drag ends.
    void showDragPosition(int row, int axis, double distance);
    /// Say that the point just picked can be dragged or typed at.
    void announceEditing(int current);

    // --- holding a distance to the chassis -----------------------------------
    /// Open the distance field on the point at @p row, holding how far it is
    /// from the chassis now.
    void openChassisDistance(int row);
    /// Where @p from comes to at @p distance from the chassis, square to it.
    /// Nothing, and said, when there is no such place.
    std::optional<Vec3> atChassisDistance(const Vec3& from, double distance) const;
    /// Show the marker where the distance field puts @p from. Not an edit.
    /// @return where that is, or @p from when there is no such place.
    Vec3 previewChassisDistance(const Vec3& from);
    /// A drag of the held point let go: put it at the distance, as one edit.
    void dropAtChassisDistance(int row, int axis, double distance);
    /// Enter: put the point at the distance if it is not there already.
    void finishChassisDistance(double distance);
    /// The field closed: the marker goes back to where the table has it.
    void chassisDistanceClosed();
    /// The point at @p row, as the table has it.
    Vec3 tablePosition(int row) const;

    ViewportWidget* m_viewport = nullptr;
    HardpointModel* m_model = nullptr;
    StatusLine m_status;
    /// The field X, Y and Z open over the viewport. A child of the viewport, so
    /// it sits over the marker it is about.
    CoordinateEntry* m_entry = nullptr;
    /// The row that field is open on, because the selection is not allowed to
    /// answer for it: what was typed belongs to the point it was opened on.
    int m_entryRow = -1;

    ChassisSource m_chassis;
    /// The field C opens, and the row it holds a distance for.
    ChassisDistanceEntry* m_distanceEntry = nullptr;
    int m_distanceRow = -1;
    /// Where the held point's marker is shown: the table's position, or the
    /// preview of a distance typed but not yet applied. A drag starts from the
    /// marker, so this is what its distance is added to.
    Vec3 m_distanceBase;
};

} // namespace suspkin
