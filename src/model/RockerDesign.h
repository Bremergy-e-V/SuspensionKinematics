#pragma once

#include "geom/Vec3.h"
#include "model/HardpointGenerator.h"

#include <QJsonArray>
#include <QString>
#include <QStringList>

#include <optional>
#include <vector>

namespace suspkin {

struct LinkageTemplate;
struct MirrorSpec;

/// One rocker and its damper, designed as a part of its own: the chalk-board
/// sketch from the 2025 car. Not part of Generate from Design, which builds
/// the double wishbone and nothing else -- a rocker is packaging, set out by
/// these numbers and then moved into place by hand, as one piece.
///
/// Every arm lies in the rocker's plane: the plane through the pivot square to
/// its axis. Angles in that plane are positive turning inboard, toward the
/// car's centreline, which is what lets one set of numbers mean the same
/// rocker on either side of the car.
///
/// The pushrod itself -- its length and the angle it meets the rocker at --
/// is not set here. Its outer end is on a wishbone and is placed by hand.
struct RockerDesign {
    /// The linkage template's corner the points are named through.
    QString corner;

    /// The pivot, in vehicle coordinates on the side being designed. Which side
    /// that is follows from its y: the far side comes from the mirror rule.
    Vec3 pivot{ 0.0, 150.0, 500.0 };
    /// The axis it turns about: its angle in top view from straight ahead
    /// (+X), positive toward +Y, and its angle above the horizontal.
    double axisYaw = 0.0;
    double axisPitch = 0.0;

    /// L1, the arm to the pushrod, and where it points: turned about the axis
    /// from straight up. This is the angle that turns the whole rocker.
    double pushrodArm = 70.0;
    double pushrodArmAngle = 0.0;

    /// L2, the arm to the damper's eye, at this angle from L1.
    double damperArm = 60.0;
    double damperArmAngle = 120.0;
    /// The damper: at the eye, the angle from the arm back to the pivot to
    /// the damper (beta in the sketch), and its length eye to eye (Ld). Its
    /// other end is the chassis mount.
    double damperAngle = 90.0;
    double damperLength = 200.0;

    /// L3, an arm for an anti-roll bar's drop link, at this angle from L1.
    /// Written only when asked for and when the template hangs the drop link
    /// on the rocker.
    bool antiRollArmUsed = false;
    double antiRollArm = 50.0;
    double antiRollArmAngle = -60.0;

    bool operator==(const RockerDesign& other) const = default;
};

/// The points one rocker comes to, on the side its pivot is on.
struct RockerPoints {
    bool ok = false;
    QString error;
    Vec3 pivot;
    Vec3 axisPoint;     ///< a second point on the axis, ahead of the pivot
    Vec3 pushrodInner;  ///< the end of L1
    Vec3 damperEye;     ///< the end of L2
    Vec3 damperMount;   ///< the damper's other end, on the chassis
    bool hasAntiRollArm = false;
    Vec3 antiRollPickup; ///< the end of L3
};

/// Lay @p design out. Pure.
RockerPoints layOutRocker(const RockerDesign& design);

/// Every rocker in @p designs, named through @p templ's mechanism, folded into
/// @p current with the far side through @p mirror -- the same plan Generate
/// from Design makes, so the dialog previews and the window applies it the
/// same way. @p baseline is what tells a hand edit about to be overwritten.
DesignPlan planRockers(const std::vector<RockerDesign>& designs, const LinkageTemplate& templ,
                       const MirrorSpec& mirror, const HardpointTable& current,
                       const HardpointTable& baseline);

/// The points of each designed rocker that move as one part -- the pivot, its
/// axis point, the ends of its arms and the damper with both its ends -- by
/// name, one group per corner and side. What HardpointModel::setRigidGroups()
/// is given, so that dragging any of them drags the rocker.
std::vector<QStringList> rockerGroups(const std::vector<RockerDesign>& designs,
                                      const LinkageTemplate& templ, const MirrorSpec& mirror,
                                      const HardpointTable& table);

/// The rocker @p table already has in @p corner, read off its points on the
/// side the template's names are on: what a rocker placed by hand starts the
/// dialog from, so that designing it does not first throw it somewhere else.
/// Empty when the table lacks the pivot, the axis point or the pushrod's end.
/// An arm the table has no point for keeps @p fallback's numbers; a point off
/// the rocker's plane is read as where it falls on that plane.
std::optional<RockerDesign> measureRocker(const QString& corner, const LinkageTemplate& templ,
                                          const MirrorSpec& mirror, const HardpointTable& table,
                                          const RockerDesign& fallback);

/// @p design with its pivot where @p table has the rocker's pivot now. A rocker
/// is moved as a whole by hand, so the table is where its position is kept;
/// the design keeps its shape. Unchanged when the table has no such point.
RockerDesign followTable(RockerDesign design, const LinkageTemplate& templ,
                         const MirrorSpec& mirror, const HardpointTable& table);

/// The manifest's form, a key missing reading back as its default.
QJsonArray rockerDesignsToJson(const std::vector<RockerDesign>& designs);
std::vector<RockerDesign> rockerDesignsFromJson(const QJsonArray& array);

} // namespace suspkin
