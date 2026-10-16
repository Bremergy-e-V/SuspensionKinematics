#pragma once

#include "geom/Vec3.h"
#include "model/HardpointConfig.h"
#include "model/HardpointGenerator.h"
#include "model/HardpointMirror.h"
#include "model/Linkage.h"

#include <QJsonArray>
#include <QString>
#include <QStringList>

#include <optional>
#include <vector>

namespace suspkin {

class MeshQuery;

/// How a corner's spring is worked.
enum class PushrodLayout {
    Rocker,       ///< pushrod to a rocker, which works the damper (Linkage > Rocker)
    InlineDamper, ///< pushrod bolted to the damper's shaft: a strut, in line
    DamperOnly,   ///< no pushrod: the damper straight off the wishbone
};

QString pushrodLayoutToString(PushrodLayout layout);
PushrodLayout pushrodLayoutFromString(const QString& text,
                                      PushrodLayout fallback = PushrodLayout::Rocker);

/// Which wishbone's outer ball joint the outboard end is laid off from, and so
/// which wishbone carries it.
enum class PickupArm { Upper, Lower };

/// One corner's pushrod or damper, the way it is laid out on a car:
///
/// - the **inboard** end on the chassis at a given height -- found by looking
///   inboard from the ball joint at that height until the chassis is hit, and
///   stood off it by a clearance; with a rocker it is the rocker's pushrod arm,
///   placed by Linkage > Rocker;
/// - the **outboard** end on the line from the chosen ball joint to the inboard
///   end, a given distance inboard of the ball joint in y.
///
/// In line, the damper's outer eye is on the same line, a damper's length from
/// the chassis end.
struct PushrodDesign {
    QString corner;
    PushrodLayout layout = PushrodLayout::Rocker;
    PickupArm arm = PickupArm::Upper;
    double inboardHeight = 450.0; ///< z of the chassis end, mm
    double clearance = 0.0;       ///< how far the chassis end stands off the chassis, mm
    /// Where the chassis end goes across the car when there is no chassis to
    /// find, as a distance from the centreline (the far side mirrored).
    double inboardY = 150.0;
    double outboardOffset = 40.0; ///< inboard of the ball joint in y, mm
    double damperLength = 200.0;  ///< eye to eye, for a damper in line, mm

    bool operator==(const PushrodDesign& other) const = default;
};

/// Where one design puts its points, on the near side.
struct PushrodPoints {
    bool ok = false;
    QString error;
    Vec3 ballJoint;
    Vec3 outboard; ///< the pushrod's outer end, or the damper's in DamperOnly
    Vec3 inboard;  ///< the chassis end, or the rocker's pushrod arm
    Vec3 damperEye;              ///< InlineDamper: where pushrod and damper meet
    bool inboardOnChassis = false; ///< found on the imported chassis, not set by inboardY
};

/// The points @p design gives, from @p ballJoint and -- with a rocker --
/// @p rockerPickup. @p chassis is what the chassis end is looked for on.
PushrodPoints layOutPushrod(const PushrodDesign& design, const Vec3& ballJoint,
                            const std::optional<Vec3>& rockerPickup, const MeshQuery* chassis);

/// What placing @p designs does to the project: the points, both sides; what
/// each of them is (the settings the solve reads); the points a layout without
/// a rocker has no use for; and which template parts become optional because
/// some corner no longer has them.
struct PushrodPlan {
    DesignPlan points;
    HardpointConfigMap config;    ///< entries to write, overwriting what is there
    QStringList removed;          ///< names taken out of the table
    QStringList unassigned;       ///< names left in, but no longer solved
    QStringList optionalParts;    ///< template part ids that must become optional

    bool ok() const { return points.ok(); }
};

PushrodPlan planPushrods(const std::vector<PushrodDesign>& designs, const LinkageTemplate& templ,
                         const MirrorSpec& mirror, const HardpointTable& current,
                         const HardpointTable& baseline, const MeshQuery* chassis);

QJsonArray pushrodDesignsToJson(const std::vector<PushrodDesign>& designs);
std::vector<PushrodDesign> pushrodDesignsFromJson(const QJsonArray& array);

} // namespace suspkin
