#pragma once

#include "geom/Vec3.h"
#include "model/Hardpoint.h"
#include "model/Linkage.h"
#include "model/Wheels.h"

#include <QJsonObject>
#include <QString>

#include <optional>
#include <vector>

namespace suspkin {

class MeshQuery;
struct TriMesh;

/// What a collision check can measure a distance to.
enum class CollisionKind { Part, Rim, Tyre, Chassis };

/// One of the two things a collision check measures between: a part of the
/// linkage, the rim or tyre at one corner, or the imported chassis.
struct CollisionTarget {
    CollisionKind kind = CollisionKind::Part;
    QString partId;                              ///< @ref LinkagePart::id, for a part
    WheelCorner corner = WheelCorner::FrontLeft; ///< for a rim or a tyre

    bool isPart() const { return kind == CollisionKind::Part; }
    bool isWheel() const { return kind == CollisionKind::Rim || kind == CollisionKind::Tyre; }

    bool operator==(const CollisionTarget& other) const;
    bool operator!=(const CollisionTarget& other) const { return !(*this == other); }
};

/// How a target is written into the project: the part's id
/// ("tieRod@F:base"), @c #rim:frontLeft, @c #tyre:rearRight or @c #chassis.
/// The hash keeps them apart from a part a template happens to call "rim".
QString collisionTargetKey(const CollisionTarget& target);
/// The target @p key names, or nothing for an empty key.
std::optional<CollisionTarget> collisionTargetFromKey(const QString& key);

/// How a collision check sees the members, and which two things it measures
/// between. Stored in the manifest under @c collision.
struct CollisionSettings {
    /// Every member is taken as a round tube this thick around the line drawn
    /// for it: half its diameter, mm.
    double memberRadius = 10.0;
    /// How far round a joint two parts that meet there -- or meet a third part
    /// there -- are not compared. They are bolted together at it, so the
    /// clearance there is the joint's, not a collision, mm.
    double jointClearance = 30.0;
    /// The two things measured, by @ref collisionTargetKey. Empty until the
    /// user has chosen.
    QString first;
    QString second;

    bool operator==(const CollisionSettings& other) const = default;
};

QJsonObject collisionSettingsToJson(const CollisionSettings& settings);
CollisionSettings collisionSettingsFromJson(const QJsonObject& object);

/// The car at one position of the travel: the table, the same points in the
/// same order as the table the linkage was resolved against, moved; and where
/// the wheel models are.
struct CollisionFrame {
    double input = 0.0;
    HardpointTable table;
    std::vector<WheelPlacement> wheels;
};

/// A mesh a distance can be measured to: its triangles, for its vertices, and
/// the query built over them.
struct CollisionMesh {
    const TriMesh* mesh = nullptr;
    const MeshQuery* query = nullptr;

    bool isEmpty() const;
};

/// The meshes a collision check can measure to. The chassis is in vehicle
/// coordinates; the rim and tyre are in their models' own, placed per frame by
/// @ref CollisionFrame::wheels the way the viewport places them.
struct CollisionMeshes {
    CollisionMesh chassis;
    CollisionMesh rim;
    CollisionMesh tyre;
    /// @ref WheelSpec::alignToCenter, which says where a wheel model's anchor is.
    bool alignToCenter = true;
};

/// What the user is offered: every part with a member, the rim and tyre at
/// each corner @p wheels names when that model is loaded, and the chassis
/// when there is one. In that order.
std::vector<CollisionTarget> collisionTargets(const Linkage& linkage, const WheelSpec& wheels,
                                              const CollisionMeshes& meshes);
/// What @p target is called in front of the user.
QString collisionTargetLabel(const CollisionTarget& target, const Linkage& linkage);

/// The least distance between two targets over a travel.
struct CollisionResult {
    bool valid = false;
    /// Why there is no distance, when there is none.
    QString problem;
    double minDistance = 0.0; ///< mm, surface to surface; negative overlaps
    double atInput = 0.0;     ///< the position it was closest at
    bool designKnown = false;
    double designDistance = 0.0; ///< mm, at input 0, when the travel passes it
    int positions = 0;

    bool touches() const { return valid && minDistance < 0.0; }
};

/// Shortest distance between the segments @p a0-@p a1 and @p b0-@p b1.
double segmentDistance(const Vec3& a0, const Vec3& a1, const Vec3& b0, const Vec3& b1);

/// Walk @p frames and report the least distance between @p first and
/// @p second.
///
/// A part is its members, tubes of @ref CollisionSettings::memberRadius round
/// the lines drawn for it; @p grounded flags the chassis points by table row,
/// as @ref groundedPoints gives them, so a line between two of them is not a
/// member and a member is not compared with the chassis round its own mounts.
/// Two parts are not compared within the joint clearance of a point they
/// share, or of a third part they both meet. Between two meshes the distance
/// is taken from the vertices of the smaller one, so it is as fine as that
/// mesh's tessellation.
CollisionResult minimumDistance(const Linkage& linkage, const std::vector<CollisionFrame>& frames,
                                const std::vector<bool>& grounded, const CollisionMeshes& meshes,
                                const CollisionSettings& settings, const CollisionTarget& first,
                                const CollisionTarget& second);

} // namespace suspkin
