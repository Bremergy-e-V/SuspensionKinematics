#pragma once

#include "geom/Vec3.h"
#include "model/Hardpoint.h"
#include "model/Linkage.h"

#include <QJsonObject>
#include <QString>

#include <vector>

namespace suspkin {

class MeshQuery;

/// How a collision check sees the members, and what it calls too close.
/// Stored in the manifest under @c collision.
struct CollisionSettings {
    /// Every member is taken as a round tube this thick around the line drawn
    /// for it: half its diameter, mm.
    double memberRadius = 10.0;
    /// Clearance below this is reported as tight, mm; below zero the two touch.
    double warnClearance = 5.0;
    /// How far round a joint two parts that meet there -- or meet a third part
    /// there -- are not compared. They are bolted together at it, so the
    /// clearance there is the joint's, not a collision, mm.
    double jointClearance = 30.0;
    /// Compare the members with the imported chassis as well.
    bool checkChassis = true;

    bool operator==(const CollisionSettings& other) const = default;
};

QJsonObject collisionSettingsToJson(const CollisionSettings& settings);
CollisionSettings collisionSettingsFromJson(const QJsonObject& object);

/// The table at one position of the travel: the same points in the same order
/// as the table the linkage was resolved against, moved.
struct CollisionFrame {
    double input = 0.0;
    HardpointTable table;
};

/// The closest two members came over the whole travel.
struct CollisionPair {
    QString first;  ///< a part's label
    QString second; ///< another part's label, or "Chassis"
    bool chassis = false;
    double minClearance = 0.0; ///< mm, surface to surface; negative overlaps
    double atInput = 0.0;      ///< the position it was closest at
    bool designKnown = false;
    double designClearance = 0.0; ///< mm, at input 0, when the travel passes it

    bool touches() const { return minClearance < 0.0; }
};

struct CollisionReport {
    /// Every pair compared, closest first.
    std::vector<CollisionPair> pairs;
    int positions = 0;
};

/// Shortest distance between the segments @p a0-@p a1 and @p b0-@p b1.
double segmentDistance(const Vec3& a0, const Vec3& a1, const Vec3& b0, const Vec3& b1);

/// Walk @p frames and report, for every two parts of @p linkage that are not
/// joined to each other, the least clearance between them -- and between each
/// part and @p chassis, when there is one and @p settings asks.
///
/// @p grounded flags the chassis points by table row, as
/// @ref groundedPoints gives them: a line between two of them is not a member,
/// and a member's chassis end is not compared with the chassis it is bolted to.
/// Wheel construction lines are not members either.
CollisionReport checkCollisions(const Linkage& linkage, const std::vector<CollisionFrame>& frames,
                                const std::vector<bool>& grounded, const MeshQuery* chassis,
                                const CollisionSettings& settings);

} // namespace suspkin
