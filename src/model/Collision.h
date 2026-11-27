#pragma once

#include "geom/Vec3.h"
#include "model/Hardpoint.h"
#include "model/Linkage.h"
#include "model/Wheels.h"

#include <QHash>
#include <QJsonObject>
#include <QString>

#include <functional>
#include <optional>
#include <vector>

namespace suspkin {

class MeshQuery;
struct TriMesh;

/// What a collision check can measure a distance to.
enum class CollisionKind { Part, Member, Rim, Tyre, Chassis };

/// One of the two things a collision check measures between: a part of the
/// linkage, one tube of a part, the rim or tyre at one corner, or the
/// imported chassis.
struct CollisionTarget {
    CollisionKind kind = CollisionKind::Part;
    QString partId;                              ///< @ref LinkagePart::id, for a part or a tube
    QString from;                                ///< a tube's two ends, by hardpoint name,
    QString to;                                  ///< in the order the part draws them
    WheelCorner corner = WheelCorner::FrontLeft; ///< for a rim or a tyre

    /// Whether it is made of tubes: a whole part, or one tube of one.
    bool isLinkage() const { return kind == CollisionKind::Part || kind == CollisionKind::Member; }
    bool isWheel() const { return kind == CollisionKind::Rim || kind == CollisionKind::Tyre; }

    bool operator==(const CollisionTarget& other) const;
    bool operator!=(const CollisionTarget& other) const { return !(*this == other); }
};

/// How a target is written into the project: the part's id
/// ("tieRod@F:base"), @c #member:<part id>|<from>|<to>, @c #rim:frontLeft,
/// @c #tyre:rearRight or @c #chassis. The hash keeps them apart from a part a
/// template happens to call "rim".
QString collisionTargetKey(const CollisionTarget& target);
/// The target @p key names, or nothing for an empty key.
std::optional<CollisionTarget> collisionTargetFromKey(const QString& key);

/// The key a tube's own radius is kept under: its two ends by name, in
/// alphabetical order, so it does not matter which way round it is drawn.
QString collisionMemberKey(const QString& from, const QString& to);

/// How a collision check sees the members, and which two things it measures
/// between. Stored in the manifest under @c collision.
struct CollisionSettings {
    /// Every member is taken as a round tube this thick around the line drawn
    /// for it -- half its diameter, mm -- unless @ref memberRadii says
    /// otherwise for that one.
    double memberRadius = 10.0;
    /// A tube's own radius, by @ref collisionMemberKey, mm.
    QHash<QString, double> memberRadii;
    /// Two parts bolted together touch at their joint. Within this distance
    /// of a joint they share -- or of a part both are bolted to -- they are
    /// not compared, so the joint itself is not read as a collision, mm.
    double jointClearance = 30.0;
    /// The two things measured, by @ref collisionTargetKey. Empty until the
    /// user has chosen.
    QString first;
    QString second;

    /// The radius of the tube from @p from to @p to.
    double radiusOf(const QString& from, const QString& to) const;

    bool operator==(const CollisionSettings& other) const = default;
};

QJsonObject collisionSettingsToJson(const CollisionSettings& settings);
CollisionSettings collisionSettingsFromJson(const QJsonObject& object);

/// Where the car is in its travel: the rack, and every wheel's travel from
/// ride height, both mm.
struct CollisionPosition {
    double rack = 0.0;
    double travel = 0.0;

    bool operator==(const CollisionPosition& other) const = default;
};

/// How far the car is checked through: the rack either side of centre, and
/// the wheels up into bump and down into rebound, all written positive, mm.
/// The analysis panel's sweep parameters, whichever kind of sweep it shows.
struct CollisionTravel {
    double rack = 30.0;
    double bump = 25.0;
    double rebound = 25.0;
};

/// The car at one position of the travel: the table, the same points in the
/// same order as the table the linkage was resolved against, moved; and where
/// the wheel models are.
struct CollisionFrame {
    CollisionPosition position;
    HardpointTable table;
    std::vector<WheelPlacement> wheels;
};

/// The car at @p position, or nothing where a corner does not assemble. Asked
/// for whatever positions the search wants, so whoever answers it caches.
using CollisionFrameSource = std::function<std::optional<CollisionFrame>(const CollisionPosition&)>;

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

/// What the user is offered: every part with a member, followed -- when it
/// has more than one -- by each of its tubes; the rim and tyre at each corner
/// @p wheels names when that model is loaded; and the chassis when there is
/// one. In that order. @p table is the one the linkage was resolved against,
/// for the names of the tubes' ends; @p grounded is as for
/// @ref minimumDistance, so a line between two chassis points is no tube.
std::vector<CollisionTarget> collisionTargets(const Linkage& linkage, const HardpointTable& table,
                                              const std::vector<bool>& grounded,
                                              const WheelSpec& wheels,
                                              const CollisionMeshes& meshes);
/// What @p target is called in front of the user.
QString collisionTargetLabel(const CollisionTarget& target, const Linkage& linkage);

/// The tubes @p target is made of, each by its two ends' names, in the order
/// the part draws them: the whole part's, or the one. Empty for anything else.
std::vector<std::pair<QString, QString>> collisionMembers(const CollisionTarget& target,
                                                          const Linkage& linkage,
                                                          const HardpointTable& table,
                                                          const std::vector<bool>& grounded);

/// The least distance found over some stretch of the travel, and where.
struct CollisionExtreme {
    bool valid = false;
    double distance = 0.0; ///< mm, surface to surface; negative overlaps
    CollisionPosition at;
    /// Where on the two it is: the tube of each that comes closest, written
    /// "from - to" (empty for a mesh), and how far along it from its first
    /// end the closest point is, mm.
    QString firstMember;
    double firstAlong = 0.0;
    QString secondMember;
    double secondAlong = 0.0;

    bool touches() const { return valid && distance < 0.0; }
};

/// The least distance between two targets, three ways.
struct CollisionResult {
    bool valid = false;
    /// Why there is no distance, when there is none.
    QString problem;
    /// Straight ahead at ride height.
    CollisionExtreme design;
    /// At ride height, the rack anywhere in its travel: the static clearance
    /// a car is signed off on at any steering position.
    CollisionExtreme rideHeight;
    /// The rack anywhere, the wheels anywhere from full rebound to full bump.
    CollisionExtreme overTravel;
    /// How many positions were measured to find them.
    int positions = 0;

    bool touches() const { return overTravel.touches(); }
};

/// Shortest distance between the segments @p a0-@p a1 and @p b0-@p b1.
double segmentDistance(const Vec3& a0, const Vec3& a1, const Vec3& b0, const Vec3& b1);

/// The least distance between @p first and @p second over @p travel, the car
/// at each position as @p source has it.
///
/// A part is its members, tubes round the lines drawn for it as thick as
/// @ref CollisionSettings::radiusOf says; @p grounded flags the chassis points by table row,
/// as @ref groundedPoints gives them, so a line between two of them is not a
/// member and a member is not compared with the chassis round its own mounts.
/// Two parts are not compared within the joint clearance of a point they
/// share, or of a third part they both meet. Between two meshes the distance
/// is taken from the vertices of the smaller one, so it is as fine as that
/// mesh's tessellation.
///
/// The travel is searched on a coarse grid, and round the closest point of it
/// in ever smaller steps down to a tenth of a millimetre.
CollisionResult minimumDistance(const Linkage& linkage, const std::vector<bool>& grounded,
                                const CollisionMeshes& meshes, const CollisionSettings& settings,
                                const CollisionTarget& first, const CollisionTarget& second,
                                const CollisionTravel& travel, const CollisionFrameSource& source);

} // namespace suspkin
