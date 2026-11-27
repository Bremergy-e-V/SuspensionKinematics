#include "model/Collision.h"

#include "geom/MeshQuery.h"
#include "geom/TriMesh.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("Collision", text); }

constexpr double kInfinity = std::numeric_limits<double>::infinity();
/// Two rows this close at design are one place -- a strut's pushrod end and
/// its damper eye -- and join the parts that name them, mm.
constexpr double kCoincident = 0.5;
/// How finely a member is walked along against a mesh, mm, and the most
/// samples one member gets.
constexpr double kMeshStep = 4.0;
constexpr int kMaxMeshSamples = 60;
/// The most vertices of one mesh measured against another, per position. A
/// finer mesh is walked with a stride.
constexpr std::size_t kMaxMeshVertices = 20000;

const char kRadius[] = "memberRadius";
const char kJoint[] = "jointClearance";
const char kFirst[] = "first";
const char kSecond[] = "second";

const char kRimPrefix[] = "#rim:";
const char kTyrePrefix[] = "#tyre:";
const char kChassisKey[] = "#chassis";

Vec3 positionOf(const HardpointTable& table, int row)
{
    const Hardpoint& point = table.points[static_cast<std::size_t>(row)];
    return Vec3(point.coord[0], point.coord[1], point.coord[2]);
}

Vec3 toVec3(const QVector3D& v) { return Vec3(v.x(), v.y(), v.z()); }
QVector3D toQVector(const Vec3& v)
{
    return QVector3D(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

/// One straight piece of a part, by table row.
struct Member {
    int a = 0;
    int b = 0;
};

/// A stretch of a segment, as fractions of the way along it.
struct Interval {
    double from = 0.0;
    double to = 1.0;
};

/// What of @p a0-@p a1 lies outside every sphere of @p radius round
/// @p centres, as fractions of the way along it.
std::vector<Interval> outsideSpheres(const Vec3& a0, const Vec3& a1,
                                     const std::vector<Vec3>& centres, double radius)
{
    std::vector<Interval> kept = { Interval{} };
    const Vec3 along = a1 - a0;
    const double lengthSquared = along.lengthSquared();
    for (const Vec3& centre : centres) {
        // |a0 + t along - centre|^2 <= radius^2, a quadratic in t.
        const Vec3 offset = a0 - centre;
        const double b = dot(offset, along);
        const double c = offset.lengthSquared() - radius * radius;
        if (lengthSquared < 1e-12) {
            if (c > 0.0) continue;
            return {};
        }
        const double discriminant = b * b - lengthSquared * c;
        if (discriminant <= 0.0) continue;
        const double root = std::sqrt(discriminant);
        const double cutFrom = (-b - root) / lengthSquared;
        const double cutTo = (-b + root) / lengthSquared;

        std::vector<Interval> next;
        for (const Interval& piece : kept) {
            if (cutTo <= piece.from || cutFrom >= piece.to) {
                next.push_back(piece);
                continue;
            }
            if (cutFrom > piece.from) next.push_back(Interval{ piece.from, cutFrom });
            if (cutTo < piece.to) next.push_back(Interval{ cutTo, piece.to });
        }
        kept = std::move(next);
        if (kept.empty()) break;
    }
    return kept;
}

Vec3 along(const Vec3& a0, const Vec3& a1, double t) { return a0 + (a1 - a0) * t; }

/// Union-find over table rows, so rows that are one place are one point.
class Places {
public:
    explicit Places(int rows) : m_parent(static_cast<std::size_t>(rows))
    {
        std::iota(m_parent.begin(), m_parent.end(), 0);
    }
    int of(int row)
    {
        while (m_parent[static_cast<std::size_t>(row)] != row)
            row = m_parent[static_cast<std::size_t>(row)] =
                m_parent[static_cast<std::size_t>(m_parent[static_cast<std::size_t>(row)])];
        return row;
    }
    void join(int a, int b) { m_parent[static_cast<std::size_t>(of(a))] = of(b); }

private:
    std::vector<int> m_parent;
};

bool isMember(const LinkagePart& part) { return part.kind != PartKind::Wheel; }

/// The members of @p part: every line drawn for it, except one between two
/// chassis points. A wheel's construction lines are no part of anything.
std::vector<Member> membersOf(const LinkagePart& part, const std::vector<bool>& grounded)
{
    const auto isGrounded = [&](int row) {
        return row >= 0 && row < static_cast<int>(grounded.size())
               && grounded[static_cast<std::size_t>(row)];
    };
    std::vector<Member> members;
    if (!isMember(part)) return members;
    for (const ResolvedChain& chain : part.chains) {
        const int count = static_cast<int>(chain.points.size());
        for (int i = 0; i < chain.segmentCount(); ++i) {
            const int a = chain.points[static_cast<std::size_t>(i)];
            const int b = chain.points[static_cast<std::size_t>((i + 1) % count)];
            if (isGrounded(a) && isGrounded(b)) continue;
            members.push_back(Member{ a, b });
        }
    }
    return members;
}

std::set<int> rowsOf(const LinkagePart& part)
{
    std::set<int> rows;
    for (const ResolvedChain& chain : part.chains) rows.insert(chain.points.begin(), chain.points.end());
    return rows;
}

int partIndex(const Linkage& linkage, const QString& id)
{
    for (std::size_t p = 0; p < linkage.parts.size(); ++p)
        if (linkage.parts[p].id == id) return static_cast<int>(p);
    return -1;
}

/// The rows round which two parts are not compared: where they meet each
/// other, and every point of a third part that meets them both. Rows that are
/// one place at @p design count as one point.
std::vector<int> jointRows(const Linkage& linkage, int first, int second,
                           const HardpointTable& design)
{
    std::set<int> used;
    for (const LinkagePart& part : linkage.parts) {
        const std::set<int> rows = rowsOf(part);
        used.insert(rows.begin(), rows.end());
    }
    Places places(static_cast<int>(design.size()));
    const std::vector<int> rows(used.begin(), used.end());
    for (std::size_t i = 0; i < rows.size(); ++i)
        for (std::size_t j = i + 1; j < rows.size(); ++j)
            if (distance(positionOf(design, rows[i]), positionOf(design, rows[j])) < kCoincident)
                places.join(rows[i], rows[j]);

    const auto placesOf = [&](int part) {
        std::set<int> out;
        for (const int row : rowsOf(linkage.parts[static_cast<std::size_t>(part)]))
            out.insert(places.of(row));
        return out;
    };
    const std::set<int> placesOfFirst = placesOf(first);
    const std::set<int> placesOfSecond = placesOf(second);
    const auto meets = [&](const std::set<int>& a, const std::set<int>& b) {
        return std::any_of(a.begin(), a.end(), [&](int place) { return b.count(place) > 0; });
    };

    std::vector<int> excluded;
    for (const int part : { first, second })
        for (const int row : rowsOf(linkage.parts[static_cast<std::size_t>(part)]))
            if (placesOfFirst.count(places.of(row)) && placesOfSecond.count(places.of(row)))
                excluded.push_back(row);
    for (int r = 0; r < static_cast<int>(linkage.parts.size()); ++r) {
        if (r == first || r == second) continue;
        const std::set<int> placesOfThird = placesOf(r);
        if (!meets(placesOfThird, placesOfFirst) || !meets(placesOfThird, placesOfSecond)) continue;
        const std::set<int> through = rowsOf(linkage.parts[static_cast<std::size_t>(r)]);
        excluded.insert(excluded.end(), through.begin(), through.end());
    }
    return excluded;
}

/// The pieces of @p members in @p table that lie outside the spheres of
/// @p radius round @p rows, as segments.
std::vector<std::pair<Vec3, Vec3>> piecesOutside(const std::vector<Member>& members,
                                                 const HardpointTable& table,
                                                 const std::vector<int>& rows, double radius)
{
    std::vector<Vec3> centres;
    centres.reserve(rows.size());
    for (const int row : rows) centres.push_back(positionOf(table, row));

    std::vector<std::pair<Vec3, Vec3>> pieces;
    for (const Member& member : members) {
        const Vec3 a = positionOf(table, member.a);
        const Vec3 b = positionOf(table, member.b);
        for (const Interval& piece : outsideSpheres(a, b, centres, radius))
            pieces.emplace_back(along(a, b, piece.from), along(a, b, piece.to));
    }
    return pieces;
}

/// The least axis-to-axis distance between two sets of segments.
double piecesDistance(const std::vector<std::pair<Vec3, Vec3>>& first,
                      const std::vector<std::pair<Vec3, Vec3>>& second)
{
    double best = kInfinity;
    for (const auto& a : first)
        for (const auto& b : second)
            best = std::min(best, segmentDistance(a.first, a.second, b.first, b.second));
    return best;
}

/// A mesh where one frame puts it. A chassis stays in vehicle coordinates, so
/// it needs no transform and keeps its doubles exact.
struct PlacedMesh {
    const CollisionMesh* mesh = nullptr;
    std::optional<QMatrix4x4> toWorld;
    std::optional<QMatrix4x4> toModel;

    Vec3 worldToModel(const Vec3& point) const
    {
        return toModel ? toVec3(toModel->map(toQVector(point))) : point;
    }
    double distanceTo(const Vec3& worldPoint) const
    {
        return mesh->query->distanceTo(worldToModel(worldPoint));
    }
};

/// The least distance from the samples along @p pieces to @p mesh.
double piecesToMesh(const std::vector<std::pair<Vec3, Vec3>>& pieces, const PlacedMesh& mesh)
{
    double best = kInfinity;
    for (const auto& [a, b] : pieces) {
        const int samples =
            std::clamp(static_cast<int>(std::ceil(distance(a, b) / kMeshStep)), 1, kMaxMeshSamples);
        for (int i = 0; i <= samples; ++i)
            best = std::min(best, mesh.distanceTo(along(a, b, static_cast<double>(i) / samples)));
    }
    return best;
}

/// The least distance from the vertices of @p from to @p to, both placed.
double meshToMesh(const PlacedMesh& from, const PlacedMesh& to)
{
    const std::vector<QVector3D>& vertices = from.mesh->mesh->positions;
    const std::size_t stride = std::max<std::size_t>(1, vertices.size() / kMaxMeshVertices);
    double best = kInfinity;
    for (std::size_t i = 0; i < vertices.size(); i += stride) {
        const QVector3D world = from.toWorld ? from.toWorld->map(vertices[i]) : vertices[i];
        best = std::min(best, to.distanceTo(toVec3(world)));
    }
    return best;
}

/// The running record of the distance over the frames.
struct Tally {
    double min = kInfinity;
    double at = 0.0;
    bool designKnown = false;
    double design = 0.0;

    void add(double value, double input)
    {
        if (value < min) {
            min = value;
            at = input;
        }
        if (std::abs(input) < 1e-9) {
            designKnown = true;
            design = value;
        }
    }
};

/// The frame the coincident points are read off: the design position when
/// the travel passes it, else the first.
const CollisionFrame& referenceFrame(const std::vector<CollisionFrame>& frames)
{
    return *std::min_element(frames.begin(), frames.end(),
                             [](const CollisionFrame& a, const CollisionFrame& b) {
                                 return std::abs(a.input) < std::abs(b.input);
                             });
}

const CollisionMesh* meshOf(const CollisionTarget& target, const CollisionMeshes& meshes)
{
    switch (target.kind) {
    case CollisionKind::Rim:     return &meshes.rim;
    case CollisionKind::Tyre:    return &meshes.tyre;
    case CollisionKind::Chassis: return &meshes.chassis;
    case CollisionKind::Part:    break;
    }
    return nullptr;
}

/// Measures one pair of targets at one frame. Everything that does not
/// change from frame to frame is worked out once, on construction.
class PairMeasure {
public:
    PairMeasure(const Linkage& linkage, const std::vector<bool>& grounded,
                const CollisionMeshes& meshes, const CollisionSettings& settings,
                const CollisionTarget& first, const CollisionTarget& second,
                const HardpointTable& design)
        : m_meshes(meshes), m_settings(settings), m_first(first), m_second(second)
    {
        if (first.isPart()) setUpPart(linkage, grounded, first, &m_firstMembers, &m_firstMounts);
        if (second.isPart()) setUpPart(linkage, grounded, second, &m_secondMembers, &m_secondMounts);
        if (first.isPart() && second.isPart())
            m_joints = jointRows(linkage, partIndex(linkage, first.partId),
                                 partIndex(linkage, second.partId), design);
    }

    /// Surface to surface, or nothing when this frame has nothing to compare.
    std::optional<double> at(const CollisionFrame& frame) const
    {
        if (m_first.isPart() && m_second.isPart()) return partToPart(frame);
        if (m_first.isPart()) return partToMesh(frame, m_firstMembers, m_firstMounts, m_second);
        if (m_second.isPart()) return partToMesh(frame, m_secondMembers, m_secondMounts, m_first);
        return meshToMeshAt(frame);
    }

private:
    static void setUpPart(const Linkage& linkage, const std::vector<bool>& grounded,
                          const CollisionTarget& target, std::vector<Member>* members,
                          std::vector<int>* mounts)
    {
        const LinkagePart& part = linkage.parts[static_cast<std::size_t>(partIndex(linkage, target.partId))];
        *members = membersOf(part, grounded);
        for (const int row : rowsOf(part))
            if (row < static_cast<int>(grounded.size()) && grounded[static_cast<std::size_t>(row)])
                mounts->push_back(row);
    }

    std::optional<double> partToPart(const CollisionFrame& frame) const
    {
        const double gap = piecesDistance(
            piecesOutside(m_firstMembers, frame.table, m_joints, m_settings.jointClearance),
            piecesOutside(m_secondMembers, frame.table, m_joints, m_settings.jointClearance));
        if (gap == kInfinity) return std::nullopt;
        return gap - 2.0 * m_settings.memberRadius;
    }

    std::optional<double> partToMesh(const CollisionFrame& frame, const std::vector<Member>& members,
                                     const std::vector<int>& mounts,
                                     const CollisionTarget& meshTarget) const
    {
        const std::optional<PlacedMesh> mesh = place(meshTarget, frame);
        if (!mesh) return std::nullopt;
        // A member is bolted to the chassis at its mounts; a wheel is not a
        // place any member is bolted to.
        const std::vector<int> excluded =
            meshTarget.kind == CollisionKind::Chassis ? mounts : std::vector<int>{};
        const double gap =
            piecesToMesh(piecesOutside(members, frame.table, excluded, m_settings.jointClearance), *mesh);
        if (gap == kInfinity) return std::nullopt;
        return gap - m_settings.memberRadius;
    }

    std::optional<double> meshToMeshAt(const CollisionFrame& frame) const
    {
        const std::optional<PlacedMesh> first = place(m_first, frame);
        const std::optional<PlacedMesh> second = place(m_second, frame);
        if (!first || !second) return std::nullopt;
        // The smaller mesh's vertices against the larger mesh's surface: the
        // larger one is the finer, and a query against it costs a log.
        const bool firstIsSmaller =
            first->mesh->mesh->vertexCount() <= second->mesh->mesh->vertexCount();
        const double gap = firstIsSmaller ? meshToMesh(*first, *second) : meshToMesh(*second, *first);
        if (gap == kInfinity) return std::nullopt;
        return gap;
    }

    /// Where @p target's mesh is at @p frame. Nothing for a corner that has no
    /// wheel there.
    std::optional<PlacedMesh> place(const CollisionTarget& target, const CollisionFrame& frame) const
    {
        PlacedMesh placed;
        placed.mesh = meshOf(target, m_meshes);
        if (!target.isWheel()) return placed;
        const auto wheel = std::find_if(frame.wheels.begin(), frame.wheels.end(),
                                        [&](const WheelPlacement& p) { return p.corner == target.corner; });
        if (wheel == frame.wheels.end()) return std::nullopt;
        placed.toWorld = wheelTransform(*wheel, placed.mesh->mesh->bounds, m_meshes.alignToCenter);
        placed.toModel = placed.toWorld->inverted();
        return placed;
    }

    const CollisionMeshes& m_meshes;
    const CollisionSettings& m_settings;
    CollisionTarget m_first;
    CollisionTarget m_second;
    std::vector<Member> m_firstMembers;
    std::vector<Member> m_secondMembers;
    std::vector<int> m_firstMounts;
    std::vector<int> m_secondMounts;
    std::vector<int> m_joints;
};

/// Why @p target cannot be measured against anything, or empty when it can.
QString targetProblem(const CollisionTarget& target, const Linkage& linkage,
                      const std::vector<bool>& grounded, const CollisionMeshes& meshes)
{
    switch (target.kind) {
    case CollisionKind::Part: {
        const int index = partIndex(linkage, target.partId);
        if (index < 0) return tr("The linkage has no part %1.").arg(target.partId);
        if (membersOf(linkage.parts[static_cast<std::size_t>(index)], grounded).empty())
            return tr("%1 has no member to measure from.")
                .arg(linkage.parts[static_cast<std::size_t>(index)].label);
        return {};
    }
    case CollisionKind::Rim:
        return meshes.rim.isEmpty() ? tr("No rim model is loaded (Geometry > Add Wheels).") : QString();
    case CollisionKind::Tyre:
        return meshes.tyre.isEmpty() ? tr("No tyre model is loaded (Geometry > Add Wheels).")
                                     : QString();
    case CollisionKind::Chassis:
        return meshes.chassis.isEmpty() ? tr("No chassis is imported.") : QString();
    }
    return {};
}

/// Why the two targets cannot be measured against each other, or empty.
QString pairProblem(const CollisionTarget& first, const CollisionTarget& second)
{
    if (first == second) return tr("Choose two different things to measure between.");
    const bool rimAndTyre =
        (first.kind == CollisionKind::Rim && second.kind == CollisionKind::Tyre)
        || (first.kind == CollisionKind::Tyre && second.kind == CollisionKind::Rim);
    if (rimAndTyre && first.corner == second.corner)
        return tr("A tyre sits on its own rim: the two always touch.");
    return {};
}

} // namespace

bool CollisionTarget::operator==(const CollisionTarget& other) const
{
    if (kind != other.kind) return false;
    if (kind == CollisionKind::Part) return partId == other.partId;
    if (isWheel()) return corner == other.corner;
    return true;
}

QString collisionTargetKey(const CollisionTarget& target)
{
    switch (target.kind) {
    case CollisionKind::Part:    return target.partId;
    case CollisionKind::Rim:     return QLatin1String(kRimPrefix) + wheelCornerToString(target.corner);
    case CollisionKind::Tyre:    return QLatin1String(kTyrePrefix) + wheelCornerToString(target.corner);
    case CollisionKind::Chassis: break;
    }
    return QLatin1String(kChassisKey);
}

std::optional<CollisionTarget> collisionTargetFromKey(const QString& key)
{
    if (key.isEmpty()) return std::nullopt;
    CollisionTarget target;
    if (key == QLatin1String(kChassisKey)) {
        target.kind = CollisionKind::Chassis;
    } else if (key.startsWith(QLatin1String(kRimPrefix))) {
        target.kind = CollisionKind::Rim;
        target.corner = wheelCornerFromString(key.mid(int(sizeof(kRimPrefix) - 1)));
    } else if (key.startsWith(QLatin1String(kTyrePrefix))) {
        target.kind = CollisionKind::Tyre;
        target.corner = wheelCornerFromString(key.mid(int(sizeof(kTyrePrefix) - 1)));
    } else {
        target.partId = key;
    }
    return target;
}

QJsonObject collisionSettingsToJson(const CollisionSettings& settings)
{
    QJsonObject object;
    object.insert(QLatin1String(kRadius), settings.memberRadius);
    object.insert(QLatin1String(kJoint), settings.jointClearance);
    if (!settings.first.isEmpty()) object.insert(QLatin1String(kFirst), settings.first);
    if (!settings.second.isEmpty()) object.insert(QLatin1String(kSecond), settings.second);
    return object;
}

CollisionSettings collisionSettingsFromJson(const QJsonObject& object)
{
    const CollisionSettings defaults;
    CollisionSettings settings;
    settings.memberRadius = object.value(QLatin1String(kRadius)).toDouble(defaults.memberRadius);
    settings.jointClearance = object.value(QLatin1String(kJoint)).toDouble(defaults.jointClearance);
    settings.first = object.value(QLatin1String(kFirst)).toString();
    settings.second = object.value(QLatin1String(kSecond)).toString();
    return settings;
}

bool CollisionMesh::isEmpty() const { return !mesh || !query || mesh->isEmpty() || query->isEmpty(); }

std::vector<CollisionTarget> collisionTargets(const Linkage& linkage, const WheelSpec& wheels,
                                              const CollisionMeshes& meshes)
{
    std::vector<CollisionTarget> targets;
    for (const LinkagePart& part : linkage.parts)
        if (isMember(part)) targets.push_back(CollisionTarget{ CollisionKind::Part, part.id, {} });
    for (const CollisionKind kind : { CollisionKind::Rim, CollisionKind::Tyre }) {
        if (meshOf(CollisionTarget{ kind, {}, {} }, meshes)->isEmpty()) continue;
        for (const WheelCorner corner : kWheelCorners)
            if (!wheels.point(corner).isEmpty()) targets.push_back(CollisionTarget{ kind, {}, corner });
    }
    if (!meshes.chassis.isEmpty()) targets.push_back(CollisionTarget{ CollisionKind::Chassis, {}, {} });
    return targets;
}

QString collisionTargetLabel(const CollisionTarget& target, const Linkage& linkage)
{
    switch (target.kind) {
    case CollisionKind::Part: {
        const int index = partIndex(linkage, target.partId);
        return index < 0 ? target.partId : linkage.parts[static_cast<std::size_t>(index)].label;
    }
    case CollisionKind::Rim:     return tr("%1 rim").arg(wheelCornerLabel(target.corner));
    case CollisionKind::Tyre:    return tr("%1 tyre").arg(wheelCornerLabel(target.corner));
    case CollisionKind::Chassis: break;
    }
    return tr("Chassis");
}

double segmentDistance(const Vec3& a0, const Vec3& a1, const Vec3& b0, const Vec3& b1)
{
    // The closest points of two segments (Ericson, Real-Time Collision
    // Detection, 5.1.9), clamped onto both.
    const Vec3 d1 = a1 - a0;
    const Vec3 d2 = b1 - b0;
    const Vec3 r = a0 - b0;
    const double a = d1.lengthSquared();
    const double e = d2.lengthSquared();
    const double f = dot(d2, r);
    constexpr double kPoint = 1e-12;

    double s = 0.0;
    double t = 0.0;
    if (a <= kPoint && e <= kPoint) return distance(a0, b0);
    if (a <= kPoint) {
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = dot(d1, r);
        if (e <= kPoint) {
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = dot(d1, d2);
            const double denominator = a * e - b * b;
            s = denominator > kPoint ? std::clamp((b * f - c * e) / denominator, 0.0, 1.0) : 0.0;
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    return distance(a0 + d1 * s, b0 + d2 * t);
}

CollisionResult minimumDistance(const Linkage& linkage, const std::vector<CollisionFrame>& frames,
                                const std::vector<bool>& grounded, const CollisionMeshes& meshes,
                                const CollisionSettings& settings, const CollisionTarget& first,
                                const CollisionTarget& second)
{
    CollisionResult result;
    result.positions = static_cast<int>(frames.size());
    for (const QString& problem : { targetProblem(first, linkage, grounded, meshes),
                                    targetProblem(second, linkage, grounded, meshes),
                                    pairProblem(first, second) }) {
        if (problem.isEmpty()) continue;
        result.problem = problem;
        return result;
    }
    if (frames.empty()) {
        result.problem = tr("There is no sweep to walk: the car does not solve.");
        return result;
    }

    const PairMeasure measure(linkage, grounded, meshes, settings, first, second,
                              referenceFrame(frames).table);
    Tally tally;
    for (const CollisionFrame& frame : frames)
        if (const std::optional<double> value = measure.at(frame)) tally.add(*value, frame.input);

    if (tally.min == kInfinity) {
        result.problem = tr("Nothing is left to compare: the two meet within the joint "
                            "clearance, or there is no wheel at that corner.");
        return result;
    }
    result.valid = true;
    result.minDistance = tally.min;
    result.atInput = tally.at;
    result.designKnown = tally.designKnown;
    result.designDistance = tally.design;
    return result;
}

} // namespace suspkin
