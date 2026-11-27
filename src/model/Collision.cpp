#include "model/Collision.h"

#include "geom/MeshQuery.h"
#include "geom/TriMesh.h"

#include <QCoreApplication>
#include <QJsonValue>

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
constexpr double kMeshStep = 1.0;
constexpr int kMaxMeshSamples = 2000;
/// The most vertices of one mesh measured against another, per position. A
/// finer mesh is walked with a stride.
constexpr std::size_t kMaxMeshVertices = 20000;
/// The coarse grid the travel is first searched on, mm of rack and of wheel
/// travel, and the step the search round its closest point stops at.
constexpr double kCoarseRackStep = 2.0;
constexpr double kCoarseTravelStep = 2.0;
constexpr double kFinestStep = 0.1;
/// A bound on how often the fine search may move before it must halve.
constexpr int kMaxRefineMoves = 200;

const char kRadius[] = "memberRadius";
const char kRadii[] = "memberRadii";
const char kJoint[] = "jointClearance";
const char kFirst[] = "first";
const char kSecond[] = "second";

const char kMemberPrefix[] = "#member:";
const char kRimPrefix[] = "#rim:";
const char kTyrePrefix[] = "#tyre:";
const char kChassisKey[] = "#chassis";
const QLatin1Char kMemberSeparator('|');

Vec3 positionOf(const HardpointTable& table, int row)
{
    const Hardpoint& point = table.points[static_cast<std::size_t>(row)];
    return Vec3(point.coord[0], point.coord[1], point.coord[2]);
}

QString nameOf(const HardpointTable& table, int row)
{
    return table.points[static_cast<std::size_t>(row)].name;
}

Vec3 toVec3(const QVector3D& v) { return Vec3(v.x(), v.y(), v.z()); }
QVector3D toQVector(const Vec3& v)
{
    return QVector3D(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

Vec3 along(const Vec3& a0, const Vec3& a1, double t) { return a0 + (a1 - a0) * t; }

/// The fractions along two segments their closest points are at
/// (Ericson, Real-Time Collision Detection, 5.1.9).
struct ClosestPoints {
    double s = 0.0;
    double t = 0.0;
};

ClosestPoints closestPoints(const Vec3& a0, const Vec3& a1, const Vec3& b0, const Vec3& b1)
{
    const Vec3 d1 = a1 - a0;
    const Vec3 d2 = b1 - b0;
    const Vec3 r = a0 - b0;
    const double a = d1.lengthSquared();
    const double e = d2.lengthSquared();
    const double f = dot(d2, r);
    constexpr double kPoint = 1e-12;

    ClosestPoints out;
    if (a <= kPoint && e <= kPoint) return out;
    if (a <= kPoint) {
        out.t = std::clamp(f / e, 0.0, 1.0);
        return out;
    }
    const double c = dot(d1, r);
    if (e <= kPoint) {
        out.s = std::clamp(-c / a, 0.0, 1.0);
        return out;
    }
    const double b = dot(d1, d2);
    const double denominator = a * e - b * b;
    out.s = denominator > kPoint ? std::clamp((b * f - c * e) / denominator, 0.0, 1.0) : 0.0;
    out.t = (b * out.s + f) / e;
    if (out.t < 0.0) {
        out.t = 0.0;
        out.s = std::clamp(-c / a, 0.0, 1.0);
    } else if (out.t > 1.0) {
        out.t = 1.0;
        out.s = std::clamp((b - c) / a, 0.0, 1.0);
    }
    return out;
}

/// One straight tube of a part, by table row, and how thick it is.
struct Tube {
    int a = 0;
    int b = 0;
    double radius = 0.0;
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
    const Vec3 direction = a1 - a0;
    const double lengthSquared = direction.lengthSquared();
    for (const Vec3& centre : centres) {
        // |a0 + t direction - centre|^2 <= radius^2, a quadratic in t.
        const Vec3 offset = a0 - centre;
        const double b = dot(offset, direction);
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

bool isGrounded(const std::vector<bool>& grounded, int row)
{
    return row >= 0 && row < static_cast<int>(grounded.size())
           && grounded[static_cast<std::size_t>(row)];
}

/// A line drawn for a part, by table row.
struct Line {
    int a = 0;
    int b = 0;
};

/// The members of @p part: every line drawn for it, except one between two
/// chassis points. A wheel's construction lines are no part of anything.
std::vector<Line> linesOf(const LinkagePart& part, const std::vector<bool>& grounded)
{
    std::vector<Line> lines;
    if (!isMember(part)) return lines;
    for (const ResolvedChain& chain : part.chains) {
        const int count = static_cast<int>(chain.points.size());
        for (int i = 0; i < chain.segmentCount(); ++i) {
            const int a = chain.points[static_cast<std::size_t>(i)];
            const int b = chain.points[static_cast<std::size_t>((i + 1) % count)];
            if (isGrounded(grounded, a) && isGrounded(grounded, b)) continue;
            lines.push_back(Line{ a, b });
        }
    }
    return lines;
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

const LinkagePart* partOf(const CollisionTarget& target, const Linkage& linkage)
{
    const int index = target.isLinkage() ? partIndex(linkage, target.partId) : -1;
    return index < 0 ? nullptr : &linkage.parts[static_cast<std::size_t>(index)];
}

/// The lines @p target is made of: the part's, or the one it names.
std::vector<Line> targetLines(const CollisionTarget& target, const Linkage& linkage,
                              const HardpointTable& table, const std::vector<bool>& grounded)
{
    const LinkagePart* part = partOf(target, linkage);
    if (!part) return {};
    std::vector<Line> lines = linesOf(*part, grounded);
    if (target.kind == CollisionKind::Part) return lines;
    const auto named = [&](const Line& line) {
        return nameOf(table, line.a) == target.from && nameOf(table, line.b) == target.to;
    };
    const auto found = std::find_if(lines.begin(), lines.end(), named);
    return found == lines.end() ? std::vector<Line>{} : std::vector<Line>{ *found };
}

QString lineLabel(const HardpointTable& table, const Line& line)
{
    return nameOf(table, line.a) + QStringLiteral(" - ") + nameOf(table, line.b);
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

/// What of one tube is compared at one frame: a stretch of it outside the
/// joints, as a segment and as fractions of the tube.
struct Piece {
    int tube = 0;
    Interval span;
    Vec3 a;
    Vec3 b;
};

/// The pieces of @p tubes in @p table that lie outside the spheres of
/// @p radius round @p rows.
std::vector<Piece> piecesOutside(const std::vector<Tube>& tubes, const HardpointTable& table,
                                 const std::vector<int>& rows, double radius)
{
    std::vector<Vec3> centres;
    centres.reserve(rows.size());
    for (const int row : rows) centres.push_back(positionOf(table, row));

    std::vector<Piece> pieces;
    for (std::size_t i = 0; i < tubes.size(); ++i) {
        const Vec3 a = positionOf(table, tubes[i].a);
        const Vec3 b = positionOf(table, tubes[i].b);
        for (const Interval& span : outsideSpheres(a, b, centres, radius))
            pieces.push_back(Piece{ static_cast<int>(i), span, along(a, b, span.from),
                                    along(a, b, span.to) });
    }
    return pieces;
}

/// Where the closest approach at one frame is, and how close.
struct Contact {
    double distance = kInfinity;
    int firstTube = -1;
    double firstAlong = 0.0; ///< mm from the tube's first end
    int secondTube = -1;
    double secondAlong = 0.0;

    bool found() const { return distance < kInfinity; }
    Contact swapped() const { return Contact{ distance, secondTube, secondAlong, firstTube, firstAlong }; }
};

/// How far along its tube, from the tube's first end, the point at @p t of
/// @p piece is, mm.
double alongTube(const Piece& piece, const std::vector<Tube>& tubes, const HardpointTable& table,
                 double t)
{
    const Tube& tube = tubes[static_cast<std::size_t>(piece.tube)];
    const double fraction = piece.span.from + (piece.span.to - piece.span.from) * t;
    return fraction * distance(positionOf(table, tube.a), positionOf(table, tube.b));
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

/// The least distance from a run of points to a mesh, asking the mesh only
/// where it could matter. The distance to a surface changes no faster than
/// the point moves, so a point close to the last one measured cannot come
/// under the record by more than the step between them: such a point is not
/// measured. Nothing above @p ceiling is worth knowing exactly.
class SkippingWalk {
public:
    SkippingWalk(const PlacedMesh& mesh, double ceiling) : m_mesh(mesh), m_ceiling(ceiling) {}

    /// Whether @p point came under the record, which it then is.
    bool add(const Vec3& point)
    {
        const double bar = std::min(m_best, m_ceiling);
        if (m_anchored && m_anchorDistance - distance(point, m_anchor) >= bar) return false;
        m_anchor = point;
        m_anchorDistance = m_mesh.distanceTo(point);
        m_anchored = true;
        if (m_anchorDistance >= m_best) return false;
        m_best = m_anchorDistance;
        return true;
    }
    double best() const { return m_best; }

private:
    const PlacedMesh& m_mesh;
    double m_ceiling;
    double m_best = kInfinity;
    bool m_anchored = false;
    Vec3 m_anchor;
    double m_anchorDistance = 0.0;
};

const CollisionMesh* meshOf(const CollisionTarget& target, const CollisionMeshes& meshes)
{
    switch (target.kind) {
    case CollisionKind::Rim:     return &meshes.rim;
    case CollisionKind::Tyre:    return &meshes.tyre;
    case CollisionKind::Chassis: return &meshes.chassis;
    case CollisionKind::Part:
    case CollisionKind::Member:  break;
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
        : m_meshes(meshes), m_settings(settings), m_first(first), m_second(second),
          m_design(design)
    {
        m_firstTubes = tubesOf(first, linkage, grounded);
        m_secondTubes = tubesOf(second, linkage, grounded);
        m_firstMounts = mountsOf(first, linkage, grounded);
        m_secondMounts = mountsOf(second, linkage, grounded);
        if (first.isLinkage() && second.isLinkage())
            m_joints = jointRows(linkage, partIndex(linkage, first.partId),
                                 partIndex(linkage, second.partId), design);
    }

    /// Surface to surface, or nothing when this frame has nothing to
    /// compare. Above @p ceiling the answer is only known not to be lower.
    std::optional<Contact> at(const CollisionFrame& frame, double ceiling) const
    {
        if (m_first.isLinkage() && m_second.isLinkage()) return partToPart(frame);
        if (m_first.isLinkage())
            return partToMesh(frame, m_firstTubes, m_firstMounts, m_second, ceiling);
        if (m_second.isLinkage()) {
            const std::optional<Contact> contact =
                partToMesh(frame, m_secondTubes, m_secondMounts, m_first, ceiling);
            return contact ? std::optional<Contact>(contact->swapped()) : std::nullopt;
        }
        return meshToMeshAt(frame, ceiling);
    }

    /// The tube of the first or second target @p index names, as text.
    QString tubeLabel(bool first, int index) const
    {
        const std::vector<Tube>& tubes = first ? m_firstTubes : m_secondTubes;
        if (index < 0 || index >= static_cast<int>(tubes.size())) return {};
        const Tube& tube = tubes[static_cast<std::size_t>(index)];
        return lineLabel(m_design, Line{ tube.a, tube.b });
    }

private:
    std::vector<Tube> tubesOf(const CollisionTarget& target, const Linkage& linkage,
                              const std::vector<bool>& grounded) const
    {
        std::vector<Tube> tubes;
        for (const Line& line : targetLines(target, linkage, m_design, grounded))
            tubes.push_back(Tube{ line.a, line.b,
                                  m_settings.radiusOf(nameOf(m_design, line.a),
                                                      nameOf(m_design, line.b)) });
        return tubes;
    }

    static std::vector<int> mountsOf(const CollisionTarget& target, const Linkage& linkage,
                                     const std::vector<bool>& grounded)
    {
        std::vector<int> mounts;
        if (const LinkagePart* part = partOf(target, linkage))
            for (const int row : rowsOf(*part))
                if (isGrounded(grounded, row)) mounts.push_back(row);
        return mounts;
    }

    std::optional<Contact> partToPart(const CollisionFrame& frame) const
    {
        const std::vector<Piece> first =
            piecesOutside(m_firstTubes, frame.table, m_joints, m_settings.jointClearance);
        const std::vector<Piece> second =
            piecesOutside(m_secondTubes, frame.table, m_joints, m_settings.jointClearance);
        Contact best;
        for (const Piece& a : first)
            for (const Piece& b : second) {
                const ClosestPoints closest = closestPoints(a.a, a.b, b.a, b.b);
                const double gap = distance(along(a.a, a.b, closest.s), along(b.a, b.b, closest.t))
                                   - m_firstTubes[static_cast<std::size_t>(a.tube)].radius
                                   - m_secondTubes[static_cast<std::size_t>(b.tube)].radius;
                if (gap >= best.distance) continue;
                best = Contact{ gap, a.tube, alongTube(a, m_firstTubes, frame.table, closest.s),
                                b.tube, alongTube(b, m_secondTubes, frame.table, closest.t) };
            }
        return best.found() ? std::optional<Contact>(best) : std::nullopt;
    }

    std::optional<Contact> partToMesh(const CollisionFrame& frame, const std::vector<Tube>& tubes,
                                      const std::vector<int>& mounts,
                                      const CollisionTarget& meshTarget, double ceiling) const
    {
        const std::optional<PlacedMesh> mesh = place(meshTarget, frame);
        if (!mesh) return std::nullopt;
        // A member is bolted to the chassis at its mounts; a wheel is not a
        // place any member is bolted to.
        const std::vector<int> excluded =
            meshTarget.kind == CollisionKind::Chassis ? mounts : std::vector<int>{};
        Contact best;
        for (const Piece& piece : piecesOutside(tubes, frame.table, excluded, m_settings.jointClearance)) {
            const double radius = tubes[static_cast<std::size_t>(piece.tube)].radius;
            // The walk is of axis distances, so its ceiling is the surface's
            // plus the tube.
            SkippingWalk walk(*mesh, std::min(best.distance, ceiling) + radius);
            const int samples = std::clamp(
                static_cast<int>(std::ceil(distance(piece.a, piece.b) / kMeshStep)), 1, kMaxMeshSamples);
            for (int i = 0; i <= samples; ++i) {
                const double t = static_cast<double>(i) / samples;
                if (!walk.add(along(piece.a, piece.b, t))) continue;
                const double gap = walk.best() - radius;
                if (gap < best.distance)
                    best = Contact{ gap, piece.tube, alongTube(piece, tubes, frame.table, t), -1, 0.0 };
            }
        }
        return best.found() ? std::optional<Contact>(best) : std::nullopt;
    }

    std::optional<Contact> meshToMeshAt(const CollisionFrame& frame, double ceiling) const
    {
        const std::optional<PlacedMesh> first = place(m_first, frame);
        const std::optional<PlacedMesh> second = place(m_second, frame);
        if (!first || !second) return std::nullopt;
        // The smaller mesh's vertices against the larger mesh's surface: the
        // larger one is the finer, and a query against it costs a log.
        const bool firstIsSmaller =
            first->mesh->mesh->vertexCount() <= second->mesh->mesh->vertexCount();
        const double gap = firstIsSmaller ? meshToMesh(*first, *second, ceiling)
                                          : meshToMesh(*second, *first, ceiling);
        if (gap == kInfinity) return std::nullopt;
        return Contact{ gap, -1, 0.0, -1, 0.0 };
    }

    /// The least distance from the vertices of @p from to @p to, both placed.
    static double meshToMesh(const PlacedMesh& from, const PlacedMesh& to, double ceiling)
    {
        const std::vector<QVector3D>& vertices = from.mesh->mesh->positions;
        const std::size_t stride = std::max<std::size_t>(1, vertices.size() / kMaxMeshVertices);
        SkippingWalk walk(to, ceiling);
        for (std::size_t i = 0; i < vertices.size(); i += stride)
            walk.add(toVec3(from.toWorld ? from.toWorld->map(vertices[i]) : vertices[i]));
        return walk.best();
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
    const HardpointTable& m_design;
    std::vector<Tube> m_firstTubes;
    std::vector<Tube> m_secondTubes;
    std::vector<int> m_firstMounts;
    std::vector<int> m_secondMounts;
    std::vector<int> m_joints;
};

/// The values a coarse pass visits from -@p below to +@p above: nought, every
/// @p step out from it, and both ends.
std::vector<double> coarseValues(double below, double above, double step)
{
    std::vector<double> values = { 0.0 };
    for (const double end : { above, -below }) {
        const double sign = end < 0.0 ? -1.0 : 1.0;
        const double reach = std::abs(end);
        for (double value = step; value < reach - 1e-9; value += step) values.push_back(sign * value);
        if (reach > 1e-9) values.push_back(end);
    }
    std::sort(values.begin(), values.end());
    return values;
}

/// Searches the travel for where one pair comes closest.
class EnvelopeSearch {
public:
    EnvelopeSearch(const PairMeasure& measure, const CollisionFrameSource& source,
                   const CollisionTravel& travel)
        : m_measure(measure), m_source(source), m_travel(travel)
    {
    }

    /// Straight ahead at ride height, measured exactly.
    CollisionExtreme design() { return extremeAt(CollisionPosition{}, kInfinity); }

    /// At ride height, the rack anywhere.
    CollisionExtreme rideHeight()
    {
        CollisionExtreme best;
        for (const double rack : coarseValues(m_travel.rack, m_travel.rack, kCoarseRackStep))
            consider(CollisionPosition{ rack, 0.0 }, &best);
        return refine(best, kCoarseRackStep / 2.0, false);
    }

    /// Anywhere, starting from what is already known at ride height.
    CollisionExtreme overTravel(const CollisionExtreme& atRideHeight)
    {
        CollisionExtreme best = atRideHeight;
        const std::vector<double> racks = coarseValues(m_travel.rack, m_travel.rack, kCoarseRackStep);
        for (const double travel : coarseValues(m_travel.rebound, m_travel.bump, kCoarseTravelStep)) {
            if (travel == 0.0) continue; // the ride height line is already done
            for (const double rack : racks) consider(CollisionPosition{ rack, travel }, &best);
        }
        return refine(best, std::max(kCoarseRackStep, kCoarseTravelStep) / 2.0, true);
    }

    int positions() const { return m_positions; }

private:
    /// @p position measured, kept in @p best when it is closer.
    void consider(const CollisionPosition& position, CollisionExtreme* best)
    {
        const CollisionExtreme here = extremeAt(position, best->valid ? best->distance : kInfinity);
        if (here.valid && (!best->valid || here.distance < best->distance)) *best = here;
    }

    /// Ever finer steps round @p best, across the rack and -- with
    /// @p travelToo -- the wheel travel, until a step is finer than it need be.
    CollisionExtreme refine(CollisionExtreme best, double step, bool travelToo)
    {
        if (!best.valid) return best;
        int moves = 0;
        while (step >= kFinestStep && moves < kMaxRefineMoves) {
            const CollisionPosition centre = best.at;
            for (int dr = -1; dr <= 1; ++dr)
                for (int dt = travelToo ? -1 : 0; dt <= (travelToo ? 1 : 0); ++dt) {
                    const CollisionPosition next =
                        clamped(CollisionPosition{ centre.rack + dr * step, centre.travel + dt * step });
                    if (next == centre) continue;
                    consider(next, &best);
                }
            if (best.at == centre) step /= 2.0;
            else ++moves;
        }
        return best;
    }

    CollisionPosition clamped(const CollisionPosition& position) const
    {
        return CollisionPosition{ std::clamp(position.rack, -m_travel.rack, m_travel.rack),
                                  std::clamp(position.travel, -m_travel.rebound, m_travel.bump) };
    }

    CollisionExtreme extremeAt(const CollisionPosition& position, double ceiling)
    {
        CollisionExtreme extreme;
        const std::optional<CollisionFrame> frame = m_source(position);
        if (!frame) return extreme;
        ++m_positions;
        const std::optional<Contact> contact = m_measure.at(*frame, ceiling);
        if (!contact) return extreme;
        extreme.valid = true;
        extreme.distance = contact->distance;
        extreme.at = position;
        extreme.firstMember = m_measure.tubeLabel(true, contact->firstTube);
        extreme.firstAlong = contact->firstAlong;
        extreme.secondMember = m_measure.tubeLabel(false, contact->secondTube);
        extreme.secondAlong = contact->secondAlong;
        return extreme;
    }

    const PairMeasure& m_measure;
    const CollisionFrameSource& m_source;
    CollisionTravel m_travel;
    int m_positions = 0;
};

/// Why @p target cannot be measured against anything, or empty when it can.
QString targetProblem(const CollisionTarget& target, const Linkage& linkage,
                      const HardpointTable& table, const std::vector<bool>& grounded,
                      const CollisionMeshes& meshes)
{
    switch (target.kind) {
    case CollisionKind::Part:
    case CollisionKind::Member: {
        const LinkagePart* part = partOf(target, linkage);
        if (!part) return tr("The linkage has no part %1.").arg(target.partId);
        if (targetLines(target, linkage, table, grounded).empty())
            return target.kind == CollisionKind::Part
                       ? tr("%1 has no member to measure from.").arg(part->label)
                       : tr("%1 has no tube from %2 to %3.").arg(part->label, target.from, target.to);
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
    if (first.isLinkage() && second.isLinkage() && first.partId == second.partId)
        return tr("Both are the same part: choose two different ones.");
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
    if (kind == CollisionKind::Member)
        return partId == other.partId && from == other.from && to == other.to;
    if (isWheel()) return corner == other.corner;
    return true;
}

QString collisionTargetKey(const CollisionTarget& target)
{
    switch (target.kind) {
    case CollisionKind::Part:    return target.partId;
    case CollisionKind::Member:
        return QLatin1String(kMemberPrefix) + target.partId + kMemberSeparator + target.from
               + kMemberSeparator + target.to;
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
    } else if (key.startsWith(QLatin1String(kMemberPrefix))) {
        // The part id comes first and may hold anything; the two names are
        // the last two fields.
        const QString rest = key.mid(int(sizeof(kMemberPrefix) - 1));
        const int last = rest.lastIndexOf(kMemberSeparator);
        const int middle = last > 0 ? rest.lastIndexOf(kMemberSeparator, last - 1) : -1;
        if (middle < 0) return std::nullopt;
        target.kind = CollisionKind::Member;
        target.partId = rest.left(middle);
        target.from = rest.mid(middle + 1, last - middle - 1);
        target.to = rest.mid(last + 1);
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

QString collisionMemberKey(const QString& from, const QString& to)
{
    return from < to ? from + kMemberSeparator + to : to + kMemberSeparator + from;
}

double CollisionSettings::radiusOf(const QString& from, const QString& to) const
{
    return memberRadii.value(collisionMemberKey(from, to), memberRadius);
}

QJsonObject collisionSettingsToJson(const CollisionSettings& settings)
{
    QJsonObject object;
    object.insert(QLatin1String(kRadius), settings.memberRadius);
    if (!settings.memberRadii.isEmpty()) {
        QJsonObject radii;
        for (auto it = settings.memberRadii.constBegin(); it != settings.memberRadii.constEnd(); ++it)
            radii.insert(it.key(), it.value());
        object.insert(QLatin1String(kRadii), radii);
    }
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
    const QJsonObject radii = object.value(QLatin1String(kRadii)).toObject();
    for (auto it = radii.constBegin(); it != radii.constEnd(); ++it)
        if (it.value().isDouble()) settings.memberRadii.insert(it.key(), it.value().toDouble());
    settings.jointClearance = object.value(QLatin1String(kJoint)).toDouble(defaults.jointClearance);
    settings.first = object.value(QLatin1String(kFirst)).toString();
    settings.second = object.value(QLatin1String(kSecond)).toString();
    return settings;
}

bool CollisionMesh::isEmpty() const { return !mesh || !query || mesh->isEmpty() || query->isEmpty(); }

std::vector<CollisionTarget> collisionTargets(const Linkage& linkage, const HardpointTable& table,
                                              const std::vector<bool>& grounded,
                                              const WheelSpec& wheels,
                                              const CollisionMeshes& meshes)
{
    std::vector<CollisionTarget> targets;
    for (const LinkagePart& part : linkage.parts) {
        const std::vector<Line> lines = linesOf(part, grounded);
        if (lines.empty()) continue;
        targets.push_back(CollisionTarget{ CollisionKind::Part, part.id, {}, {}, {} });
        if (lines.size() < 2) continue;
        for (const Line& line : lines)
            targets.push_back(CollisionTarget{ CollisionKind::Member, part.id, nameOf(table, line.a),
                                               nameOf(table, line.b), {} });
    }
    for (const CollisionKind kind : { CollisionKind::Rim, CollisionKind::Tyre }) {
        if (meshOf(CollisionTarget{ kind, {}, {}, {}, {} }, meshes)->isEmpty()) continue;
        for (const WheelCorner corner : kWheelCorners)
            if (!wheels.point(corner).isEmpty())
                targets.push_back(CollisionTarget{ kind, {}, {}, {}, corner });
    }
    if (!meshes.chassis.isEmpty())
        targets.push_back(CollisionTarget{ CollisionKind::Chassis, {}, {}, {}, {} });
    return targets;
}

QString collisionTargetLabel(const CollisionTarget& target, const Linkage& linkage)
{
    switch (target.kind) {
    case CollisionKind::Part: {
        const LinkagePart* part = partOf(target, linkage);
        return part ? part->label : target.partId;
    }
    case CollisionKind::Member: {
        const LinkagePart* part = partOf(target, linkage);
        return tr("%1: %2 - %3").arg(part ? part->label : target.partId, target.from, target.to);
    }
    case CollisionKind::Rim:     return tr("%1 rim").arg(wheelCornerLabel(target.corner));
    case CollisionKind::Tyre:    return tr("%1 tyre").arg(wheelCornerLabel(target.corner));
    case CollisionKind::Chassis: break;
    }
    return tr("Chassis");
}

std::vector<std::pair<QString, QString>> collisionMembers(const CollisionTarget& target,
                                                          const Linkage& linkage,
                                                          const HardpointTable& table,
                                                          const std::vector<bool>& grounded)
{
    std::vector<std::pair<QString, QString>> members;
    for (const Line& line : targetLines(target, linkage, table, grounded))
        members.emplace_back(nameOf(table, line.a), nameOf(table, line.b));
    return members;
}

double segmentDistance(const Vec3& a0, const Vec3& a1, const Vec3& b0, const Vec3& b1)
{
    const ClosestPoints closest = closestPoints(a0, a1, b0, b1);
    return distance(along(a0, a1, closest.s), along(b0, b1, closest.t));
}

CollisionResult minimumDistance(const Linkage& linkage, const std::vector<bool>& grounded,
                                const CollisionMeshes& meshes, const CollisionSettings& settings,
                                const CollisionTarget& first, const CollisionTarget& second,
                                const CollisionTravel& travel, const CollisionFrameSource& source)
{
    CollisionResult result;
    const std::optional<CollisionFrame> design = source(CollisionPosition{});
    if (!design) {
        result.problem = tr("The car does not solve at ride height.");
        return result;
    }
    for (const QString& problem : { targetProblem(first, linkage, design->table, grounded, meshes),
                                    targetProblem(second, linkage, design->table, grounded, meshes),
                                    pairProblem(first, second) }) {
        if (problem.isEmpty()) continue;
        result.problem = problem;
        return result;
    }

    const PairMeasure measure(linkage, grounded, meshes, settings, first, second, design->table);
    EnvelopeSearch search(measure, source, travel);
    result.design = search.design();
    result.rideHeight = search.rideHeight();
    result.overTravel = search.overTravel(result.rideHeight);
    result.positions = search.positions();

    if (!result.overTravel.valid) {
        result.problem = tr("Nothing is left to compare: the two meet within the joint "
                            "clearance, or there is no wheel at that corner.");
        return result;
    }
    result.valid = true;
    return result;
}

} // namespace suspkin
