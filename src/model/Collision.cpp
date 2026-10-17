#include "model/Collision.h"

#include "geom/MeshQuery.h"

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
/// How finely a member is walked along against the chassis, mm, and the most
/// samples one member gets.
constexpr double kChassisStep = 4.0;
constexpr int kMaxChassisSamples = 60;

const char kRadius[] = "memberRadius";
const char kWarn[] = "warnClearance";
const char kJoint[] = "jointClearance";
const char kChassis[] = "checkChassis";

Vec3 positionOf(const HardpointTable& table, int row)
{
    const Hardpoint& point = table.points[static_cast<std::size_t>(row)];
    return Vec3(point.coord[0], point.coord[1], point.coord[2]);
}

/// One straight piece of a part, by table row.
struct Member {
    int part = 0;
    int a = 0;
    int b = 0;
};

/// Two parts that are compared, and what is left out round the joints that
/// connect them through a third.
struct PartPair {
    int first = 0;
    int second = 0;
    std::vector<int> excluded; ///< rows round which neither is compared
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
        double cutFrom = 0.0;
        double cutTo = 0.0;
        if (lengthSquared < 1e-12) {
            if (c > 0.0) continue;
            return {};
        }
        const double discriminant = b * b - lengthSquared * c;
        if (discriminant <= 0.0) continue;
        const double root = std::sqrt(discriminant);
        cutFrom = (-b - root) / lengthSquared;
        cutTo = (-b + root) / lengthSquared;

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

/// Everything worked out once from the linkage and the design position.
struct Layout {
    std::vector<Member> members;
    std::vector<std::vector<int>> membersOfPart;
    std::vector<std::set<int>> placesOfPart; ///< rows, by the place they are
    std::vector<std::vector<int>> rowsOfPart;
    std::vector<PartPair> pairs;
    std::vector<std::vector<int>> groundedOfPart;
};

bool isMember(const LinkagePart& part) { return part.kind != PartKind::Wheel; }

Layout layOut(const Linkage& linkage, const HardpointTable& design,
              const std::vector<bool>& grounded)
{
    Layout layout;
    const int parts = static_cast<int>(linkage.parts.size());
    layout.membersOfPart.resize(static_cast<std::size_t>(parts));
    layout.placesOfPart.resize(static_cast<std::size_t>(parts));
    layout.rowsOfPart.resize(static_cast<std::size_t>(parts));
    layout.groundedOfPart.resize(static_cast<std::size_t>(parts));
    const auto isGrounded = [&](int row) {
        return row >= 0 && row < static_cast<int>(grounded.size())
               && grounded[static_cast<std::size_t>(row)];
    };

    // The members: every line drawn, except one between two chassis points
    // and the wheel's construction lines, which are no part of anything.
    std::set<int> used;
    for (int p = 0; p < parts; ++p) {
        const LinkagePart& part = linkage.parts[static_cast<std::size_t>(p)];
        for (const ResolvedChain& chain : part.chains) {
            const int count = static_cast<int>(chain.points.size());
            for (int i = 0; i < count; ++i) used.insert(chain.points[static_cast<std::size_t>(i)]);
            if (!isMember(part)) continue;
            for (int i = 0; i < chain.segmentCount(); ++i) {
                const int a = chain.points[static_cast<std::size_t>(i)];
                const int b = chain.points[static_cast<std::size_t>((i + 1) % count)];
                if (isGrounded(a) && isGrounded(b)) continue;
                layout.membersOfPart[static_cast<std::size_t>(p)].push_back(
                    static_cast<int>(layout.members.size()));
                layout.members.push_back(Member{ p, a, b });
            }
        }
    }

    Places places(static_cast<int>(design.size()));
    const std::vector<int> rows(used.begin(), used.end());
    for (std::size_t i = 0; i < rows.size(); ++i)
        for (std::size_t j = i + 1; j < rows.size(); ++j)
            if (distance(positionOf(design, rows[i]), positionOf(design, rows[j])) < kCoincident)
                places.join(rows[i], rows[j]);

    for (int p = 0; p < parts; ++p) {
        std::set<int> own;
        for (const ResolvedChain& chain : linkage.parts[static_cast<std::size_t>(p)].chains)
            for (const int row : chain.points) own.insert(row);
        for (const int row : own) {
            layout.placesOfPart[static_cast<std::size_t>(p)].insert(places.of(row));
            layout.rowsOfPart[static_cast<std::size_t>(p)].push_back(row);
            if (isGrounded(row)) layout.groundedOfPart[static_cast<std::size_t>(p)].push_back(row);
        }
    }

    const auto joined = [&](int p, int q) {
        const std::set<int>& a = layout.placesOfPart[static_cast<std::size_t>(p)];
        const std::set<int>& b = layout.placesOfPart[static_cast<std::size_t>(q)];
        return std::any_of(a.begin(), a.end(), [&](int place) { return b.count(place) > 0; });
    };

    // Two parts joined at a point are not compared: they meet there by
    // design. Two that each meet a third are, except round that third.
    for (int p = 0; p < parts; ++p) {
        if (layout.membersOfPart[static_cast<std::size_t>(p)].empty()) continue;
        for (int q = p + 1; q < parts; ++q) {
            if (layout.membersOfPart[static_cast<std::size_t>(q)].empty() || joined(p, q)) continue;
            PartPair pair{ p, q, {} };
            for (int r = 0; r < parts; ++r) {
                if (r == p || r == q || !joined(r, p) || !joined(r, q)) continue;
                const std::vector<int>& through = layout.rowsOfPart[static_cast<std::size_t>(r)];
                pair.excluded.insert(pair.excluded.end(), through.begin(), through.end());
            }
            layout.pairs.push_back(std::move(pair));
        }
    }
    return layout;
}

/// The least distance between the members of @p pair in @p table, axis to
/// axis, round its joints left out. Infinity when nothing is left to compare.
double pairDistance(const Layout& layout, const PartPair& pair, const HardpointTable& table,
                    double jointClearance)
{
    std::vector<Vec3> centres;
    centres.reserve(pair.excluded.size());
    for (const int row : pair.excluded) centres.push_back(positionOf(table, row));

    const auto pieces = [&](const Member& member) {
        const Vec3 a = positionOf(table, member.a);
        const Vec3 b = positionOf(table, member.b);
        std::vector<std::pair<Vec3, Vec3>> out;
        for (const Interval& piece : outsideSpheres(a, b, centres, jointClearance))
            out.emplace_back(along(a, b, piece.from), along(a, b, piece.to));
        return out;
    };

    double best = kInfinity;
    for (const int m : layout.membersOfPart[static_cast<std::size_t>(pair.first)]) {
        const auto first = pieces(layout.members[static_cast<std::size_t>(m)]);
        if (first.empty()) continue;
        for (const int n : layout.membersOfPart[static_cast<std::size_t>(pair.second)])
            for (const auto& b : pieces(layout.members[static_cast<std::size_t>(n)]))
                for (const auto& a : first)
                    best = std::min(best, segmentDistance(a.first, a.second, b.first, b.second));
    }
    return best;
}

/// The least distance from @p part's members to @p chassis in @p table, away
/// from the points it is bolted to the chassis by.
double chassisDistance(const Layout& layout, int part, const HardpointTable& table,
                       const MeshQuery& chassis, double jointClearance)
{
    std::vector<Vec3> mounts;
    for (const int row : layout.groundedOfPart[static_cast<std::size_t>(part)])
        mounts.push_back(positionOf(table, row));

    double best = kInfinity;
    for (const int m : layout.membersOfPart[static_cast<std::size_t>(part)]) {
        const Member& member = layout.members[static_cast<std::size_t>(m)];
        const Vec3 a = positionOf(table, member.a);
        const Vec3 b = positionOf(table, member.b);
        for (const Interval& piece : outsideSpheres(a, b, mounts, jointClearance)) {
            const double length = distance(a, b) * (piece.to - piece.from);
            const int samples =
                std::clamp(static_cast<int>(std::ceil(length / kChassisStep)), 1, kMaxChassisSamples);
            for (int i = 0; i <= samples; ++i) {
                const double t = piece.from + (piece.to - piece.from) * i / samples;
                best = std::min(best, chassis.distanceTo(along(a, b, t)));
            }
        }
    }
    return best;
}

/// The running record of one pair over the frames.
struct Tally {
    double min = kInfinity;
    double at = 0.0;
    bool designKnown = false;
    double design = 0.0;

    void add(double clearance, double input, bool isDesign)
    {
        if (clearance < min) {
            min = clearance;
            at = input;
        }
        if (isDesign) {
            designKnown = true;
            design = clearance;
        }
    }
};

/// The frame the coincident points are read off: the design position when
/// the travel passes it, else the first.
const CollisionFrame& referenceFrame(const std::vector<CollisionFrame>& frames)
{
    const auto design = std::min_element(
        frames.begin(), frames.end(), [](const CollisionFrame& a, const CollisionFrame& b) {
            return std::abs(a.input) < std::abs(b.input);
        });
    return *design;
}

} // namespace

QJsonObject collisionSettingsToJson(const CollisionSettings& settings)
{
    QJsonObject object;
    object.insert(QLatin1String(kRadius), settings.memberRadius);
    object.insert(QLatin1String(kWarn), settings.warnClearance);
    object.insert(QLatin1String(kJoint), settings.jointClearance);
    object.insert(QLatin1String(kChassis), settings.checkChassis);
    return object;
}

CollisionSettings collisionSettingsFromJson(const QJsonObject& object)
{
    const CollisionSettings defaults;
    CollisionSettings settings;
    settings.memberRadius = object.value(QLatin1String(kRadius)).toDouble(defaults.memberRadius);
    settings.warnClearance = object.value(QLatin1String(kWarn)).toDouble(defaults.warnClearance);
    settings.jointClearance = object.value(QLatin1String(kJoint)).toDouble(defaults.jointClearance);
    settings.checkChassis = object.value(QLatin1String(kChassis)).toBool(defaults.checkChassis);
    return settings;
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

CollisionReport checkCollisions(const Linkage& linkage, const std::vector<CollisionFrame>& frames,
                                const std::vector<bool>& grounded, const MeshQuery* chassis,
                                const CollisionSettings& settings)
{
    CollisionReport report;
    if (frames.empty() || linkage.isEmpty()) return report;
    report.positions = static_cast<int>(frames.size());

    const Layout layout = layOut(linkage, referenceFrame(frames).table, grounded);
    const bool withChassis = settings.checkChassis && chassis && !chassis->isEmpty();
    const double bothRadii = 2.0 * settings.memberRadius;

    std::vector<Tally> pairs(layout.pairs.size());
    std::vector<Tally> walls(linkage.parts.size());
    for (const CollisionFrame& frame : frames) {
        const bool isDesign = std::abs(frame.input) < 1e-9;
        for (std::size_t i = 0; i < layout.pairs.size(); ++i) {
            const double gap =
                pairDistance(layout, layout.pairs[i], frame.table, settings.jointClearance);
            if (gap < kInfinity) pairs[i].add(gap - bothRadii, frame.input, isDesign);
        }
        if (!withChassis) continue;
        for (std::size_t p = 0; p < linkage.parts.size(); ++p) {
            if (layout.membersOfPart[p].empty()) continue;
            const double gap = chassisDistance(layout, static_cast<int>(p), frame.table, *chassis,
                                               settings.jointClearance);
            if (gap < kInfinity) walls[p].add(gap - settings.memberRadius, frame.input, isDesign);
        }
    }

    const auto record = [&](const Tally& tally, const QString& first, const QString& second,
                            bool againstChassis) {
        if (tally.min == kInfinity) return;
        report.pairs.push_back(CollisionPair{ first, second, againstChassis, tally.min, tally.at,
                                              tally.designKnown, tally.design });
    };
    for (std::size_t i = 0; i < layout.pairs.size(); ++i)
        record(pairs[i], linkage.parts[static_cast<std::size_t>(layout.pairs[i].first)].label,
               linkage.parts[static_cast<std::size_t>(layout.pairs[i].second)].label, false);
    for (std::size_t p = 0; p < walls.size(); ++p)
        record(walls[p], linkage.parts[p].label, tr("Chassis"), true);

    std::sort(report.pairs.begin(), report.pairs.end(),
              [](const CollisionPair& a, const CollisionPair& b) {
                  return a.minClearance < b.minClearance;
              });
    return report;
}

} // namespace suspkin
