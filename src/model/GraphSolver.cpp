#include "model/GraphSolver.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("GraphSolver", text); }

/// What a turn of one radian is worth in millimetres, for the solve's own
/// bookkeeping: turns and shifts then come out of the normal equations on one
/// scale, about the size of a suspension member.
constexpr double kTurnScale = 100.0;

/// A joint held to this is held: far below anything measured, far above where
/// doubles at a few metres from the origin stop resolving.
constexpr double kHeld = 1e-11;

constexpr int kMaxIterations = 60;
constexpr int kMaxHalvings = 20;

/// A motion is free when the normal equations give it less than this fraction
/// of their stiffest.
constexpr double kFreeTolerance = 1e-10;

/// A 3x3 matrix by rows, for the handful of products the Jacobian needs.
struct M3 {
    Vec3 r[3];

    Vec3 operator*(const Vec3& v) const { return Vec3(dot(r[0], v), dot(r[1], v), dot(r[2], v)); }
    M3 operator*(const M3& o) const
    {
        M3 out;
        for (int i = 0; i < 3; ++i)
            out.r[i] = Vec3(r[i].x * o.r[0].x + r[i].y * o.r[1].x + r[i].z * o.r[2].x,
                            r[i].x * o.r[0].y + r[i].y * o.r[1].y + r[i].z * o.r[2].y,
                            r[i].x * o.r[0].z + r[i].y * o.r[1].z + r[i].z * o.r[2].z);
        return out;
    }
    M3 operator*(double s) const { return M3{ { r[0] * s, r[1] * s, r[2] * s } }; }
};

M3 identity3() { return M3{ { Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1) } }; }

/// The matrix of v x (.).
M3 skew(const Vec3& v)
{
    return M3{ { Vec3(0, -v.z, v.y), Vec3(v.z, 0, -v.x), Vec3(-v.y, v.x, 0) } };
}

/// The rotation by the rotation vector @p w, Rodrigues' way.
M3 exponential(const Vec3& w)
{
    const double angle = w.length();
    if (angle < 1e-300) return identity3();
    const Vec3 k = w / angle;
    const M3 K = skew(k);
    const M3 KK = K * K;
    const double s = std::sin(angle);
    const double c = 1.0 - std::cos(angle);
    M3 out = identity3();
    for (int i = 0; i < 3; ++i) out.r[i] = out.r[i] + K.r[i] * s + KK.r[i] * c;
    return out;
}

Rigid turnedBy(const M3& e, const Rigid& motion, const Vec3& about, const Vec3& shift)
{
    // x' = E (x - c) + c + shift, with x = R p + t.
    Rigid out;
    const M3 rotation{ { motion.r[0], motion.r[1], motion.r[2] } };
    const M3 turned = e * rotation;
    for (int i = 0; i < 3; ++i) out.r[i] = turned.r[i];
    out.t = e * (motion.t - about) + about + shift;
    return out;
}

/// The rows of @p motion made orthonormal again, which a long sweep of small
/// turns slowly stops them being.
void reorthonormalise(Rigid& motion)
{
    Vec3 x = motion.r[0].normalized();
    Vec3 y = (motion.r[1] - x * dot(x, motion.r[1])).normalized();
    motion.r[0] = x;
    motion.r[1] = y;
    motion.r[2] = cross(x, y);
}

/// Solve the symmetric positive definite @p a (n x n, by rows) against @p b in
/// place. False when @p a is not positive definite.
bool choleskySolve(std::vector<double> a, std::vector<double>& b, int n)
{
    for (int j = 0; j < n; ++j) {
        double diagonal = a[std::size_t(j * n + j)];
        for (int k = 0; k < j; ++k) diagonal -= a[std::size_t(j * n + k)] * a[std::size_t(j * n + k)];
        if (diagonal <= 0.0) return false;
        const double root = std::sqrt(diagonal);
        a[std::size_t(j * n + j)] = root;
        for (int i = j + 1; i < n; ++i) {
            double sum = a[std::size_t(i * n + j)];
            for (int k = 0; k < j; ++k) sum -= a[std::size_t(i * n + k)] * a[std::size_t(j * n + k)];
            a[std::size_t(i * n + j)] = sum / root;
        }
    }
    for (int i = 0; i < n; ++i) {
        double sum = b[std::size_t(i)];
        for (int k = 0; k < i; ++k) sum -= a[std::size_t(i * n + k)] * b[std::size_t(k)];
        b[std::size_t(i)] = sum / a[std::size_t(i * n + i)];
    }
    for (int i = n - 1; i >= 0; --i) {
        double sum = b[std::size_t(i)];
        for (int k = i + 1; k < n; ++k) sum -= a[std::size_t(k * n + i)] * b[std::size_t(k)];
        b[std::size_t(i)] = sum / a[std::size_t(i * n + i)];
    }
    return true;
}

/// The eigenvalues of the symmetric @p a (n x n), with their eigenvectors as
/// the columns of @p vectors, by Jacobi rotations. Small n and run once per
/// question, so the simplest method that is always right is the one used.
void symmetricEigen(std::vector<double> a, int n, std::vector<double>* values,
                    std::vector<double>* vectors)
{
    std::vector<double>& v = *vectors;
    v.assign(std::size_t(n * n), 0.0);
    for (int i = 0; i < n; ++i) v[std::size_t(i * n + i)] = 1.0;
    const auto at = [&a, n](int i, int j) -> double& { return a[std::size_t(i * n + j)]; };

    for (int sweep = 0; sweep < 100; ++sweep) {
        double off = 0.0;
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j) off += at(i, j) * at(i, j);
        if (off < 1e-30) break;
        for (int p = 0; p < n; ++p) {
            for (int q = p + 1; q < n; ++q) {
                if (std::abs(at(p, q)) < 1e-300) continue;
                const double theta = (at(q, q) - at(p, p)) / (2.0 * at(p, q));
                const double t = (theta >= 0 ? 1.0 : -1.0)
                                 / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0);
                const double s = t * c;
                for (int k = 0; k < n; ++k) {
                    const double akp = at(k, p);
                    const double akq = at(k, q);
                    at(k, p) = c * akp - s * akq;
                    at(k, q) = s * akp + c * akq;
                }
                for (int k = 0; k < n; ++k) {
                    const double apk = at(p, k);
                    const double aqk = at(q, k);
                    at(p, k) = c * apk - s * aqk;
                    at(q, k) = s * apk + c * aqk;
                }
                for (int k = 0; k < n; ++k) {
                    const double vkp = v[std::size_t(k * n + p)];
                    const double vkq = v[std::size_t(k * n + q)];
                    v[std::size_t(k * n + p)] = c * vkp - s * vkq;
                    v[std::size_t(k * n + q)] = s * vkp + c * vkq;
                }
            }
        }
    }
    values->resize(std::size_t(n));
    for (int i = 0; i < n; ++i) (*values)[std::size_t(i)] = at(i, i);
}

double sumOfSquares(const std::vector<double>& r)
{
    return std::accumulate(r.begin(), r.end(), 0.0, [](double s, double x) { return s + x * x; });
}

double largest(const std::vector<double>& r)
{
    double out = 0.0;
    for (double x : r) out = std::max(out, std::abs(x));
    return out;
}

/// A disjoint-set forest over body indices.
struct Groups {
    std::vector<int> parent;

    explicit Groups(int count) : parent(std::size_t(count)) { std::iota(parent.begin(), parent.end(), 0); }
    int find(int i)
    {
        while (parent[std::size_t(i)] != i) i = parent[std::size_t(i)] = parent[std::size_t(parent[std::size_t(i)])];
        return i;
    }
    void unite(int a, int b) { parent[std::size_t(find(a))] = find(b); }
};

} // namespace

std::optional<GraphSolver> GraphSolver::compile(const MechanismGraph& graph, int drivenBody,
                                                QString* error)
{
    GraphSolver solver;
    solver.m_graph = graph;
    if (solver.m_graph.bodies.empty()) solver.m_graph.bodyFor(QString());

    solver.groupBodies();
    const bool drivable = drivenBody > MechanismGraph::kGround
                          && drivenBody < int(solver.m_graph.bodies.size())
                          && solver.m_groupOfBody[std::size_t(drivenBody)] >= 0
                          && solver.m_groupOfBody[std::size_t(drivenBody)] != solver.m_groundGroup;
    if (!drivable) {
        if (error) *error = tr("The lower wishbone is not a body that can move.");
        return std::nullopt;
    }
    solver.m_drivenGroup = solver.m_groupOfBody[std::size_t(drivenBody)];

    solver.reduceLinks();
    solver.findCarriers();
    solver.buildConstraints();
    solver.findCentroids();
    solver.findLineGroups();
    solver.m_unknowns = solver.unknowns(false);
    return solver;
}

void GraphSolver::groupBodies()
{
    const int bodyCount = int(m_graph.bodies.size());
    const auto rigid = [this, bodyCount](int body) {
        return body >= 0 && body < bodyCount && !m_graph.bodies[std::size_t(body)].compressible;
    };

    // Two bodies welded together, or a point both of them carry, are one body
    // as far as the motion goes: the wheel bolted to its upright.
    Groups forest(bodyCount);
    for (const GraphPoint& point : m_graph.points) {
        const bool joins = point.type == PointType::Welded
                           || (point.type == PointType::Dependent && point.body2 >= 0);
        if (joins && rigid(point.body1) && rigid(point.body2)) forest.unite(point.body1, point.body2);
    }

    m_groupOfBody.assign(std::size_t(bodyCount), -1);
    std::vector<int> groupOfRoot(std::size_t(bodyCount), -1);
    m_groupCount = 0;
    for (int body = 0; body < bodyCount; ++body) {
        if (!rigid(body)) continue;
        int& group = groupOfRoot[std::size_t(forest.find(body))];
        if (group < 0) group = m_groupCount++;
        m_groupOfBody[std::size_t(body)] = group;
    }
    m_groundGroup = m_groupOfBody[std::size_t(MechanismGraph::kGround)];
    m_reduced.assign(std::size_t(m_groupCount), false);
}

void GraphSolver::reduceLinks()
{
    // A group touched by exactly two points, each a plain joint to something
    // else rigid, is a two-force member: all it does is hold those two points a
    // length apart, and its own spin about that line is nobody's business.
    std::vector<std::vector<int>> touching(static_cast<std::size_t>(m_groupCount));
    for (std::size_t p = 0; p < m_graph.points.size(); ++p) {
        const GraphPoint& point = m_graph.points[p];
        for (const int body : { point.body1, point.body2 }) {
            if (body < 0) continue;
            const int group = m_groupOfBody[std::size_t(body)];
            if (group < 0) continue;
            std::vector<int>& list = touching[std::size_t(group)];
            if (std::find(list.begin(), list.end(), int(p)) == list.end()) list.push_back(int(p));
        }
    }

    for (int group = 0; group < m_groupCount; ++group) {
        if (group == m_groundGroup || group == m_drivenGroup) continue;
        const std::vector<int>& points = touching[std::size_t(group)];
        if (points.size() != 2) continue;
        bool link = true;
        for (const int p : points) {
            const GraphPoint& point = m_graph.points[std::size_t(p)];
            const bool joint = point.type == PointType::Solved || point.type == PointType::ToBody;
            const int g1 = point.body1 >= 0 ? m_groupOfBody[std::size_t(point.body1)] : -1;
            const int g2 = point.body2 >= 0 ? m_groupOfBody[std::size_t(point.body2)] : -1;
            const int other = g1 == group ? g2 : g1;
            link = link && joint && other >= 0 && other != group;
        }
        if (link) m_reduced[std::size_t(group)] = true;
    }
}

void GraphSolver::findCarriers()
{
    const auto usable = [this](int body) {
        if (body < 0) return -1;
        const int group = m_groupOfBody[std::size_t(body)];
        return group >= 0 && !m_reduced[std::size_t(group)] ? group : -1;
    };

    m_carrier.assign(m_graph.points.size(), -1);
    for (std::size_t p = 0; p < m_graph.points.size(); ++p) {
        const GraphPoint& point = m_graph.points[p];
        if (point.type == PointType::ToBody
            && (point.body1 == MechanismGraph::kGround || point.body2 == MechanismGraph::kGround)) {
            m_carrier[p] = m_groundGroup;
            continue;
        }
        const int first = usable(point.body1);
        m_carrier[p] = first >= 0 ? first : usable(point.body2);
    }
}

void GraphSolver::buildConstraints()
{
    const auto usable = [this](int body) {
        if (body < 0) return -1;
        const int group = m_groupOfBody[std::size_t(body)];
        return group >= 0 && !m_reduced[std::size_t(group)] ? group : -1;
    };
    const auto compressible = [this](int body) {
        return body >= 0 && m_graph.bodies[std::size_t(body)].compressible;
    };

    m_constraints.clear();
    for (std::size_t p = 0; p < m_graph.points.size(); ++p) {
        const GraphPoint& point = m_graph.points[p];
        if (!isJoint(point.type)) continue;
        const int a = usable(point.body1);
        const int b = usable(point.body2);
        if (a >= 0 && b >= 0 && a != b) {
            Constraint joint;
            joint.kind = Kind::Coincide;
            joint.a = int(p);
            joint.groupA = a;
            joint.groupB = b;
            m_constraints.push_back(joint);
        }

        // Bolted rigidly to a strut, a body keeps the strut on its own line:
        // the far end of the strut stays on the line it started on, turned with
        // the body.
        if (point.type != PointType::Welded) continue;
        const int strut = compressible(point.body1) ? point.body1
                          : compressible(point.body2) ? point.body2 : -1;
        const int holder = strut == point.body1 ? b : a;
        if (strut < 0 || holder < 0) continue;
        int farEnd = -1;
        double reach = 0.0;
        for (std::size_t q = 0; q < m_graph.points.size(); ++q) {
            const GraphPoint& other = m_graph.points[q];
            if (q == p || (other.body1 != strut && other.body2 != strut)) continue;
            const double d = distance(other.design, point.design);
            if (d > reach) {
                reach = d;
                farEnd = int(q);
            }
        }
        if (farEnd < 0 || m_carrier[std::size_t(farEnd)] < 0
            || m_carrier[std::size_t(farEnd)] == holder || reach < 1e-9)
            continue;
        Constraint line;
        line.kind = Kind::Collinear;
        line.a = int(p);
        line.b = farEnd;
        line.groupA = holder;
        line.direction = (m_graph.points[std::size_t(farEnd)].design - point.design) / reach;
        m_constraints.push_back(line);
    }

    for (int group = 0; group < m_groupCount; ++group) {
        if (!m_reduced[std::size_t(group)]) continue;
        std::vector<int> ends;
        for (std::size_t p = 0; p < m_graph.points.size(); ++p) {
            const GraphPoint& point = m_graph.points[p];
            const bool on = (point.body1 >= 0 && m_groupOfBody[std::size_t(point.body1)] == group)
                            || (point.body2 >= 0 && m_groupOfBody[std::size_t(point.body2)] == group);
            if (on) ends.push_back(int(p));
        }
        Constraint length;
        length.kind = Kind::Distance;
        length.a = ends[0];
        length.b = ends[1];
        length.length = distance(m_graph.points[std::size_t(ends[0])].design,
                                 m_graph.points[std::size_t(ends[1])].design);
        m_constraints.push_back(length);
    }
}

void GraphSolver::findCentroids()
{
    std::vector<Vec3> sum(static_cast<std::size_t>(m_groupCount));
    std::vector<int> count(std::size_t(m_groupCount), 0);
    for (std::size_t p = 0; p < m_graph.points.size(); ++p) {
        const int group = m_carrier[p];
        if (group < 0) continue;
        sum[std::size_t(group)] += m_graph.points[p].design;
        ++count[std::size_t(group)];
    }
    m_centroid.assign(std::size_t(m_groupCount), Vec3());
    for (int group = 0; group < m_groupCount; ++group)
        if (count[std::size_t(group)] > 0)
            m_centroid[std::size_t(group)] = sum[std::size_t(group)] / count[std::size_t(group)];
}

void GraphSolver::findLineGroups()
{
    std::vector<std::vector<Vec3>> touched(static_cast<std::size_t>(m_groupCount));
    for (const GraphPoint& point : m_graph.points) {
        for (const int body : { point.body1, point.body2 }) {
            const int group = body >= 0 ? m_groupOfBody[std::size_t(body)] : -1;
            if (group >= 0) touched[std::size_t(group)].push_back(point.design);
        }
    }

    m_lineAxis.assign(std::size_t(m_groupCount), Vec3());
    for (int group = 0; group < m_groupCount; ++group) {
        const std::vector<Vec3>& points = touched[std::size_t(group)];
        if (points.size() < 2) continue;
        Vec3 axis;
        for (const Vec3& p : points)
            if (distance(p, points.front()) > axis.length()) axis = p - points.front();
        if (axis.lengthSquared() < 1e-12) continue;
        const Vec3 unit = axis.normalized();
        bool straight = true;
        for (const Vec3& p : points)
            straight = straight && cross(p - points.front(), unit).length() < 1e-6;
        if (straight) m_lineAxis[std::size_t(group)] = unit;
    }
}

GraphSolver::Unknowns GraphSolver::unknowns(bool includeDriven) const
{
    std::vector<bool> used(std::size_t(m_groupCount), false);
    for (const int group : m_carrier)
        if (group >= 0) used[std::size_t(group)] = true;
    for (const Constraint& c : m_constraints) {
        if (c.groupA >= 0) used[std::size_t(c.groupA)] = true;
        if (c.groupB >= 0) used[std::size_t(c.groupB)] = true;
    }

    Unknowns out;
    out.first.assign(std::size_t(m_groupCount), -1);
    for (int group = 0; group < m_groupCount; ++group) {
        if (group == m_groundGroup || m_reduced[std::size_t(group)] || !used[std::size_t(group)])
            continue;
        if (group == m_drivenGroup && !includeDriven) continue;
        out.first[std::size_t(group)] = out.count;
        out.count += 6;
    }
    return out;
}

int GraphSolver::rows() const
{
    int count = 0;
    for (const Constraint& c : m_constraints) count += c.kind == Kind::Distance ? 1 : 3;
    return count;
}

Vec3 GraphSolver::carriedBy(int point, int group, const GraphState& state, int rackPoint,
                            const Vec3& rackOffset) const
{
    const Vec3& design = m_graph.points[std::size_t(point)].design;
    if (group == m_groundGroup) return point == rackPoint ? design + rackOffset : design;
    return state.motions[std::size_t(group)].map(design);
}

Vec3 GraphSolver::positionOf(int point, const GraphState& state, int rackPoint,
                             const Vec3& rackOffset) const
{
    const int group = m_carrier[std::size_t(point)];
    if (group < 0) return m_graph.points[std::size_t(point)].design;
    return carriedBy(point, group, state, rackPoint, rackOffset);
}

bool GraphSolver::isPosed(int point) const { return m_carrier[std::size_t(point)] >= 0; }

bool GraphSolver::moves(int point, int rackPoint) const
{
    const int group = m_carrier[std::size_t(point)];
    return group >= 0 && (group != m_groundGroup || point == rackPoint);
}

Rigid GraphSolver::motionOf(int body, const GraphState& state) const
{
    if (body < 0 || body >= int(m_groupOfBody.size())) return Rigid();
    const int group = m_groupOfBody[std::size_t(body)];
    if (group < 0 || m_reduced[std::size_t(group)] || group >= int(state.motions.size()))
        return Rigid();
    return state.motions[std::size_t(group)];
}

void GraphSolver::residual(const GraphState& state, int rackPoint, const Vec3& rackOffset,
                           std::vector<double>* r) const
{
    r->clear();
    const auto push = [r](const Vec3& v) {
        r->push_back(v.x);
        r->push_back(v.y);
        r->push_back(v.z);
    };
    for (const Constraint& c : m_constraints) {
        switch (c.kind) {
        case Kind::Coincide:
            push(carriedBy(c.a, c.groupA, state, rackPoint, rackOffset)
                 - carriedBy(c.a, c.groupB, state, rackPoint, rackOffset));
            break;
        case Kind::Distance:
            r->push_back(distance(positionOf(c.a, state, rackPoint, rackOffset),
                                  positionOf(c.b, state, rackPoint, rackOffset))
                         - c.length);
            break;
        case Kind::Collinear: {
            const Vec3 along = positionOf(c.b, state, rackPoint, rackOffset)
                               - carriedBy(c.a, c.groupA, state, rackPoint, rackOffset);
            push(cross(along, state.motions[std::size_t(c.groupA)].rotate(c.direction)));
            break;
        }
        }
    }
}

void GraphSolver::jacobian(const GraphState& state, int rackPoint, const Vec3& rackOffset,
                           const Unknowns& unknowns, std::vector<double>* j) const
{
    const int n = unknowns.count;
    j->assign(std::size_t(rows() * n), 0.0);

    // d(point carried by group) / d(unknowns of group): a turn about the group's
    // posed centroid, scaled, then a shift.
    const auto add = [&](int row, int group, const M3& weight, int point) {
        if (group < 0 || unknowns.first[std::size_t(group)] < 0) return;
        const Vec3 x = carriedBy(point, group, state, rackPoint, rackOffset);
        const Vec3 about = state.motions[std::size_t(group)].map(m_centroid[std::size_t(group)]);
        const M3 turn = weight * (skew(x - about) * (-1.0 / kTurnScale));
        const int first = unknowns.first[std::size_t(group)];
        for (int i = 0; i < 3; ++i) {
            double* line = j->data() + std::size_t((row + i) * n + first);
            line[0] += turn.r[i].x;
            line[1] += turn.r[i].y;
            line[2] += turn.r[i].z;
            line[3] += weight.r[i].x;
            line[4] += weight.r[i].y;
            line[5] += weight.r[i].z;
        }
    };
    // The same for a row that is one number: @p weight is its gradient.
    const auto addScalar = [&](int row, int group, const Vec3& weight, int point) {
        if (group < 0 || unknowns.first[std::size_t(group)] < 0) return;
        const Vec3 x = carriedBy(point, group, state, rackPoint, rackOffset);
        const Vec3 about = state.motions[std::size_t(group)].map(m_centroid[std::size_t(group)]);
        // w . (omega x (x - c)) = omega . ((x - c) x w)
        const Vec3 turn = cross(x - about, weight) / kTurnScale;
        double* line = j->data() + std::size_t(row * n + unknowns.first[std::size_t(group)]);
        line[0] += turn.x;
        line[1] += turn.y;
        line[2] += turn.z;
        line[3] += weight.x;
        line[4] += weight.y;
        line[5] += weight.z;
    };

    int row = 0;
    for (const Constraint& c : m_constraints) {
        switch (c.kind) {
        case Kind::Coincide:
            add(row, c.groupA, identity3(), c.a);
            add(row, c.groupB, identity3() * -1.0, c.a);
            row += 3;
            break;
        case Kind::Distance: {
            const Vec3 a = positionOf(c.a, state, rackPoint, rackOffset);
            const Vec3 b = positionOf(c.b, state, rackPoint, rackOffset);
            const double length = distance(a, b);
            const Vec3 u = length > 1e-300 ? (a - b) / length : Vec3();
            addScalar(row, m_carrier[std::size_t(c.a)], u, c.a);
            addScalar(row, m_carrier[std::size_t(c.b)], u * -1.0, c.b);
            row += 1;
            break;
        }
        case Kind::Collinear: {
            // r = e x n with e = b - a and n the direction turned with group A:
            // dr = -[n]x de + [e]x dn, and dn = omega x n.
            const Vec3 n3 = state.motions[std::size_t(c.groupA)].rotate(c.direction);
            const Vec3 e = positionOf(c.b, state, rackPoint, rackOffset)
                           - carriedBy(c.a, c.groupA, state, rackPoint, rackOffset);
            const M3 alongWeight = skew(n3) * -1.0;
            add(row, m_carrier[std::size_t(c.b)], alongWeight, c.b);
            add(row, c.groupA, alongWeight * -1.0, c.a);
            if (unknowns.first[std::size_t(c.groupA)] >= 0) {
                const M3 turn = skew(e) * skew(n3) * (-1.0 / kTurnScale);
                const int first = unknowns.first[std::size_t(c.groupA)];
                for (int i = 0; i < 3; ++i) {
                    double* line = j->data() + std::size_t((row + i) * n + first);
                    line[0] += turn.r[i].x;
                    line[1] += turn.r[i].y;
                    line[2] += turn.r[i].z;
                }
            }
            row += 3;
            break;
        }
        }
    }
}

GraphState GraphSolver::stepped(const GraphState& state, const Unknowns& unknowns,
                                const std::vector<double>& step) const
{
    GraphState out = state;
    for (int group = 0; group < m_groupCount; ++group) {
        const int first = unknowns.first[std::size_t(group)];
        if (first < 0) continue;
        const double* s = step.data() + first;
        const Vec3 turn = Vec3(s[0], s[1], s[2]) / kTurnScale;
        const Vec3 shift(s[3], s[4], s[5]);
        const Rigid& motion = state.motions[std::size_t(group)];
        out.motions[std::size_t(group)] = turnedBy(
            exponential(turn), motion, motion.map(m_centroid[std::size_t(group)]), shift);
    }
    return out;
}

bool GraphSolver::solve(const Rigid& driven, int rackPoint, const Vec3& rackOffset,
                        const GraphState* previous, GraphState* out, QString* error) const
{
    GraphState state;
    if (previous && int(previous->motions.size()) == m_groupCount)
        state = *previous;
    else
        state.motions.assign(std::size_t(m_groupCount), Rigid());
    state.motions[std::size_t(m_groundGroup)] = Rigid();
    state.motions[std::size_t(m_drivenGroup)] = driven;

    const int n = m_unknowns.count;
    std::vector<double> r;
    residual(state, rackPoint, rackOffset, &r);
    double cost = sumOfSquares(r);

    std::vector<double> j;
    std::vector<double> normal;
    std::vector<double> step;
    const int m = int(r.size());
    // One step more once the joints are held: Newton doubles the digits each
    // step, so the answer is then as exact as doubles allow and no longer
    // depends on the path taken to it -- on which bodies happened to be in the
    // solve, or where it started.
    int polishing = 1;
    for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
        if (largest(r) <= kHeld && polishing-- <= 0) break;
        jacobian(state, rackPoint, rackOffset, m_unknowns, &j);

        // The normal equations, damped just enough to give a free motion --
        // a body nothing holds yet -- no step at all rather than any step.
        normal.assign(std::size_t(n * n), 0.0);
        step.assign(std::size_t(n), 0.0);
        for (int row = 0; row < m; ++row) {
            const double* line = j.data() + std::size_t(row * n);
            for (int a = 0; a < n; ++a) {
                if (line[a] == 0.0) continue;
                step[std::size_t(a)] -= line[a] * r[std::size_t(row)];
                for (int b = 0; b < n; ++b) normal[std::size_t(a * n + b)] += line[a] * line[b];
            }
        }
        for (int a = 0; a < n; ++a)
            normal[std::size_t(a * n + a)] += 1e-12 * normal[std::size_t(a * n + a)] + 1e-14;
        if (!choleskySolve(normal, step, n)) break;

        // Halve until the joints are held better than they were: a full step
        // that overshoots near the end of the travel is normal.
        // The polishing step is taken whole: it is far below anything that
        // could overshoot, and what it changes is rounding.
        const bool held = largest(r) <= kHeld;
        bool improved = false;
        for (int halving = 0; halving < kMaxHalvings; ++halving) {
            GraphState trial = stepped(state, m_unknowns, step);
            std::vector<double> trialResidual;
            residual(trial, rackPoint, rackOffset, &trialResidual);
            const double trialCost = sumOfSquares(trialResidual);
            if (held || trialCost < cost) {
                state = std::move(trial);
                r = std::move(trialResidual);
                cost = trialCost;
                improved = true;
                break;
            }
            for (double& s : step) s *= 0.5;
        }
        if (!improved) break;
    }

    if (largest(r) > kHeld * 100.0) {
        if (error) *error = tr("The linkage does not assemble here.");
        return false;
    }
    for (Rigid& motion : state.motions) reorthonormalise(motion);
    state.motions[std::size_t(m_drivenGroup)] = driven;
    *out = std::move(state);
    return true;
}

int GraphSolver::freeMotions(bool includeDriven, QStringList* looseBodies) const
{
    const Unknowns map = unknowns(includeDriven);
    const int n = map.count;
    if (n == 0) return 0;

    GraphState design;
    design.motions.assign(std::size_t(m_groupCount), Rigid());
    std::vector<double> j;
    jacobian(design, -1, Vec3(), map, &j);
    const int m = rows();

    std::vector<double> normal(std::size_t(n * n), 0.0);
    for (int row = 0; row < m; ++row)
        for (int a = 0; a < n; ++a)
            for (int b = 0; b < n; ++b)
                normal[std::size_t(a * n + b)] +=
                    j[std::size_t(row * n + a)] * j[std::size_t(row * n + b)];

    std::vector<double> values;
    std::vector<double> vectors;
    // A body whose points all lie on one line may spin about it without moving
    // any of them. That is not a motion of the suspension, so it is held.
    double stiffness = 0.0;
    for (int a = 0; a < n; ++a) stiffness = std::max(stiffness, normal[std::size_t(a * n + a)]);
    for (int group = 0; group < m_groupCount; ++group) {
        const int first = map.first[std::size_t(group)];
        const Vec3& line = m_lineAxis[std::size_t(group)];
        if (first < 0 || line.lengthSquared() == 0.0) continue;
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b)
                normal[std::size_t((first + a) * n + first + b)] += stiffness * line[a] * line[b];
    }

    symmetricEigen(normal, n, &values, &vectors);
    const double stiffest = std::max(1e-300, *std::max_element(values.begin(), values.end()));

    int free = 0;
    for (int k = 0; k < n; ++k) {
        if (values[std::size_t(k)] > kFreeTolerance * stiffest) continue;
        ++free;
        if (!looseBodies) continue;
        for (int group = 0; group < m_groupCount; ++group) {
            const int first = map.first[std::size_t(group)];
            if (first < 0) continue;
            double weight = 0.0;
            for (int i = 0; i < 6; ++i)
                weight += std::pow(vectors[std::size_t((first + i) * n + k)], 2);
            if (weight < 0.01) continue;
            for (std::size_t body = 0; body < m_groupOfBody.size(); ++body) {
                if (m_groupOfBody[body] != group) continue;
                const QString& name = m_graph.bodies[body].name;
                if (!looseBodies->contains(name)) looseBodies->append(name);
            }
        }
    }
    return free;
}

GraphMobility GraphSolver::mobility() const
{
    GraphMobility out;
    out.freeWithDriveHeld = freeMotions(false, &out.looseBodies);
    out.freeWithDriveLoose = freeMotions(true, nullptr);
    for (std::size_t p = 0; p < m_graph.points.size(); ++p)
        if (m_carrier[p] < 0 && m_graph.points[p].type != PointType::Unassigned)
            out.unposedPoints << m_graph.points[p].name;
    return out;
}

} // namespace suspkin
