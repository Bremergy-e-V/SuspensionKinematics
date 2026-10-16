#include "model/SuspensionSolver.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("SuspensionSolver", text); }

constexpr double kRadToDeg = 57.295779513082320876798154814105;

/// How close to the asked-for height counts as arrived. A micron: far below
/// anything a suspension is measured to, and far above where the doubles stop.
constexpr double kTargetTolerance = 1e-7;

/// Bigger than any single Newton step should ever need on a suspension, and
/// small enough that overshooting cannot fling the solve past a link's limit
/// into a region where it does not assemble.
constexpr double kMaxStep = 0.35;

/// Where the road is. A hardpoint workbook is measured in vehicle coordinates
/// with z up from the ground, so this is not a guess so much as the frame's own
/// definition -- and a corner whose workbook says otherwise names a contact
/// patch, which is taken over it.
constexpr double kGroundZ = 0.0;

/// The direction of an anti-roll bar's axis, @p along it either way, turned so
/// that the two sides of an axle measure their arms the same way round.
///
/// A mirrored side does not want the mirror image of the near side's direction:
/// mirroring turns a rotation the other way, and a pure bump would then read as
/// the arms twisting against each other. It wants that image reversed -- which,
/// for a bar straight across the car, is the same +y both sides already use. So
/// the direction is seen from the left (a right side's is mirrored over),
/// pointed left, and taken back to this side as the reversed mirror image.
Vec3 barAxisPointingLeft(const Vec3& along, double side)
{
    const Vec3 unit = along.normalized();
    Vec3 seenFromLeft(unit.x, unit.y * side, unit.z);
    // A bar along the car has no left to point to; forward decides instead, so
    // both sides still agree.
    const bool reversed = std::abs(seenFromLeft.y) > 1e-9 ? seenFromLeft.y < 0.0
                                                          : seenFromLeft.x < 0.0;
    if (reversed) seenFromLeft = seenFromLeft * -1.0;
    return side > 0.0 ? seenFromLeft : Vec3(-seenFromLeft.x, seenFromLeft.y, -seenFromLeft.z);
}

double pickWheelHeight(const CornerPose& pose) { return pose.wheelCenter.z; }
double pickContactHeight(const CornerPose& pose) { return pose.contactPatch.z; }

} // namespace

Vec3 spinAxisFor(const StaticAlignment& alignment, double side)
{
    // Camber tips the axis out of the ground plane, toe turns what is left of
    // it in plan view. Both measures read this straight back: camber is minus
    // the arcsine of z, toe the angle of (x, side * y).
    const double camber = alignment.camber / kRadToDeg;
    const double toe = alignment.toe / kRadToDeg;
    return Vec3(std::sin(toe) * std::cos(camber), side * std::cos(toe) * std::cos(camber),
                -std::sin(camber));
}

Vec3 wheelPlaneDown(const Vec3& spinAxis)
{
    const Vec3 axis = spinAxis.normalized();
    const Vec3 down(0.0, 0.0, -1.0);
    const Vec3 inPlane = down - axis * dot(down, axis);
    // A wheel whose axis stands vertical has no down in its own plane. It is not
    // a suspension anybody is drawing, but it is a division by zero if ignored.
    if (inPlane.lengthSquared() < 1e-12) return Vec3();
    return inPlane.normalized();
}

Vec3 contactPatchFor(const Vec3& wheelCenter, const Vec3& spinAxis, double tireRadius)
{
    const Vec3 down = wheelPlaneDown(spinAxis);
    if (down.lengthSquared() < 0.5) return wheelCenter;
    return wheelCenter + down * tireRadius;
}

double tireRadiusToGround(const Vec3& wheelCenter, const Vec3& spinAxis, double groundZ, bool* ok)
{
    if (ok) *ok = false;
    const Vec3 down = wheelPlaneDown(spinAxis);
    if (down.z > -1e-9) return 0.0; // the wheel plane runs away from the ground
    const double radius = (groundZ - wheelCenter.z) / down.z;
    if (radius <= 0.0) return 0.0; // a wheel centre at or below the ground
    if (ok) *ok = true;
    return radius;
}

const Vec3* CornerPose::find(const QString& name) const
{
    for (const PosedPoint& point : points)
        if (point.name == name) return &point.position;
    return nullptr;
}

std::optional<CornerSolver> CornerSolver::bind(const MechanismTemplate& mechanism,
                                               const HardpointTable& table, QString* error,
                                               const std::optional<StaticAlignment>& alignment,
                                               const MechanismGraph* graph)
{
    const MechanismCoverage coverage = coverMechanism(mechanism, table);
    if (coverage.absent) {
        if (error) *error = tr("None of this corner's hardpoints are in the table.");
        return std::nullopt;
    }
    if (!coverage.missingRequired.isEmpty()) {
        if (error)
            *error = tr("Cannot solve this corner: the table has no %1.")
                         .arg(coverage.missingRequired.join(QStringLiteral(", ")));
        return std::nullopt;
    }

    CornerSolver solver;
    solver.m_mechanism = mechanism;

    const auto at = [&table](const QString& name) {
        const Hardpoint* point = table.find(name);
        return point ? Vec3(point->coord[0], point->coord[1], point->coord[2]) : Vec3();
    };
    const auto has = [&table](const QString& name) {
        return !name.isEmpty() && table.indexOf(name) >= 0;
    };

    solver.m_lowerFront = at(mechanism.lowerFront);
    solver.m_lowerRear = at(mechanism.lowerRear);
    solver.m_lowerOuter = at(mechanism.lowerOuter);
    solver.m_upperFront = at(mechanism.upperFront);
    solver.m_upperRear = at(mechanism.upperRear);
    solver.m_upperOuter = at(mechanism.upperOuter);
    solver.m_tieRodInboard = at(mechanism.tieRodInboard);
    solver.m_tieRodOutboard = at(mechanism.tieRodOutboard);
    solver.m_wheelCenter = at(mechanism.wheelCenter);

    // A rack drives this corner only if the template says one does, and only
    // through the point this solve knows how to move. Anything else is caught
    // where there is somewhere to report it -- AxleSolver::build().
    solver.m_steered = !mechanism.steeringRack.isEmpty()
                       && mechanism.steeringRack == mechanism.tieRodInboard;

    solver.m_hasWheelAxis = has(mechanism.wheelAxis);
    if (solver.m_hasWheelAxis) solver.m_wheelAxisPoint = at(mechanism.wheelAxis);

    solver.m_hasContactPatch = has(mechanism.contactPatch);
    if (solver.m_hasContactPatch) solver.m_contactPatch = at(mechanism.contactPatch);

    solver.m_hasRocker = mechanism.hasRocker() && has(mechanism.pushrodOuter)
                         && has(mechanism.pushrodInner) && has(mechanism.rockerPivot)
                         && has(mechanism.rockerAxis);
    if (solver.m_hasRocker) {
        solver.m_pushrodOuter = at(mechanism.pushrodOuter);
        solver.m_pushrodInner = at(mechanism.pushrodInner);
        solver.m_rockerPivot = at(mechanism.rockerPivot);
        solver.m_rockerAxisPoint = at(mechanism.rockerAxis);
    }

    // The damper hangs off the rocker, so without one there is nothing to
    // measure its length against.
    solver.m_hasDamper = solver.m_hasRocker && has(mechanism.damperInboard)
                         && has(mechanism.damperOutboard);
    if (solver.m_hasDamper) {
        solver.m_damperInboard = at(mechanism.damperInboard);
        solver.m_damperOutboard = at(mechanism.damperOutboard);
    }

    // A drop link on the rocker needs the rocker bound; one on the wheel's own
    // members does not, and is checked against the rocker further down.
    solver.m_hasAntiRoll = mechanism.hasAntiRoll() && has(mechanism.antiRollDropLinkOuter)
                           && has(mechanism.antiRollArmEnd) && has(mechanism.antiRollArmRoot);
    if (solver.m_hasAntiRoll) {
        solver.m_antiRollDropLinkOuter = at(mechanism.antiRollDropLinkOuter);
        solver.m_antiRollArmEnd = at(mechanism.antiRollArmEnd);
        solver.m_antiRollArmRoot = at(mechanism.antiRollArmRoot);
        solver.m_antiRollAxisFromBearing = has(mechanism.antiRollBearing);
        if (solver.m_antiRollAxisFromBearing)
            solver.m_antiRollBearing = at(mechanism.antiRollBearing);
    }

    for (const QString& name : mechanism.carried)
        if (has(name)) solver.m_carried.push_back(PosedPoint{ name, at(name) });

    // +y is left in ISO 8855, and that is the whole of what decides which way
    // camber and toe are signed.
    solver.m_left = solver.m_wheelCenter.y >= 0.0;

    solver.m_lowerAxis = axisThrough(solver.m_lowerFront, solver.m_lowerRear);
    solver.m_upperAxis = axisThrough(solver.m_upperFront, solver.m_upperRear);
    if (!solver.m_lowerAxis.isValid() || !solver.m_upperAxis.isValid()) {
        if (error)
            *error = tr("A wishbone's two chassis pivots are at the same place, so it has no "
                        "axis to turn about.");
        return std::nullopt;
    }
    solver.m_upperCircle = circleAbout(solver.m_upperOuter, solver.m_upperAxis);
    if (!solver.m_upperCircle.isValid()) {
        if (error) *error = tr("The upper ball joint sits on its own pivot axis.");
        return std::nullopt;
    }

    if (solver.m_hasRocker) {
        solver.m_rockerAxis = axisThrough(solver.m_rockerPivot, solver.m_rockerAxisPoint);
        solver.m_pushrodInnerCircle = circleAbout(solver.m_pushrodInner, solver.m_rockerAxis);
        if (!solver.m_pushrodInnerCircle.isValid()) solver.m_hasRocker = false;
    }

    if (solver.m_hasAntiRoll) {
        // A bearing in the table says where the bar's axis is outright. Without
        // one, a U-bar runs across the car, so until the far side's arm root
        // gives the real axis y is the right guess and not much of one.
        const Vec3 alongBar = solver.m_antiRollAxisFromBearing
                                  ? solver.m_antiRollBearing - solver.m_antiRollArmRoot
                                  : Vec3();
        if (alongBar.lengthSquared() < 1e-12) solver.m_antiRollAxisFromBearing = false;
        const Vec3 direction = solver.m_antiRollAxisFromBearing
                                   ? barAxisPointingLeft(alongBar, solver.side())
                                   : Vec3(0, 1, 0);
        solver.m_antiRollAxis =
            axisThrough(solver.m_antiRollArmRoot, solver.m_antiRollArmRoot + direction);
        solver.m_antiRollCircle = circleAbout(solver.m_antiRollArmEnd, solver.m_antiRollAxis);
        if (!solver.m_antiRollCircle.isValid()) solver.m_hasAntiRoll = false;
    }

    // Which way the wheel points, as a unit vector outboard along its own axis of
    // rotation.
    //
    // Static angles the project states say it outright, and win: they are the
    // number the user typed, and a point that disagrees with them is a point
    // nobody has moved since. Next, a second point on the axis says it, camber
    // and toe together -- the only way a hardpoint table can state static toe
    // at all. Without either it is inferred the old way, from the contact patch
    // sitting under the wheel centre: that carries the workbook's static camber
    // and assumes zero toe, which is why toe is also reported as a change from
    // this position.
    Vec3 spin;
    if (alignment) {
        spin = spinAxisFor(*alignment, solver.side());
        solver.m_attitude = WheelAttitude::Stated;
    } else if (solver.m_hasWheelAxis) {
        spin = (solver.m_wheelAxisPoint - solver.m_wheelCenter).normalized();
        // Either end of the axle may have been measured; outboard is the end the
        // measures are read against.
        if (spin.y * solver.side() < 0.0) spin = spin * -1.0;
        solver.m_attitude = WheelAttitude::WheelAxis;
    } else if (solver.m_hasContactPatch) {
        const Vec3 up = solver.m_wheelCenter - solver.m_contactPatch;
        spin = cross(up, Vec3(1, 0, 0)).normalized() * solver.side();
        solver.m_attitude = WheelAttitude::ContactPatch;
    }
    if (spin.lengthSquared() < 0.5) {
        spin = Vec3(0, solver.side(), 0);
        solver.m_attitude = alignment ? WheelAttitude::Stated : WheelAttitude::Upright;
    }
    solver.m_designSpinAxis = spin;

    // With the angles stated, a wheel axis point in the table is one more point
    // on the upright, and it is carried on the axis those angles give -- as far
    // from the centre as the table has it, on the same side -- so that what is
    // drawn while simulating is the axis the numbers are actually read off.
    if (alignment && solver.m_hasWheelAxis) {
        const Vec3 offset = solver.m_wheelAxisPoint - solver.m_wheelCenter;
        const double reach = dot(offset, spin) < 0.0 ? -offset.length() : offset.length();
        solver.m_wheelAxisPoint = solver.m_wheelCenter + spin * reach;
    }

    // The tyre radius, which is what turns a wheel centre and an axis into a
    // contact patch: the drop from the centre down the wheel's own plane onto
    // the road. What a patch in the workbook supplies is the height of that
    // road -- a table measured from a chassis datum rather than from the ground
    // has no other way of saying where the ground is -- and not the patch's own
    // position, which under a cambered wheel is not where the tyre touches.
    //
    // With no axis point named the two come to the same thing: the drop is then
    // straight down the line the patch itself defined, so the patch is
    // reproduced exactly and nothing about an older project moves.
    const double groundZ = solver.m_hasContactPatch ? solver.m_contactPatch.z : kGroundZ;
    bool grounded = false;
    solver.m_tireRadius = tireRadiusToGround(solver.m_wheelCenter, spin, groundZ, &grounded);
    solver.m_hasGround = grounded;

    // Computed from here on, at design as well as through the travel, so that
    // the rise a roll sweep drives is measured against the same thing it moves.
    if (grounded)
        solver.m_contactPatch = contactPatchFor(solver.m_wheelCenter, spin, solver.m_tireRadius);
    else if (!solver.m_hasContactPatch)
        solver.m_contactPatch = solver.m_wheelCenter - Vec3(0, 0, 1);

    if (!solver.bindGraph(graph ? *graph : roleGraph(mechanism, table), alignment.has_value(),
                          error))
        return std::nullopt;

    CornerPose design;
    if (!solver.solveAt(0.0, 0.0, nullptr, &design)) {
        if (error)
            *error = design.error.isEmpty()
                         ? tr("This corner does not assemble at the coordinates given.")
                         : design.error;
        return std::nullopt;
    }
    // Measured against itself, so every change is zero here by construction.
    design.camberChange = 0.0;
    design.toeChange = 0.0;
    design.damperTravel = 0.0;
    solver.m_design = design;

    return solver;
}

namespace {

/// The one of @p point's two bodies that @p other also names, or -1.
int sharedBody(const GraphPoint& point, const GraphPoint& other)
{
    for (const int body : { point.body1, point.body2 })
        if (body > MechanismGraph::kGround && (body == other.body1 || body == other.body2))
            return body;
    return MechanismGraph::kNoBody;
}

/// The moving body @p point is carried by: its first, unless that is the chassis.
int carryingBody(const GraphPoint& point)
{
    return point.body1 > MechanismGraph::kGround ? point.body1 : point.body2;
}

/// How far out along the bar's axis its made-up second point is put. Any
/// distance does; one of the size of the bar keeps the solve's numbers even.
constexpr double kBarAxisReach = 100.0;

} // namespace

int CornerSolver::pointIndex(const QString& name) const
{
    return name.isEmpty() ? -1 : m_graphSource.indexOfPoint(name);
}

bool CornerSolver::bindGraph(MechanismGraph graph, bool stated, QString* error)
{
    m_graphSource = std::move(graph);

    // With the angles stated the axis point is carried on the axis they give,
    // which is where bind() has already put it.
    const int axisPoint = pointIndex(m_mechanism.wheelAxis);
    if (stated && m_hasWheelAxis && axisPoint >= 0)
        m_graphSource.points[std::size_t(axisPoint)].design = m_wheelAxisPoint;

    // The lower wishbone is the body its ball joint shares with its pivots.
    const int outer = pointIndex(m_mechanism.lowerOuter);
    const int front = pointIndex(m_mechanism.lowerFront);
    const int center = pointIndex(m_mechanism.wheelCenter);
    if (outer < 0 || front < 0 || center < 0) {
        if (error)
            *error = tr("The configuration says nothing about the lower wishbone or the wheel "
                        "centre, so there is nothing to drive the corner by or to read it off.");
        return false;
    }
    m_drivenBody = sharedBody(m_graphSource.points[std::size_t(outer)],
                              m_graphSource.points[std::size_t(front)]);
    m_uprightBody = carryingBody(m_graphSource.points[std::size_t(center)]);
    m_rackPoint = m_steered ? pointIndex(m_mechanism.tieRodInboard) : -1;

    // A bar with no bearing in the table turns about an axis the axle decides;
    // the linkage is given a second point on it to turn about.
    const int root = pointIndex(m_mechanism.antiRollArmRoot);
    if (m_hasAntiRoll && !m_antiRollAxisFromBearing && root >= 0) {
        GraphPoint axisEnd = m_graphSource.points[std::size_t(root)];
        axisEnd.name.clear();
        axisEnd.design = m_antiRollArmRoot + m_antiRollAxis.direction * kBarAxisReach;
        m_graphSource.points.push_back(axisEnd);
        m_barAxisPoint = int(m_graphSource.points.size()) - 1;
    }

    m_graph = GraphSolver::compile(m_graphSource, m_drivenBody, error);
    if (!m_graph) return false;

    const auto moving = [this](const QString& name) {
        const int index = pointIndex(name);
        return index >= 0 && m_graph->moves(index, m_rackPoint);
    };
    const int damperInboard = pointIndex(m_mechanism.damperInboard);
    m_hasRocker = m_hasRocker && moving(m_mechanism.pushrodInner);
    m_hasDamper = damperInboard >= 0 && moving(m_mechanism.damperOutboard);
    if (m_hasDamper) {
        m_damperInboard = m_graphSource.points[std::size_t(damperInboard)].design;
        m_damperOutboard =
            m_graphSource.points[std::size_t(pointIndex(m_mechanism.damperOutboard))].design;
    }
    m_hasAntiRoll = m_hasAntiRoll && moving(m_mechanism.antiRollArmEnd);
    return true;
}

bool CornerSolver::placeBarAxis(const Vec3& direction)
{
    if (m_barAxisPoint < 0) return false;
    MechanismGraph moved = m_graphSource;
    moved.points[std::size_t(m_barAxisPoint)].design =
        m_antiRollArmRoot + direction.normalized() * kBarAxisReach;
    std::optional<GraphSolver> compiled = GraphSolver::compile(moved, m_drivenBody, nullptr);
    if (!compiled) return false;
    m_graphSource = std::move(moved);
    m_graph = std::move(compiled);
    return true;
}

void CornerSolver::setAntiRollAxis(const Vec3& direction)
{
    if (!m_hasAntiRoll || m_antiRollAxisFromBearing) return;
    const Axis axis = axisThrough(m_antiRollArmRoot, m_antiRollArmRoot + direction);
    if (!axis.isValid()) return;
    const Circle circle = circleAbout(m_antiRollArmEnd, axis);
    if (!circle.isValid() || !placeBarAxis(direction)) return;
    m_antiRollAxis = axis;
    m_antiRollCircle = circle;
    // The design pose's bar angle is zero either way, so nothing else has to be
    // rebuilt: at design every body is where the table put it.
}

bool CornerSolver::solveAt(double angle, double rackTravel, const CornerPose* previous,
                           CornerPose* out) const
{
    CornerPose pose;
    pose.armAngle = angle;
    // An axle with no rack ignores rack travel rather than pretending to steer.
    // The pose reports what actually happened, not what was asked for.
    const double rack = m_steered ? rackTravel : 0.0;
    pose.rackTravel = rack;
    const Vec3 rackOffset(0.0, rack, 0.0);

    // The lower wishbone turns about its pivots by the one free variable; the
    // linkage does the rest.
    GraphState state;
    if (!m_graph->solve(rotationAbout(m_lowerAxis, angle), m_rackPoint, rackOffset,
                        previous ? &previous->graphState : nullptr, &state, &pose.error)) {
        *out = pose;
        return false;
    }

    const Rigid upright = m_graph->motionOf(m_uprightBody, state);
    pose.wheelCenter = upright.map(m_wheelCenter);
    pose.spinAxis = upright.rotate(m_designSpinAxis).normalized();
    pose.uprightMotion = upright;
    pose.wheelCenterName = m_mechanism.wheelCenter;

    // The wheel is rigid with the upright; the contact patch is not. It is the
    // bottom of a tyre that stays on the road, so it is recomputed from the
    // wheel's new attitude rather than carried round with the upright -- which
    // is what makes it walk outboard as the wheel gains camber instead of
    // lifting off the ground with it.
    pose.contactPatch = m_hasGround
                            ? contactPatchFor(pose.wheelCenter, pose.spinAxis, m_tireRadius)
                            : upright.map(m_contactPatch);

    readJoints(state, rackOffset, &pose);
    pose.graphState = std::move(state);

    pose.hasDamper = m_hasDamper;
    pose.hasAntiRoll = m_hasAntiRoll;
    pose.valid = true;
    measure(&pose);
    *out = pose;
    return true;
}

void CornerSolver::readJoints(const GraphState& state, const Vec3& rackOffset,
                              CornerPose* pose) const
{
    const auto at = [&](const QString& name) {
        const int index = pointIndex(name);
        return index < 0 ? Vec3() : m_graph->positionOf(index, state, m_rackPoint, rackOffset);
    };
    pose->lowerOuter = at(m_mechanism.lowerOuter);
    pose->upperOuter = at(m_mechanism.upperOuter);
    pose->tieRodInboard = at(m_mechanism.tieRodInboard);
    pose->tieRodOutboard = at(m_mechanism.tieRodOutboard);
    pose->upperArmAngle = m_upperCircle.angleOf(pose->upperOuter);
    if (m_hasRocker) {
        pose->pushrodOuter = at(m_mechanism.pushrodOuter);
        pose->pushrodInner = at(m_mechanism.pushrodInner);
        pose->rockerAngle = m_pushrodInnerCircle.angleOf(pose->pushrodInner);
    }
    if (m_hasDamper) pose->damperOutboard = at(m_mechanism.damperOutboard);
    if (m_hasAntiRoll) {
        pose->antiRollDropLinkOuter = at(m_mechanism.antiRollDropLinkOuter);
        pose->antiRollArmEnd = at(m_mechanism.antiRollArmEnd);
        pose->antiRollArmAngle = m_antiRollCircle.angleOf(pose->antiRollArmEnd);
    }

    // Every point the linkage moved, for laying over the table. A contact
    // patch is the computed one, not the one the upright would carry.
    const MechanismGraph& graph = m_graph->graph();
    for (std::size_t i = 0; i < graph.points.size(); ++i) {
        const GraphPoint& point = graph.points[i];
        // The inboard tie rod end is reported steered or not, as it always was.
        const bool reported = m_graph->moves(int(i), m_rackPoint)
                              || point.name == m_mechanism.tieRodInboard;
        if (point.name.isEmpty() || !reported) continue;
        const bool patch = point.type == PointType::ContactPatch
                           || point.name == m_mechanism.contactPatch;
        pose->points.push_back(PosedPoint{
            point.name, patch ? pose->contactPatch
                              : m_graph->positionOf(int(i), state, m_rackPoint, rackOffset) });
    }
}

void CornerSolver::measure(CornerPose* pose) const
{
    const double s = side();

    // Camber and toe are the spin axis in two different views. The axis points
    // outboard, so the same two formulas do both sides.
    const Vec3 n = pose->spinAxis;
    pose->camber = -std::asin(std::clamp(n.z, -1.0, 1.0)) * kRadToDeg;
    pose->camberToGround = pose->camber;
    pose->toe = std::atan2(n.x, s * n.y) * kRadToDeg;
    pose->camberChange = pose->camber - m_design.camber;
    pose->toeChange = pose->toe - m_design.toe;

    // The steering axis runs from the lower ball joint up to the upper one.
    const Vec3 steering = pose->upperOuter - pose->lowerOuter;
    pose->caster = std::atan2(-steering.x, steering.z) * kRadToDeg;
    pose->kingpinInclination = std::atan2(-s * steering.y, steering.z) * kRadToDeg;

    pose->wheelTravel = pose->wheelCenter.z - m_wheelCenter.z;
    pose->contactPatchRise = pose->contactPatch.z - m_contactPatch.z;

    // Track and wheelbase are read at the ground when there is a contact patch
    // to read them at -- named or computed -- and at the wheel centre when the
    // wheel is lying so flat that there is none.
    const Vec3& ground = m_hasGround ? pose->contactPatch : pose->wheelCenter;
    const Vec3& groundDesign = m_hasGround ? m_contactPatch : m_wheelCenter;
    pose->halfTrackChange = s * (ground.y - groundDesign.y);
    pose->wheelbaseChange = ground.x - groundDesign.x;

    bool ok = false;
    const Vec3 pierce = linePlaneCrossing(pose->lowerOuter, pose->upperOuter, 2, ground.z, &ok);
    if (ok) {
        pose->scrubRadius = s * (ground.y - pierce.y);
        pose->mechanicalTrail = pierce.x - ground.x;
    }

    if (m_hasDamper) {
        pose->damperLength = distance(pose->damperOutboard, m_damperInboard);
        pose->damperTravel = pose->damperLength - m_design.damperLength;
    }

    // The front-view instant centre, the way RCVD constructs it. A wishbone
    // moves its ball joint at right angles to the plane through that joint and
    // the arm's pivot axis, so the upright turns, for an instant, about the line
    // the two arm planes share; the front-view centre is where that line pierces
    // the transverse plane through the wheel centre. The hardpoint generator
    // places its points by the same construction, so the two are inverses here
    // as well.
    //
    // It is not the same as drawing each arm from its pivot to its ball joint
    // and flattening the result, which is what this used to do: that ignores
    // how far a joint sits ahead of or behind the wheel centre, and on an arm
    // whose pivot axis is inclined a caster's worth of offset moved a real
    // car's roll centre from 10 mm to 6.
    const Vec3 lowerNormal = cross(m_lowerRear - m_lowerFront, pose->lowerOuter - m_lowerFront);
    const Vec3 upperNormal = cross(m_upperRear - m_upperFront, pose->upperOuter - m_upperFront);
    bool centerOk = false;
    pose->instantCenter = planesCrossing(lowerNormal, m_lowerFront, upperNormal, m_upperFront, 0,
                                         pose->wheelCenter.x, &centerOk);
    pose->instantCenterValid = centerOk;
}

CornerPose CornerSolver::driveTo(double value, double rackTravel, const CornerPose* previous,
                                 double (*pick)(const CornerPose&)) const
{
    CornerPose pose;
    double angle = previous && previous->valid ? previous->armAngle : 0.0;
    if (!solveAt(angle, rackTravel, previous, &pose)) return pose;

    double residual = pick(pose) - value;
    constexpr double kProbe = 1e-5;

    for (int iteration = 0; iteration < 40 && std::abs(residual) > kTargetTolerance; ++iteration) {
        // The slope of height against arm angle, by a probe to whichever side
        // assembles. Near a travel limit only one of them does.
        CornerPose probe;
        double slope = 0.0;
        if (solveAt(angle + kProbe, rackTravel, &pose, &probe)) {
            slope = (pick(probe) - pick(pose)) / kProbe;
        } else if (solveAt(angle - kProbe, rackTravel, &pose, &probe)) {
            slope = (pick(pose) - pick(probe)) / kProbe;
        } else {
            pose.valid = false;
            pose.error = tr("The mechanism jams before it reaches that position.");
            return pose;
        }
        if (std::abs(slope) < 1e-12) break;

        double step = std::clamp(-residual / slope, -kMaxStep, kMaxStep);

        // Back off until it assembles again. A step that overshoots the end of
        // the travel is the normal way to find where that end is.
        CornerPose next;
        bool moved = false;
        for (int backoff = 0; backoff < 30; ++backoff) {
            if (solveAt(angle + step, rackTravel, &pose, &next)) {
                moved = true;
                break;
            }
            step *= 0.5;
        }
        if (!moved) {
            pose.valid = false;
            pose.error = tr("The mechanism jams before it reaches that position.");
            return pose;
        }

        angle += step;
        pose = next;
        residual = pick(pose) - value;
    }

    if (std::abs(residual) > 1e-4) {
        pose.valid = false;
        pose.error = tr("That position is outside this corner's travel.");
    }
    return pose;
}

CornerPose CornerSolver::poseAtArmAngle(double angle, double rackTravel,
                                        const CornerPose* previous) const
{
    CornerPose pose;
    solveAt(angle, rackTravel, previous, &pose);
    return pose;
}

CornerPose CornerSolver::poseAtWheelTravel(double travel, double rackTravel,
                                           const CornerPose* previous) const
{
    return driveTo(m_wheelCenter.z + travel, rackTravel, previous, &pickWheelHeight);
}

CornerPose CornerSolver::poseAtContactPatchRise(double rise, double rackTravel,
                                                const CornerPose* previous) const
{
    return driveTo(m_contactPatch.z + rise, rackTravel, previous, &pickContactHeight);
}

} // namespace suspkin
