#include "io/LinkageTemplate.h"
#include "model/GraphSolver.h"
#include "model/HardpointConfig.h"
#include "model/HardpointMirror.h"
#include "model/MechanismGraph.h"
#include "model/SuspensionSolver.h"
#include "model/Sweep.h"
#include "support/ReferenceCorner.h"

#include <QTest>

#include <cmath>

using namespace suspkin;

namespace {

/// Every number a pose reports is held to this: far below anything measured,
/// and what is left between two solves that took different paths to the same
/// answer.
constexpr double kAgree = 1e-9;

Hardpoint make(const char* name, double x, double y, double z)
{
    Hardpoint point;
    point.name = QLatin1String(name);
    point.coord[0] = x;
    point.coord[1] = y;
    point.coord[2] = z;
    return point;
}

/// The front left corner test_kinematics solves, named the way the built-in
/// template names its points now.
HardpointTable frontLeftCorner()
{
    HardpointTable table;
    table.points = {
        make("F_LCA_IF", 100.0, 200.0, 120.0),     make("F_LCA_IR", -100.0, 200.0, 120.0),
        make("F_LCA_O", 0.0, 560.0, 110.0),        make("F_UCA_IF", 80.0, 230.0, 280.0),
        make("F_UCA_IR", -80.0, 230.0, 280.0),     make("F_UCA_O", 0.0, 540.0, 300.0),
        make("F_TieRod_I", -120.0, 220.0, 150.0),  make("F_TieRod_O", -120.0, 555.0, 145.0),
        make("F_WheelCenter", 0.0, 600.0, 220.0),  make("F_ContactPatch", 0.0, 600.0, 0.0),
        make("F_PushRod_O", 0.0, 500.0, 295.0),    make("F_PushRod_I", 0.0, 185.4, 560.4),
        make("F_Rocker_Center", 0.0, 150.0, 500.0), make("F_Rocker_AxisPoint", 100.0, 150.0, 500.0),
        make("F_Damper_O", 0.0, 95.0, 524.0),      make("F_Damper_I", 0.0, 15.0, 341.0),
        make("F_ARB_DropLink_O", 0.0, 180.0, 540.0), make("F_ARB_ArmEnd", -20.0, 200.0, 470.0),
        make("F_ARB_ArmRoot", -100.0, 200.0, 450.0),
    };
    return table;
}

HardpointTable withPoint(HardpointTable table, const Hardpoint& point)
{
    table.points.push_back(point);
    return table;
}

MechanismTemplate frontMechanism(bool mirrored = false)
{
    MechanismTemplate templ = builtinMechanismTemplate();
    templ.steeringRack = templ.tieRodInboard;
    return instantiateMechanism(templ, QStringLiteral("F"), mirrored, MirrorSpec{});
}

void expectSame(const CornerPose& a, const CornerPose& b, const char* what)
{
    QVERIFY2(a.valid == b.valid, what);
    if (!a.valid) return;
    const auto near = [](double x, double y) { return std::abs(x - y) < kAgree; };
    const auto nearVec = [&](const Vec3& x, const Vec3& y) {
        return near(x.x, y.x) && near(x.y, y.y) && near(x.z, y.z);
    };
    QVERIFY2(near(a.armAngle, b.armAngle), what);
    QVERIFY2(near(a.upperArmAngle, b.upperArmAngle), what);
    QVERIFY2(near(a.rockerAngle, b.rockerAngle), what);
    QVERIFY2(near(a.antiRollArmAngle, b.antiRollArmAngle), what);
    for (const auto& [x, y] : { std::pair{ a.lowerOuter, b.lowerOuter },
                                { a.upperOuter, b.upperOuter },
                                { a.tieRodInboard, b.tieRodInboard },
                                { a.tieRodOutboard, b.tieRodOutboard },
                                { a.wheelCenter, b.wheelCenter },
                                { a.contactPatch, b.contactPatch },
                                { a.pushrodOuter, b.pushrodOuter },
                                { a.pushrodInner, b.pushrodInner },
                                { a.damperOutboard, b.damperOutboard },
                                { a.antiRollDropLinkOuter, b.antiRollDropLinkOuter },
                                { a.antiRollArmEnd, b.antiRollArmEnd },
                                { a.spinAxis, b.spinAxis } })
        QVERIFY2(nearVec(x, y), what);
    for (const auto& [x, y] :
         { std::pair{ a.camber, b.camber }, { a.toe, b.toe }, { a.caster, b.caster },
           { a.kingpinInclination, b.kingpinInclination }, { a.scrubRadius, b.scrubRadius },
           { a.mechanicalTrail, b.mechanicalTrail }, { a.halfTrackChange, b.halfTrackChange },
           { a.wheelbaseChange, b.wheelbaseChange }, { a.wheelTravel, b.wheelTravel },
           { a.damperLength, b.damperLength }, { a.camberChange, b.camberChange },
           { a.toeChange, b.toeChange } })
        QVERIFY2(near(x, y), what);
    QVERIFY2(a.instantCenterValid == b.instantCenterValid, what);
    if (a.instantCenterValid) QVERIFY2(nearVec(a.instantCenter, b.instantCenter), what);
    // Every point the reference moved, the graph moved to the same place.
    for (const PosedPoint& point : a.points) {
        const Vec3* other = b.find(point.name);
        QVERIFY2(other, qPrintable(QStringLiteral("%1: %2").arg(QLatin1String(what), point.name)));
        QVERIFY2(nearVec(point.position, *other), what);
    }
}

/// Bump, roll and steer through both solvers, step by step from the pose
/// before, the way a sweep runs them.
void expectSweepsAgree(const ReferenceCornerSolver& reference, const CornerSolver& graph)
{
    expectSame(reference.designPose(), graph.designPose(), "design");
    for (const double rack : { -12.0, 0.0, 12.0 }) {
        CornerPose a, b;
        bool first = true;
        for (double travel = -40.0; travel <= 40.0; travel += 4.0) {
            a = reference.poseAtWheelTravel(travel, rack, first ? nullptr : &a);
            b = graph.poseAtWheelTravel(travel, rack, first ? nullptr : &b);
            first = false;
            expectSame(a, b, "bump");
        }
        first = true;
        for (double rise = -30.0; rise <= 30.0; rise += 5.0) {
            a = reference.poseAtContactPatchRise(rise, rack, first ? nullptr : &a);
            b = graph.poseAtContactPatchRise(rise, rack, first ? nullptr : &b);
            first = false;
            expectSame(a, b, "roll");
        }
    }
}

struct Case {
    const char* name;
    HardpointTable table;
    MechanismTemplate mechanism;
    std::optional<StaticAlignment> alignment;
};

std::vector<Case> cases()
{
    std::vector<Case> out;
    out.push_back({ "plain", frontLeftCorner(), frontMechanism(), std::nullopt });
    out.push_back({ "wheel axis",
                    withPoint(frontLeftCorner(), make("F_WheelAxis", 5.0, 749.0, 228.0)),
                    frontMechanism(), std::nullopt });
    out.push_back({ "stated angles",
                    withPoint(frontLeftCorner(), make("F_WheelAxis", 5.0, 749.0, 228.0)),
                    frontMechanism(), StaticAlignment{ -1.5, 0.3 } });
    out.push_back({ "bar bearing",
                    withPoint(frontLeftCorner(), make("F_ARB_Bearing", -100.0, 120.0, 452.0)),
                    frontMechanism(), std::nullopt });
    MechanismTemplate unsteered = frontMechanism();
    unsteered.steeringRack.clear();
    out.push_back({ "unsteered", frontLeftCorner(), unsteered, std::nullopt });
    MechanismTemplate lowerMount = frontMechanism();
    lowerMount.pushrodMount = PushrodMount::LowerArm;
    HardpointTable lower = frontLeftCorner();
    for (Hardpoint& point : lower.points)
        if (point.name == QLatin1String("F_PushRod_O")) point.coord[2] = 120.0;
    out.push_back({ "pushrod on the lower arm", lower, lowerMount, std::nullopt });
    return out;
}

} // namespace

class TestGraphSolver : public QObject {
    Q_OBJECT

private slots:
    void theRoleGraphSolvesExactlyAsTheClosedFormDid();
    void theFarSideSolvesExactlyAsTheClosedFormDid();
    void theBuiltinConfigurationSolvesLikeTheRoles();
    void theBuiltinConfigurationLeavesNothingFree();
    void aCornerMissingItsTieRodIsFreeToSteerItself();
    void aStrutInLineKeepsItsPushrodOnItsLine();
    void aDamperOnTheWishboneIsMeasuredWithoutARocker();
};

void TestGraphSolver::theRoleGraphSolvesExactlyAsTheClosedFormDid()
{
    for (const Case& c : cases()) {
        QString error;
        const std::optional<ReferenceCornerSolver> reference =
            ReferenceCornerSolver::bind(c.mechanism, c.table, &error, c.alignment);
        QVERIFY2(reference, qPrintable(error));
        const std::optional<CornerSolver> graph =
            CornerSolver::bind(c.mechanism, c.table, &error, c.alignment);
        QVERIFY2(graph, qPrintable(error));
        QCOMPARE(graph->hasRocker(), reference->hasRocker());
        QCOMPARE(graph->hasDamper(), reference->hasDamper());
        QCOMPARE(graph->hasAntiRoll(), reference->hasAntiRoll());
        expectSweepsAgree(*reference, *graph);
        if (QTest::currentTestFailed()) qWarning("case: %s", c.name);
    }
}

void TestGraphSolver::theFarSideSolvesExactlyAsTheClosedFormDid()
{
    const HardpointTable axle = mirrorHardpoints(frontLeftCorner(), {}, MirrorSpec{}).table;
    const MechanismTemplate far = frontMechanism(true);
    QString error;
    std::optional<ReferenceCornerSolver> reference =
        ReferenceCornerSolver::bind(far, axle, &error);
    std::optional<CornerSolver> graph = CornerSolver::bind(far, axle, &error);
    QVERIFY2(reference && graph, qPrintable(error));
    // The axle sets a bar without a bearing turning about the line between the
    // two arm roots.
    reference->setAntiRollAxis(Vec3(0.0, 1.0, 0.1));
    graph->setAntiRollAxis(Vec3(0.0, 1.0, 0.1));
    expectSweepsAgree(*reference, *graph);
}

void TestGraphSolver::theBuiltinConfigurationSolvesLikeTheRoles()
{
    const LinkageTemplate templ = builtinLinkageTemplate();
    const HardpointTable table = frontLeftCorner();
    const HardpointConfigMap config = inferHardpointConfig(table, templ, MirrorSpec{});
    const MechanismGraph graph =
        configGraph(cornerPointNames(templ, QStringLiteral("F"), false, MirrorSpec{}, table),
                    config, templ, table);

    QString error;
    const std::optional<ReferenceCornerSolver> reference =
        ReferenceCornerSolver::bind(frontMechanism(), table, &error);
    const std::optional<CornerSolver> configured =
        CornerSolver::bind(frontMechanism(), table, &error, std::nullopt, &graph);
    QVERIFY2(reference && configured, qPrintable(error));
    QVERIFY(configured->hasRocker());
    QVERIFY(configured->hasDamper());
    QVERIFY(configured->hasAntiRoll());
    expectSweepsAgree(*reference, *configured);
}

void TestGraphSolver::theBuiltinConfigurationLeavesNothingFree()
{
    const LinkageTemplate templ = builtinLinkageTemplate();
    const HardpointTable table = frontLeftCorner();
    const MechanismGraph graph =
        configGraph(cornerPointNames(templ, QStringLiteral("F"), false, MirrorSpec{}, table),
                    inferHardpointConfig(table, templ, MirrorSpec{}), templ, table);
    QString error;
    const std::optional<CornerSolver> corner =
        CornerSolver::bind(frontMechanism(), table, &error, std::nullopt, &graph);
    QVERIFY2(corner, qPrintable(error));
    const GraphMobility mobility = corner->mobility();
    QVERIFY2(mobility.isDetermined(), qPrintable(mobility.looseBodies.join(", ")));
    QCOMPARE(mobility.freeWithDriveLoose, 1);
}

void TestGraphSolver::aCornerMissingItsTieRodIsFreeToSteerItself()
{
    const LinkageTemplate templ = builtinLinkageTemplate();
    const HardpointTable table = frontLeftCorner();
    HardpointConfigMap config = inferHardpointConfig(table, templ, MirrorSpec{});
    config.insert(QStringLiteral("F_TieRod_O"), HardpointConfig{}); // cleared by the user
    const MechanismGraph graph =
        configGraph(cornerPointNames(templ, QStringLiteral("F"), false, MirrorSpec{}, table),
                    config, templ, table);
    QString error;
    const std::optional<CornerSolver> corner =
        CornerSolver::bind(frontMechanism(), table, &error, std::nullopt, &graph);
    QVERIFY2(corner, qPrintable(error));
    const GraphMobility mobility = corner->mobility();
    QVERIFY(!mobility.isDetermined());
    // The upright turns about its two ball joints, and the tie rod hangs from
    // its inboard end, free to swing every way.
    QCOMPARE(mobility.freeWithDriveHeld, 4);
    QVERIFY(mobility.looseBodies.contains(QStringLiteral("Upright")));
    QVERIFY(mobility.looseBodies.contains(QStringLiteral("Tie rod")));
}

namespace {

/// A corner with no rocker: the pushrod bolted rigidly to a damper's shaft, the
/// damper's body pinned to the chassis -- a strut, in line.
struct InlineStrut {
    HardpointTable table;
    MechanismGraph graph;
    MechanismTemplate mechanism;
};

InlineStrut inlineStrut()
{
    InlineStrut out;
    out.table = frontLeftCorner();
    std::erase_if(out.table.points, [](const Hardpoint& p) {
        return p.name.contains(QLatin1String("Rocker")) || p.name.contains(QLatin1String("ARB"))
               || p.name.contains(QLatin1String("Damper")) || p.name == QLatin1String("F_PushRod_I");
    });
    // The damper's chassis end, and the joint where pushrod meets shaft 200 mm
    // short of it, on the line from the pushrod's outer end.
    const Vec3 outer(0.0, 500.0, 295.0);
    const Vec3 mount(20.0, 120.0, 620.0);
    const Vec3 joint = mount + (outer - mount).normalized() * 200.0;
    out.table.points.push_back(make("F_Damper_I", mount.x, mount.y, mount.z));
    out.table.points.push_back(make("F_Damper_O", joint.x, joint.y, joint.z));

    out.mechanism = frontMechanism();
    out.mechanism.pushrodInner.clear();
    out.mechanism.rockerPivot.clear();
    out.mechanism.rockerAxis.clear();
    out.mechanism.antiRollArmEnd.clear();
    out.mechanism.antiRollArmRoot.clear();
    out.mechanism.antiRollDropLinkOuter.clear();
    out.mechanism.antiRollBearing.clear();

    out.graph = roleGraph(out.mechanism, out.table);
    const int pushrod = out.graph.bodyFor(QStringLiteral("Pushrod"));
    const int damper = out.graph.bodyFor(QStringLiteral("Damper"), true);
    const int upperArm = out.graph.bodyFor(QStringLiteral("Upper wishbone"));
    out.graph.points.push_back(
        GraphPoint{ QStringLiteral("F_PushRod_O"), outer, PointType::Solved, pushrod, upperArm });
    out.graph.points.push_back(
        GraphPoint{ QStringLiteral("F_Damper_O"), joint, PointType::Welded, pushrod, damper });
    out.graph.points.push_back(GraphPoint{ QStringLiteral("F_Damper_I"), mount, PointType::ToBody,
                                           damper, MechanismGraph::kGround });
    return out;
}

} // namespace

void TestGraphSolver::aStrutInLineKeepsItsPushrodOnItsLine()
{
    const InlineStrut strut = inlineStrut();
    QString error;
    const std::optional<CornerSolver> corner =
        CornerSolver::bind(strut.mechanism, strut.table, &error, std::nullopt, &strut.graph);
    QVERIFY2(corner, qPrintable(error));
    QVERIFY(corner->hasDamper());
    QVERIFY(corner->mobility().isDetermined());

    const Vec3 mount(20.0, 120.0, 620.0);
    const double pushrod = distance(Vec3(0.0, 500.0, 295.0),
                                    corner->designPose().damperOutboard);
    double before = corner->designPose().damperLength;
    for (double travel = 5.0; travel <= 30.0; travel += 5.0) {
        const CornerPose pose = corner->poseAtWheelTravel(travel);
        QVERIFY2(pose.valid, qPrintable(pose.error));
        const Vec3* outer = pose.find(QStringLiteral("F_PushRod_O"));
        QVERIFY(outer);
        // The pushrod keeps its length, and the three points stay on one line.
        QVERIFY(std::abs(distance(*outer, pose.damperOutboard) - pushrod) < 1e-9);
        const Vec3 offLine = cross(pose.damperOutboard - mount, *outer - mount);
        QVERIFY(offLine.length() / distance(*outer, mount) < 1e-9);
        // Bump compresses it.
        QVERIFY(pose.damperLength < before);
        before = pose.damperLength;
    }
}

void TestGraphSolver::aDamperOnTheWishboneIsMeasuredWithoutARocker()
{
    InlineStrut base = inlineStrut();
    MechanismGraph graph = roleGraph(base.mechanism, base.table);
    const int damper = graph.bodyFor(QStringLiteral("Damper"), true);
    const int lowerArm = graph.bodyFor(QStringLiteral("Lower wishbone"));
    const Vec3 onArm(0.0, 480.0, 125.0);
    const Vec3 mount(20.0, 160.0, 560.0);
    graph.points.push_back(
        GraphPoint{ QStringLiteral("F_Damper_O"), onArm, PointType::Solved, damper, lowerArm });
    graph.points.push_back(GraphPoint{ QStringLiteral("F_Damper_I"), mount, PointType::ToBody,
                                       damper, MechanismGraph::kGround });
    HardpointTable table = base.table;
    for (Hardpoint& point : table.points) {
        const Vec3* at = point.name == QLatin1String("F_Damper_O") ? &onArm
                         : point.name == QLatin1String("F_Damper_I") ? &mount : nullptr;
        if (!at) continue;
        point.coord[0] = at->x;
        point.coord[1] = at->y;
        point.coord[2] = at->z;
    }

    QString error;
    const std::optional<CornerSolver> corner =
        CornerSolver::bind(base.mechanism, table, &error, std::nullopt, &graph);
    QVERIFY2(corner, qPrintable(error));
    QVERIFY(corner->hasDamper());
    QVERIFY(!corner->hasRocker());

    const CornerPose pose = corner->poseAtWheelTravel(20.0);
    QVERIFY(pose.valid);
    // The damper's outer end turns with the lower wishbone, exactly.
    const Axis lowerAxis = axisThrough(Vec3(100.0, 200.0, 120.0), Vec3(-100.0, 200.0, 120.0));
    const Vec3 expected = rotateAbout(onArm, lowerAxis, pose.armAngle);
    QVERIFY(distance(pose.damperOutboard, expected) < 1e-9);
    QVERIFY(std::abs(pose.damperLength - distance(expected, mount)) < 1e-9);
}

QTEST_GUILESS_MAIN(TestGraphSolver)
#include "test_graph_solver.moc"
