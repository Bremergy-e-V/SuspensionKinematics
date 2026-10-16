#include "geom/MeshQuery.h"
#include "io/LinkageTemplate.h"
#include "model/HardpointConfig.h"
#include "model/MechanismGraph.h"
#include "model/PushrodDesign.h"
#include "model/SuspensionSolver.h"

#include <QTest>

#include <cmath>

using namespace suspkin;

namespace {

Hardpoint make(const char* name, double x, double y, double z)
{
    Hardpoint point;
    point.name = QLatin1String(name);
    point.coord[0] = x;
    point.coord[1] = y;
    point.coord[2] = z;
    return point;
}

/// The front left corner the solver tests use, rocker and all.
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

/// A chassis tub: a box from y -180 to +180, so a ray inboard from either side
/// meets its wall at |y| = 180.
TriMesh tub()
{
    const Vec3 min(-800.0, -180.0, 0.0);
    const Vec3 max(800.0, 180.0, 800.0);
    TriMesh mesh;
    for (int i = 0; i < 8; ++i)
        mesh.positions.push_back(QVector3D(float((i & 1) ? max.x : min.x),
                                           float((i & 2) ? max.y : min.y),
                                           float((i & 4) ? max.z : min.z)));
    const uint32_t faces[12][3] = { { 0, 2, 1 }, { 1, 2, 3 }, { 4, 5, 6 }, { 5, 7, 6 },
                                    { 0, 1, 4 }, { 1, 5, 4 }, { 2, 6, 3 }, { 3, 6, 7 },
                                    { 0, 4, 2 }, { 2, 4, 6 }, { 1, 3, 5 }, { 3, 7, 5 } };
    for (const auto& face : faces)
        for (const uint32_t index : face) mesh.indices.push_back(index);
    return mesh;
}

Vec3 at(const HardpointTable& table, const char* name)
{
    const Hardpoint* point = table.find(QLatin1String(name));
    return point ? Vec3(point->coord[0], point->coord[1], point->coord[2]) : Vec3(1e9, 1e9, 1e9);
}

PushrodDesign design(PushrodLayout layout, PickupArm arm = PickupArm::Upper)
{
    PushrodDesign out;
    out.corner = QStringLiteral("F");
    out.layout = layout;
    out.arm = arm;
    out.inboardHeight = 520.0;
    out.clearance = 10.0;
    out.outboardOffset = 40.0;
    out.damperLength = 200.0;
    return out;
}

/// The plan's table and settings, bound and solved the way an axle would.
std::optional<CornerSolver> solvePlanned(const PushrodPlan& plan, const HardpointConfigMap& base)
{
    const LinkageTemplate templ = builtinLinkageTemplate();
    HardpointConfigMap config = base;
    for (auto it = plan.config.constBegin(); it != plan.config.constEnd(); ++it)
        config.insert(it.key(), it.value());
    const HardpointTable& table = plan.points.table;
    const MechanismGraph graph =
        configGraph(cornerPointNames(templ, QStringLiteral("F"), false, MirrorSpec{}, table),
                    config, templ, table);
    MechanismTemplate roles = templ.mechanism;
    roles.steeringRack = roles.tieRodInboard;
    QString error;
    std::optional<CornerSolver> corner = CornerSolver::bind(
        instantiateMechanism(roles, QStringLiteral("F"), false, MirrorSpec{}, table), table,
        &error, std::nullopt, &graph);
    if (!corner) qWarning("did not bind: %s", qPrintable(error));
    return corner;
}

} // namespace

class TestPushrodDesign : public QObject {
    Q_OBJECT

private slots:
    void theChassisEndIsFoundInboardAtItsHeight();
    void withoutAChassisTheChassisEndIsSetAcrossTheCar();
    void theOutboardEndIsOnTheLineToTheChassisEnd();
    void aRockerLayoutPlacesOnlyThePushrodsOuterEnd();
    void aStrutInLineSolvesWithItsDamperMeasured();
    void aDamperAloneHangsOffTheWishbone();
    void aCustomOffsetPutsTheOutboardEndAnywhere();
    void theStrutsInnerEndAndEyeMoveAsOne();
    void theFarSideIsPlacedAndDescribedToo();
    void anOffsetPastTheChassisIsRefused();
    void designsRoundTripThroughJson();
};

void TestPushrodDesign::theChassisEndIsFoundInboardAtItsHeight()
{
    const MeshQuery chassis(tub());
    const PushrodPoints points = layOutPushrod(design(PushrodLayout::DamperOnly),
                                               Vec3(0.0, 540.0, 300.0), std::nullopt, &chassis);
    QVERIFY2(points.ok, qPrintable(points.error));
    QVERIFY(points.inboardOnChassis);
    // The wall at y = 180, stood 10 mm off it towards the wheel.
    QVERIFY(std::abs(points.inboard.y - 190.0) < 1e-6);
    QVERIFY(std::abs(points.inboard.z - 520.0) < 1e-9);
    QVERIFY(std::abs(points.inboard.x - 0.0) < 1e-9);
}

void TestPushrodDesign::withoutAChassisTheChassisEndIsSetAcrossTheCar()
{
    PushrodDesign d = design(PushrodLayout::DamperOnly);
    d.inboardY = 160.0;
    const PushrodPoints left = layOutPushrod(d, Vec3(0.0, 540.0, 300.0), std::nullopt, nullptr);
    QVERIFY(left.ok);
    QVERIFY(!left.inboardOnChassis);
    QCOMPARE(left.inboard.y, 160.0);
    const PushrodPoints right = layOutPushrod(d, Vec3(0.0, -540.0, 300.0), std::nullopt, nullptr);
    QCOMPARE(right.inboard.y, -160.0);
}

void TestPushrodDesign::theOutboardEndIsOnTheLineToTheChassisEnd()
{
    const Vec3 ballJoint(10.0, 540.0, 300.0);
    const PushrodPoints points =
        layOutPushrod(design(PushrodLayout::DamperOnly), ballJoint, std::nullopt, nullptr);
    QVERIFY(points.ok);
    // 40 mm inboard across the car...
    QVERIFY(std::abs((ballJoint.y - points.outboard.y) - 40.0) < 1e-9);
    // ...and on the line from the ball joint to the chassis end.
    const Vec3 off = cross(points.outboard - ballJoint, points.inboard - ballJoint);
    QVERIFY(off.length() < 1e-9);
}

void TestPushrodDesign::aRockerLayoutPlacesOnlyThePushrodsOuterEnd()
{
    const HardpointTable table = frontLeftCorner();
    const PushrodPlan plan = planPushrods({ design(PushrodLayout::Rocker) }, builtinLinkageTemplate(),
                                          MirrorSpec{}, table, table, nullptr);
    QVERIFY2(plan.ok(), qPrintable(plan.points.error));
    QVERIFY(plan.removed.isEmpty());
    QCOMPARE(plan.config.value(QStringLiteral("F_PushRod_O")).part2,
             QStringLiteral("Upper wishbone"));
    // On the line from the upper ball joint to the rocker's pushrod arm.
    const Vec3 outer = at(plan.points.table, "F_PushRod_O");
    const Vec3 off = cross(outer - at(table, "F_UCA_O"), at(table, "F_PushRod_I") - at(table, "F_UCA_O"));
    QVERIFY(off.length() < 1e-6);
    QCOMPARE(at(plan.points.table, "F_Rocker_Center").y, 150.0); // the rocker stays
}

void TestPushrodDesign::aStrutInLineSolvesWithItsDamperMeasured()
{
    const HardpointTable table = frontLeftCorner();
    const LinkageTemplate templ = builtinLinkageTemplate();
    const PushrodPlan plan = planPushrods({ design(PushrodLayout::InlineDamper) }, templ,
                                          MirrorSpec{}, table, table, nullptr);
    QVERIFY2(plan.ok(), qPrintable(plan.points.error));
    QVERIFY(plan.removed.contains(QStringLiteral("F_Rocker_Center")));
    QVERIFY(plan.unassigned.contains(QStringLiteral("F_ARB_DropLink_O")));
    QVERIFY(plan.optionalParts.contains(QStringLiteral("rocker")));
    QCOMPARE(plan.config.value(QStringLiteral("F_Damper_O")).type, PointType::Welded);

    // The eye is a damper's length from the chassis end, on the pushrod's line.
    const Vec3 eye = at(plan.points.table, "F_Damper_O");
    QVERIFY(std::abs(distance(eye, at(plan.points.table, "F_Damper_I")) - 200.0) < 1e-9);
    QCOMPARE(eye, at(plan.points.table, "F_PushRod_I"));

    const std::optional<CornerSolver> corner =
        solvePlanned(plan, inferHardpointConfig(table, templ, MirrorSpec{}));
    QVERIFY(corner);
    QVERIFY2(corner->mobility().isDetermined(),
             qPrintable(describeMobilityProblem(corner->mobility())));
    QVERIFY(corner->hasDamper());
    const CornerPose bump = corner->poseAtWheelTravel(20.0);
    QVERIFY(bump.valid);
    QVERIFY(bump.damperLength < corner->designPose().damperLength); // bump compresses it
}

void TestPushrodDesign::aCustomOffsetPutsTheOutboardEndAnywhere()
{
    PushrodDesign custom = design(PushrodLayout::InlineDamper);
    custom.customOutboard = true;
    custom.outboardShift = Vec3(15.0, 30.0, -10.0); // forward, inboard, down

    const Vec3 left(0.0, 540.0, 300.0);
    const PushrodPoints near = layOutPushrod(custom, left, std::nullopt, nullptr);
    QVERIFY2(near.ok, qPrintable(near.error));
    QCOMPARE(near.outboard, Vec3(15.0, 510.0, 290.0));
    // Inboard on the right is +y.
    const PushrodPoints far = layOutPushrod(custom, Vec3(0.0, -540.0, 300.0), std::nullopt, nullptr);
    QCOMPARE(far.outboard, Vec3(15.0, -510.0, 290.0));
    // The damper's eye is still on the pushrod's own line.
    const Vec3 along = (near.outboard - near.inboard).normalized();
    QVERIFY(distance(near.damperEye, near.inboard + along * custom.damperLength) < 1e-9);

    // Off the ball joint's line, the strut still solves.
    const HardpointTable table = frontLeftCorner();
    const LinkageTemplate templ = builtinLinkageTemplate();
    const PushrodPlan plan = planPushrods({ custom }, templ, MirrorSpec{}, table, table, nullptr);
    QVERIFY2(plan.ok(), qPrintable(plan.points.error));
    const std::optional<CornerSolver> corner =
        solvePlanned(plan, inferHardpointConfig(table, templ, MirrorSpec{}));
    QVERIFY(corner);
    QVERIFY2(corner->mobility().isDetermined(),
             qPrintable(describeMobilityProblem(corner->mobility())));

    const std::vector<PushrodDesign> back =
        pushrodDesignsFromJson(pushrodDesignsToJson({ custom }));
    QCOMPARE(back.size(), std::size_t(1));
    QVERIFY(back.front() == custom);
}

void TestPushrodDesign::theStrutsInnerEndAndEyeMoveAsOne()
{
    const HardpointTable table = frontLeftCorner();
    const LinkageTemplate templ = builtinLinkageTemplate();
    const MirrorSpec mirror;
    const PushrodPlan plan = planPushrods({ design(PushrodLayout::InlineDamper) }, templ, mirror,
                                          table, table, nullptr);
    QVERIFY2(plan.ok(), qPrintable(plan.points.error));
    const std::vector<QStringList> groups =
        pushrodGroups({ design(PushrodLayout::InlineDamper) }, templ, mirror, plan.points.table);
    QCOMPARE(groups.size(), std::size_t(2)); // both sides
    QCOMPARE(groups.front(), (QStringList{ QStringLiteral("F_PushRod_I"),
                                           QStringLiteral("F_Damper_O") }));
    QVERIFY(pushrodGroups({ design(PushrodLayout::Rocker) }, templ, mirror, plan.points.table)
                .empty());
}

void TestPushrodDesign::aDamperAloneHangsOffTheWishbone()
{
    const HardpointTable table = frontLeftCorner();
    const LinkageTemplate templ = builtinLinkageTemplate();
    const PushrodPlan plan =
        planPushrods({ design(PushrodLayout::DamperOnly, PickupArm::Lower) }, templ, MirrorSpec{},
                     table, table, nullptr);
    QVERIFY2(plan.ok(), qPrintable(plan.points.error));
    QVERIFY(plan.removed.contains(QStringLiteral("F_PushRod_O")));
    QVERIFY(plan.removed.contains(QStringLiteral("F_PushRod_I")));
    QVERIFY(plan.points.table.indexOf(QStringLiteral("F_PushRod_O")) < 0);
    QCOMPARE(plan.config.value(QStringLiteral("F_Damper_O")).part2,
             QStringLiteral("Lower wishbone"));

    const std::optional<CornerSolver> corner =
        solvePlanned(plan, inferHardpointConfig(table, templ, MirrorSpec{}));
    QVERIFY(corner);
    QVERIFY2(corner->mobility().isDetermined(),
             qPrintable(describeMobilityProblem(corner->mobility())));
    QVERIFY(corner->hasDamper());
    QVERIFY(!corner->hasRocker());
    QVERIFY(corner->poseAtWheelTravel(15.0).valid);
}

void TestPushrodDesign::theFarSideIsPlacedAndDescribedToo()
{
    const HardpointTable table = mirrorHardpoints(frontLeftCorner(), {}, MirrorSpec{}).table;
    const PushrodPlan plan = planPushrods({ design(PushrodLayout::InlineDamper) },
                                          builtinLinkageTemplate(), MirrorSpec{}, table, table,
                                          nullptr);
    QVERIFY2(plan.ok(), qPrintable(plan.points.error));
    const QString far = mirroredName(QStringLiteral("F_Damper_O"), MirrorSpec{});
    QCOMPARE(plan.config.value(far).type, PointType::Welded);
    const Vec3 near = at(plan.points.table, "F_Damper_O");
    const Hardpoint* mirrored = plan.points.table.find(far);
    QVERIFY(mirrored);
    QCOMPARE(mirrored->coord[1], -near.y);
    QVERIFY(plan.removed.contains(mirroredName(QStringLiteral("F_Rocker_Center"), MirrorSpec{})));
}

void TestPushrodDesign::anOffsetPastTheChassisIsRefused()
{
    PushrodDesign d = design(PushrodLayout::DamperOnly);
    d.outboardOffset = 1000.0;
    const PushrodPoints points = layOutPushrod(d, Vec3(0.0, 540.0, 300.0), std::nullopt, nullptr);
    QVERIFY(!points.ok);
    QVERIFY(!points.error.isEmpty());

    PushrodDesign tooLong = design(PushrodLayout::InlineDamper);
    tooLong.damperLength = 5000.0;
    QVERIFY(!layOutPushrod(tooLong, Vec3(0.0, 540.0, 300.0), std::nullopt, nullptr).ok);
}

void TestPushrodDesign::designsRoundTripThroughJson()
{
    PushrodDesign a = design(PushrodLayout::InlineDamper, PickupArm::Lower);
    a.inboardY = 123.0;
    PushrodDesign b = design(PushrodLayout::DamperOnly);
    b.corner = QStringLiteral("R");
    const std::vector<PushrodDesign> back = pushrodDesignsFromJson(pushrodDesignsToJson({ a, b }));
    QCOMPARE(back.size(), std::size_t(2));
    QVERIFY(back[0] == a);
    QVERIFY(back[1] == b);
}

QTEST_GUILESS_MAIN(TestPushrodDesign)
#include "test_pushrod_design.moc"
