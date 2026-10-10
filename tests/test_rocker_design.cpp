#include "io/LinkageTemplate.h"
#include "model/GeomSolve.h"
#include "model/HardpointMirror.h"
#include "model/Linkage.h"
#include "model/Mechanism.h"
#include "model/RockerDesign.h"

#include <QJsonArray>
#include <QTest>

#include <cmath>

using namespace suspkin;

namespace {

bool near(double a, double b, double tolerance = 1e-9) { return std::abs(a - b) <= tolerance; }

bool near(const Vec3& a, const Vec3& b, double tolerance = 1e-9)
{
    return distance(a, b) <= tolerance;
}

/// The angle between two directions, in degrees.
double angleBetween(const Vec3& a, const Vec3& b)
{
    const double c = dot(a.normalized(), b.normalized());
    return std::acos(std::max(-1.0, std::min(1.0, c))) * 180.0 / M_PI;
}

RockerDesign front()
{
    RockerDesign d;
    d.corner = QStringLiteral("F");
    d.pivot = Vec3(-150.0, 160.0, 480.0);
    d.axisYaw = 8.0;
    d.axisPitch = -5.0;
    d.pushrodArm = 72.0;
    d.pushrodArmAngle = 25.0;
    d.damperArm = 58.0;
    d.damperArmAngle = 110.0;
    d.damperAngle = 80.0;
    d.damperLength = 205.0;
    d.antiRollArmUsed = true;
    d.antiRollArm = 45.0;
    d.antiRollArmAngle = -70.0;
    return d;
}

MechanismTemplate namesFor(const QString& corner, bool mirrored)
{
    return instantiateMechanism(builtinLinkageTemplate().mechanism, corner, mirrored, MirrorSpec{});
}

} // namespace

class TestRockerDesign : public QObject {
    Q_OBJECT

private slots:
    void theArmsHaveTheLengthsAndAnglesAskedFor();
    void positiveTurnsInboardOnEitherSide();
    void theWholeRockerTurnsWithItsPushrodArm();
    void bothSidesArePlacedUnderTheTemplatesNames();
    void anExistingRockerReadsBackAsItsDesign();
    void theRockerIsOneRigidGroupPerSide();
    void thePivotFollowsTheTable();
    void designsRoundTripThroughTheManifest();
    void aRockerWithNoLengthIsRefused();
};

void TestRockerDesign::theArmsHaveTheLengthsAndAnglesAskedFor()
{
    const RockerDesign d = front();
    const RockerPoints p = layOutRocker(d);
    QVERIFY2(p.ok, qPrintable(p.error));

    const Vec3 axis = (p.axisPoint - p.pivot).normalized();
    const Vec3 l1 = p.pushrodInner - p.pivot;
    const Vec3 l2 = p.damperEye - p.pivot;
    const Vec3 l3 = p.antiRollPickup - p.pivot;
    const Vec3 damper = p.damperMount - p.damperEye;

    QVERIFY(near(l1.length(), 72.0));
    QVERIFY(near(l2.length(), 58.0));
    QVERIFY(near(l3.length(), 45.0));
    QVERIFY(near(damper.length(), 205.0));
    // Everything in the rocker's plane.
    for (const Vec3& v : { l1, l2, l3, damper }) QVERIFY(near(dot(v, axis), 0.0));
    // The angles between the arms, and beta at the eye.
    QVERIFY(near(angleBetween(l1, l2), 110.0, 1e-7));
    QVERIFY(near(angleBetween(l1, l3), 70.0, 1e-7));
    QVERIFY(near(angleBetween(l2 * -1.0, damper), 80.0, 1e-7));
    // The axis where it was asked for.
    QVERIFY(near(std::atan2(axis.y, axis.x) * 180.0 / M_PI, 8.0, 1e-9));
    QVERIFY(near(std::asin(axis.z) * 180.0 / M_PI, -5.0, 1e-9));
}

void TestRockerDesign::positiveTurnsInboardOnEitherSide()
{
    RockerDesign left = front();
    left.axisYaw = 0.0;
    left.axisPitch = 0.0;
    left.pushrodArmAngle = 30.0;
    const RockerPoints l = layOutRocker(left);
    QVERIFY(l.pushrodInner.y < left.pivot.y); // toward the centreline
    QVERIFY(l.pushrodInner.z > left.pivot.z);

    RockerDesign right = left;
    right.pivot.y = -left.pivot.y;
    const RockerPoints r = layOutRocker(right);
    QVERIFY(r.pushrodInner.y > right.pivot.y);
    // The mirror image of the left, exactly.
    QVERIFY(near(r.pushrodInner.y, -l.pushrodInner.y));
    QVERIFY(near(r.damperMount.z, l.damperMount.z));
}

void TestRockerDesign::theWholeRockerTurnsWithItsPushrodArm()
{
    // Turning L1 turns the rocker as a body: every point the same angle about
    // the axis, every distance on it unchanged.
    RockerDesign d = front();
    const RockerPoints before = layOutRocker(d);
    d.pushrodArmAngle += 15.0;
    const RockerPoints after = layOutRocker(d);
    const Axis axis = axisThrough(before.pivot, before.axisPoint);
    const double turn = std::acos(-1.0) / 180.0 * 15.0;
    // Inboard on the left is right-handed about a forward axis.
    for (const auto& [was, now] : { std::pair{ before.pushrodInner, after.pushrodInner },
                                    std::pair{ before.damperEye, after.damperEye },
                                    std::pair{ before.damperMount, after.damperMount },
                                    std::pair{ before.antiRollPickup, after.antiRollPickup } })
        QVERIFY(near(rotateAbout(was, axis, turn), now, 1e-9));
}

void TestRockerDesign::bothSidesArePlacedUnderTheTemplatesNames()
{
    const LinkageTemplate templ = builtinLinkageTemplate();
    const DesignPlan plan = planRockers({ front() }, templ, MirrorSpec{}, HardpointTable{}, HardpointTable{});
    QVERIFY2(plan.ok(), qPrintable(plan.error));

    const MechanismTemplate left = namesFor(QStringLiteral("F"), false);
    const MechanismTemplate right = namesFor(QStringLiteral("F"), true);
    for (const MechanismTemplate* names : { &left, &right }) {
        for (const QString& name : { names->rockerPivot, names->rockerAxis, names->pushrodInner,
                                     names->damperOutboard, names->damperInboard })
            QVERIFY2(plan.table.find(name), qPrintable(name));
    }
    // The pushrod's outer end is on a wishbone: placed by hand, never here.
    QVERIFY(!plan.table.find(left.pushrodOuter));
    // The bar hangs off the rocker in the built-in template, so its pickup is
    // written when asked for...
    QVERIFY(plan.table.find(left.antiRollDropLinkOuter));
    // ...and not when not.
    RockerDesign without = front();
    without.antiRollArmUsed = false;
    const DesignPlan bare = planRockers({ without }, templ, MirrorSpec{}, HardpointTable{}, HardpointTable{});
    QVERIFY(!bare.table.find(left.antiRollDropLinkOuter));

    const Hardpoint* pivot = plan.table.find(left.rockerPivot);
    const Hardpoint* mirrored = plan.table.find(right.rockerPivot);
    QVERIFY(pivot && mirrored);
    QCOMPARE(mirrored->coord[1], -pivot->coord[1]);
}

void TestRockerDesign::anExistingRockerReadsBackAsItsDesign()
{
    const LinkageTemplate templ = builtinLinkageTemplate();
    const RockerDesign d = front();
    const DesignPlan plan = planRockers({ d }, templ, MirrorSpec{}, HardpointTable{}, HardpointTable{});
    QVERIFY2(plan.ok(), qPrintable(plan.error));

    const std::optional<RockerDesign> read =
        measureRocker(QStringLiteral("F"), templ, MirrorSpec{}, plan.table, RockerDesign{});
    QVERIFY(read.has_value());
    QVERIFY(near(read->pivot, d.pivot));
    QVERIFY(near(read->axisYaw, d.axisYaw, 1e-7));
    QVERIFY(near(read->axisPitch, d.axisPitch, 1e-7));
    QVERIFY(near(read->pushrodArm, d.pushrodArm, 1e-7));
    QVERIFY(near(read->pushrodArmAngle, d.pushrodArmAngle, 1e-7));
    QVERIFY(near(read->damperArm, d.damperArm, 1e-7));
    QVERIFY(near(read->damperArmAngle, d.damperArmAngle, 1e-7));
    QVERIFY(near(read->damperAngle, d.damperAngle, 1e-7));
    QVERIFY(near(read->damperLength, d.damperLength, 1e-7));
    QVERIFY(read->antiRollArmUsed);
    QVERIFY(near(read->antiRollArmAngle, d.antiRollArmAngle, 1e-7));

    // Placing what was read back moves nothing.
    const DesignPlan again = planRockers({ *read }, templ, MirrorSpec{}, plan.table, plan.table);
    QVERIFY2(again.ok(), qPrintable(again.error));
    for (const DesignChange& change : again.changes)
        QVERIFY2(change.distance < 1e-6, qPrintable(change.name));

    // A table with no rocker has nothing to read.
    QVERIFY(!measureRocker(QStringLiteral("F"), templ, MirrorSpec{}, HardpointTable{}, d));
}

void TestRockerDesign::theRockerIsOneRigidGroupPerSide()
{
    const LinkageTemplate templ = builtinLinkageTemplate();
    const DesignPlan plan = planRockers({ front() }, templ, MirrorSpec{}, HardpointTable{}, HardpointTable{});
    QVERIFY2(plan.ok(), qPrintable(plan.error));

    const std::vector<QStringList> groups = rockerGroups({ front() }, templ, MirrorSpec{}, plan.table);
    QCOMPARE(int(groups.size()), 2);
    const MechanismTemplate left = namesFor(QStringLiteral("F"), false);
    // The pivot, its axis point, the three arms' ends, and both ends of the
    // damper: the damper's chassis mount goes with the rocker.
    QCOMPARE(groups[0].size(), 6);
    QVERIFY(groups[0].contains(left.damperInboard));
    QVERIFY(groups[0].contains(left.rockerAxis));
    QVERIFY(!groups[0].contains(left.pushrodOuter));

    // A corner nobody designed is not a part.
    QVERIFY(rockerGroups({}, templ, MirrorSpec{}, plan.table).empty());
}

void TestRockerDesign::thePivotFollowsTheTable()
{
    const LinkageTemplate templ = builtinLinkageTemplate();
    const DesignPlan plan = planRockers({ front() }, templ, MirrorSpec{}, HardpointTable{}, HardpointTable{});
    HardpointTable moved = plan.table;
    const MechanismTemplate left = namesFor(QStringLiteral("F"), false);
    Hardpoint& pivot = moved.points[std::size_t(moved.indexOf(left.rockerPivot))];
    pivot.coord[2] += 12.0;

    const RockerDesign followed = followTable(front(), templ, MirrorSpec{}, moved);
    QVERIFY(near(followed.pivot.z, front().pivot.z + 12.0));
    QCOMPARE(followed.pushrodArm, front().pushrodArm);
}

void TestRockerDesign::designsRoundTripThroughTheManifest()
{
    RockerDesign rear = front();
    rear.corner = QStringLiteral("R");
    rear.antiRollArmUsed = false;
    const std::vector<RockerDesign> designs{ front(), rear };
    QCOMPARE(rockerDesignsFromJson(rockerDesignsToJson(designs)), designs);
    // A key a later release adds, or an older manifest lacks, reads as its
    // default.
    QJsonArray partial = rockerDesignsToJson({ front() });
    QJsonObject first = partial[0].toObject();
    first.remove(QStringLiteral("damperLength"));
    partial[0] = first;
    QCOMPARE(rockerDesignsFromJson(partial).front().damperLength, RockerDesign{}.damperLength);
}

void TestRockerDesign::aRockerWithNoLengthIsRefused()
{
    RockerDesign d = front();
    d.damperLength = 0.0;
    QVERIFY(!layOutRocker(d).ok);
    d = front();
    d.axisPitch = 89.0;
    QVERIFY(!layOutRocker(d).ok);

    const DesignPlan plan = planRockers({ d }, builtinLinkageTemplate(), MirrorSpec{},
                                        HardpointTable{}, HardpointTable{});
    QVERIFY(!plan.ok());
    QVERIFY(!plan.error.isEmpty());
}

QTEST_APPLESS_MAIN(TestRockerDesign)
#include "test_rocker_design.moc"
