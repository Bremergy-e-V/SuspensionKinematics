#include "geom/MeshQuery.h"
#include "geom/TriMesh.h"
#include "model/Collision.h"

#include <QTest>

#include <cmath>

using namespace suspkin;

namespace {

Hardpoint make(const QString& name, const Vec3& at)
{
    Hardpoint point;
    point.name = name;
    point.coord[0] = at.x;
    point.coord[1] = at.y;
    point.coord[2] = at.z;
    return point;
}

/// A part of one straight member between two rows.
LinkagePart link(const QString& label, int a, int b, PartKind kind = PartKind::Link)
{
    LinkagePart part;
    part.id = label;
    part.label = label;
    part.kind = kind;
    part.chains.push_back(ResolvedChain{ { a, b } });
    return part;
}

TriMesh box(const Vec3& min, const Vec3& max)
{
    TriMesh mesh;
    for (int i = 0; i < 8; ++i) {
        mesh.positions.push_back(QVector3D(float((i & 1) ? max.x : min.x),
                                           float((i & 2) ? max.y : min.y),
                                           float((i & 4) ? max.z : min.z)));
    }
    const uint32_t faces[12][3] = { { 0, 2, 1 }, { 1, 2, 3 }, { 4, 5, 6 }, { 5, 7, 6 },
                                    { 0, 1, 4 }, { 1, 5, 4 }, { 2, 6, 3 }, { 3, 6, 7 },
                                    { 0, 4, 2 }, { 2, 4, 6 }, { 1, 3, 5 }, { 3, 7, 5 } };
    for (const auto& face : faces)
        for (const uint32_t index : face) mesh.indices.push_back(index);
    for (const QVector3D& p : mesh.positions) mesh.bounds.expand(p);
    return mesh;
}

CollisionTarget part(const QString& id) { return CollisionTarget{ CollisionKind::Part, id, {} }; }
CollisionTarget rim(WheelCorner corner = WheelCorner::FrontLeft)
{
    return CollisionTarget{ CollisionKind::Rim, {}, corner };
}
CollisionTarget tyre(WheelCorner corner = WheelCorner::FrontLeft)
{
    return CollisionTarget{ CollisionKind::Tyre, {}, corner };
}
CollisionTarget chassis() { return CollisionTarget{ CollisionKind::Chassis, {}, {} }; }

/// A frame with the front left wheel at @p center.
CollisionFrame withWheel(std::vector<Hardpoint> points, const QVector3D& center,
                         const QQuaternion& rotation = {}, bool mirrored = false)
{
    CollisionFrame frame;
    frame.table.points = std::move(points);
    frame.wheels.push_back(WheelPlacement{ WheelCorner::FrontLeft, QStringLiteral("FL_WC"), center,
                                           rotation, mirrored });
    return frame;
}

bool near(double a, double b, double tolerance = 1e-3) { return std::abs(a - b) < tolerance; }

} // namespace

class TestCollision : public QObject {
    Q_OBJECT

private slots:
    void segmentsAreMeasuredBetweenTheirClosestPoints();
    void twoMembersCrossingMidTravelTouch();
    void partsJoinedAtAPointAreComparedAwayFromIt();
    void partsJoinedThroughAThirdAreComparedAwayFromIt();
    void aMemberIsComparedWithTheChassisAwayFromItsMounts();
    void aMemberIsMeasuredToTheRimWhereTheWheelIsTurned();
    void aMirroredRimIsMeasuredAsItsMirrorImage();
    void twoMeshesAreMeasuredBetweenTheirSurfaces();
    void whatCannotBeMeasuredSaysWhy();
    void theTargetsOfferedAreWhatTheProjectHas();
    void targetKeysRoundTrip();
    void settingsRoundTrip();
};

void TestCollision::segmentsAreMeasuredBetweenTheirClosestPoints()
{
    // Skew, crossing over one another 10 apart.
    QCOMPARE(segmentDistance(Vec3(-1, 0, 0), Vec3(1, 0, 0), Vec3(0, -1, 10), Vec3(0, 1, 10)), 10.0);
    // Parallel and overlapping.
    QCOMPARE(segmentDistance(Vec3(0, 0, 0), Vec3(10, 0, 0), Vec3(5, 3, 0), Vec3(15, 3, 0)), 3.0);
    // End to end, past each other's ends.
    QCOMPARE(segmentDistance(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(4, 4, 0), Vec3(4, 5, 0)), 5.0);
    // A point.
    QCOMPARE(segmentDistance(Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(-1, 2, 0), Vec3(1, 2, 0)), 2.0);
}

void TestCollision::twoMembersCrossingMidTravelTouch()
{
    // A link that rises through another as the travel goes from -10 to +10:
    // 30 mm apart at the start, crossing at +10.
    std::vector<CollisionFrame> frames;
    for (int step = -10; step <= 10; step += 5) {
        CollisionFrame frame;
        frame.input = step;
        frame.table.points = { make("A0", Vec3(-100, 0, 0)), make("A1", Vec3(100, 0, 0)),
                               make("B0", Vec3(0, -100, 20.0 - 2.0 * step)),
                               make("B1", Vec3(0, 100, 20.0 - 2.0 * step)) };
        frames.push_back(frame);
    }
    Linkage linkage;
    linkage.parts = { link("a", 0, 1), link("b", 2, 3) };

    CollisionSettings settings;
    settings.memberRadius = 5.0;
    const CollisionResult result = minimumDistance(linkage, frames, std::vector<bool>(4, false),
                                                   CollisionMeshes{}, settings, part("a"), part("b"));
    QVERIFY2(result.valid, qPrintable(result.problem));
    QCOMPARE(result.positions, 5);
    QVERIFY(result.touches());
    QCOMPARE(result.minDistance, -10.0); // axes meet, less two radii
    QCOMPARE(result.atInput, 10.0);
    QVERIFY(result.designKnown);
    QCOMPARE(result.designDistance, 10.0); // 20 apart, less two radii
}

void TestCollision::partsJoinedAtAPointAreComparedAwayFromIt()
{
    // Two links bolted together at Q, at right angles: they touch at the bolt,
    // which is the joint's business, and are measured from the joint clearance
    // out. b and c meet at a different row in the same place, as a strut's
    // pushrod and damper do.
    CollisionFrame frame;
    frame.table.points = { make("P", Vec3(0, 0, 0)), make("Q", Vec3(100, 0, 0)),
                           make("R", Vec3(100, 50, 0)), make("S", Vec3(100, 50, 0)),
                           make("T", Vec3(100, 50, 100)) };
    Linkage linkage;
    linkage.parts = { link("a", 0, 1), link("b", 1, 2), link("c", 3, 4) };

    CollisionSettings settings;
    settings.memberRadius = 0.0;
    settings.jointClearance = 30.0;
    const std::vector<bool> grounded(5, false);
    const CollisionResult ab =
        minimumDistance(linkage, { frame }, grounded, {}, settings, part("a"), part("b"));
    QVERIFY2(ab.valid, qPrintable(ab.problem));
    // Closest: a's end 30 short of Q, and b's start 30 past it.
    QVERIFY2(near(ab.minDistance, std::sqrt(2.0) * 30.0), qPrintable(QString::number(ab.minDistance)));

    const CollisionResult bc =
        minimumDistance(linkage, { frame }, grounded, {}, settings, part("b"), part("c"));
    QVERIFY2(bc.valid, qPrintable(bc.problem));
    QVERIFY(bc.minDistance > 0.0);
}

void TestCollision::partsJoinedThroughAThirdAreComparedAwayFromIt()
{
    // A pushrod picking up 15 mm off a wishbone's ball joint through a short
    // bracket: right beside the wishbone there, which is the joint's business,
    // and clear of it further in.
    CollisionFrame frame;
    frame.table.points = {
        make("Arm_I", Vec3(0, 0, 0)),        make("Arm_O", Vec3(300, 0, 0)),
        make("Rod_O", Vec3(285, 0, 8)),      make("Rod_I", Vec3(100, 0, 200)),
    };
    Linkage linkage;
    linkage.parts = { link("arm", 0, 1), link("bracket", 1, 2), link("rod", 2, 3) };

    CollisionSettings settings;
    settings.memberRadius = 5.0;
    settings.jointClearance = 30.0;
    const std::vector<bool> grounded(4, false);
    const CollisionResult clear =
        minimumDistance(linkage, { frame }, grounded, {}, settings, part("arm"), part("rod"));
    QVERIFY2(clear.valid, qPrintable(clear.problem));
    QVERIFY2(clear.minDistance > 5.0, qPrintable(QString::number(clear.minDistance)));

    // Without the joint left out, they would read as touching.
    settings.jointClearance = 0.0;
    QVERIFY(minimumDistance(linkage, { frame }, grounded, {}, settings, part("arm"), part("rod"))
                .touches());
}

void TestCollision::aMemberIsComparedWithTheChassisAwayFromItsMounts()
{
    // A tub from y = -200 to 200; a link from a pickup on its side outward,
    // and another that dips back towards it as the travel goes on.
    const TriMesh tub = box(Vec3(-500, -200, 0), Vec3(500, 200, 300));
    const MeshQuery query(tub);
    CollisionMeshes meshes;
    meshes.chassis = { &tub, &query };

    std::vector<CollisionFrame> frames;
    for (const double input : { 0.0, 10.0 }) {
        CollisionFrame frame;
        frame.input = input;
        frame.table.points = {
            make("Pivot", Vec3(0, 200, 150)), make("Outer", Vec3(0, 500, 150)),
            make("Far_I", Vec3(200, 400, 100)), make("Far_O", Vec3(200, 600, 100)),
        };
        // At 10 the second link's inner end comes to 8 mm off the tub.
        if (input > 0.0) frame.table.points[2].coord[1] = 208.0;
        frames.push_back(frame);
    }
    Linkage linkage;
    linkage.parts = { link("arm", 0, 1), link("other", 2, 3) };
    const std::vector<bool> grounded = { true, false, false, false };

    CollisionSettings settings;
    settings.memberRadius = 5.0;
    const CollisionResult arm =
        minimumDistance(linkage, frames, grounded, meshes, settings, part("arm"), chassis());
    QVERIFY2(arm.valid, qPrintable(arm.problem));
    // Bolted to the tub at its pivot, so measured from the joint clearance out.
    QVERIFY(near(arm.minDistance, settings.jointClearance - 5.0, 1e-6));

    // Either way round.
    const CollisionResult other =
        minimumDistance(linkage, frames, grounded, meshes, settings, chassis(), part("other"));
    QVERIFY2(other.valid, qPrintable(other.problem));
    QVERIFY(near(other.minDistance, 3.0, 1e-6));
    QCOMPARE(other.atInput, 10.0);
    QVERIFY(near(other.designDistance, 195.0, 1e-6));
}

void TestCollision::aMemberIsMeasuredToTheRimWhereTheWheelIsTurned()
{
    // A rim 100 wide along x and 40 along y, about its own centre, at
    // (0, 600, 0); a rod standing upright beside it at x = 100.
    const TriMesh model = box(Vec3(-50, -20, -50), Vec3(50, 20, 50));
    const MeshQuery query(model);
    CollisionMeshes meshes;
    meshes.rim = { &model, &query };
    const std::vector<Hardpoint> points = { make("Rod_L", Vec3(100, 600, -200)),
                                            make("Rod_U", Vec3(100, 600, 200)) };
    Linkage linkage;
    linkage.parts = { link("rod", 0, 1) };
    CollisionSettings settings;
    settings.memberRadius = 5.0;

    CollisionFrame straight = withWheel(points, QVector3D(0, 600, 0));
    straight.input = 0.0;
    // Turned a quarter about z, its 40 now lies along x.
    CollisionFrame turned =
        withWheel(points, QVector3D(0, 600, 0), QQuaternion::fromAxisAndAngle(0, 0, 1, 90));
    turned.input = 20.0;

    const CollisionResult result = minimumDistance(linkage, { straight, turned },
                                                   std::vector<bool>(2, false), meshes, settings,
                                                   part("rod"), rim());
    QVERIFY2(result.valid, qPrintable(result.problem));
    QVERIFY2(near(result.minDistance, 45.0), qPrintable(QString::number(result.minDistance)));
    QCOMPARE(result.atInput, 0.0);

    const CollisionResult onlyTurned = minimumDistance(
        linkage, { turned }, std::vector<bool>(2, false), meshes, settings, part("rod"), rim());
    QVERIFY2(near(onlyTurned.minDistance, 75.0), qPrintable(QString::number(onlyTurned.minDistance)));
}

void TestCollision::aMirroredRimIsMeasuredAsItsMirrorImage()
{
    // A rim drawn entirely on +y of its origin, placed by that origin: dished.
    // On the far side it is the mirror image, reaching to -y instead.
    const TriMesh model = box(Vec3(-50, 0, -50), Vec3(50, 40, 50));
    const MeshQuery query(model);
    CollisionMeshes meshes;
    meshes.rim = { &model, &query };
    meshes.alignToCenter = false;
    const std::vector<Hardpoint> points = { make("Rod_A", Vec3(-200, 540, 0)),
                                            make("Rod_B", Vec3(200, 540, 0)) };
    Linkage linkage;
    linkage.parts = { link("rod", 0, 1) };
    CollisionSettings settings;
    settings.memberRadius = 5.0;
    const std::vector<bool> grounded(2, false);

    const CollisionResult drawn =
        minimumDistance(linkage, { withWheel(points, QVector3D(0, 600, 0)) }, grounded, meshes,
                        settings, part("rod"), rim());
    QVERIFY2(near(drawn.minDistance, 55.0), qPrintable(QString::number(drawn.minDistance)));

    const CollisionResult mirrored =
        minimumDistance(linkage, { withWheel(points, QVector3D(0, 600, 0), {}, true) }, grounded,
                        meshes, settings, part("rod"), rim());
    QVERIFY2(near(mirrored.minDistance, 15.0), qPrintable(QString::number(mirrored.minDistance)));
}

void TestCollision::twoMeshesAreMeasuredBetweenTheirSurfaces()
{
    const TriMesh tub = box(Vec3(-500, -200, 0), Vec3(500, 200, 300));
    const MeshQuery tubQuery(tub);
    const TriMesh model = box(Vec3(-50, -20, -50), Vec3(50, 20, 50));
    const MeshQuery rimQuery(model);
    CollisionMeshes meshes;
    meshes.chassis = { &tub, &tubQuery };
    meshes.rim = { &model, &rimQuery };

    const CollisionResult result =
        minimumDistance(Linkage{}, { withWheel({}, QVector3D(0, 600, 100)) }, {}, meshes,
                        CollisionSettings{}, rim(), chassis());
    QVERIFY2(result.valid, qPrintable(result.problem));
    // The rim's inner face at y = 580, the tub's side at y = 200.
    QVERIFY2(near(result.minDistance, 380.0), qPrintable(QString::number(result.minDistance)));
}

void TestCollision::whatCannotBeMeasuredSaysWhy()
{
    const TriMesh model = box(Vec3(-50, -20, -50), Vec3(50, 20, 50));
    const MeshQuery query(model);
    CollisionMeshes withRimAndTyre;
    withRimAndTyre.rim = { &model, &query };
    withRimAndTyre.tyre = { &model, &query };

    const std::vector<Hardpoint> points = { make("A", Vec3(0, 0, 0)), make("B", Vec3(100, 0, 0)) };
    Linkage linkage;
    linkage.parts = { link("rod", 0, 1), link("wheel", 0, 1, PartKind::Wheel) };
    const std::vector<bool> grounded(2, false);
    const std::vector<CollisionFrame> frames = { withWheel(points, QVector3D(0, 600, 0)) };
    const auto measure = [&](const CollisionMeshes& meshes, const CollisionTarget& a,
                             const CollisionTarget& b) {
        return minimumDistance(linkage, frames, grounded, meshes, CollisionSettings{}, a, b);
    };

    const auto refused = [](const CollisionResult& result) {
        return !result.valid && !result.problem.isEmpty();
    };
    QVERIFY(refused(measure(withRimAndTyre, part("rod"), part("rod"))));
    QVERIFY(refused(measure(withRimAndTyre, part("rod"), part("nothing"))));
    QVERIFY(refused(measure(withRimAndTyre, part("wheel"), rim()))); // no member
    QVERIFY(refused(measure(CollisionMeshes{}, part("rod"), rim())));
    QVERIFY(refused(measure(withRimAndTyre, part("rod"), chassis())));
    QVERIFY(refused(measure(withRimAndTyre, rim(), tyre())));
    // No wheel at that corner in any frame.
    QVERIFY(refused(measure(withRimAndTyre, part("rod"), rim(WheelCorner::RearRight))));
    // A part against a rim that is there is measured.
    QVERIFY(!refused(measure(withRimAndTyre, part("rod"), rim())));
}

void TestCollision::theTargetsOfferedAreWhatTheProjectHas()
{
    const TriMesh model = box(Vec3(-50, -20, -50), Vec3(50, 20, 50));
    const MeshQuery query(model);
    Linkage linkage;
    linkage.parts = { link("rod", 0, 1), link("wheel", 0, 1, PartKind::Wheel) };
    WheelSpec spec;
    spec.setPoint(WheelCorner::FrontLeft, QStringLiteral("FL_WC"));
    spec.setPoint(WheelCorner::RearRight, QStringLiteral("RR_WC"));

    CollisionMeshes meshes;
    meshes.rim = { &model, &query };
    const std::vector<CollisionTarget> targets = collisionTargets(linkage, spec, meshes);
    // The rod, and a rim at the two corners named; no tyre, no chassis, and
    // no wheel construction line.
    QCOMPARE(targets.size(), std::size_t(3));
    QVERIFY(targets[0] == part("rod"));
    QVERIFY(targets[1] == rim(WheelCorner::FrontLeft));
    QVERIFY(targets[2] == rim(WheelCorner::RearRight));
    QCOMPARE(collisionTargetLabel(targets[0], linkage), QStringLiteral("rod"));
    QVERIFY(collisionTargetLabel(targets[1], linkage).contains(QStringLiteral("rim")));
}

void TestCollision::targetKeysRoundTrip()
{
    for (const CollisionTarget& target :
         { part(QStringLiteral("tieRod@F:base")), rim(WheelCorner::RearLeft),
           tyre(WheelCorner::FrontRight), chassis(), part(QStringLiteral("rim")) }) {
        const std::optional<CollisionTarget> back = collisionTargetFromKey(collisionTargetKey(target));
        QVERIFY(back.has_value());
        QVERIFY2(*back == target, qPrintable(collisionTargetKey(target)));
    }
    QVERIFY(!collisionTargetFromKey(QString()).has_value());
}

void TestCollision::settingsRoundTrip()
{
    CollisionSettings settings;
    settings.memberRadius = 7.5;
    settings.jointClearance = 25.0;
    settings.first = QStringLiteral("tieRod@F:base");
    settings.second = collisionTargetKey(rim());
    QVERIFY(collisionSettingsFromJson(collisionSettingsToJson(settings)) == settings);
    QVERIFY(collisionSettingsFromJson(QJsonObject{}) == CollisionSettings{});
}

QTEST_MAIN(TestCollision)
#include "test_collision.moc"
