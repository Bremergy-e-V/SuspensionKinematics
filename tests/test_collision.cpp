#include "geom/MeshQuery.h"
#include "geom/TriMesh.h"
#include "model/Collision.h"

#include <QTest>

#include <QtMath>

#include <cmath>
#include <functional>

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

CollisionTarget part(const QString& id) { return CollisionTarget{ CollisionKind::Part, id, {}, {}, {} }; }
CollisionTarget tube(const QString& id, const QString& from, const QString& to)
{
    return CollisionTarget{ CollisionKind::Member, id, from, to, {} };
}
CollisionTarget rim(WheelCorner corner = WheelCorner::FrontLeft)
{
    return CollisionTarget{ CollisionKind::Rim, {}, {}, {}, corner };
}
CollisionTarget tyre(WheelCorner corner = WheelCorner::FrontLeft)
{
    return CollisionTarget{ CollisionKind::Tyre, {}, {}, {}, corner };
}
CollisionTarget chassis() { return CollisionTarget{ CollisionKind::Chassis, {}, {}, {}, {} }; }

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

/// No travel at all: the car only at ride height, straight ahead.
const CollisionTravel kStill{ 0.0, 0.0, 0.0 };

/// A car that is @p frame wherever it is asked to be.
CollisionFrameSource fixed(const CollisionFrame& frame)
{
    return [frame](const CollisionPosition& position) {
        CollisionFrame at = frame;
        at.position = position;
        return std::optional<CollisionFrame>(at);
    };
}

/// A car whose points @p points makes for each position.
CollisionFrameSource moving(std::function<std::vector<Hardpoint>(const CollisionPosition&)> points)
{
    return [points](const CollisionPosition& position) {
        CollisionFrame frame;
        frame.position = position;
        frame.table.points = points(position);
        return std::optional<CollisionFrame>(frame);
    };
}

bool near(double a, double b, double tolerance = 1e-3) { return std::abs(a - b) < tolerance; }

} // namespace

class TestCollision : public QObject {
    Q_OBJECT

private slots:
    void segmentsAreMeasuredBetweenTheirClosestPoints();
    void twoMembersCrossingInBumpTouch();
    void theClosestPlaceIsFoundBetweenTheGridsPoints();
    void eachTubeHasItsOwnRadiusAndSaysWhereItIsClosest();
    void oneTubeIsMeasuredWithoutTheRestOfItsPart();
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

void TestCollision::twoMembersCrossingInBumpTouch()
{
    // A link that comes down through another as the wheel rises: 20 mm apart
    // at ride height, crossing at 10 mm of bump. The rack changes nothing.
    const CollisionFrameSource source = moving([](const CollisionPosition& at) {
        const double z = 20.0 - 2.0 * at.travel;
        return std::vector<Hardpoint>{ make("A0", Vec3(-100, 0, 0)), make("A1", Vec3(100, 0, 0)),
                                       make("B0", Vec3(0, -100, z)), make("B1", Vec3(0, 100, z)) };
    });
    Linkage linkage;
    linkage.parts = { link("a", 0, 1), link("b", 2, 3) };

    CollisionSettings settings;
    settings.memberRadius = 5.0;
    const CollisionResult result =
        minimumDistance(linkage, std::vector<bool>(4, false), CollisionMeshes{}, settings,
                        part("a"), part("b"), CollisionTravel{ 20.0, 10.0, 10.0 }, source);
    QVERIFY2(result.valid, qPrintable(result.problem));
    QVERIFY(result.touches());
    QCOMPARE(result.overTravel.distance, -10.0); // axes meet, less two radii
    QCOMPARE(result.overTravel.at.travel, 10.0);
    QCOMPARE(result.rideHeight.distance, 10.0); // 20 apart, less two radii
    QCOMPARE(result.rideHeight.at.travel, 0.0);
    QVERIFY(result.design.valid);
    QCOMPARE(result.design.distance, 10.0);
    QVERIFY(result.positions > 1);
}

void TestCollision::theClosestPlaceIsFoundBetweenTheGridsPoints()
{
    // Two links whose gap is least at 7.3 mm of rack and 3.7 mm of bump,
    // neither of them on the coarse grid.
    const CollisionFrameSource source = moving([](const CollisionPosition& at) {
        const double z = 20.0 + 0.05 * (std::pow(at.rack - 7.3, 2.0) + std::pow(at.travel - 3.7, 2.0));
        return std::vector<Hardpoint>{ make("A0", Vec3(-100, 0, 0)), make("A1", Vec3(100, 0, 0)),
                                       make("B0", Vec3(0, -100, z)), make("B1", Vec3(0, 100, z)) };
    });
    Linkage linkage;
    linkage.parts = { link("a", 0, 1), link("b", 2, 3) };
    CollisionSettings settings;
    settings.memberRadius = 0.0;

    const CollisionResult result =
        minimumDistance(linkage, std::vector<bool>(4, false), CollisionMeshes{}, settings,
                        part("a"), part("b"), CollisionTravel{ 30.0, 25.0, 25.0 }, source);
    QVERIFY2(result.valid, qPrintable(result.problem));
    QVERIFY2(near(result.overTravel.at.rack, 7.3, 0.1),
             qPrintable(QString::number(result.overTravel.at.rack)));
    QVERIFY2(near(result.overTravel.at.travel, 3.7, 0.1),
             qPrintable(QString::number(result.overTravel.at.travel)));
    QVERIFY(near(result.overTravel.distance, 20.0, 1e-3));
    // At ride height the wheel cannot reach 3.7 mm of bump.
    QVERIFY(near(result.rideHeight.at.rack, 7.3, 0.1));
    QVERIFY(near(result.rideHeight.distance, 20.0 + 0.05 * 3.7 * 3.7, 1e-3));
    QVERIFY(near(result.design.distance, 20.0 + 0.05 * (7.3 * 7.3 + 3.7 * 3.7), 1e-9));
    // Far fewer than every tenth of a millimetre in both directions.
    QVERIFY2(result.positions < 2000, qPrintable(QString::number(result.positions)));
}

/// An A-arm, P to Q to R, above a rod along x at z = 0: its front tube P-Q
/// 30 up, its rear one Q-R coming down to 25 at R.
struct ArmAboveRod {
    CollisionFrame frame;
    Linkage linkage;

    ArmAboveRod()
    {
        frame.table.points = {
            make("P", Vec3(-200, 0, 30)), make("Q", Vec3(0, 300, 30)), make("R", Vec3(200, 0, 25)),
            make("Rod0", Vec3(-300, 0, 0)), make("Rod1", Vec3(300, 0, 0)),
        };
        LinkagePart arm;
        arm.id = arm.label = QStringLiteral("arm");
        arm.kind = PartKind::Wishbone;
        arm.chains.push_back(ResolvedChain{ { 0, 1, 2 } });
        linkage.parts = { arm, link("rod", 3, 4) };
    }

    CollisionResult measure(const CollisionSettings& settings, const CollisionTarget& first) const
    {
        return minimumDistance(linkage, std::vector<bool>(5, false), {}, settings, first,
                               part("rod"), kStill, fixed(frame));
    }
};

void TestCollision::eachTubeHasItsOwnRadiusAndSaysWhereItIsClosest()
{
    const ArmAboveRod car;
    CollisionSettings settings;
    settings.memberRadius = 6.0;
    settings.jointClearance = 0.0;

    const CollisionResult thick = car.measure(settings, part("arm"));
    QVERIFY2(thick.valid, qPrintable(thick.problem));
    QVERIFY(near(thick.overTravel.distance, 25.0 - 12.0));
    // At R, the far end of the rear tube, half a metre along the rod.
    QCOMPARE(thick.overTravel.firstMember, QStringLiteral("Q - R"));
    QVERIFY(near(thick.overTravel.firstAlong, distance(Vec3(0, 300, 30), Vec3(200, 0, 25))));
    QCOMPARE(thick.overTravel.secondMember, QStringLiteral("Rod0 - Rod1"));
    QVERIFY(near(thick.overTravel.secondAlong, 500.0));

    // With the rear tube a bare line, the front one comes closer: 30 less
    // two radii of 6 against 25 less one.
    settings.memberRadii.insert(collisionMemberKey(QStringLiteral("R"), QStringLiteral("Q")), 0.0);
    const CollisionResult thin = car.measure(settings, part("arm"));
    QVERIFY(near(thin.overTravel.distance, 30.0 - 12.0));
    QCOMPARE(thin.overTravel.firstMember, QStringLiteral("P - Q"));
    QVERIFY(near(thin.overTravel.firstAlong, 0.0));
}

void TestCollision::oneTubeIsMeasuredWithoutTheRestOfItsPart()
{
    const ArmAboveRod car;
    CollisionSettings settings;
    settings.memberRadius = 0.0;
    settings.jointClearance = 0.0;

    const CollisionResult front = car.measure(settings, tube("arm", "P", "Q"));
    QVERIFY2(front.valid, qPrintable(front.problem));
    QVERIFY(near(front.overTravel.distance, 30.0));
    QCOMPARE(front.overTravel.firstMember, QStringLiteral("P - Q"));

    // A tube the part does not draw is refused.
    QVERIFY(!car.measure(settings, tube("arm", "P", "R")).valid);
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
    const CollisionResult ab = minimumDistance(linkage, grounded, {}, settings, part("a"),
                                               part("b"), kStill, fixed(frame));
    QVERIFY2(ab.valid, qPrintable(ab.problem));
    // Closest: a's end 30 short of Q, and b's start 30 past it.
    QVERIFY2(near(ab.overTravel.distance, std::sqrt(2.0) * 30.0),
             qPrintable(QString::number(ab.overTravel.distance)));

    const CollisionResult bc = minimumDistance(linkage, grounded, {}, settings, part("b"),
                                               part("c"), kStill, fixed(frame));
    QVERIFY2(bc.valid, qPrintable(bc.problem));
    QVERIFY(bc.overTravel.distance > 0.0);
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
    const CollisionResult clear = minimumDistance(linkage, grounded, {}, settings, part("arm"),
                                                  part("rod"), kStill, fixed(frame));
    QVERIFY2(clear.valid, qPrintable(clear.problem));
    QVERIFY2(clear.overTravel.distance > 5.0, qPrintable(QString::number(clear.overTravel.distance)));

    // Without the joint left out, they would read as touching.
    settings.jointClearance = 0.0;
    QVERIFY(minimumDistance(linkage, grounded, {}, settings, part("arm"), part("rod"), kStill,
                            fixed(frame))
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

    // At 10 mm of bump the second link's inner end comes to 8 mm off the tub.
    const CollisionFrameSource source = moving([](const CollisionPosition& at) {
        return std::vector<Hardpoint>{
            make("Pivot", Vec3(0, 200, 150)), make("Outer", Vec3(0, 500, 150)),
            make("Far_I", Vec3(200, 400 - 19.2 * at.travel, 100)), make("Far_O", Vec3(200, 600, 100)),
        };
    });
    const CollisionTravel travel{ 0.0, 10.0, 0.0 };
    Linkage linkage;
    linkage.parts = { link("arm", 0, 1), link("other", 2, 3) };
    const std::vector<bool> grounded = { true, false, false, false };

    CollisionSettings settings;
    settings.memberRadius = 5.0;
    const CollisionResult arm = minimumDistance(linkage, grounded, meshes, settings, part("arm"),
                                                chassis(), travel, source);
    QVERIFY2(arm.valid, qPrintable(arm.problem));
    // Bolted to the tub at its pivot, so measured from the joint clearance out.
    QVERIFY(near(arm.overTravel.distance, settings.jointClearance - 5.0, 1e-6));

    // Either way round.
    const CollisionResult other = minimumDistance(linkage, grounded, meshes, settings, chassis(),
                                                  part("other"), travel, source);
    QVERIFY2(other.valid, qPrintable(other.problem));
    QVERIFY(near(other.overTravel.distance, 3.0, 1e-6));
    QCOMPARE(other.overTravel.at.travel, 10.0);
    // The second target is the tube, so that is the side that says where.
    QVERIFY(other.overTravel.firstMember.isEmpty());
    QCOMPARE(other.overTravel.secondMember, QStringLiteral("Far_I - Far_O"));
    QVERIFY(near(other.design.distance, 195.0, 1e-6));
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

    // The rack turns the wheel 9 degrees a millimetre about z. Its corner
    // reaches furthest out in x, sqrt(50^2 + 20^2), at atan(20 / 50).
    const CollisionFrameSource steered = [&](const CollisionPosition& at) {
        CollisionFrame frame = withWheel(points, QVector3D(0, 600, 0),
                                         QQuaternion::fromAxisAndAngle(0, 0, 1, 9.0 * at.rack));
        frame.position = at;
        return std::optional<CollisionFrame>(frame);
    };
    const std::vector<bool> grounded(2, false);
    const CollisionResult result = minimumDistance(linkage, grounded, meshes, settings, part("rod"),
                                                   rim(), CollisionTravel{ 10.0, 0.0, 0.0 }, steered);
    QVERIFY2(result.valid, qPrintable(result.problem));
    QVERIFY2(near(result.design.distance, 45.0), qPrintable(QString::number(result.design.distance)));
    const double reach = std::sqrt(50.0 * 50.0 + 20.0 * 20.0);
    QVERIFY2(near(result.rideHeight.distance, 100.0 - reach - 5.0, 1e-2),
             qPrintable(QString::number(result.rideHeight.distance)));
    // Either way round: the box is symmetric, so is turning it.
    QVERIFY(near(9.0 * std::abs(result.rideHeight.at.rack), qRadiansToDegrees(std::atan(20.0 / 50.0)), 1.0));

    // Turned a quarter about z, its 40 now lies along x.
    const CollisionResult onlyTurned = minimumDistance(
        linkage, grounded, meshes, settings, part("rod"), rim(), kStill,
        fixed(withWheel(points, QVector3D(0, 600, 0), QQuaternion::fromAxisAndAngle(0, 0, 1, 90))));
    QVERIFY2(near(onlyTurned.overTravel.distance, 75.0),
             qPrintable(QString::number(onlyTurned.overTravel.distance)));
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
        minimumDistance(linkage, grounded, meshes, settings, part("rod"), rim(), kStill,
                        fixed(withWheel(points, QVector3D(0, 600, 0))));
    QVERIFY2(near(drawn.overTravel.distance, 55.0),
             qPrintable(QString::number(drawn.overTravel.distance)));

    const CollisionResult mirrored =
        minimumDistance(linkage, grounded, meshes, settings, part("rod"), rim(), kStill,
                        fixed(withWheel(points, QVector3D(0, 600, 0), {}, true)));
    QVERIFY2(near(mirrored.overTravel.distance, 15.0),
             qPrintable(QString::number(mirrored.overTravel.distance)));
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
        minimumDistance(Linkage{}, {}, meshes, CollisionSettings{}, rim(), chassis(), kStill,
                        fixed(withWheel({}, QVector3D(0, 600, 100))));
    QVERIFY2(result.valid, qPrintable(result.problem));
    // The rim's inner face at y = 580, the tub's side at y = 200.
    QVERIFY2(near(result.overTravel.distance, 380.0),
             qPrintable(QString::number(result.overTravel.distance)));
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
    const CollisionFrameSource source = fixed(withWheel(points, QVector3D(0, 600, 0)));
    const auto measure = [&](const CollisionMeshes& meshes, const CollisionTarget& a,
                             const CollisionTarget& b) {
        return minimumDistance(linkage, grounded, meshes, CollisionSettings{}, a, b, kStill, source);
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
    // A tube against its own part.
    QVERIFY(refused(measure(withRimAndTyre, part("rod"), tube("rod", "A", "B"))));
    // No wheel at that corner in any frame.
    QVERIFY(refused(measure(withRimAndTyre, part("rod"), rim(WheelCorner::RearRight))));
    // A part against a rim that is there is measured.
    QVERIFY(!refused(measure(withRimAndTyre, part("rod"), rim())));
}

void TestCollision::theTargetsOfferedAreWhatTheProjectHas()
{
    const TriMesh model = box(Vec3(-50, -20, -50), Vec3(50, 20, 50));
    const MeshQuery query(model);
    HardpointTable table;
    table.points = { make("A", Vec3(0, 0, 0)), make("B", Vec3(100, 0, 0)), make("C", Vec3(0, 100, 0)) };
    LinkagePart arm;
    arm.id = arm.label = QStringLiteral("arm");
    arm.kind = PartKind::Wishbone;
    arm.chains.push_back(ResolvedChain{ { 0, 1, 2 }, true });
    Linkage linkage;
    linkage.parts = { link("rod", 0, 1), link("wheel", 0, 1, PartKind::Wheel), arm };
    WheelSpec spec;
    spec.setPoint(WheelCorner::FrontLeft, QStringLiteral("FL_WC"));
    spec.setPoint(WheelCorner::RearRight, QStringLiteral("RR_WC"));
    // A and C are on the chassis: the arm's closing line between them is no tube.
    const std::vector<bool> grounded = { true, false, true };

    CollisionMeshes meshes;
    meshes.rim = { &model, &query };
    const std::vector<CollisionTarget> targets =
        collisionTargets(linkage, table, grounded, spec, meshes);
    // The rod, the arm and its two tubes, and a rim at the two corners named;
    // no tyre, no chassis, and no wheel construction line.
    QCOMPARE(targets.size(), std::size_t(6));
    QVERIFY(targets[0] == part("rod"));
    QVERIFY(targets[1] == part("arm"));
    QVERIFY(targets[2] == tube("arm", "A", "B"));
    QVERIFY(targets[3] == tube("arm", "B", "C"));
    QVERIFY(targets[4] == rim(WheelCorner::FrontLeft));
    QVERIFY(targets[5] == rim(WheelCorner::RearRight));
    QCOMPARE(collisionTargetLabel(targets[0], linkage), QStringLiteral("rod"));
    QVERIFY(collisionTargetLabel(targets[2], linkage).contains(QStringLiteral("A - B")));
    QVERIFY(collisionTargetLabel(targets[4], linkage).contains(QStringLiteral("rim")));

    const auto members = collisionMembers(part("arm"), linkage, table, grounded);
    QCOMPARE(members.size(), std::size_t(2));
    QCOMPARE(members[1].first, QStringLiteral("B"));
    QCOMPARE(members[1].second, QStringLiteral("C"));
}

void TestCollision::targetKeysRoundTrip()
{
    for (const CollisionTarget& target :
         { part(QStringLiteral("tieRod@F:base")), tube(QStringLiteral("lca@F:base"), "F_LCA_IF", "F_LCA_O"),
           rim(WheelCorner::RearLeft),
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
    settings.memberRadii.insert(collisionMemberKey(QStringLiteral("B"), QStringLiteral("A")), 6.0);
    settings.first = QStringLiteral("tieRod@F:base");
    settings.second = collisionTargetKey(rim());
    QVERIFY(collisionSettingsFromJson(collisionSettingsToJson(settings)) == settings);
    QVERIFY(collisionSettingsFromJson(QJsonObject{}) == CollisionSettings{});
    QCOMPARE(settings.radiusOf(QStringLiteral("A"), QStringLiteral("B")), 6.0);
    QCOMPARE(settings.radiusOf(QStringLiteral("A"), QStringLiteral("C")), 7.5);
}

QTEST_MAIN(TestCollision)
#include "test_collision.moc"
