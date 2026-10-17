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
    return mesh;
}

const CollisionPair* pairOf(const CollisionReport& report, const QString& a, const QString& b)
{
    for (const CollisionPair& pair : report.pairs)
        if ((pair.first == a && pair.second == b) || (pair.first == b && pair.second == a))
            return &pair;
    return nullptr;
}

} // namespace

class TestCollision : public QObject {
    Q_OBJECT

private slots:
    void segmentsAreMeasuredBetweenTheirClosestPoints();
    void twoMembersCrossingMidTravelTouch();
    void partsJoinedAtAPointAreNotCompared();
    void partsJoinedThroughAThirdAreComparedAwayFromIt();
    void aMemberIsComparedWithTheChassisAwayFromItsMounts();
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
    const CollisionReport report =
        checkCollisions(linkage, frames, std::vector<bool>(4, false), nullptr, settings);
    QCOMPARE(report.positions, 5);
    QCOMPARE(report.pairs.size(), std::size_t(1));
    const CollisionPair& pair = report.pairs.front();
    QVERIFY(pair.touches());
    QCOMPARE(pair.atInput, 10.0);
    QVERIFY(pair.designKnown);
    QCOMPARE(pair.designClearance, 10.0); // 20 apart, less two radii
}

void TestCollision::partsJoinedAtAPointAreNotCompared()
{
    CollisionFrame frame;
    frame.table.points = { make("P", Vec3(0, 0, 0)), make("Q", Vec3(100, 0, 0)),
                           make("R", Vec3(100, 50, 0)), make("S", Vec3(100, 50, 0)),
                           make("T", Vec3(100, 50, 100)) };
    Linkage linkage;
    // b shares Q with a, and c starts where b ends -- at a different row, but
    // the same place, as a strut's pushrod and damper do.
    linkage.parts = { link("a", 0, 1), link("b", 1, 2), link("c", 3, 4) };
    linkage.parts.push_back(link("wheel", 0, 1, PartKind::Wheel));

    const CollisionReport report = checkCollisions(
        linkage, { frame }, std::vector<bool>(5, false), nullptr, CollisionSettings{});
    QVERIFY(!pairOf(report, QStringLiteral("a"), QStringLiteral("b")));
    QVERIFY(!pairOf(report, QStringLiteral("b"), QStringLiteral("c")));
    // a and c meet only through b: compared, away from b's points.
    QVERIFY(pairOf(report, QStringLiteral("a"), QStringLiteral("c")));
    // A wheel's construction line is not a member.
    QVERIFY(!pairOf(report, QStringLiteral("a"), QStringLiteral("wheel")));
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
    const CollisionReport report =
        checkCollisions(linkage, { frame }, std::vector<bool>(4, false), nullptr, settings);
    const CollisionPair* pair = pairOf(report, QStringLiteral("arm"), QStringLiteral("rod"));
    QVERIFY(pair);
    QVERIFY2(pair->minClearance > settings.warnClearance, qPrintable(QString::number(pair->minClearance)));

    // Without the joint left out, they would read as touching.
    settings.jointClearance = 0.0;
    const CollisionReport tight =
        checkCollisions(linkage, { frame }, std::vector<bool>(4, false), nullptr, settings);
    QVERIFY(pairOf(tight, QStringLiteral("arm"), QStringLiteral("rod"))->touches());
}

void TestCollision::aMemberIsComparedWithTheChassisAwayFromItsMounts()
{
    // A tub from y = -200 to 200; a link from a pickup on its side outward,
    // and another that dips back towards it as the travel goes on.
    const MeshQuery chassis(box(Vec3(-500, -200, 0), Vec3(500, 200, 300)));
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
    std::vector<bool> grounded = { true, false, false, false };

    CollisionSettings settings;
    settings.memberRadius = 5.0;
    const CollisionReport report = checkCollisions(linkage, frames, grounded, &chassis, settings);

    const CollisionPair* arm = pairOf(report, QStringLiteral("arm"), QStringLiteral("Chassis"));
    QVERIFY(arm);
    QVERIFY(arm->chassis);
    // Bolted to the tub at its pivot, so measured from the joint clearance out.
    QVERIFY(std::abs(arm->minClearance - (settings.jointClearance - 5.0)) < 1e-6);

    const CollisionPair* other = pairOf(report, QStringLiteral("other"), QStringLiteral("Chassis"));
    QVERIFY(other);
    QVERIFY(std::abs(other->minClearance - 3.0) < 1e-6);
    QCOMPARE(other->atInput, 10.0);
    QVERIFY(std::abs(other->designClearance - 195.0) < 1e-6);

    settings.checkChassis = false;
    QVERIFY(!pairOf(checkCollisions(linkage, frames, grounded, &chassis, settings),
                    QStringLiteral("arm"), QStringLiteral("Chassis")));
}

void TestCollision::settingsRoundTrip()
{
    CollisionSettings settings{ 7.5, 3.0, 25.0, false };
    QVERIFY(collisionSettingsFromJson(collisionSettingsToJson(settings)) == settings);
    QVERIFY(collisionSettingsFromJson(QJsonObject{}) == CollisionSettings{});
}

QTEST_MAIN(TestCollision)
#include "test_collision.moc"
