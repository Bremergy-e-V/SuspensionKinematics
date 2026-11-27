#include "io/LinkageTemplate.h"
#include "io/XlsxHardpoints.h"
#include "model/HardpointMirror.h"
#include "model/PoseExport.h"
#include "model/Simulation.h"

#include <QJsonArray>
#include <QTemporaryDir>
#include <QTest>

#include <cmath>
#include <set>

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

/// Both sides of a steered front axle, the far side mirrored with the tool's
/// own rule: the car test_simulation poses.
HardpointTable frontAxle()
{
    HardpointTable corner;
    corner.points = {
        make("F_LCA_IF", 100.0, 200.0, 120.0),  make("F_LCA_IR", -100.0, 200.0, 120.0),
        make("F_LCA_O", 0.0, 560.0, 110.0),     make("F_UCA_IF", 80.0, 230.0, 280.0),
        make("F_UCA_IR", -80.0, 230.0, 280.0),  make("F_UCA_O", 0.0, 540.0, 300.0),
        make("F_TieRod_I", -120.0, 220.0, 150.0), make("F_TieRod_O", -120.0, 555.0, 145.0),
        make("F_WheelCenter", 0.0, 600.0, 220.0), make("F_ContactPatch", 0.0, 600.0, 0.0),
    };
    return mirrorHardpoints(corner, {}, MirrorSpec{}).table;
}

LinkageTemplate steeredFrontTemplate()
{
    LinkageTemplate templ;
    templ.mechanism.lowerFront = QStringLiteral("{corner}_LCA_IF");
    templ.mechanism.lowerRear = QStringLiteral("{corner}_LCA_IR");
    templ.mechanism.lowerOuter = QStringLiteral("{corner}_LCA_O");
    templ.mechanism.upperFront = QStringLiteral("{corner}_UCA_IF");
    templ.mechanism.upperRear = QStringLiteral("{corner}_UCA_IR");
    templ.mechanism.upperOuter = QStringLiteral("{corner}_UCA_O");
    templ.mechanism.tieRodInboard = QStringLiteral("{corner}_TieRod_I");
    templ.mechanism.tieRodOutboard = QStringLiteral("{corner}_TieRod_O");
    templ.mechanism.wheelCenter = QStringLiteral("{corner}_WheelCenter");
    templ.mechanism.contactPatch = QStringLiteral("{corner}_ContactPatch");

    CornerSpec front;
    front.token = QStringLiteral("F");
    front.label = QStringLiteral("Front");
    front.steeringRack = QStringLiteral("{corner}_TieRod_I");
    front.steeringStated = true;
    templ.corners = { front };
    return templ;
}

const Hardpoint& pointNamed(const HardpointTable& table, const QString& name)
{
    const int index = table.indexOf(name);
    Q_ASSERT(index >= 0);
    return table.points[std::size_t(index)];
}

} // namespace

class TestPoseExport : public QObject {
    Q_OBJECT

private slots:
    void thereAreNineDistinctPoses();
    void aKeyReadsBackAsItsPose();
    void aKeyThatIsNotOneIsRefused();
    void aPoseIsTheEndOfTheTravelItNames();
    void aFileSaysTheMillimetresOfItsPose();
    void aProjectThatNeverExportedTicksEveryPose();
    void anEmptyListReadsBackEmpty();
    void settingsRoundTripThroughJson();
    void aPosedWorkbookHoldsThePointsWhereTheSolvePutsThem();
};

void TestPoseExport::thereAreNineDistinctPoses()
{
    const std::vector<ExportPose> poses = allExportPoses();
    QCOMPARE(poses.size(), std::size_t(9));
    std::set<QString> keys;
    for (const ExportPose& pose : poses) keys.insert(exportPoseKey(pose));
    QCOMPARE(keys.size(), std::size_t(9));
}

void TestPoseExport::aKeyReadsBackAsItsPose()
{
    for (const ExportPose& pose : allExportPoses()) {
        const std::optional<ExportPose> read = exportPoseFromKey(exportPoseKey(pose));
        QVERIFY(read.has_value());
        QVERIFY(*read == pose);
    }
    QCOMPARE(exportPoseKey(ExportPose{ TravelEnd::Positive, TravelEnd::Negative }),
             QStringLiteral("+-"));
}

void TestPoseExport::aKeyThatIsNotOneIsRefused()
{
    QVERIFY(!exportPoseFromKey(QString()).has_value());
    QVERIFY(!exportPoseFromKey(QStringLiteral("+")).has_value());
    QVERIFY(!exportPoseFromKey(QStringLiteral("+x")).has_value());
    QVERIFY(!exportPoseFromKey(QStringLiteral("+-0")).has_value());
}

void TestPoseExport::aPoseIsTheEndOfTheTravelItNames()
{
    const CollisionTravel travel{ 30.0, 25.0, 20.0 };
    const CollisionPosition leftBump =
        exportPosition(ExportPose{ TravelEnd::Positive, TravelEnd::Positive }, travel);
    QCOMPARE(leftBump.rack, 30.0);
    QCOMPARE(leftBump.travel, 25.0);

    const CollisionPosition rightRebound =
        exportPosition(ExportPose{ TravelEnd::Negative, TravelEnd::Negative }, travel);
    QCOMPARE(rightRebound.rack, -30.0);
    QCOMPARE(rightRebound.travel, -20.0);

    const CollisionPosition design = exportPosition(ExportPose{}, travel);
    QCOMPARE(design.rack, 0.0);
    QCOMPARE(design.travel, 0.0);
}

void TestPoseExport::aFileSaysTheMillimetresOfItsPose()
{
    const CollisionTravel travel{ 30.0, 25.0, 12.5 };
    QCOMPARE(exportPoseSuffix(ExportPose{ TravelEnd::Positive, TravelEnd::Positive }, travel),
             QStringLiteral("rack+30_bump25"));
    QCOMPARE(exportPoseSuffix(ExportPose{ TravelEnd::Negative, TravelEnd::Negative }, travel),
             QStringLiteral("rack-30_rebound12.5"));
    QCOMPARE(exportPoseSuffix(ExportPose{}, travel), QStringLiteral("rack0_ride"));
}

void TestPoseExport::aProjectThatNeverExportedTicksEveryPose()
{
    const PoseExportSettings settings = poseExportSettingsFromJson(QJsonObject());
    QVERIFY(settings == PoseExportSettings{});
    QCOMPARE(settings.poses.size(), std::size_t(9));
}

void TestPoseExport::anEmptyListReadsBackEmpty()
{
    PoseExportSettings settings;
    settings.poses.clear();
    const PoseExportSettings read = poseExportSettingsFromJson(poseExportSettingsToJson(settings));
    QVERIFY(read.poses.empty());
}

void TestPoseExport::settingsRoundTripThroughJson()
{
    PoseExportSettings settings;
    settings.poses = { ExportPose{ TravelEnd::Negative, TravelEnd::Positive },
                       ExportPose{ TravelEnd::Positive, TravelEnd::Centre } };
    settings.directory = QStringLiteral("C:/cad/check");
    settings.baseName = QStringLiteral("26_DY");
    QVERIFY(poseExportSettingsFromJson(poseExportSettingsToJson(settings)) == settings);
}

void TestPoseExport::aPosedWorkbookHoldsThePointsWhereTheSolvePutsThem()
{
    const HardpointTable design = frontAxle();
    const Simulation simulation =
        Simulation::build(steeredFrontTemplate(), design, MirrorSpec{}, {});
    QVERIFY(!simulation.isEmpty());

    const CollisionTravel travel{ 20.0, 25.0, 25.0 };
    const CollisionPosition position =
        exportPosition(ExportPose{ TravelEnd::Positive, TravelEnd::Positive }, travel);
    const std::optional<SimulationPose> pose =
        simulation.poseAtWheelTravel(position.travel, position.rack);
    QVERIFY(pose.has_value());
    const HardpointTable posed = pose->layOver(design);

    QTemporaryDir directory;
    QVERIFY(directory.isValid());
    const QString path = directory.filePath(QStringLiteral("posed.xlsx"));
    QCOMPARE(writeNewHardpointsXlsx(path, posed), QString());
    const HardpointLoadResult read = readHardpointsXlsx(path);
    QVERIFY2(read.ok(), qPrintable(read.error));

    // Every point is there, where the solve put it -- both sides, which steer
    // differently, so the far side is not the near side's mirror image.
    QCOMPARE(read.table->points.size(), posed.points.size());
    for (const Hardpoint& point : posed.points) {
        const Hardpoint& back = pointNamed(*read.table, point.name);
        for (int axis = 0; axis < 3; ++axis)
            QVERIFY2(std::abs(back.coord[axis] - point.coord[axis]) < 1e-9,
                     qPrintable(point.name));
    }
    const Hardpoint& left = pointNamed(*read.table, QStringLiteral("F_WheelCenter"));
    const Hardpoint& designLeft = pointNamed(design, QStringLiteral("F_WheelCenter"));
    QVERIFY(std::abs(left.coord[2] - designLeft.coord[2] - 25.0) < 0.5);

    const QString farName = mirroredName(QStringLiteral("F_TieRod_O"), MirrorSpec{});
    const Hardpoint& near = pointNamed(*read.table, QStringLiteral("F_TieRod_O"));
    const Hardpoint& far = pointNamed(*read.table, farName);
    QVERIFY(std::abs(near.coord[0] - far.coord[0]) > 0.1);
}

QTEST_GUILESS_MAIN(TestPoseExport)
#include "test_pose_export.moc"
