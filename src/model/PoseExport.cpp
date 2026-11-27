#include "model/PoseExport.h"

#include <QJsonArray>
#include <QJsonValue>

namespace suspkin {
namespace {

const char kPoses[] = "poses";
const char kDirectory[] = "directory";
const char kBaseName[] = "baseName";

const TravelEnd kEnds[] = { TravelEnd::Negative, TravelEnd::Centre, TravelEnd::Positive };

QChar endChar(TravelEnd end)
{
    switch (end) {
    case TravelEnd::Negative: return QLatin1Char('-');
    case TravelEnd::Positive: return QLatin1Char('+');
    case TravelEnd::Centre: break;
    }
    return QLatin1Char('0');
}

std::optional<TravelEnd> endFromChar(QChar c)
{
    if (c == QLatin1Char('-')) return TravelEnd::Negative;
    if (c == QLatin1Char('0')) return TravelEnd::Centre;
    if (c == QLatin1Char('+')) return TravelEnd::Positive;
    return std::nullopt;
}

/// A length for a file name: as many digits as it has and no more.
QString millimetres(double value) { return QString::number(value, 'g', 6); }

QString rackSuffix(TravelEnd end, double travel)
{
    switch (end) {
    case TravelEnd::Negative: return QStringLiteral("rack-") + millimetres(travel);
    case TravelEnd::Positive: return QStringLiteral("rack+") + millimetres(travel);
    case TravelEnd::Centre: break;
    }
    return QStringLiteral("rack0");
}

QString wheelSuffix(TravelEnd end, const CollisionTravel& travel)
{
    switch (end) {
    case TravelEnd::Negative: return QStringLiteral("rebound") + millimetres(travel.rebound);
    case TravelEnd::Positive: return QStringLiteral("bump") + millimetres(travel.bump);
    case TravelEnd::Centre: break;
    }
    return QStringLiteral("ride");
}

} // namespace

std::vector<ExportPose> allExportPoses()
{
    std::vector<ExportPose> poses;
    for (TravelEnd wheels : kEnds)
        for (TravelEnd rack : kEnds) poses.push_back(ExportPose{ rack, wheels });
    return poses;
}

QString exportPoseKey(const ExportPose& pose)
{
    return QString(endChar(pose.rack)) + endChar(pose.wheels);
}

std::optional<ExportPose> exportPoseFromKey(const QString& key)
{
    if (key.size() != 2) return std::nullopt;
    const std::optional<TravelEnd> rack = endFromChar(key.at(0));
    const std::optional<TravelEnd> wheels = endFromChar(key.at(1));
    if (!rack || !wheels) return std::nullopt;
    return ExportPose{ *rack, *wheels };
}

CollisionPosition exportPosition(const ExportPose& pose, const CollisionTravel& travel)
{
    CollisionPosition position;
    position.rack = static_cast<int>(pose.rack) * travel.rack;
    if (pose.wheels == TravelEnd::Positive) position.travel = travel.bump;
    if (pose.wheels == TravelEnd::Negative) position.travel = -travel.rebound;
    return position;
}

QString exportPoseSuffix(const ExportPose& pose, const CollisionTravel& travel)
{
    return rackSuffix(pose.rack, travel.rack) + QLatin1Char('_') + wheelSuffix(pose.wheels, travel);
}

QJsonObject poseExportSettingsToJson(const PoseExportSettings& settings)
{
    QJsonArray poses;
    for (const ExportPose& pose : settings.poses) poses.append(exportPoseKey(pose));
    QJsonObject object;
    object.insert(QLatin1String(kPoses), poses);
    if (!settings.directory.isEmpty()) object.insert(QLatin1String(kDirectory), settings.directory);
    if (!settings.baseName.isEmpty()) object.insert(QLatin1String(kBaseName), settings.baseName);
    return object;
}

PoseExportSettings poseExportSettingsFromJson(const QJsonObject& object)
{
    PoseExportSettings settings;
    // No list at all is a project that never exported; an empty one is a
    // user who unticked everything, and gets that back.
    if (object.contains(QLatin1String(kPoses))) {
        settings.poses.clear();
        for (const QJsonValue& value : object.value(QLatin1String(kPoses)).toArray())
            if (const std::optional<ExportPose> pose = exportPoseFromKey(value.toString()))
                settings.poses.push_back(*pose);
    }
    settings.directory = object.value(QLatin1String(kDirectory)).toString();
    settings.baseName = object.value(QLatin1String(kBaseName)).toString();
    return settings;
}

} // namespace suspkin
