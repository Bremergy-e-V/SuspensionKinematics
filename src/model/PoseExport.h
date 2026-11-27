#pragma once

#include "model/Collision.h"

#include <QJsonObject>
#include <QString>

#include <optional>
#include <vector>

namespace suspkin {

/// Where one exported pose takes the rack or the wheels: to the end of their
/// travel one way, the other way, or nowhere.
enum class TravelEnd { Negative = -1, Centre = 0, Positive = 1 };

/// One position of the car Hardpoints > Export Posed Workbooks writes a
/// workbook for: the rack at an end of its travel or centred (positive to the
/// left, +Y, as in a steer sweep), and every wheel at full bump, full rebound
/// or ride height (positive is bump).
struct ExportPose {
    TravelEnd rack = TravelEnd::Centre;
    TravelEnd wheels = TravelEnd::Centre;

    bool operator==(const ExportPose& other) const = default;
};

/// The nine poses there are: every end of the steering against every end of
/// the wheel travel, ride height and straight ahead included.
std::vector<ExportPose> allExportPoses();

/// @p pose as the manifest writes it: one character each for the rack and
/// the wheels, `-`, `0` or `+` -- "+-" is full left lock at full rebound.
QString exportPoseKey(const ExportPose& pose);
std::optional<ExportPose> exportPoseFromKey(const QString& key);

/// The rack travel and the wheel travel @p pose stands for, mm, with the
/// travel the sweep parameters give.
CollisionPosition exportPosition(const ExportPose& pose, const CollisionTravel& travel);

/// What a workbook of @p pose is called after the project's own name:
/// "rack+30_bump25", "rack0_ride", "rack-30_rebound25". Says the millimetres,
/// so two exports made with different travel never read as the same car.
QString exportPoseSuffix(const ExportPose& pose, const CollisionTravel& travel);

/// What the user last asked Export Posed Workbooks for.
struct PoseExportSettings {
    /// The poses ticked, in the order allExportPoses() gives them.
    std::vector<ExportPose> poses = allExportPoses();
    /// The folder the workbooks go to. Empty until the user has chosen one.
    QString directory;
    /// What every workbook's name starts with. Empty means the workbook's own.
    QString baseName;

    bool operator==(const PoseExportSettings& other) const = default;
};

QJsonObject poseExportSettingsToJson(const PoseExportSettings& settings);
PoseExportSettings poseExportSettingsFromJson(const QJsonObject& object);

} // namespace suspkin
