#pragma once

#include <QHash>
#include <QJsonObject>
#include <QString>

namespace suspkin {

struct CornerPose;

/// What an axle is sprung with: the rates the forces at its wheels come from.
///
/// Each is optional -- zero is "not given" -- because each answers a question
/// of its own: the spring what load a bump puts on the wheel, the bar what a
/// roll does, the tyre how much of the motion it takes up itself. Stored in the
/// manifest under @c springs, keyed by corner token, like the static angles.
struct AxleSprings {
    double springRate = 0.0;   ///< N/mm, the coil on the damper
    double antiRollRate = 0.0; ///< N·m per degree of twist between the bar's arms
    double tyreRate = 0.0;     ///< N/mm, the tyre's vertical stiffness
    /// N on each wheel at the design position. With it the loads are absolute
    /// and the spring carries it as preload; without it they are changes from
    /// design.
    double staticLoad = 0.0;

    bool isEmpty() const { return springRate <= 0.0 && antiRollRate <= 0.0 && tyreRate <= 0.0; }
    bool operator==(const AxleSprings& other) const = default;
};

QJsonObject axleSpringsToJson(const QHash<QString, AxleSprings>& springs);
QHash<QString, AxleSprings> axleSpringsFromJson(const QJsonObject& object);

/// The forces at one wheel at one position, and what the tyre makes of them.
///
/// The forces come from virtual work: a wheel that rises dx while the spring
/// closes dc = IR dx needs F_spring IR at the wheel, and one that turns the
/// bar's arm by d(alpha) = r dx against a twist theta needs k theta r. That is
/// the motion the solve already has, and nothing about the car's mass.
struct WheelLoad {
    bool valid = false;
    double springForce = 0.0; ///< N along the spring, preload included
    double barForce = 0.0;    ///< N at the wheel from the bar's twist
    double load = 0.0;        ///< N at the contact patch: static plus change
    double loadChange = 0.0;  ///< N from the design position
    double wheelRate = 0.0;   ///< N/mm, spring rate times the installation ratio squared

    bool tyreValid = false;
    double tyreDeflection = 0.0; ///< mm the tyre is squashed: load over its rate
    double rideRate = 0.0;       ///< N/mm, wheel rate and tyre in series
    /// mm the body comes down towards the road: the wheel's own travel plus
    /// what the tyre gives on top, which is the bump a road input has to make.
    double travelWithTyre = 0.0;
};

/// What one side of an axle is at, for @ref wheelLoadAt: the pose, the design
/// pose it is measured from, and the two ratios the solve has differentiated.
struct WheelMotion {
    const CornerPose* pose = nullptr;
    const CornerPose* design = nullptr;
    double installationRatio = 0.0;       ///< mm of spring per mm of wheel, here
    double designInstallationRatio = 0.0; ///< and at design, for the preload
    double barRatio = 0.0;                ///< degrees of bar arm per mm of wheel
};

/// The load at one wheel. @p twistDeg is the bar's twist, left arm less right,
/// and @p leftSide says which way it pushes this wheel; @p hasBar is whether
/// there is a twist to read at all.
WheelLoad wheelLoadAt(const WheelMotion& motion, const AxleSprings& springs, double twistDeg,
                      bool hasBar, bool leftSide);

/// N·m of moment the two wheels' load changes make about the centreline,
/// positive resisting positive roll (the left side lifting). @p leftY and
/// @p rightY are the contact patches across the car, mm.
double rollMomentOf(const WheelLoad& left, double leftY, const WheelLoad& right, double rightY);

} // namespace suspkin
