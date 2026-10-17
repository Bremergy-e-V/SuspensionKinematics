#include "model/SpringLoads.h"

#include "model/SuspensionSolver.h"

#include <cmath>

namespace suspkin {
namespace {

constexpr double kMillimetresPerMetre = 1000.0;
constexpr double kDegToRad = 0.017453292519943295769236907684886;
/// A ratio smaller than this has no spring to speak of behind it, and dividing
/// a static load by it would invent a preload of mega-newtons.
constexpr double kNoRatio = 1e-6;

const char kSpring[] = "spring";
const char kAntiRoll[] = "antiRoll";
const char kTyre[] = "tyre";
const char kStaticLoad[] = "staticLoad";

/// The spring's share: its force, what it puts on the wheel here, what it put
/// there at design, and the wheel rate it gives.
struct SpringShare {
    bool valid = false;
    double force = 0.0;
    double atWheel = 0.0;
    double atWheelDesign = 0.0;
    double wheelRate = 0.0;
};

SpringShare springShare(const WheelMotion& motion, const AxleSprings& springs)
{
    SpringShare share;
    const CornerPose& pose = *motion.pose;
    const CornerPose& design = *motion.design;
    if (springs.springRate <= 0.0 || !pose.hasDamper || !design.hasDamper) return share;

    // The static load is carried by a preload, which is what a spring set to
    // ride height is: whatever force puts that load on the wheel at design.
    const double preload = springs.staticLoad > 0.0
                                   && std::abs(motion.designInstallationRatio) > kNoRatio
                               ? springs.staticLoad / motion.designInstallationRatio
                               : 0.0;
    const double compression = design.damperLength - pose.damperLength;
    share.valid = true;
    share.force = preload + springs.springRate * compression;
    share.atWheel = share.force * motion.installationRatio;
    share.atWheelDesign = preload * motion.designInstallationRatio;
    share.wheelRate = springs.springRate * motion.installationRatio * motion.installationRatio;
    return share;
}

} // namespace

QJsonObject axleSpringsToJson(const QHash<QString, AxleSprings>& springs)
{
    QJsonObject object;
    for (auto it = springs.begin(); it != springs.end(); ++it) {
        QJsonObject axle;
        axle.insert(QLatin1String(kSpring), it->springRate);
        axle.insert(QLatin1String(kAntiRoll), it->antiRollRate);
        axle.insert(QLatin1String(kTyre), it->tyreRate);
        axle.insert(QLatin1String(kStaticLoad), it->staticLoad);
        object.insert(it.key(), axle);
    }
    return object;
}

QHash<QString, AxleSprings> axleSpringsFromJson(const QJsonObject& object)
{
    QHash<QString, AxleSprings> springs;
    for (auto it = object.begin(); it != object.end(); ++it) {
        const QJsonObject axle = it.value().toObject();
        AxleSprings rates;
        rates.springRate = axle.value(QLatin1String(kSpring)).toDouble();
        rates.antiRollRate = axle.value(QLatin1String(kAntiRoll)).toDouble();
        rates.tyreRate = axle.value(QLatin1String(kTyre)).toDouble();
        rates.staticLoad = axle.value(QLatin1String(kStaticLoad)).toDouble();
        springs.insert(it.key(), rates);
    }
    return springs;
}

WheelLoad wheelLoadAt(const WheelMotion& motion, const AxleSprings& springs, double twistDeg,
                      bool hasBar, bool leftSide)
{
    WheelLoad load;
    if (!motion.pose || !motion.design || !motion.pose->valid || springs.isEmpty()) return load;

    const SpringShare spring = springShare(motion, springs);
    const bool bar = hasBar && springs.antiRollRate > 0.0;
    // The torque's work over the arm's turn, in radians: dU/dx for
    // U = k theta^2 / 2, where theta is left arm less right, so the right arm's
    // turn counts against the twist.
    const double torque = springs.antiRollRate * kMillimetresPerMetre * twistDeg; // N·mm
    const double armTurn = motion.barRatio * kDegToRad;                          // rad/mm
    load.barForce = bar ? (leftSide ? 1.0 : -1.0) * torque * armTurn : 0.0;

    load.valid = spring.valid || bar || springs.tyreRate > 0.0;
    load.springForce = spring.force;
    load.wheelRate = spring.wheelRate;
    load.loadChange = (spring.valid ? spring.atWheel - spring.atWheelDesign : 0.0) + load.barForce;
    load.load = springs.staticLoad + load.loadChange;

    if (springs.tyreRate > 0.0) {
        load.tyreValid = true;
        load.tyreDeflection = load.load / springs.tyreRate;
        load.travelWithTyre = motion.pose->wheelTravel + load.loadChange / springs.tyreRate;
        if (load.wheelRate > 0.0)
            load.rideRate =
                load.wheelRate * springs.tyreRate / (load.wheelRate + springs.tyreRate);
    }
    return load;
}

double rollMomentOf(const WheelLoad& left, double leftY, const WheelLoad& right, double rightY)
{
    // The left wheel unloading on the positive side of the car resists a roll
    // that lifts it; so does the right one loading on the negative side.
    return -(left.loadChange * leftY + right.loadChange * rightY) / kMillimetresPerMetre;
}

} // namespace suspkin
