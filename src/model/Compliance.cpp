#include "model/Compliance.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("Compliance", text); }

constexpr double kPi = 3.14159265358979323846264338327950288;
constexpr double kRadToDeg = 57.295779513082320876798154814105;

/// Millimetres of patch rise either side of a pose that a slope is taken over:
/// small enough to be a derivative, large enough to stand clear of the corner
/// solve's own tolerance.
constexpr double kGradientProbeMm = 0.25;

/// A tyre deflection settled to this is settled: a thousandth of a micron.
constexpr double kSettledMm = 1e-6;
/// What is still accepted when the search runs out of turns -- well under
/// anything a plot or a workbook shows.
constexpr double kAcceptedMm = 1e-3;
constexpr int kMaxSettlingSteps = 30;
/// The furthest one step moves a patch. The search is all but linear, so this
/// only ever bites on rates that make no sense.
constexpr double kMaxSettlingStepMm = 20.0;

/// The bar's rate in the units the energy is in: N mm per radian.
double barRatePerRadian(const AxleRates& rates)
{
    return rates.antiRollRate * 1000.0 * kRadToDeg;
}

/// @p angle brought into (-pi, pi], so that two arm angles either side of the
/// wrap still differ by the little they do.
double wrappedAngle(double angle) { return std::remainder(angle, 2.0 * kPi); }

/// How much the bar is twisted between its two arms, in radians, beyond what
/// it is at design. Zero on an axle that has no bar on both sides.
double barTwist(const AxleSides& sides, const AxlePoses& poses)
{
    if (!sides.left || !sides.right) return 0.0;
    if (!poses.left.hasAntiRoll || !poses.right.hasAntiRoll) return 0.0;
    const double posed = poses.left.antiRollArmAngle - poses.right.antiRollArmAngle;
    const double design = sides.left->designPose().antiRollArmAngle
                          - sides.right->designPose().antiRollArmAngle;
    return wrappedAngle(posed - design);
}

/// What a millimetre of contact patch rise does to one corner's spring and to
/// its bar arm.
struct RiseGradient {
    bool valid = false;
    double compression = 0.0; ///< mm of damper compression per mm of rise
    double barAngle = 0.0;    ///< radians of bar arm per mm of rise
};

/// The gradient at @p at, across a probe either side of it. One side of the
/// probe is enough near the end of the travel, where the other does not
/// assemble.
RiseGradient riseGradient(const CornerSolver& solver, const CornerPose& at, double rackTravel)
{
    CornerPose before =
        solver.poseAtContactPatchRise(at.contactPatchRise - kGradientProbeMm, rackTravel, &at);
    CornerPose after =
        solver.poseAtContactPatchRise(at.contactPatchRise + kGradientProbeMm, rackTravel, &at);
    if (!before.valid) before = at;
    if (!after.valid) after = at;

    RiseGradient gradient;
    const double span = after.contactPatchRise - before.contactPatchRise;
    if (span < 0.5 * kGradientProbeMm) return gradient;
    gradient.valid = true;
    gradient.compression = (before.damperLength - after.damperLength) / span;
    gradient.barAngle = wrappedAngle(after.antiRollArmAngle - before.antiRollArmAngle) / span;
    return gradient;
}

/// One corner's share of the axle's springing at one pose.
struct SideSpringing {
    bool present = false; ///< the axle has this side at all
    bool valid = false;   ///< and it has a damper, and its slopes could be taken
    double compression = 0.0; ///< mm the spring is shorter than at design
    RiseGradient gradient;
};

SideSpringing sideSpringing(const CornerSolver* solver, const CornerPose& pose, double rackTravel)
{
    SideSpringing side;
    if (!solver) return side;
    side.present = true;
    if (!pose.valid || !pose.hasDamper) return side;
    side.compression = -pose.damperTravel;
    side.gradient = riseGradient(*solver, pose, rackTravel);
    side.valid = side.gradient.valid;
    return side;
}

/// The axle's springing at one pose, taken as far as a search needs it: what
/// pushes on each patch, and how that changes as the patches move.
struct AxleSpringing {
    bool valid = false;
    SideSpringing left;
    SideSpringing right;
    double twist = 0.0; ///< radians, beyond design
    SidePair patchForce; ///< N
    /// N/mm: how each patch's force answers each patch's rise. The cross term
    /// is the bar, and is the same both ways round.
    double leftStiffness = 0.0;
    double rightStiffness = 0.0;
    double crossStiffness = 0.0;
};

AxleSpringing axleSpringing(const AxleSides& sides, const AxleRates& rates,
                            const AxlePoses& poses, double rackTravel)
{
    AxleSpringing axle;
    axle.left = sideSpringing(sides.left, poses.left, rackTravel);
    axle.right = sideSpringing(sides.right, poses.right, rackTravel);
    if (!axle.left.present && !axle.right.present) return axle;
    if (axle.left.present && !axle.left.valid) return axle;
    if (axle.right.present && !axle.right.valid) return axle;
    axle.valid = true;
    axle.twist = barTwist(sides, poses);

    const double spring = rates.springRate;
    const double bar = barRatePerRadian(rates);
    const RiseGradient& left = axle.left.gradient;
    const RiseGradient& right = axle.right.gradient;

    // Each is the stored energy differentiated against that patch's rise. The
    // bar's twist is left minus right, so it pushes the two patches opposite
    // ways.
    axle.patchForce.left =
        spring * axle.left.compression * left.compression + bar * axle.twist * left.barAngle;
    axle.patchForce.right =
        spring * axle.right.compression * right.compression - bar * axle.twist * right.barAngle;

    axle.leftStiffness =
        spring * left.compression * left.compression + bar * left.barAngle * left.barAngle;
    axle.rightStiffness =
        spring * right.compression * right.compression + bar * right.barAngle * right.barAngle;
    axle.crossStiffness = -bar * left.barAngle * right.barAngle;
    return axle;
}

/// The loads of @p axle with tyres squashed by @p deflection.
AxleLoads loadsOf(const AxleSpringing& axle, const AxleRates& rates, const SidePair& deflection)
{
    AxleLoads loads;
    if (!axle.valid) return loads;
    loads.valid = true;
    loads.leftSpringForce = rates.springRate * axle.left.compression;
    loads.rightSpringForce = rates.springRate * axle.right.compression;
    loads.antiRollTorque = barRatePerRadian(rates) * axle.twist / 1000.0;
    loads.leftWheelLoad = axle.patchForce.left;
    loads.rightWheelLoad = axle.patchForce.right;
    loads.leftTyreDeflection = deflection.left;
    loads.rightTyreDeflection = deflection.right;
    return loads;
}

/// Both sides with their rigid contact patches @p rise above design.
AxlePoses poseAxle(const AxleSides& sides, const SidePair& rise, double rackTravel,
                   const AxlePoses* previous)
{
    AxlePoses poses;
    if (sides.left)
        poses.left = sides.left->poseAtContactPatchRise(rise.left, rackTravel,
                                                        previous ? &previous->left : nullptr);
    if (sides.right)
        poses.right = sides.right->poseAtContactPatchRise(rise.right, rackTravel,
                                                          previous ? &previous->right : nullptr);
    return poses;
}

bool everySideAssembled(const AxleSides& sides, const AxlePoses& poses)
{
    return (!sides.left || poses.left.valid) && (!sides.right || poses.right.valid);
}

/// How far each patch's springing is from what its tyre makes of the
/// deflection left over, in newtons. Zero on a side the axle has not got.
SidePair imbalance(const AxleSpringing& axle, const AxleRates& rates, const SidePair& groundRise,
                   const SidePair& rise)
{
    SidePair residual;
    if (axle.left.present)
        residual.left = axle.patchForce.left - rates.tyreRate * (groundRise.left - rise.left);
    if (axle.right.present)
        residual.right = axle.patchForce.right - rates.tyreRate * (groundRise.right - rise.right);
    return residual;
}

/// The move of the two patches that takes @p residual out, to first order.
SidePair settlingStep(const AxleSpringing& axle, const AxleRates& rates, const SidePair& residual)
{
    const double ll = axle.leftStiffness + rates.tyreRate;
    const double rr = axle.rightStiffness + rates.tyreRate;
    const double lr = axle.left.present && axle.right.present ? axle.crossStiffness : 0.0;
    const double determinant = ll * rr - lr * lr;

    SidePair step;
    step.left = -(rr * residual.left - lr * residual.right) / determinant;
    step.right = -(ll * residual.right - lr * residual.left) / determinant;
    step.left = std::clamp(step.left, -kMaxSettlingStepMm, kMaxSettlingStepMm);
    step.right = std::clamp(step.right, -kMaxSettlingStepMm, kMaxSettlingStepMm);
    return step;
}

/// Move @p pose's contact patch up onto the road its tyre has been squashed
/// against: the suspension put it @p deflection lower, where a rigid tyre
/// would have reached.
void settleOnRoad(const CornerSolver& solver, double deflection, CornerPose* pose)
{
    pose->contactPatch.z += deflection;
    pose->contactPatchRise += deflection;
    const QString& name = solver.mechanism().contactPatch;
    if (name.isEmpty()) return;
    for (PosedPoint& point : pose->points)
        if (point.name == name) point.position.z += deflection;
}

void failToSettle(const AxleSides& sides, AxlePoses* poses)
{
    const QString reason = tr("The springs and the tyres did not settle at that position.");
    for (CornerPose* pose : { sides.left ? &poses->left : nullptr,
                              sides.right ? &poses->right : nullptr }) {
        if (!pose || !pose->valid) continue;
        pose->valid = false;
        pose->error = reason;
    }
}

} // namespace

AxleLoads suspensionLoads(const AxleSides& sides, const AxleRates& rates, const AxlePoses& poses,
                          double rackTravel)
{
    if (!rates.hasSpring() || !everySideAssembled(sides, poses)) return AxleLoads{};
    return loadsOf(axleSpringing(sides, rates, poses, rackTravel), rates, SidePair{});
}

CompliantAxle solveCompliantAxle(const AxleSides& sides, const AxleRates& rates,
                                 const SidePair& groundRise, double rackTravel,
                                 const AxlePoses* previous)
{
    CompliantAxle result;
    SidePair rise = groundRise;
    result.poses = poseAxle(sides, rise, rackTravel, previous);
    if (!rates.hasSpring() || !rates.hasTyre()) return result;

    AxleSpringing springing;
    double unsettledMm = 0.0;
    for (int turn = 0; turn < kMaxSettlingSteps; ++turn) {
        if (!everySideAssembled(sides, result.poses)) return result;
        springing = axleSpringing(sides, rates, result.poses, rackTravel);
        if (!springing.valid) return result;

        const SidePair residual = imbalance(springing, rates, groundRise, rise);
        unsettledMm =
            std::max(std::abs(residual.left), std::abs(residual.right)) / rates.tyreRate;
        if (unsettledMm < kSettledMm) break;

        const SidePair step = settlingStep(springing, rates, residual);
        rise.left += step.left;
        rise.right += step.right;
        const AxlePoses last = result.poses;
        result.poses = poseAxle(sides, rise, rackTravel, &last);
    }
    if (!everySideAssembled(sides, result.poses)) return result;
    if (unsettledMm > kAcceptedMm) {
        failToSettle(sides, &result.poses);
        return result;
    }

    const SidePair deflection{ groundRise.left - rise.left, groundRise.right - rise.right };
    result.loads = loadsOf(springing, rates, deflection);
    if (sides.left) settleOnRoad(*sides.left, deflection.left, &result.poses.left);
    if (sides.right) settleOnRoad(*sides.right, deflection.right, &result.poses.right);
    return result;
}

} // namespace suspkin
