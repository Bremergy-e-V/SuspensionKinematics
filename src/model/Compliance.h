#pragma once

#include "model/SuspensionSolver.h"

namespace suspkin {

/// How stiff one axle's springs, bar and tyres are.
///
/// One set per axle -- the far side takes the same, as it does the static
/// angles. The spring is taken to sit on the damper, so its travel is the
/// damper's; the bar is stated at the bar, the way its data sheet does, and the
/// kinematics take it to the wheel.
struct AxleRates {
    double springRate = 0.0;   ///< N/mm, along the damper
    double antiRollRate = 0.0; ///< Nm per degree of twist; zero for no bar
    double tyreRate = 0.0;     ///< N/mm, vertical

    /// Whether there is enough here to work a force out of a movement. A tyre
    /// rate is what a tyre deflection needs on top (@ref hasTyre).
    bool hasSpring() const { return springRate > 0.0; }
    bool hasTyre() const { return tyreRate > 0.0; }

    bool operator==(const AxleRates& other) const
    {
        return springRate == other.springRate && antiRollRate == other.antiRollRate
               && tyreRate == other.tyreRate;
    }
    bool operator!=(const AxleRates& other) const { return !(*this == other); }
};

/// What the springs, the bar and the tyres of an axle are carrying at one
/// position, as changes from the design position. That position is the car
/// standing at its static ride height, so every preload drops out and none has
/// to be stated.
struct AxleLoads {
    bool valid = false;

    /// Newtons along each damper; positive pushes back against bump.
    double leftSpringForce = 0.0;
    double rightSpringForce = 0.0;
    /// Newton metres in the bar, signed like its twist: left arm minus right.
    double antiRollTorque = 0.0;
    /// Newtons at each contact patch, vertical; positive is more load.
    double leftWheelLoad = 0.0;
    double rightWheelLoad = 0.0;
    /// Millimetres each tyre is squashed further than at design. Zero when the
    /// tyres are taken as rigid.
    double leftTyreDeflection = 0.0;
    double rightTyreDeflection = 0.0;
};

/// The two corners of one axle, either of which may be missing.
struct AxleSides {
    const CornerSolver* left = nullptr;
    const CornerSolver* right = nullptr;
};

/// One number for each side of an axle, in millimetres.
struct SidePair {
    double left = 0.0;
    double right = 0.0;
};

/// An axle posed with both of its sides.
struct AxlePoses {
    CornerPose left;
    CornerPose right;
};

/// What @p rates make of the axle standing in @p poses, the tyres taken as
/// rigid: the spring forces, the bar's torque, and what the two of them come to
/// at each contact patch.
///
/// The patch load is the springs' stored energy differentiated against the
/// patch's own rise, so a motion ratio that changes through the travel and a
/// bar that couples the two sides are both in it without being named.
/// Invalid when there is no damper for a spring to sit on.
AxleLoads suspensionLoads(const AxleSides& sides, const AxleRates& rates, const AxlePoses& poses,
                          double rackTravel);

/// An axle in equilibrium between its springs and its tyres.
struct CompliantAxle {
    AxlePoses poses;
    AxleLoads loads;
};

/// Pose an axle whose road has risen by @p groundRise under each contact patch,
/// measured from the body, with the tyres giving way.
///
/// The rise is shared out between suspension and tyre so that each patch
/// carries what its tyre's deflection makes: the springs and the bar push on
/// the patch, the tyre pushes back, and the two meet. Each returned pose is the
/// suspension's own, with its contact patch moved up onto the road the tyre has
/// been squashed against.
///
/// @p previous picks the assembly branch and starts the search, as it does for
/// a corner alone. Loads come back invalid when the rates do not allow it -- no
/// spring, no tyre rate, no damper -- or the mechanism jams on the way.
CompliantAxle solveCompliantAxle(const AxleSides& sides, const AxleRates& rates,
                                 const SidePair& groundRise, double rackTravel,
                                 const AxlePoses* previous = nullptr);

} // namespace suspkin
