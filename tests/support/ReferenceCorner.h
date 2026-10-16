#pragma once

// The closed-form corner solve as it was before the solver was built from the
// configuration, kept unchanged as the reference the generic solve is held to.
// Nothing in the application uses it.

#include "model/SuspensionSolver.h"

namespace suspkin {

class ReferenceCornerSolver {
public:
    /// Resolve @p mechanism against @p table. Returns nothing, and sets
    /// @p error, when the table does not hold enough of it to solve.
    ///
    /// @p alignment, when given, is which way the wheel points at design, and
    /// it wins over anything the table says: a wheel axis point is then carried
    /// on the axis these angles give rather than read for its direction, and a
    /// contact patch only says how high the ground is. Without one the table is
    /// read the way it always was -- the wheel axis point, else the patch under
    /// the wheel centre, else an upright wheel.
    static std::optional<ReferenceCornerSolver> bind(const MechanismTemplate& mechanism,
                                            const HardpointTable& table, QString* error,
                                            const std::optional<StaticAlignment>& alignment
                                            = std::nullopt);

    const MechanismTemplate& mechanism() const { return m_mechanism; }

    /// Where the design camber and toe came from.
    WheelAttitude wheelAttitude() const { return m_attitude; }

    /// Whether a steering rack drives this corner. False for an axle whose toe
    /// link inboard end is simply bolted to the chassis: it then ignores rack
    /// travel entirely rather than being dragged sideways by a rack it has not
    /// got.
    bool isSteered() const { return m_steered; }

    /// Which side of the car this is, from the sign of the wheel centre's y.
    /// It is what camber and toe signs are read against: +y is left in ISO 8855.
    bool isLeft() const { return m_left; }
    /// +1 on the left, -1 on the right. Multiplying a lateral number by this
    /// turns it from "toward +y" into "outboard", which is what the measures
    /// actually mean.
    double side() const { return m_left ? 1.0 : -1.0; }

    /// The direction of the anti-roll bar's own axis, which runs through this
    /// corner's arm root. Without it the bar is taken to run along y, which is
    /// what a transverse U-bar does anyway.
    ///
    /// It is a direction rather than the far side's root on purpose. Both arms
    /// are on one straight bar, so both have to measure their rotation about the
    /// same direction: give the two of them opposite ones and a pure bump, where
    /// the bar does nothing, reads as the arms twisting against each other.
    ///
    /// A corner whose table names the bar's bearing already has its axis, and
    /// keeps it: this does nothing there.
    void setAntiRollAxis(const Vec3& direction);

    /// Whether the pushrod and rocker are bound, and the damper and the bar
    /// with them. Without a rocker the corner still solves -- the wheel's own
    /// kinematics are the wishbones and the tie rod -- and only the measures
    /// read off those bodies are missing.
    bool hasRocker() const { return m_hasRocker; }
    bool hasDamper() const { return m_hasDamper; }
    bool hasAntiRoll() const { return m_hasAntiRoll; }

    /// Whether the bar's axis came from a bearing point in the table, rather
    /// than from a guess or from the far side's arm root.
    bool antiRollAxisFromBearing() const { return m_antiRollAxisFromBearing; }

    /// The corner as the workbook holds it: the zero every change is measured
    /// from.
    const CornerPose& designPose() const { return m_design; }

    /// Pose at a given lower-wishbone angle. @p previous picks the assembly
    /// branch: a circle meets a sphere twice, and which of the two is this
    /// suspension is decided by where it was a moment ago. Passing nothing
    /// continues from the design position, which is right for any single pose
    /// inside the normal travel.
    CornerPose poseAtArmAngle(double angle, double rackTravel = 0.0,
                              const CornerPose* previous = nullptr) const;

    /// Pose with the wheel centre @p travel millimetres above where the workbook
    /// put it. Bump is positive.
    CornerPose poseAtWheelTravel(double travel, double rackTravel = 0.0,
                                 const CornerPose* previous = nullptr) const;

    /// Pose with the contact patch @p rise millimetres above where it started,
    /// which is how a roll sweep asks for a corner: the ground moves, not the
    /// wheel.
    CornerPose poseAtContactPatchRise(double rise, double rackTravel = 0.0,
                                      const CornerPose* previous = nullptr) const;

private:
    ReferenceCornerSolver() = default;

    /// The single-degree-of-freedom solve, from an angle to everything else.
    bool solveAt(double angle, double rackTravel, const CornerPose* previous,
                 CornerPose* out) const;
    /// Fill in camber, toe, caster and the rest from a solved geometry.
    void measure(CornerPose* pose) const;
    /// Drive @p target(pose) to @p value by moving the arm angle. @p pick is
    /// what is being aimed at -- the wheel centre's height, or the contact
    /// patch's.
    CornerPose driveTo(double value, double rackTravel, const CornerPose* previous,
                       double (*pick)(const CornerPose&)) const;
    /// Where the drop link's upper end is in @p pose, carried by whichever body
    /// the mechanism mounts it on. @p upright is that pose's upright motion.
    Vec3 dropLinkOuterAt(const CornerPose& pose, const Rigid& upright) const;
    /// Turn the bar's arm until it is a drop link's length from where the drop
    /// link's upper end has gone. False, with the error set, when it cannot be.
    bool solveAntiRoll(const Rigid& upright, const CornerPose* previous, CornerPose* pose) const;

    MechanismTemplate m_mechanism;
    bool m_left = true;
    /// The mechanism names a rack point, and it is the point this solve can
    /// actually move: the inboard tie rod end.
    bool m_steered = false;

    // Design coordinates, resolved once.
    Vec3 m_lowerFront, m_lowerRear, m_lowerOuter;
    Vec3 m_upperFront, m_upperRear, m_upperOuter;
    Vec3 m_tieRodInboard, m_tieRodOutboard;
    Vec3 m_wheelCenter, m_contactPatch;
    Vec3 m_pushrodOuter, m_pushrodInner;
    Vec3 m_rockerPivot, m_rockerAxisPoint;
    Vec3 m_damperInboard, m_damperOutboard;
    Vec3 m_antiRollDropLinkOuter, m_antiRollArmEnd, m_antiRollArmRoot, m_antiRollBearing;
    std::vector<PosedPoint> m_carried; ///< design positions of the carried points

    Axis m_lowerAxis, m_upperAxis, m_rockerAxis, m_antiRollAxis;
    Circle m_upperCircle, m_pushrodInnerCircle, m_antiRollCircle;

    // Link lengths, which are the constraints.
    double m_uprightLowerUpper = 0.0;
    double m_uprightLowerTie = 0.0;
    double m_uprightUpperTie = 0.0;
    double m_tieRodLength = 0.0;
    double m_pushrodLength = 0.0;
    double m_dropLinkLength = 0.0;

    /// The table names a point on the wheel's axis, so the wheel's orientation
    /// is measured rather than inferred from a patch under it.
    bool m_hasWheelAxis = false;
    Vec3 m_wheelAxisPoint;
    /// The table names a contact patch of its own. It is still drawn and still
    /// moved; what it supplies to the solve is the tyre radius.
    bool m_hasContactPatch = false;
    /// A contact patch can be had at all -- named or computed. Only a wheel
    /// lying flat has none.
    bool m_hasGround = false;
    double m_tireRadius = 0.0;
    bool m_hasRocker = false;
    bool m_hasDamper = false;
    bool m_hasAntiRoll = false;
    /// The table names the bar's bearing, so its axis is known on this side
    /// alone and nothing the axle guesses replaces it.
    bool m_antiRollAxisFromBearing = false;

    /// The design wheel spin axis, outboard. From the static angles when the
    /// project states them; otherwise from the wheel axis point when the table
    /// has one -- the only way a table can state static toe -- and otherwise
    /// from the contact patch sitting under the wheel centre, which carries the
    /// workbook's static camber but knows nothing about toe. That is why toe is
    /// reported as a change as well as an absolute.
    Vec3 m_designSpinAxis;
    WheelAttitude m_attitude = WheelAttitude::Upright;
    CornerPose m_design;
};

} // namespace suspkin
