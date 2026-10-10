#pragma once

#include "geom/Vec3.h"

#include <QString>

#include <optional>

namespace suspkin {

class MeshQuery;

/// Where a point goes to be a given distance from the chassis.
struct ChassisOffset {
    /// Empty when there is no such place to go; @ref error says why.
    std::optional<Vec3> position;
    QString error;
    /// How far the point was from the chassis before it was moved.
    double distanceBefore = 0.0;
};

/// Move @p point square to the chassis surface until it is @p clearance from
/// it -- the shortest move that does it. The point stays on the side of the
/// surface it was on: the distance is unsigned, as an STL has no reliable
/// inside, so "further from the surface" is all there is to go by.
///
/// The surface square under the point is not always square under where it
/// ends up -- a corner, a curved tube -- so the move is repeated from there
/// until the distance holds. A point lying on the surface has no side to be
/// moved to, and is refused rather than guessed at.
///
/// Pure, so it is tested without a window.
ChassisOffset offsetFromChassis(const MeshQuery& chassis, const Vec3& point, double clearance);

} // namespace suspkin
