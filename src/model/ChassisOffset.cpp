#include "model/ChassisOffset.h"

#include "geom/MeshQuery.h"

#include <QCoreApplication>

#include <cmath>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("ChassisOffset", text); }

/// Closer to the surface than this, a point has no side of it to be on.
constexpr double kOnSurface = 1e-6;
/// How close to the distance asked for counts as there. Far below anything a
/// workbook is measured to.
constexpr double kTolerance = 1e-6;
/// Enough for any surface a chassis is made of; one that has not settled by
/// then has no single place to put the point.
constexpr int kMaxSteps = 64;

} // namespace

ChassisOffset offsetFromChassis(const MeshQuery& chassis, const Vec3& point, double clearance)
{
    ChassisOffset out;
    if (!(clearance >= 0.0)) {
        out.error = tr("A distance from the chassis cannot be negative.");
        return out;
    }
    const std::optional<Vec3> first = chassis.nearestPoint(point);
    if (!first) {
        out.error = tr("There is no chassis to measure from.");
        return out;
    }
    out.distanceBefore = distance(point, *first);
    if (out.distanceBefore < kOnSurface && clearance > 0.0) {
        out.error = tr("The point is on the chassis surface, so there is no telling which side "
                       "of it to move it to. Move it off the surface a little first.");
        return out;
    }

    Vec3 at = point;
    Vec3 nearest = *first;
    for (int step = 0; step < kMaxSteps; ++step) {
        const double gap = distance(at, nearest);
        if (std::abs(gap - clearance) < kTolerance) {
            out.position = at;
            return out;
        }
        // Along the line from the surface through the point: square to it.
        const Vec3 away = gap < kOnSurface ? Vec3() : (at - nearest) / gap;
        at = nearest + away * clearance;
        nearest = *chassis.nearestPoint(at);
    }
    out.error = tr("The chassis around the point has no single place %1 mm from it -- a sharp "
                   "inside corner, perhaps. Move the point nearer where it should go first.")
                    .arg(clearance, 0, 'f', 1);
    return out;
}

} // namespace suspkin
