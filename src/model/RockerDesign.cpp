#include "model/RockerDesign.h"

#include "model/GeomSolve.h"
#include "model/HardpointMirror.h"
#include "model/Linkage.h"
#include "model/Mechanism.h"

#include <QCoreApplication>
#include <QJsonObject>

#include <algorithm>
#include <cmath>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("RockerDesign", text); }

constexpr double kDegToRad = 0.017453292519943295769236907684886;

/// How far ahead of the pivot the second axis point is put. Only the
/// direction means anything to the solver; this is far enough to pick in the
/// viewport.
constexpr double kAxisReach = 50.0;

double rad(double degrees) { return degrees * kDegToRad; }

/// The rocker's axis from its two angles: forward, turned in top view, then
/// tilted up.
Vec3 axisDirection(const RockerDesign& d)
{
    const double yaw = rad(d.axisYaw);
    const double pitch = rad(d.axisPitch);
    return Vec3(std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch));
}

/// The rocker's own frame: its axis, and the "up" its pushrod arm's angle is
/// measured from.
///
/// Both are a rocker lying along the car with up straight up, turned as one
/// body: first about the vertical by the yaw, then tilted by the tilt. "Up"
/// therefore tilts with the axis rather than being the world's up squeezed
/// into the rocker's plane -- which has no answer for a steep axis and turned
/// the arms half way round as the axis passed 80 degrees. Every axis, standing
/// upright included, has a frame, and the rocker turns smoothly into it.
struct RockerFrame {
    Axis axis;      ///< through the origin: arms are directions
    Vec3 up;        ///< unit, square to the axis
    double inboard; ///< +1 where turning right-handed about the axis is inboard
};

RockerFrame frameOf(const RockerDesign& d)
{
    const double yaw = rad(d.axisYaw);
    const double tilt = rad(d.axisPitch);
    RockerFrame frame;
    frame.axis = Axis{ Vec3(), axisDirection(d) };
    frame.up = Vec3(-std::sin(tilt) * std::cos(yaw), -std::sin(tilt) * std::sin(yaw), std::cos(tilt));
    // Lying along the car, right-handed about +x turns up toward -y: inboard on
    // the left (+y) side. Turning the whole body keeps that, so the side alone
    // says which way inboard is.
    frame.inboard = d.pivot.y >= 0.0 ? 1.0 : -1.0;
    return frame;
}

/// @p direction turned inboard by @p degrees about the rocker's axis.
Vec3 turnInboard(const RockerFrame& frame, const Vec3& direction, double degrees)
{
    return rotateAbout(direction, frame.axis, frame.inboard * rad(degrees));
}

/// @p v with its component along @p axis taken out: where it falls on the
/// rocker's plane.
Vec3 onPlane(const Vec3& v, const Vec3& axis) { return v - axis * dot(v, axis); }

/// The angle, in degrees and positive inboard, from @p from to @p to -- both on
/// the rocker's plane.
double inboardAngle(const RockerFrame& frame, const Vec3& from, const Vec3& to)
{
    const Vec3& a = frame.axis.direction;
    const double turned = std::atan2(dot(a, cross(from, to)), dot(from, to));
    return frame.inboard * turned / kDegToRad;
}

/// The rocker's roles in @p names, as they are written: the points it lays out.
struct RockerNames {
    QString pivot, axis, pushrodInner, damperEye, damperMount, antiRollPickup;
};

RockerNames namesOf(const MechanismTemplate& names)
{
    RockerNames out{ names.rockerPivot,    names.rockerAxis,    names.pushrodInner,
                     names.damperOutboard, names.damperInboard, QString() };
    if (names.antiRollMount == DropLinkMount::Rocker) out.antiRollPickup = names.antiRollDropLinkOuter;
    return out;
}

void addPoint(HardpointTable* table, const QString& name, const Vec3& at)
{
    // A role the template does not name is not written; nor is one named twice.
    if (name.isEmpty() || table->indexOf(name) >= 0) return;
    Hardpoint point;
    point.name = name;
    point.coord[0] = at.x;
    point.coord[1] = at.y;
    point.coord[2] = at.z;
    table->points.push_back(point);
}

HardpointTable bindRocker(const RockerPoints& points, const RockerNames& names)
{
    HardpointTable table;
    addPoint(&table, names.pivot, points.pivot);
    addPoint(&table, names.axis, points.axisPoint);
    addPoint(&table, names.pushrodInner, points.pushrodInner);
    addPoint(&table, names.damperEye, points.damperEye);
    addPoint(&table, names.damperMount, points.damperMount);
    if (points.hasAntiRollArm) addPoint(&table, names.antiRollPickup, points.antiRollPickup);
    return table;
}

QStringList groupOf(const RockerNames& names, const HardpointTable& table)
{
    QStringList group;
    for (const QString& name : { names.pivot, names.axis, names.pushrodInner, names.damperEye,
                                 names.damperMount, names.antiRollPickup })
        if (!name.isEmpty() && table.indexOf(name) >= 0 && !group.contains(name)) group << name;
    return group;
}

// --- the manifest ------------------------------------------------------------

struct NumberKey {
    const char* key;
    double RockerDesign::*member;
};
constexpr NumberKey kNumbers[] = {
    { "axisYaw", &RockerDesign::axisYaw },
    { "axisPitch", &RockerDesign::axisPitch },
    { "pushrodArm", &RockerDesign::pushrodArm },
    { "pushrodArmAngle", &RockerDesign::pushrodArmAngle },
    { "damperArm", &RockerDesign::damperArm },
    { "damperArmAngle", &RockerDesign::damperArmAngle },
    { "damperAngle", &RockerDesign::damperAngle },
    { "damperLength", &RockerDesign::damperLength },
    { "antiRollArm", &RockerDesign::antiRollArm },
    { "antiRollArmAngle", &RockerDesign::antiRollArmAngle },
};

QJsonObject rockerToJson(const RockerDesign& d)
{
    QJsonObject object;
    object.insert(QStringLiteral("corner"), d.corner);
    object.insert(QStringLiteral("pivot"), QJsonArray{ d.pivot.x, d.pivot.y, d.pivot.z });
    for (const NumberKey& number : kNumbers) object.insert(QLatin1String(number.key), d.*number.member);
    object.insert(QStringLiteral("antiRollArmUsed"), d.antiRollArmUsed);
    return object;
}

RockerDesign rockerFromJson(const QJsonObject& object)
{
    RockerDesign d;
    d.corner = object.value(QStringLiteral("corner")).toString();
    const QJsonArray pivot = object.value(QStringLiteral("pivot")).toArray();
    if (pivot.size() == 3)
        d.pivot = Vec3(pivot[0].toDouble(d.pivot.x), pivot[1].toDouble(d.pivot.y),
                       pivot[2].toDouble(d.pivot.z));
    for (const NumberKey& number : kNumbers)
        d.*number.member = object.value(QLatin1String(number.key)).toDouble(d.*number.member);
    d.antiRollArmUsed = object.value(QStringLiteral("antiRollArmUsed")).toBool(d.antiRollArmUsed);
    return d;
}

} // namespace

RockerPoints layOutRocker(const RockerDesign& d)
{
    RockerPoints out;
    if (!(d.pushrodArm > 0.0 && d.damperArm > 0.0 && d.damperLength > 0.0)) {
        out.error = tr("The rocker's arms and the damper all need a length.");
        return out;
    }
    if (d.antiRollArmUsed && !(d.antiRollArm > 0.0)) {
        out.error = tr("The anti-roll bar's arm on the rocker needs a length.");
        return out;
    }

    const RockerFrame frame = frameOf(d);
    const Vec3 toPushrod = turnInboard(frame, frame.up, d.pushrodArmAngle);
    const Vec3 toDamper = turnInboard(frame, toPushrod, d.damperArmAngle);
    // At the eye, from the arm back to the pivot, by beta.
    const Vec3 alongDamper = turnInboard(frame, toDamper * -1.0, d.damperAngle);

    out.pivot = d.pivot;
    out.axisPoint = d.pivot + frame.axis.direction * kAxisReach;
    out.pushrodInner = d.pivot + toPushrod * d.pushrodArm;
    out.damperEye = d.pivot + toDamper * d.damperArm;
    out.damperMount = out.damperEye + alongDamper * d.damperLength;
    if (d.antiRollArmUsed) {
        out.hasAntiRollArm = true;
        out.antiRollPickup =
            d.pivot + turnInboard(frame, toPushrod, d.antiRollArmAngle) * d.antiRollArm;
    }
    out.ok = true;
    return out;
}

DesignPlan planRockers(const std::vector<RockerDesign>& designs, const LinkageTemplate& templ,
                       const MirrorSpec& mirror, const HardpointTable& current,
                       const HardpointTable& baseline)
{
    DesignPlan plan;
    plan.table = current;
    plan.steering = templ.corners;
    if (!templ.mechanism.hasRocker()) {
        plan.error = tr("This project's linkage template has no rocker, so there are no names "
                        "to give the rocker's points.");
        return plan;
    }

    HardpointTable points;
    for (const RockerDesign& design : designs) {
        const RockerPoints laid = layOutRocker(design);
        if (!laid.ok) {
            plan.error = QStringLiteral("%1: %2").arg(design.corner, laid.error);
            return plan;
        }
        // Named against the table, so a point it still holds under a former
        // name is moved under that name rather than joined by a second one.
        const MechanismTemplate names =
            instantiateMechanism(templ.mechanism, design.corner, false, mirror, current);
        const RockerNames rocker = namesOf(names);
        if (design.antiRollArmUsed && rocker.antiRollPickup.isEmpty())
            plan.warnings << tr("%1: the linkage template does not hang the anti-roll bar's drop "
                                "link on the rocker, so its arm on the rocker is not written.")
                                 .arg(design.corner);
        const HardpointTable bound = bindRocker(laid, rocker);
        points.points.insert(points.points.end(), bound.points.begin(), bound.points.end());
    }
    if (points.isEmpty()) {
        plan.error = tr("There is no rocker to place.");
        return plan;
    }

    placeGeneratedPoints(&plan, points, mirror, current, baseline);
    // Nothing to write into the template, but a plan with no corners is
    // read as one that failed.
    plan.corners.push_back(GeneratedCorner{});
    return plan;
}

std::vector<QStringList> rockerGroups(const std::vector<RockerDesign>& designs,
                                      const LinkageTemplate& templ, const MirrorSpec& mirror,
                                      const HardpointTable& table)
{
    std::vector<QStringList> groups;
    if (!templ.mechanism.hasRocker()) return groups;
    for (const RockerDesign& design : designs) {
        for (const bool mirrored : { false, true }) {
            const MechanismTemplate names =
                instantiateMechanism(templ.mechanism, design.corner, mirrored, mirror, table);
            const QStringList group = groupOf(namesOf(names), table);
            if (group.size() > 1) groups.push_back(group);
        }
    }
    return groups;
}

std::optional<RockerDesign> measureRocker(const QString& corner, const LinkageTemplate& templ,
                                          const MirrorSpec& mirror, const HardpointTable& table,
                                          const RockerDesign& fallback)
{
    const MechanismTemplate names = instantiateMechanism(templ.mechanism, corner, false, mirror, table);
    const auto at = [&table](const QString& name) -> std::optional<Vec3> {
        const Hardpoint* point = name.isEmpty() ? nullptr : table.find(name);
        if (!point) return std::nullopt;
        return Vec3(point->coord[0], point->coord[1], point->coord[2]);
    };
    const std::optional<Vec3> pivot = at(names.rockerPivot);
    const std::optional<Vec3> axisPoint = at(names.rockerAxis);
    const std::optional<Vec3> pushrodEnd = at(names.pushrodInner);
    if (!pivot || !axisPoint || !pushrodEnd) return std::nullopt;
    const Vec3 axis = (*axisPoint - *pivot).normalized();
    if (axis.lengthSquared() == 0.0) return std::nullopt;

    RockerDesign d = fallback;
    d.corner = corner;
    d.pivot = *pivot;
    d.axisYaw = std::atan2(axis.y, axis.x) / kDegToRad;
    d.axisPitch = std::asin(std::clamp(axis.z, -1.0, 1.0)) / kDegToRad;
    const RockerFrame frame = frameOf(d);

    const Vec3 toPushrod = onPlane(*pushrodEnd - *pivot, axis);
    d.pushrodArm = toPushrod.length();
    d.pushrodArmAngle = inboardAngle(frame, frame.up, toPushrod);

    const std::optional<Vec3> eye = at(names.damperOutboard);
    if (eye) {
        const Vec3 toDamper = onPlane(*eye - *pivot, axis);
        d.damperArm = toDamper.length();
        d.damperArmAngle = inboardAngle(frame, toPushrod, toDamper);
        if (const std::optional<Vec3> mount = at(names.damperInboard)) {
            const Vec3 damper = onPlane(*mount - *eye, axis);
            d.damperLength = damper.length();
            d.damperAngle = inboardAngle(frame, toDamper * -1.0, damper);
        }
    }
    const std::optional<Vec3> pickup =
        names.antiRollMount == DropLinkMount::Rocker ? at(names.antiRollDropLinkOuter) : std::nullopt;
    if (pickup) {
        const Vec3 toPickup = onPlane(*pickup - *pivot, axis);
        d.antiRollArmUsed = true;
        d.antiRollArm = toPickup.length();
        d.antiRollArmAngle = inboardAngle(frame, toPushrod, toPickup);
    }
    return d;
}

RockerDesign followTable(RockerDesign design, const LinkageTemplate& templ,
                         const MirrorSpec& mirror, const HardpointTable& table)
{
    const MechanismTemplate names =
        instantiateMechanism(templ.mechanism, design.corner, false, mirror, table);
    if (const Hardpoint* pivot = table.find(names.rockerPivot))
        design.pivot = Vec3(pivot->coord[0], pivot->coord[1], pivot->coord[2]);
    return design;
}

QJsonArray rockerDesignsToJson(const std::vector<RockerDesign>& designs)
{
    QJsonArray array;
    for (const RockerDesign& design : designs) array.append(rockerToJson(design));
    return array;
}

std::vector<RockerDesign> rockerDesignsFromJson(const QJsonArray& array)
{
    std::vector<RockerDesign> designs;
    for (const QJsonValue& value : array) designs.push_back(rockerFromJson(value.toObject()));
    return designs;
}

} // namespace suspkin
