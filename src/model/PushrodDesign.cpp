#include "model/PushrodDesign.h"

#include "geom/MeshQuery.h"
#include "model/Mechanism.h"

#include <QCoreApplication>
#include <QJsonObject>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("PushrodDesign", text); }

struct LayoutName {
    PushrodLayout layout;
    const char* text;
};

constexpr LayoutName kLayoutNames[] = {
    { PushrodLayout::Rocker, "rocker" },
    { PushrodLayout::InlineDamper, "inlineDamper" },
    { PushrodLayout::DamperOnly, "damperOnly" },
};

Vec3 positionOf(const Hardpoint& point) { return Vec3(point.coord[0], point.coord[1], point.coord[2]); }

std::optional<Vec3> pointAt(const HardpointTable& table, const QString& name)
{
    const Hardpoint* point = name.isEmpty() ? nullptr : table.find(name);
    if (!point) return std::nullopt;
    return positionOf(*point);
}

/// Where the chassis end goes: looked for inboard of the ball joint at the
/// height asked for, or set across the car when there is no chassis to look on.
bool placeInboard(const PushrodDesign& design, const Vec3& ballJoint, double side,
                  const MeshQuery* chassis, PushrodPoints* out)
{
    const Vec3 start(ballJoint.x, ballJoint.y, design.inboardHeight);
    if (!chassis || chassis->isEmpty()) {
        out->inboard = Vec3(ballJoint.x, side * design.inboardY, design.inboardHeight);
        return true;
    }
    const Vec3 inboardward(0.0, -side, 0.0);
    const std::optional<RayHit> hit = chassis->castRay(start, inboardward);
    if (!hit) {
        out->error = tr("Looking inboard from the ball joint at z = %1 mm finds no chassis.")
                         .arg(design.inboardHeight);
        return false;
    }
    // Stood off back towards the wheel, which is the side it was found from.
    out->inboard = hit->point - inboardward * design.clearance;
    out->inboardOnChassis = true;
    return true;
}

/// The outboard end where the custom offset puts it: @p side turns "inboard"
/// into the sign of y.
void placeOutboardAtShift(const PushrodDesign& design, double side, PushrodPoints* out)
{
    const Vec3& shift = design.outboardShift;
    out->outboard = out->ballJoint + Vec3(shift.x, -side * shift.y, shift.z);
}

/// The outboard end: on the line from the ball joint to the chassis end, the
/// offset inboard of the ball joint across the car.
bool placeOutboard(const PushrodDesign& design, double side, PushrodPoints* out)
{
    if (design.customOutboard) {
        placeOutboardAtShift(design, side, out);
        return true;
    }
    const double span = side * (out->ballJoint.y - out->inboard.y);
    if (span <= 1e-6) {
        out->error = tr("The inboard end is not inboard of the ball joint.");
        return false;
    }
    if (design.outboardOffset < 0.0 || design.outboardOffset >= span) {
        out->error = tr("The outboard offset has to lie between the ball joint and the inboard "
                        "end, which are %1 mm apart across the car.")
                         .arg(span, 0, 'f', 1);
        return false;
    }
    out->outboard = out->ballJoint + (out->inboard - out->ballJoint) * (design.outboardOffset / span);
    return true;
}

bool placeDamperEye(const PushrodDesign& design, PushrodPoints* out)
{
    const double reach = distance(out->outboard, out->inboard);
    if (design.damperLength <= 0.0 || design.damperLength >= reach) {
        out->error = tr("A %1 mm damper does not fit in the %2 mm from the chassis to the "
                        "pushrod's outer end.")
                         .arg(design.damperLength, 0, 'f', 1)
                         .arg(reach, 0, 'f', 1);
        return false;
    }
    out->damperEye = out->inboard + (out->outboard - out->inboard).normalized() * design.damperLength;
    return true;
}

/// The body the template draws through both @p a and @p b, by name, or
/// @p fallback.
QString bodyDrawing(const LinkageTemplate& templ, const QString& a, const QString& b,
                    const QString& fallback)
{
    for (const PartTemplate& part : templ.parts)
        for (const ChainTemplate& chain : part.chains)
            if (chain.points.contains(a) && chain.points.contains(b)) return partBodyName(part);
    return fallback;
}

/// The parts that draw through @p pattern, by id.
QStringList partsDrawing(const LinkageTemplate& templ, const QString& pattern)
{
    QStringList ids;
    if (pattern.isEmpty()) return ids;
    for (const PartTemplate& part : templ.parts)
        for (const ChainTemplate& chain : part.chains)
            if (chain.points.contains(pattern) && !ids.contains(part.id)) ids << part.id;
    return ids;
}

HardpointConfig joint(PointType type, const QString& part1, const QString& part2 = QString())
{
    HardpointConfig config;
    config.type = type;
    config.part1 = part1;
    config.part2 = part2;
    return config;
}

/// What one design adds to the plan: its points, their settings, and what its
/// layout has no use for.
struct CornerWork {
    HardpointTable points;
    HardpointConfigMap config;
    QStringList removed;
    QStringList unassigned;
    QStringList optionalParts;
    QStringList warnings;

    void place(const QString& name, const Vec3& at, const HardpointConfig& settings)
    {
        if (name.isEmpty()) return;
        Hardpoint point;
        point.name = name;
        point.coord[0] = at.x;
        point.coord[1] = at.y;
        point.coord[2] = at.z;
        points.points.push_back(point);
        config.insert(name, settings);
    }
};

void planCorner(const PushrodDesign& design, const LinkageTemplate& templ, const MirrorSpec& mirror,
                const HardpointTable& current, const MeshQuery* chassis, CornerWork* work)
{
    const MechanismTemplate& roles = templ.mechanism;
    const MechanismTemplate named =
        instantiateMechanism(roles, design.corner, false, mirror, current);
    const bool upper = design.arm == PickupArm::Upper;
    const std::optional<Vec3> ballJoint =
        pointAt(current, upper ? named.upperOuter : named.lowerOuter);
    if (!ballJoint) {
        work->warnings << tr("%1: the table has no %2 ball joint to lay the pushrod off from.")
                              .arg(design.corner, upper ? tr("upper") : tr("lower"));
        return;
    }

    const PushrodPoints points =
        layOutPushrod(design, *ballJoint, pointAt(current, named.pushrodInner), chassis);
    if (!points.ok) {
        work->warnings << tr("%1: %2").arg(design.corner, points.error);
        return;
    }

    const QString ground = BodyCatalog::ground();
    const QString arm = upper ? bodyDrawing(templ, roles.upperFront, roles.upperOuter,
                                            QStringLiteral("Upper wishbone"))
                              : bodyDrawing(templ, roles.lowerFront, roles.lowerOuter,
                                            QStringLiteral("Lower wishbone"));
    const QString pushrod =
        bodyDrawing(templ, roles.pushrodOuter, roles.pushrodInner, QStringLiteral("Pushrod"));
    const QString damper =
        bodyDrawing(templ, roles.damperOutboard, roles.damperInboard, QStringLiteral("Damper"));
    const QStringList rocker = { named.rockerPivot, named.rockerAxis };
    const QStringList rockerParts = partsDrawing(templ, roles.rockerPivot);

    switch (design.layout) {
    case PushrodLayout::Rocker:
        work->place(named.pushrodOuter, points.outboard, joint(PointType::Solved, pushrod, arm));
        break;
    case PushrodLayout::InlineDamper:
        work->place(named.pushrodOuter, points.outboard, joint(PointType::Solved, pushrod, arm));
        // The pushrod's inner end and the damper's eye are one place: drawn as
        // the end of each, joined rigidly there.
        work->place(named.pushrodInner, points.damperEye, joint(PointType::Dependent, pushrod));
        work->place(named.damperOutboard, points.damperEye,
                    joint(PointType::Welded, pushrod, damper));
        work->place(named.damperInboard, points.inboard, joint(PointType::ToBody, damper, ground));
        work->removed << rocker;
        work->optionalParts << rockerParts;
        break;
    case PushrodLayout::DamperOnly:
        work->place(named.damperOutboard, points.outboard, joint(PointType::Solved, damper, arm));
        work->place(named.damperInboard, points.inboard, joint(PointType::ToBody, damper, ground));
        work->removed << rocker << named.pushrodOuter << named.pushrodInner;
        work->optionalParts << rockerParts << partsDrawing(templ, roles.pushrodOuter);
        break;
    }

    // A bar that hung on the rocker has nothing to hang on any more. Its points
    // are kept, but are no longer solved, until the drop link is put on an arm.
    if (design.layout != PushrodLayout::Rocker && roles.antiRollMount == DropLinkMount::Rocker) {
        for (const QString& name : { named.antiRollDropLinkOuter, named.antiRollArmEnd,
                                     named.antiRollArmRoot, named.antiRollBearing })
            if (!name.isEmpty() && current.indexOf(name) >= 0) work->unassigned << name;
        if (!work->unassigned.isEmpty())
            work->warnings << tr("%1: the anti-roll bar hung on the rocker. Its points are kept "
                                 "but no longer solved; give the drop link an arm to hang on in "
                                 "the Template Editor.")
                                  .arg(design.corner);
    }
}

void appendUnique(QStringList& list, const QStringList& more)
{
    for (const QString& entry : more)
        if (!entry.isEmpty() && !list.contains(entry)) list << entry;
}

} // namespace

QString pushrodLayoutToString(PushrodLayout layout)
{
    for (const LayoutName& entry : kLayoutNames)
        if (entry.layout == layout) return QLatin1String(entry.text);
    return QStringLiteral("rocker");
}

PushrodLayout pushrodLayoutFromString(const QString& text, PushrodLayout fallback)
{
    for (const LayoutName& entry : kLayoutNames)
        if (text.compare(QLatin1String(entry.text), Qt::CaseInsensitive) == 0) return entry.layout;
    return fallback;
}

PushrodPoints layOutPushrod(const PushrodDesign& design, const Vec3& ballJoint,
                            const std::optional<Vec3>& rockerPickup, const MeshQuery* chassis)
{
    PushrodPoints out;
    out.ballJoint = ballJoint;
    const double side = ballJoint.y >= 0.0 ? 1.0 : -1.0;

    if (design.layout == PushrodLayout::Rocker) {
        if (!rockerPickup) {
            out.error = tr("There is no rocker to run the pushrod to yet; place one with "
                           "Linkage > Rocker first.");
            return out;
        }
        out.inboard = *rockerPickup;
    } else if (!placeInboard(design, ballJoint, side, chassis, &out)) {
        return out;
    }

    if (!placeOutboard(design, side, &out)) return out;
    if (design.layout == PushrodLayout::InlineDamper && !placeDamperEye(design, &out)) return out;
    out.ok = true;
    return out;
}

std::vector<QStringList> pushrodGroups(const std::vector<PushrodDesign>& designs,
                                       const LinkageTemplate& templ, const MirrorSpec& mirror,
                                       const HardpointTable& table)
{
    std::vector<QStringList> groups;
    for (const PushrodDesign& design : designs) {
        if (design.layout != PushrodLayout::InlineDamper) continue;
        for (const bool mirrored : { false, true }) {
            const MechanismTemplate names =
                instantiateMechanism(templ.mechanism, design.corner, mirrored, mirror, table);
            const QStringList group = { names.pushrodInner, names.damperOutboard };
            if (table.indexOf(group[0]) >= 0 && table.indexOf(group[1]) >= 0) groups.push_back(group);
        }
    }
    return groups;
}

PushrodPlan planPushrods(const std::vector<PushrodDesign>& designs, const LinkageTemplate& templ,
                         const MirrorSpec& mirror, const HardpointTable& current,
                         const HardpointTable& baseline, const MeshQuery* chassis)
{
    PushrodPlan plan;
    plan.points.table = current;

    HardpointTable placed;
    QStringList removed;
    for (const PushrodDesign& design : designs) {
        CornerWork work;
        planCorner(design, templ, mirror, current, chassis, &work);
        placed.points.insert(placed.points.end(), work.points.points.begin(), work.points.points.end());
        for (auto it = work.config.constBegin(); it != work.config.constEnd(); ++it)
            plan.config.insert(it.key(), it.value());
        appendUnique(removed, work.removed);
        appendUnique(plan.unassigned, work.unassigned);
        appendUnique(plan.optionalParts, work.optionalParts);
        plan.points.warnings << work.warnings;
    }
    if (placed.isEmpty()) {
        plan.points.error = plan.points.warnings.isEmpty()
                                ? tr("No corner has a pushrod or damper to place.")
                                : plan.points.warnings.join(QLatin1Char('\n'));
        return plan;
    }

    placeGeneratedPoints(&plan.points, placed, mirror, current, baseline);
    // Something was planned: that is what ok() asks.
    plan.points.corners.push_back(GeneratedCorner{});

    // The far side is described by the near side alone.
    const HardpointConfigMap near = plan.config;
    for (auto it = near.constBegin(); it != near.constEnd(); ++it) {
        const QString far = mirroredName(it.key(), mirror);
        if (!far.isEmpty() && plan.points.table.indexOf(far) >= 0) plan.config.insert(far, *it);
    }
    for (const QString& name : QStringList(plan.unassigned)) {
        const QString far = mirroredName(name, mirror);
        if (!far.isEmpty() && current.indexOf(far) >= 0) appendUnique(plan.unassigned, { far });
    }
    for (const QString& name : plan.unassigned) plan.config.insert(name, HardpointConfig{});

    // What the layout has no use for goes, on both sides.
    for (const QString& name : QStringList(removed)) appendUnique(removed, { mirroredName(name, mirror) });
    for (const QString& name : removed) {
        const int row = plan.points.table.indexOf(name);
        if (row < 0) continue;
        plan.points.table.points.erase(plan.points.table.points.begin() + row);
        plan.removed << name;
    }
    return plan;
}

QJsonArray pushrodDesignsToJson(const std::vector<PushrodDesign>& designs)
{
    QJsonArray array;
    for (const PushrodDesign& design : designs) {
        QJsonObject object;
        object.insert(QStringLiteral("corner"), design.corner);
        object.insert(QStringLiteral("layout"), pushrodLayoutToString(design.layout));
        object.insert(QStringLiteral("arm"), design.arm == PickupArm::Upper
                                                 ? QStringLiteral("upper")
                                                 : QStringLiteral("lower"));
        object.insert(QStringLiteral("inboardHeight"), design.inboardHeight);
        object.insert(QStringLiteral("clearance"), design.clearance);
        object.insert(QStringLiteral("inboardY"), design.inboardY);
        object.insert(QStringLiteral("outboardOffset"), design.outboardOffset);
        object.insert(QStringLiteral("customOutboard"), design.customOutboard);
        object.insert(QStringLiteral("outboardShift"),
                      QJsonArray{ design.outboardShift.x, design.outboardShift.y,
                                  design.outboardShift.z });
        object.insert(QStringLiteral("damperLength"), design.damperLength);
        array.append(object);
    }
    return array;
}

std::vector<PushrodDesign> pushrodDesignsFromJson(const QJsonArray& array)
{
    std::vector<PushrodDesign> designs;
    const PushrodDesign defaults;
    for (const QJsonValue& value : array) {
        const QJsonObject object = value.toObject();
        PushrodDesign design;
        design.corner = object.value(QStringLiteral("corner")).toString();
        design.layout = pushrodLayoutFromString(object.value(QStringLiteral("layout")).toString());
        design.arm = object.value(QStringLiteral("arm")).toString() == QLatin1String("lower")
                         ? PickupArm::Lower
                         : PickupArm::Upper;
        design.inboardHeight =
            object.value(QStringLiteral("inboardHeight")).toDouble(defaults.inboardHeight);
        design.clearance = object.value(QStringLiteral("clearance")).toDouble(defaults.clearance);
        design.inboardY = object.value(QStringLiteral("inboardY")).toDouble(defaults.inboardY);
        design.outboardOffset =
            object.value(QStringLiteral("outboardOffset")).toDouble(defaults.outboardOffset);
        design.damperLength =
            object.value(QStringLiteral("damperLength")).toDouble(defaults.damperLength);
        design.customOutboard = object.value(QStringLiteral("customOutboard")).toBool(false);
        const QJsonArray shift = object.value(QStringLiteral("outboardShift")).toArray();
        if (shift.size() == 3)
            design.outboardShift = Vec3(shift[0].toDouble(), shift[1].toDouble(), shift[2].toDouble());
        if (!design.corner.isEmpty()) designs.push_back(design);
    }
    return designs;
}

} // namespace suspkin
