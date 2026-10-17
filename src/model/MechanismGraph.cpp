#include "model/MechanismGraph.h"

#include <QHash>

namespace suspkin {
namespace {

Vec3 positionOf(const Hardpoint& point)
{
    return Vec3(point.coord[0], point.coord[1], point.coord[2]);
}

/// The bodies the role graph is made of, named the way the built-in template's
/// parts are, so a role graph and a configuration graph of the built-in
/// template describe one car in one vocabulary.
struct RoleBodies {
    int lowerArm = -1;
    int upperArm = -1;
    int upright = -1;
    int tieRod = -1;
    int pushrod = -1;
    int rocker = -1;
    int damper = -1;
    int barArm = -1;
    int dropLink = -1;
};

RoleBodies addRoleBodies(MechanismGraph& graph)
{
    RoleBodies bodies;
    bodies.lowerArm = graph.bodyFor(QStringLiteral("Lower wishbone"));
    bodies.upperArm = graph.bodyFor(QStringLiteral("Upper wishbone"));
    bodies.upright = graph.bodyFor(QStringLiteral("Upright"));
    bodies.tieRod = graph.bodyFor(QStringLiteral("Tie rod"));
    bodies.pushrod = graph.bodyFor(QStringLiteral("Pushrod"));
    bodies.rocker = graph.bodyFor(QStringLiteral("Rocker"));
    bodies.damper = graph.bodyFor(QStringLiteral("Damper"), true);
    bodies.barArm = graph.bodyFor(QStringLiteral("Anti-roll bar arm"));
    bodies.dropLink = graph.bodyFor(QStringLiteral("Anti-roll drop link"));
    return bodies;
}

/// The body a pushrod's or a drop link's outer end is carried by.
int mountBody(const RoleBodies& bodies, DropLinkMount mount)
{
    switch (mount) {
    case DropLinkMount::Rocker: return bodies.rocker;
    case DropLinkMount::UpperArm: return bodies.upperArm;
    case DropLinkMount::LowerArm: return bodies.lowerArm;
    case DropLinkMount::Upright: return bodies.upright;
    }
    return bodies.rocker;
}

int pushrodMountBody(const RoleBodies& bodies, PushrodMount mount)
{
    switch (mount) {
    case PushrodMount::UpperArm: return bodies.upperArm;
    case PushrodMount::LowerArm: return bodies.lowerArm;
    case PushrodMount::Upright: return bodies.upright;
    }
    return bodies.upperArm;
}

} // namespace

int MechanismGraph::indexOfPoint(const QString& name) const
{
    for (std::size_t i = 0; i < points.size(); ++i)
        if (points[i].name == name) return static_cast<int>(i);
    return -1;
}

int MechanismGraph::indexOfBody(const QString& name) const
{
    for (std::size_t i = 0; i < bodies.size(); ++i)
        if (bodies[i].name == name) return static_cast<int>(i);
    return kNoBody;
}

int MechanismGraph::bodyFor(const QString& name, bool compressible)
{
    if (bodies.empty()) bodies.push_back(GraphBody{ BodyCatalog::ground(), false });
    if (name.isEmpty()) return kNoBody;
    const int existing = indexOfBody(name);
    if (existing != kNoBody) return existing;
    bodies.push_back(GraphBody{ name, compressible });
    return static_cast<int>(bodies.size()) - 1;
}

QStringList cornerPointNames(const LinkageTemplate& templ, const QString& cornerToken,
                             bool mirrored, const MirrorSpec& mirror, const HardpointTable& table)
{
    QStringList names;
    const auto take = [&](const QString& name) {
        if (!name.isEmpty() && table.indexOf(name) >= 0 && !names.contains(name)) names << name;
    };

    for (const PartTemplate& part : templ.parts) {
        if (!part.perCorner) continue;
        for (const ChainTemplate& chain : part.chains) {
            if (!chain.drawsOn(cornerToken)) continue;
            for (const QString& pattern : chain.points)
                take(resolvePointName(pattern, cornerToken, mirrored, mirror,
                                      templ.mechanism.formerNames, table));
        }
    }
    const MechanismTemplate named =
        instantiateMechanism(templ.mechanism, cornerToken, mirrored, mirror, table);
    for (const QString& name : named.allNames()) take(name);
    return names;
}

MechanismGraph configGraph(const QStringList& names, const HardpointConfigMap& config,
                           const LinkageTemplate& templ, const HardpointTable& table)
{
    QHash<QString, bool> compressible;
    for (const PartTemplate& part : templ.parts)
        compressible.insert(partBodyName(part), part.compliance == PartCompliance::Compressible);

    MechanismGraph graph;
    graph.bodyFor(QString()); // the chassis, even for a corner with no points
    for (const QString& name : names) {
        const Hardpoint* point = table.find(name);
        const auto entry = config.constFind(name);
        if (!point || entry == config.constEnd() || entry->type == PointType::Unassigned) continue;

        GraphPoint node;
        node.name = name;
        node.design = positionOf(*point);
        node.type = entry->type;
        node.body1 = graph.bodyFor(entry->part1, compressible.value(entry->part1));
        node.body2 = graph.bodyFor(entry->part2, compressible.value(entry->part2));
        graph.points.push_back(node);
    }
    return graph;
}

MechanismGraph roleGraph(const MechanismTemplate& mechanism, const HardpointTable& table)
{
    MechanismGraph graph;
    const RoleBodies bodies = addRoleBodies(graph);
    const int ground = MechanismGraph::kGround;

    const auto add = [&](const QString& name, PointType type, int body1, int body2) {
        const Hardpoint* point = name.isEmpty() ? nullptr : table.find(name);
        if (!point || graph.indexOfPoint(name) >= 0) return;
        graph.points.push_back(GraphPoint{ name, positionOf(*point), type, body1, body2 });
    };
    const auto has = [&table](const QString& name) {
        return !name.isEmpty() && table.indexOf(name) >= 0;
    };

    add(mechanism.lowerFront, PointType::ToBody, bodies.lowerArm, ground);
    add(mechanism.lowerRear, PointType::ToBody, bodies.lowerArm, ground);
    add(mechanism.lowerOuter, PointType::Solved, bodies.lowerArm, bodies.upright);
    add(mechanism.upperFront, PointType::ToBody, bodies.upperArm, ground);
    add(mechanism.upperRear, PointType::ToBody, bodies.upperArm, ground);
    add(mechanism.upperOuter, PointType::Solved, bodies.upperArm, bodies.upright);
    add(mechanism.tieRodInboard, PointType::ToBody, bodies.tieRod, ground);
    add(mechanism.tieRodOutboard, PointType::Solved, bodies.tieRod, bodies.upright);

    add(mechanism.wheelCenter, PointType::Dependent, bodies.upright, MechanismGraph::kNoBody);
    add(mechanism.wheelAxis, PointType::Dependent, bodies.upright, MechanismGraph::kNoBody);
    add(mechanism.contactPatch, PointType::ContactPatch, bodies.upright, MechanismGraph::kNoBody);
    for (const QString& name : mechanism.carried)
        add(name, PointType::Dependent, bodies.upright, MechanismGraph::kNoBody);

    // The spring end hangs together or not at all: a rocker short of a point
    // has nothing to turn about, and what hangs off it goes with it.
    const bool rocker = mechanism.hasRocker() && has(mechanism.pushrodOuter)
                        && has(mechanism.pushrodInner) && has(mechanism.rockerPivot)
                        && has(mechanism.rockerAxis);
    if (rocker) {
        add(mechanism.pushrodOuter, PointType::Solved, bodies.pushrod,
            pushrodMountBody(bodies, mechanism.pushrodMount));
        add(mechanism.pushrodInner, PointType::Solved, bodies.pushrod, bodies.rocker);
        add(mechanism.rockerPivot, PointType::ToBody, bodies.rocker, ground);
        add(mechanism.rockerAxis, PointType::ToBody, bodies.rocker, ground);
        if (has(mechanism.damperInboard) && has(mechanism.damperOutboard)) {
            add(mechanism.damperOutboard, PointType::Solved, bodies.damper, bodies.rocker);
            add(mechanism.damperInboard, PointType::ToBody, bodies.damper, ground);
        }
    }

    const bool bar = mechanism.hasAntiRoll() && has(mechanism.antiRollDropLinkOuter)
                     && has(mechanism.antiRollArmEnd) && has(mechanism.antiRollArmRoot)
                     && (mechanism.antiRollMount != DropLinkMount::Rocker || rocker);
    if (bar) {
        add(mechanism.antiRollDropLinkOuter, PointType::Solved, bodies.dropLink,
            mountBody(bodies, mechanism.antiRollMount));
        add(mechanism.antiRollArmEnd, PointType::Solved, bodies.dropLink, bodies.barArm);
        add(mechanism.antiRollArmRoot, PointType::ToBody, bodies.barArm, ground);
        add(mechanism.antiRollBearing, PointType::ToBody, bodies.barArm, ground);
    }
    return graph;
}

} // namespace suspkin
