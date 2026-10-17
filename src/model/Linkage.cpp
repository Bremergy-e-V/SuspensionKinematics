#include "model/Linkage.h"

#include <QCoreApplication>

#include <algorithm>
#include <utility>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("Linkage", text); }

struct KindName {
    PartKind kind;
    const char* text;
};

// The strings the template file uses. Adding a kind means adding a colour in
// ViewportWidget too, which is the only other place a kind means anything.
constexpr KindName kKindNames[] = {
    { PartKind::Wishbone, "wishbone" }, { PartKind::Link, "link" },
    { PartKind::Upright, "upright" },   { PartKind::Rocker, "rocker" },
    { PartKind::Damper, "damper" },     { PartKind::AntiRoll, "antiroll" },
    { PartKind::Wheel, "wheel" },       { PartKind::Other, "other" },
};

const QString kCornerToken = QStringLiteral("{corner}");
const QString kSideToken = QStringLiteral("{side}");

const char kRigid[] = "rigid";
const char kCompressible[] = "compressible";
const char kLine[] = "line";
const char kPivot[] = "pivot";

} // namespace

QString partComplianceToString(PartCompliance compliance)
{
    return QLatin1String(compliance == PartCompliance::Compressible ? kCompressible : kRigid);
}

PartCompliance partComplianceFromString(const QString& text, PartCompliance fallback)
{
    if (text.compare(QLatin1String(kRigid), Qt::CaseInsensitive) == 0)
        return PartCompliance::Rigid;
    if (text.compare(QLatin1String(kCompressible), Qt::CaseInsensitive) == 0)
        return PartCompliance::Compressible;
    return fallback;
}

PartCompliance defaultCompliance(PartKind kind)
{
    return kind == PartKind::Damper ? PartCompliance::Compressible : PartCompliance::Rigid;
}

QString chainStyleToString(ChainStyle style)
{
    return QLatin1String(style == ChainStyle::Pivot ? kPivot : kLine);
}

ChainStyle chainStyleFromString(const QString& text, ChainStyle fallback)
{
    if (text.compare(QLatin1String(kLine), Qt::CaseInsensitive) == 0) return ChainStyle::Line;
    if (text.compare(QLatin1String(kPivot), Qt::CaseInsensitive) == 0) return ChainStyle::Pivot;
    return fallback;
}

QString partKindToString(PartKind kind)
{
    for (const KindName& entry : kKindNames)
        if (entry.kind == kind) return QLatin1String(entry.text);
    return QStringLiteral("other");
}

PartKind partKindFromString(const QString& text, PartKind fallback)
{
    for (const KindName& entry : kKindNames)
        if (text.compare(QLatin1String(entry.text), Qt::CaseInsensitive) == 0) return entry.kind;
    return fallback;
}

int ResolvedChain::segmentCount() const
{
    const int count = static_cast<int>(points.size());
    if (count < 2) return 0;
    return closed && count > 2 ? count : count - 1;
}

int LinkagePart::segmentCount() const
{
    int total = 0;
    for (const ResolvedChain& chain : chains) total += chain.segmentCount();
    return total;
}

int Linkage::segmentCount() const
{
    int total = 0;
    for (const LinkagePart& part : parts) total += part.segmentCount();
    return total;
}

namespace {

/// One part for one corner and one side. Nothing is reported from here: whether
/// a missing point is worth mentioning depends on which side is being built,
/// and only the caller knows that.
struct Instance {
    LinkagePart part;
    QStringList missing; ///< names that are not in the table
    int found = 0;       ///< names that are, across every chain
    int wanted = 0;
};

constexpr int kMissing = -1;

/// What is left of a chain whose points are @p indices, some of them
/// kMissing. A chain that lost a point is still drawn through the ones it
/// kept -- a wishbone missing an inner pickup is more use as one leg than as
/// nothing -- but only between neighbours: joining the points either side of
/// a gap, or closing round it, would draw a member that is not there, such as
/// a pushrod straight to a damper on an axle without the rocker between them.
std::vector<ResolvedChain> drawableRuns(const std::vector<int>& indices, const ChainTemplate& chain)
{
    const auto gap = std::find(indices.begin(), indices.end(), kMissing);
    if (gap == indices.end()) {
        ResolvedChain whole{ indices, chain.closed, chain.style };
        if (whole.segmentCount() == 0) return {};
        return { whole };
    }

    // Closed, the run that wraps past the end goes on into the start, so the
    // walk begins just after a gap.
    std::vector<int> walk = indices;
    if (chain.closed) std::rotate(walk.begin(), walk.begin() + (gap - indices.begin()) + 1, walk.end());

    std::vector<ResolvedChain> runs;
    ResolvedChain run{ {}, false, chain.style };
    const auto finish = [&] {
        if (run.segmentCount() > 0) runs.push_back(run);
        run.points.clear();
    };
    for (const int index : walk) {
        if (index == kMissing) finish();
        else run.points.push_back(index);
    }
    finish();
    return runs;
}

Instance instantiatePart(const PartTemplate& source, const HardpointTable& table,
                         const CornerSpec& corner, bool mirrored, const MirrorSpec& mirror,
                         const FormerNames& formerNames, const QString& sideLabel)
{
    // A part that spells its points out is drawn through exactly those names:
    // no corner to substitute, no far side to mirror onto.
    const bool literal = !source.perCorner;

    Instance instance;
    instance.part.id = literal ? QStringLiteral("%1@literal").arg(source.id)
                               : QStringLiteral("%1@%2:%3")
                                     .arg(source.id, corner.token,
                                          mirrored ? QStringLiteral("mirror")
                                                   : QStringLiteral("base"));
    instance.part.label = source.label;
    instance.part.label.replace(kCornerToken, corner.label);
    instance.part.label.replace(kSideToken, sideLabel);
    instance.part.label = instance.part.label.simplified();
    if (instance.part.label.isEmpty()) instance.part.label = source.id;
    instance.part.kind = source.kind;
    instance.part.corner = corner.token;
    instance.part.mirrored = mirrored;

    for (const ChainTemplate& chain : source.chains) {
        if (!literal && !chain.drawsOn(corner.token)) continue;
        std::vector<int> indices; // kMissing where the table has no such point
        for (const QString& pattern : chain.points) {
            ++instance.wanted;
            const QString name =
                literal ? pattern
                        : resolvePointName(pattern, corner.token, mirrored, mirror, formerNames,
                                           table);
            const int index = name.isEmpty() ? kMissing : table.indexOf(name);
            indices.push_back(index < 0 ? kMissing : index);
            if (index >= 0) {
                ++instance.found;
                continue;
            }
            // An empty name means the mirror rule does not apply, which is
            // worth saying with the pattern it came from rather than blank.
            if (!chain.optional && !source.optional)
                instance.missing.append(name.isEmpty() ? pattern : name);
        }

        for (ResolvedChain& resolved : drawableRuns(indices, chain))
            instance.part.chains.push_back(std::move(resolved));
    }

    return instance;
}

} // namespace

Linkage buildLinkage(const LinkageTemplate& templ, const HardpointTable& table,
                     const MirrorSpec& mirror)
{
    Linkage linkage;
    if (templ.parts.empty() || table.isEmpty()) return linkage;

    // A template without corners is one that spells its point names out in full.
    std::vector<CornerSpec> corners = templ.corners;
    if (corners.empty()) corners.push_back(CornerSpec{});

    const auto report = [&linkage](const Instance& instance) {
        if (!instance.missing.isEmpty()) {
            linkage.warnings.append(tr("%1: no hardpoint named %2")
                                        .arg(instance.part.label,
                                             instance.missing.join(QStringLiteral(", "))));
        }
        if (!instance.part.chains.empty()) linkage.parts.push_back(instance.part);
    };

    for (const CornerSpec& corner : corners) {
        for (int side = 0; side < 2; ++side) {
            const bool mirrored = (side == 1);

            std::vector<Instance> instances;
            instances.reserve(templ.parts.size());
            int found = 0;
            for (const PartTemplate& source : templ.parts) {
                if (!source.perCorner) continue; // drawn once, below
                instances.push_back(instantiatePart(
                    source, table, corner, mirrored, mirror, templ.mechanism.formerNames,
                    mirrored ? templ.mirroredSideLabel : templ.baseSideLabel));
                found += instances.back().found;
            }

            // A corner or a side with not one point in the table is not missing
            // anything -- it is not on this car. A template describes a kind of
            // suspension, and a particular table may be one axle, or one side
            // that has not been mirrored yet. Warning per part about either
            // would bury the warnings that do matter.
            if (found == 0) continue;

            for (const Instance& instance : instances) report(instance);
        }
    }

    // Parts that name their points outright belong to no corner and no side, so
    // there is nothing for "not on this car" to mean: one that has lost a point
    // has lost it, and says so.
    for (const PartTemplate& source : templ.parts) {
        if (source.perCorner) continue;
        report(instantiatePart(source, table, CornerSpec{}, false, mirror, FormerNames{},
                               QString()));
    }

    return linkage;
}

} // namespace suspkin
