#include "model/HardpointMirror.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("HardpointMirror", text); }

int axisIndex(MirrorAxis axis)
{
    switch (axis) {
    case MirrorAxis::X: return 0;
    case MirrorAxis::Y: return 1;
    case MirrorAxis::Z: return 2;
    }
    return 1;
}

/// Closer than this, two positions are the same place. A thousandth of a
/// millimetre is below anything a workbook is measured to.
constexpr double kSamePlaceMm = 1e-3;
/// Nearer the mirror plane than this, a point is its own mirror image.
constexpr double kOnPlaneMm = 1e-6;

bool samePlace(const Hardpoint& a, const Hardpoint& b)
{
    for (int k = 0; k < 3; ++k)
        if (std::abs(a.coord[k] - b.coord[k]) > kSamePlaceMm) return false;
    return true;
}

bool occupied(const HardpointTable& table, const Hardpoint& place)
{
    return std::any_of(table.points.begin(), table.points.end(),
                       [&place](const Hardpoint& point) { return samePlace(point, place); });
}

} // namespace

bool MirrorSpec::operator==(const MirrorSpec& other) const
{
    return axis == other.axis && naming == other.naming && affix == other.affix
        && findText == other.findText && replaceText == other.replaceText
        && caseSensitive == other.caseSensitive && updateExisting == other.updateExisting
        && skipMirrored == other.skipMirrored;
}

QString mirroredName(const QString& name, const MirrorSpec& spec)
{
    if (name.isEmpty()) return {};

    QString result;
    switch (spec.naming) {
    case MirrorNaming::Suffix:
        if (spec.affix.isEmpty()) return {};
        result = name + spec.affix;
        break;
    case MirrorNaming::Prefix:
        if (spec.affix.isEmpty()) return {};
        result = spec.affix + name;
        break;
    case MirrorNaming::Replace: {
        if (spec.findText.isEmpty()) return {};
        const Qt::CaseSensitivity sensitivity =
            spec.caseSensitive ? Qt::CaseSensitive : Qt::CaseInsensitive;
        if (!name.contains(spec.findText, sensitivity)) return {};
        result = name;
        result.replace(spec.findText, spec.replaceText, sensitivity);
        break;
    }
    }

    // A rule that maps a name onto itself would have the mirror overwrite its
    // own source, which is never what was meant.
    return (result == name) ? QString() : result;
}

MirrorOutcome mirrorHardpoints(const HardpointTable& table, const std::vector<int>& rows,
                               const MirrorSpec& spec)
{
    MirrorOutcome outcome;
    outcome.table = table;

    const int count = static_cast<int>(table.points.size());

    // The selection is resolved against the input, so appending as we go cannot
    // feed a freshly mirrored point back in as a source.
    std::vector<int> sources;
    if (rows.empty()) {
        sources.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i) sources.push_back(i);
    } else {
        sources = rows;
        std::sort(sources.begin(), sources.end());
        sources.erase(std::unique(sources.begin(), sources.end()), sources.end());
    }

    QStringList unnamed;
    QStringList kept;
    int mirrorsSkipped = 0;

    for (const int row : sources) {
        if (row < 0 || row >= count) continue;
        const Hardpoint& source = table.points[static_cast<std::size_t>(row)];

        if (spec.skipMirrored && source.isMirrored()) {
            ++mirrorsSkipped;
            ++outcome.skipped;
            continue;
        }

        const QString name = mirroredName(source.name, spec);
        if (name.isEmpty()) {
            if (unnamed.size() < 3) unnamed.append(source.name);
            ++outcome.skipped;
            continue;
        }

        Hardpoint mirrored = reflectedPoint(source, spec.axis);
        mirrored.name = name;
        mirrored.mirrorOf = source.name;

        const int existing = outcome.table.indexOf(name);
        if (existing < 0) {
            outcome.table.points.push_back(std::move(mirrored));
            ++outcome.added;
            continue;
        }
        if (!spec.updateExisting) {
            if (kept.size() < 3) kept.append(name);
            ++outcome.skipped;
            continue;
        }
        outcome.table.points[static_cast<std::size_t>(existing)] = std::move(mirrored);
        ++outcome.updated;
    }

    if (mirrorsSkipped > 0) {
        outcome.notes.append(tr("%1 point(s) were already mirrors and were left alone.")
                                 .arg(mirrorsSkipped));
    }
    if (!unnamed.isEmpty()) {
        outcome.notes.append(tr("The naming rule does not apply to some points, for example %1.")
                                 .arg(unnamed.join(QStringLiteral(", "))));
    }
    if (!kept.isEmpty()) {
        outcome.notes.append(tr("Some mirrored points already existed and were kept as they are, "
                                "for example %1.")
                                 .arg(kept.join(QStringLiteral(", "))));
    }
    return outcome;
}

Hardpoint reflectedPoint(const Hardpoint& point, MirrorAxis axis)
{
    const int flipped = axisIndex(axis);
    Hardpoint reflected = point;
    reflected.coord[flipped] = -point.coord[flipped];
    // Negating -0.0 gives 0.0 back, but a point sitting on the mirror plane
    // reads better as a plain zero than as its negative.
    if (reflected.coord[flipped] == 0.0) reflected.coord[flipped] = 0.0;
    return reflected;
}

bool isGhostAt(const HardpointTable& table, int row)
{
    if (row < 0 || row >= static_cast<int>(table.points.size())) return false;
    const Hardpoint& point = table.points[static_cast<std::size_t>(row)];
    return point.isMirrored() && table.indexOf(point.mirrorOf) >= 0;
}

HardpointTable withoutGhosts(const HardpointTable& table)
{
    HardpointTable kept;
    kept.points.reserve(table.points.size());
    for (int row = 0; row < static_cast<int>(table.points.size()); ++row)
        if (!isGhostAt(table, row)) kept.points.push_back(table.points[static_cast<std::size_t>(row)]);
    return kept;
}

std::vector<Hardpoint> missingMirrors(const HardpointTable& table, const MirrorSpec& spec)
{
    const int flipped = axisIndex(spec.axis);
    HardpointTable completed = table;
    std::vector<Hardpoint> missing;
    for (const Hardpoint& source : table.points) {
        if (source.isMirrored() || std::abs(source.coord[flipped]) <= kOnPlaneMm) continue;
        const QString name = mirroredName(source.name, spec);
        if (name.isEmpty() || completed.indexOf(name) >= 0) continue;

        Hardpoint ghost = reflectedPoint(source, spec.axis);
        if (occupied(completed, ghost)) continue;
        ghost.name = name;
        ghost.mirrorOf = source.name;
        // Into the table checked against, so two sources whose names mirror
        // onto one another cannot both claim the same far-side name.
        completed.points.push_back(ghost);
        missing.push_back(std::move(ghost));
    }
    return missing;
}

QString mirrorAxisToString(MirrorAxis axis)
{
    switch (axis) {
    case MirrorAxis::X: return QStringLiteral("x");
    case MirrorAxis::Y: return QStringLiteral("y");
    case MirrorAxis::Z: return QStringLiteral("z");
    }
    return QStringLiteral("y");
}

MirrorAxis mirrorAxisFromString(const QString& text, MirrorAxis fallback)
{
    const QString key = text.trimmed().toLower();
    if (key == QLatin1String("x")) return MirrorAxis::X;
    if (key == QLatin1String("y")) return MirrorAxis::Y;
    if (key == QLatin1String("z")) return MirrorAxis::Z;
    return fallback;
}

QString mirrorNamingToString(MirrorNaming naming)
{
    switch (naming) {
    case MirrorNaming::Suffix: return QStringLiteral("suffix");
    case MirrorNaming::Prefix: return QStringLiteral("prefix");
    case MirrorNaming::Replace: return QStringLiteral("replace");
    }
    return QStringLiteral("suffix");
}

MirrorNaming mirrorNamingFromString(const QString& text, MirrorNaming fallback)
{
    const QString key = text.trimmed().toLower();
    if (key == QLatin1String("suffix")) return MirrorNaming::Suffix;
    if (key == QLatin1String("prefix")) return MirrorNaming::Prefix;
    if (key == QLatin1String("replace")) return MirrorNaming::Replace;
    return fallback;
}

} // namespace suspkin
