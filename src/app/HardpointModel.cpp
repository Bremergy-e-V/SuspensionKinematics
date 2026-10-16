#include "app/HardpointModel.h"

#include <QLocale>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <utility>

namespace suspkin {
namespace {

/// Enough to read a millimetre coordinate at the precision anyone works to,
/// without turning the column into a wall of digits. The stored value keeps its
/// full double precision either way.
constexpr int kDisplayDecimals = 3;

/// What an empty body or a rigid joint reads as. A blank cell looks like a
/// column that failed to draw; a dash looks like an answer.
const QChar kEmDash(0x2014);

int axisOf(int column) { return column - HardpointModel::XColumn; }

QStringList pointTypeLabels()
{
    QStringList labels;
    for (const PointType type : kPointTypes) labels.append(pointTypeLabel(type));
    return labels;
}

/// The int a Type cell is edited with, back into a type. Anything outside the
/// enum is refused rather than clamped: it can only be a bug on the way in.
bool pointTypeFromInt(int value, PointType* type)
{
    for (const PointType candidate : kPointTypes) {
        if (static_cast<int>(candidate) != value) continue;
        *type = candidate;
        return true;
    }
    return false;
}

} // namespace

HardpointModel::HardpointModel(QObject* parent) : QAbstractTableModel(parent) {}

bool HardpointModel::isCoordinateColumn(int column)
{
    return column >= XColumn && column <= ZColumn;
}

bool HardpointModel::isConfigColumn(int column)
{
    return column >= TypeColumn && column <= BushingColumn;
}

void HardpointModel::setTable(HardpointTable table)
{
    beginResetModel();
    m_table = std::move(table);
    endResetModel();
}

void HardpointModel::clear()
{
    beginResetModel();
    m_table = HardpointTable{};
    m_config.clear();
    endResetModel();
}

bool HardpointModel::insertPoint(int row, const Hardpoint& point)
{
    if (!hardpointNameProblem(point.name, m_table).isEmpty()) return false;

    const int at = std::clamp(row, 0, rowCount());
    beginInsertRows(QModelIndex(), at, at);
    m_table.points.insert(m_table.points.begin() + at, point);
    endInsertRows();
    return true;
}

void HardpointModel::removePoints(std::vector<int> rows)
{
    // Bottom up, so taking one row out never moves another still to be taken.
    std::sort(rows.begin(), rows.end(), std::greater<int>());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());

    for (const int row : rows) {
        if (row < 0 || row >= rowCount()) continue;
        beginRemoveRows(QModelIndex(), row, row);
        m_config.remove(m_table.points[static_cast<std::size_t>(row)].name);
        m_table.points.erase(m_table.points.begin() + row);
        endRemoveRows();
    }
}

bool HardpointModel::renamePoint(int row, const QString& requested)
{
    if (row < 0 || row >= rowCount()) return false;

    // Whatever was typed around the name is not part of it; the reader would
    // take it off again on the way back in.
    const QString name = requested.trimmed();
    Hardpoint& point = m_table.points[static_cast<std::size_t>(row)];
    if (name == point.name) return false; // nothing to store, nothing to save

    const QString problem = hardpointNameProblem(name, m_table, row);
    if (!problem.isEmpty()) {
        emit editRejected(row, problem);
        return false;
    }

    const QString from = point.name;
    point.name = name;

    // The name is the key, so what was stored under it moves with it: the
    // configuration, and the provenance of anything that was mirrored from it.
    if (m_config.contains(from)) m_config.insert(name, m_config.take(from));
    for (Hardpoint& other : m_table.points)
        if (other.mirrorOf == from) other.mirrorOf = name;

    emit dataChanged(index(row, IndexColumn), index(row, ColumnCount - 1));
    emit pointRenamed(row, from, name);
    return true;
}

void HardpointModel::setConfig(HardpointConfigMap config)
{
    m_config = std::move(config);
    if (rowCount() == 0) return;
    // Not a reset: the rows are the same rows, and resetting would throw away
    // the selection and whatever the user was scrolled to.
    emit dataChanged(index(0, IndexColumn), index(rowCount() - 1, ColumnCount - 1));
}

void HardpointModel::setBodyCatalog(BodyCatalog catalog)
{
    m_catalog = std::move(catalog);
    if (rowCount() == 0) return;
    // The values did not change, but what they are checked against did, so the
    // annotations on them may have.
    emit dataChanged(index(0, IndexColumn), index(rowCount() - 1, ColumnCount - 1),
                     { Qt::ToolTipRole, IssueLevelRole, IssueTextRole });
}

HardpointConfig HardpointModel::configAt(int row) const
{
    if (row < 0 || row >= rowCount()) return {};
    return m_config.value(m_table.points[static_cast<std::size_t>(row)].name);
}

void HardpointModel::setSolvedPoints(std::optional<QSet<QString>> names)
{
    if (m_solved == names) return;
    m_solved = std::move(names);
    if (rowCount() == 0) return;
    emit dataChanged(index(0, TypeColumn), index(rowCount() - 1, TypeColumn),
                     { Qt::DisplayRole, Qt::ToolTipRole, IssueLevelRole, IssueTextRole,
                       UnsolvedRole });
    // The status dot sits beside the name.
    emit dataChanged(index(0, NameColumn), index(rowCount() - 1, NameColumn),
                     { Qt::ToolTipRole, IssueLevelRole, IssueTextRole });
}

bool HardpointModel::isUnsolved(int row) const
{
    if (!m_solved || row < 0 || row >= rowCount()) return false;
    const PointType type = configAt(row).type;
    return (type == PointType::Solved || type == PointType::Welded)
           && !m_solved->contains(m_table.points[static_cast<std::size_t>(row)].name);
}

std::vector<ConfigIssue> HardpointModel::issuesAt(int row) const
{
    if (row < 0 || row >= rowCount()) return {};
    const HardpointConfig config = configAt(row);
    // A point nobody has touched is not a point with something wrong with it.
    if (config.isEmpty()) return {};
    std::vector<ConfigIssue> issues = validateHardpointConfig(config, m_catalog);
    if (isUnsolved(row)) {
        issues.push_back(ConfigIssue{
            ConfigIssueLevel::Warning,
            tr("Not solved: the solver does not move this point, so while simulating it stays "
               "where the table has it. The linkage it belongs to is incomplete -- a rocker "
               "without its pushrod, say -- or its corner does not solve.") });
    }
    return issues;
}

int HardpointModel::configuredCount() const
{
    int count = 0;
    for (const Hardpoint& point : m_table.points)
        if (m_config.value(point.name).type != PointType::Unassigned) ++count;
    return count;
}

int HardpointModel::issueCount() const
{
    int count = 0;
    for (int row = 0; row < rowCount(); ++row)
        if (!issuesAt(row).empty()) ++count;
    return count;
}

int HardpointModel::rowCount(const QModelIndex& parent) const
{
    return parent.isValid() ? 0 : static_cast<int>(m_table.points.size());
}

int HardpointModel::columnCount(const QModelIndex& parent) const
{
    return parent.isValid() ? 0 : ColumnCount;
}

QVariant HardpointModel::configDisplay(int row, const HardpointConfig& config, int column) const
{
    switch (column) {
    case TypeColumn: return isUnsolved(row)
                                ? tr("Not solved")
                                : pointTypeLabel(config.type);
    case Part1Column: return config.part1.isEmpty() ? QString(kEmDash) : config.part1;
    case Part2Column: return config.part2.isEmpty() ? QString(kEmDash) : config.part2;
    case BushingColumn:
        return config.bushing == kNoBushing ? QString(kEmDash) : QString::number(config.bushing);
    default: return {};
    }
}

QVariant HardpointModel::tooltipFor(int row, int column) const
{
    const Hardpoint& point = m_table.points[static_cast<std::size_t>(row)];
    const HardpointConfig config = configAt(row);

    QString text;
    switch (column) {
    case TypeColumn:
        text = QStringLiteral("%1\n%2").arg(pointTypeLabel(config.type),
                                            pointTypeDescription(config.type));
        break;
    case Part1Column:
    case Part2Column:
        text = tr("The two bodies that meet at %1.\n\nWhat this column may name comes from the "
                  "project's linkage template.")
                   .arg(point.name);
        break;
    case BushingColumn:
        text = tr("The compliance bushing acting at %1, by index into your stiffness and damping "
                  "map. %2 is a rigid joint.")
                   .arg(point.name)
                   .arg(kNoBushing);
        break;
    default:
        text = QStringLiteral("%1\nX %2\nY %3\nZ %4 (mm)")
                   .arg(point.name)
                   .arg(point.coord[0], 0, 'f', 4)
                   .arg(point.coord[1], 0, 'f', 4)
                   .arg(point.coord[2], 0, 'f', 4);
        if (isGhost(row))
            text += QLatin1String("\n\n")
                    + tr("The mirror of %1. It follows %1 and is moved by moving %1.")
                          .arg(point.mirrorOf);
        break;
    }

    const QStringList issues = issueMessages(issuesAt(row));
    if (issues.isEmpty()) return text;
    return text + QStringLiteral("\n\n") + issues.join(QStringLiteral("\n"));
}

QVariant HardpointModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid() || index.row() >= rowCount()) return {};

    const int row = index.row();
    const int column = index.column();
    const Hardpoint& point = m_table.points[static_cast<std::size_t>(row)];
    const HardpointConfig config = configAt(row);

    switch (role) {
    case Qt::DisplayRole:
        switch (column) {
        case IndexColumn: return row + 1;
        case NameColumn: return point.name;
        case XColumn:
        case YColumn:
        case ZColumn: return QLocale().toString(point.coord[axisOf(column)], 'f', kDisplayDecimals);
        default: return configDisplay(row, config, column);
        }

    case Qt::EditRole:
        // The unrounded value, so opening an editor and closing it again cannot
        // truncate a coordinate that was never touched. It is also the role the
        // view sorts on, so every column answers it with something comparable.
        switch (column) {
        case IndexColumn: return row + 1;
        case NameColumn: return point.name;
        case XColumn:
        case YColumn:
        case ZColumn: return point.coord[axisOf(column)];
        case TypeColumn: return static_cast<int>(config.type);
        case Part1Column: return config.part1;
        case Part2Column: return config.part2;
        case BushingColumn: return config.bushing;
        default: return {};
        }

    case Qt::ToolTipRole: return tooltipFor(row, column);

    case Qt::TextAlignmentRole:
        switch (column) {
        case IndexColumn:
        case BushingColumn: return int(Qt::AlignCenter);
        case XColumn:
        case YColumn:
        case ZColumn: return int(Qt::AlignRight | Qt::AlignVCenter);
        default: return int(Qt::AlignLeft | Qt::AlignVCenter);
        }

    case PointTypeRole: return static_cast<int>(config.type);

    case ChoicesRole:
        if (column == TypeColumn) return pointTypeLabels();
        if (column == Part1Column || column == Part2Column) return m_catalog.bodies;
        return {};

    case IssueLevelRole: {
        const std::vector<ConfigIssue> issues = issuesAt(row);
        if (issues.empty()) return -1;
        return static_cast<int>(hasError(issues) ? ConfigIssueLevel::Error
                                                 : ConfigIssueLevel::Warning);
    }

    case IssueTextRole: return issueMessages(issuesAt(row)).join(QLatin1Char('\n'));

    case UnsolvedRole: return isUnsolved(row);

    case GhostRole: return isGhost(row);

    default: return {};
    }
}

QVariant HardpointModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Vertical) return role == Qt::DisplayRole ? QVariant(section + 1) : QVariant();

    if (role == Qt::DisplayRole) {
        switch (section) {
        case IndexColumn: return QStringLiteral("#");
        case NameColumn: return tr("Point Name");
        case XColumn: return tr("X [mm]");
        case YColumn: return tr("Y [mm]");
        case ZColumn: return tr("Z [mm]");
        case TypeColumn: return tr("Point Type");
        case Part1Column: return tr("Part 1");
        case Part2Column: return tr("Part 2");
        case BushingColumn: return tr("Bushing");
        default: return {};
        }
    }

    if (role == Qt::ToolTipRole) {
        switch (section) {
        case IndexColumn: return tr("The point's place in the workbook, which is the order it is "
                                    "written back in.");
        case NameColumn: return tr("The name the workbook gives this point, and the key the parts, "
                                   "the solver and the configuration find it by. Renaming a point "
                                   "the workbook holds takes its old rows out of the workbook "
                                   "and adds new ones when the workbook is next written.");
        case TypeColumn: return tr("What the solver does with this point: hold it to the chassis, "
                                   "solve for it, or carry it along with a body that moves.");
        case Part1Column:
        case Part2Column: return tr("The two bodies that meet at this point, from the project's "
                                    "linkage template.");
        case BushingColumn: return tr("Which compliance bushing acts here, by index. %1 is a "
                                      "rigid joint.")
                                       .arg(kNoBushing);
        default: return {};
        }
    }

    if (role == Qt::TextAlignmentRole) {
        if (section == IndexColumn || section == BushingColumn) return int(Qt::AlignCenter);
        if (isCoordinateColumn(section)) return int(Qt::AlignRight | Qt::AlignVCenter);
        return int(Qt::AlignLeft | Qt::AlignVCenter);
    }

    return {};
}

Qt::ItemFlags HardpointModel::flags(const QModelIndex& index) const
{
    Qt::ItemFlags result = QAbstractTableModel::flags(index);
    if (!index.isValid()) return result;
    const bool lockedCoordinate = isCoordinateColumn(index.column()) && isGhost(index.row());
    if (index.column() == NameColumn || (isCoordinateColumn(index.column()) && !lockedCoordinate)
        || isConfigColumn(index.column()))
        result |= Qt::ItemIsEditable;
    return result;
}

bool HardpointModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if (role != Qt::EditRole || !index.isValid() || index.row() >= rowCount()) return false;
    if (index.column() == NameColumn) return renamePoint(index.row(), value.toString());
    if (isCoordinateColumn(index.column())) return setCoordinate(index, value);
    if (isConfigColumn(index.column())) return setConfigField(index.row(), index.column(), value);
    return false;
}

bool HardpointModel::setCoordinate(const QModelIndex& index, const QVariant& value)
{
    bool ok = false;
    const double parsed = value.toDouble(&ok);
    if (!ok) return false;

    if (isGhost(index.row())) {
        rejectGhostEdit(index.row());
        return false;
    }
    const int axis = axisOf(index.column());
    const double stored = m_table.points[static_cast<std::size_t>(index.row())].coord[axis];
    // Returning false for a no-op keeps the document from being marked dirty by
    // someone opening an editor and pressing Enter.
    if (stored == parsed) return false;

    if (rigidPartners(index.row()).isEmpty()) {
        // The value typed, not the old one plus a difference: the two can
        // differ in the last bit, and that would be an edit nobody made.
        m_table.points[static_cast<std::size_t>(index.row())].coord[axis] = parsed;
        emit dataChanged(index, index, { Qt::DisplayRole, Qt::EditRole, Qt::ToolTipRole });
    } else {
        Vec3 delta;
        (axis == 0 ? delta.x : axis == 1 ? delta.y : delta.z) = parsed - stored;
        translateWithPartners(index.row(), delta);
        m_table.points[static_cast<std::size_t>(index.row())].coord[axis] = parsed;
    }
    updateGhostsOf(index.row());
    emit coordinateChanged(index.row());
    return true;
}

bool HardpointModel::movePoint(int row, const Vec3& to)
{
    if (row < 0 || row >= rowCount()) return false;
    if (isGhost(row)) {
        rejectGhostEdit(row);
        return false;
    }
    const Hardpoint& point = m_table.points[static_cast<std::size_t>(row)];
    const Vec3 delta(to.x - point.coord[0], to.y - point.coord[1], to.z - point.coord[2]);
    if (delta.x == 0.0 && delta.y == 0.0 && delta.z == 0.0) return false;

    translateWithPartners(row, delta);
    // Exactly where it was asked to go, for the same reason as a typed value.
    Hardpoint& moved = m_table.points[static_cast<std::size_t>(row)];
    moved.coord[0] = to.x;
    moved.coord[1] = to.y;
    moved.coord[2] = to.z;
    updateGhostsOf(row);
    emit coordinateChanged(row);
    return true;
}

void HardpointModel::translateWithPartners(int row, const Vec3& delta)
{
    QList<int> rows = rigidPartners(row);
    rows.prepend(row);
    const double step[3] = { delta.x, delta.y, delta.z };
    for (const int moved : std::as_const(rows)) {
        Hardpoint& point = m_table.points[static_cast<std::size_t>(moved)];
        // An axis the move does not touch is left exactly as it was.
        for (int k = 0; k < 3; ++k)
            if (step[k] != 0.0) point.coord[k] += step[k];
        emit dataChanged(index(moved, XColumn), index(moved, ZColumn),
                         { Qt::DisplayRole, Qt::EditRole, Qt::ToolTipRole });
    }
}

void HardpointModel::setRigidGroups(std::vector<QStringList> groups)
{
    m_rigidGroups = std::move(groups);
}

QList<int> HardpointModel::rigidPartners(int row) const
{
    QList<int> partners;
    if (row < 0 || row >= rowCount()) return partners;
    const QString& name = m_table.points[static_cast<std::size_t>(row)].name;
    for (const QStringList& group : m_rigidGroups) {
        if (!group.contains(name)) continue;
        for (const QString& member : group) {
            const int other = m_table.indexOf(member);
            if (other >= 0 && other != row && !partners.contains(other) && !isGhost(other))
                partners << other;
        }
    }
    return partners;
}

void HardpointModel::setMirrorAxis(MirrorAxis axis)
{
    m_mirrorAxis = axis;
}

bool HardpointModel::isGhost(int row) const
{
    if (row < 0 || row >= rowCount()) return false;
    const Hardpoint& point = m_table.points[static_cast<std::size_t>(row)];
    return point.isMirrored() && m_table.indexOf(point.mirrorOf) >= 0;
}

std::vector<bool> HardpointModel::ghostFlags() const
{
    std::vector<bool> flags(m_table.points.size(), false);
    for (int row = 0; row < rowCount(); ++row) flags[static_cast<std::size_t>(row)] = isGhost(row);
    return flags;
}

QList<int> HardpointModel::ghostsOf(int row) const
{
    QList<int> ghosts;
    const QString& name = m_table.points[static_cast<std::size_t>(row)].name;
    for (int other = 0; other < rowCount(); ++other)
        if (other != row && m_table.points[static_cast<std::size_t>(other)].mirrorOf == name)
            ghosts << other;
    return ghosts;
}

QList<int> HardpointModel::followers(int row) const
{
    QList<int> moved = rigidPartners(row);
    QList<int> all = moved;
    moved.prepend(row);
    for (const int source : std::as_const(moved))
        for (const int ghost : ghostsOf(source))
            if (!all.contains(ghost)) all << ghost;
    return all;
}

void HardpointModel::updateGhostsOf(int row)
{
    QList<int> sources = rigidPartners(row);
    sources.prepend(row);
    const int flipped = m_mirrorAxis == MirrorAxis::X ? 0 : (m_mirrorAxis == MirrorAxis::Y ? 1 : 2);
    for (const int source : std::as_const(sources)) {
        const Hardpoint& from = m_table.points[static_cast<std::size_t>(source)];
        for (const int ghost : ghostsOf(source)) {
            Hardpoint& to = m_table.points[static_cast<std::size_t>(ghost)];
            // Exactly the reflection: the two coordinates the mirror does not
            // touch are the source's own doubles, the third its negation.
            for (int k = 0; k < 3; ++k) to.coord[k] = k == flipped ? -from.coord[k] : from.coord[k];
            emit dataChanged(index(ghost, XColumn), index(ghost, ZColumn),
                             { Qt::DisplayRole, Qt::EditRole, Qt::ToolTipRole });
        }
    }
}

void HardpointModel::rejectGhostEdit(int row)
{
    const Hardpoint& point = m_table.points[static_cast<std::size_t>(row)];
    emit editRejected(row, tr("%1 is the mirror of %2. It follows %2: move %2 and %1 moves with "
                              "it.")
                               .arg(point.name, point.mirrorOf));
}

bool HardpointModel::setConfigField(int row, int column, const QVariant& value)
{
    const QString& name = m_table.points[static_cast<std::size_t>(row)].name;
    const HardpointConfig current = m_config.value(name);
    HardpointConfig candidate = current;

    switch (column) {
    case TypeColumn: {
        PointType type = PointType::Unassigned;
        if (!pointTypeFromInt(value.toInt(), &type)) return false;
        candidate.type = type;
        break;
    }
    case Part1Column: candidate.part1 = value.toString(); break;
    case Part2Column: candidate.part2 = value.toString(); break;
    case BushingColumn: {
        bool ok = false;
        const int bushing = value.toInt(&ok);
        if (!ok) return false;
        candidate.bushing = bushing;
        break;
    }
    default: return false;
    }

    if (candidate == current) return false; // nothing to store, nothing to save

    // The boundary. Anything that could not mean something is refused here,
    // before the store holds it; anything merely unfinished goes in and comes
    // back out as a warning on the row.
    const std::vector<ConfigIssue> issues = validateHardpointConfig(candidate, m_catalog);
    if (hasError(issues)) {
        for (const ConfigIssue& issue : issues) {
            if (issue.level != ConfigIssueLevel::Error) continue;
            emit editRejected(row, issue.message);
            break;
        }
        return false;
    }

    m_config.insert(name, candidate);
    // One row, not the table: an edit to a cell can change the annotation on
    // its neighbours, and nothing beyond that.
    emit dataChanged(index(row, IndexColumn), index(row, ColumnCount - 1),
                     { Qt::DisplayRole, Qt::EditRole, Qt::ToolTipRole, PointTypeRole,
                       IssueLevelRole, IssueTextRole });
    emit configChanged(row);
    return true;
}

} // namespace suspkin
