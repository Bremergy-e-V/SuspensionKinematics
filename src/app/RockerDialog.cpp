#include "app/RockerDialog.h"

#include "app/HardpointDelegates.h"
#include "app/RockerSketch.h"
#include "model/Linkage.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QTimer>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <algorithm>

namespace suspkin {
namespace {

/// As Generate from Design: a number typed is one plan, not one per digit.
constexpr int kPlanDelayMs = 120;

QString bulleted(const QStringList& lines)
{
    QString html;
    for (const QString& line : lines) html += QStringLiteral("<li>%1</li>").arg(line.toHtmlEscaped());
    return QStringLiteral("<ul style=\"margin-left: -20px\">%1</ul>").arg(html);
}

/// Where a column starts: the design the project has, its pivot following the
/// table; else the rocker the table has, read back; else the defaults.
RockerDesign seedFor(const CornerSpec& corner, const std::vector<RockerDesign>& designs,
                     const LinkageTemplate& templ, const MirrorSpec& mirror,
                     const HardpointTable& table, bool* used)
{
    const auto stored = std::find_if(designs.begin(), designs.end(), [&corner](const RockerDesign& d) {
        return d.corner == corner.token;
    });
    *used = stored != designs.end();
    if (*used) return followTable(*stored, templ, mirror, table);

    RockerDesign fallback;
    fallback.corner = corner.token;
    return measureRocker(corner.token, templ, mirror, table, fallback).value_or(fallback);
}

} // namespace

RockerDialog::RockerDialog(const std::vector<RockerDesign>& designs, const LinkageTemplate& templ,
                           const MirrorSpec& mirror, const HardpointTable& current,
                           const HardpointTable& baseline, QWidget* parent)
    : QDialog(parent), m_templ(templ), m_mirror(mirror), m_current(current), m_baseline(baseline)
{
    setWindowTitle(tr("Rocker"));

    using D = RockerDesign;
    m_fields = {
        { "Pivot and axis", "Pivot x", [](D& d) -> double& { return d.pivot.x; }, -10000, 10000,
          false, "Where the rocker turns, along the car (X), on the side being designed." },
        { nullptr, "Pivot y", [](D& d) -> double& { return d.pivot.y; }, -3000, 3000, false,
          "Where the rocker turns, across the car (Y). Its sign says which side is being "
          "designed; the other side comes from the mirror rule." },
        { nullptr, "Pivot z", [](D& d) -> double& { return d.pivot.z; }, -1000, 3000, false,
          "Where the rocker turns, its height (Z)." },
        { nullptr, "Axis, in top view", [](D& d) -> double& { return d.axisYaw; }, -180, 180, true,
          "The rocker's axis seen from above: 0 runs straight along the car (X), positive "
          "turns its front end toward +Y. The rocker turns with it." },
        { nullptr, "Axis, tilt", [](D& d) -> double& { return d.axisPitch; }, -90, 90, true,
          "The rocker's axis against the horizontal: positive raises its front end, 90 stands "
          "it upright so the rocker lies flat. The rocker tilts with it as one piece, so its "
          "arms keep their angles." },
        { "Pushrod arm (L1)", "L1, pivot to pushrod", [](D& d) -> double& { return d.pushrodArm; },
          1, 1000, false, "Distance on the rocker from its pivot to the pushrod's inner end." },
        { nullptr, "L1, turned from up", [](D& d) -> double& { return d.pushrodArmAngle; },
          -180, 180, true,
          "Where L1 points, seen along the rocker's axis: 0 is the rocker's up -- straight up "
          "while the axis is level, tilted with the axis when it is not -- and positive turns "
          "it inboard. This turns the whole rocker; every other arm follows it." },
        { "Damper (L2)", "L2, pivot to damper eye", [](D& d) -> double& { return d.damperArm; },
          1, 1000, false, "Distance on the rocker from its pivot to the damper's eye." },
        { nullptr, "Angle from L1 to L2", [](D& d) -> double& { return d.damperArmAngle; }, -180,
          180, true,
          "Angle on the rocker between the pushrod's arm and the damper's arm, seen along the "
          "rocker's axis. Positive turns inboard." },
        { nullptr, "Beta, damper at its eye", [](D& d) -> double& { return d.damperAngle; }, -180,
          180, true,
          "At the damper's eye: the angle from the arm back to the pivot to the damper. "
          "Positive turns inboard. 90 lays the damper square to its arm." },
        { nullptr, "Ld, damper length", [](D& d) -> double& { return d.damperLength; }, 1, 2000,
          false, "The damper eye to eye. Its far end is its chassis mount." },
        { "Anti-roll bar arm (L3)", "L3, pivot to drop link",
          [](D& d) -> double& { return d.antiRollArm; }, 1, 1000, false,
          "Distance on the rocker from its pivot to the drop link's upper end." },
        { nullptr, "Angle from L1 to L3", [](D& d) -> double& { return d.antiRollArmAngle; }, -180,
          180, true,
          "Angle on the rocker between the pushrod's arm and the drop link's arm, seen along "
          "the rocker's axis. Positive turns inboard." },
    };

    for (const CornerSpec& corner : templ.corners) {
        Column column;
        column.seed = seedFor(corner, designs, templ, mirror, current, &column.startsUsed);
        column.seed.corner = corner.token;
        m_columns.push_back(column);
    }

    m_timer = new QTimer(this);
    m_timer->setSingleShot(true);
    m_timer->setInterval(kPlanDelayMs);
    connect(m_timer, &QTimer::timeout, this, &RockerDialog::rebuildPlan);

    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->addWidget(buildDesigns());
    splitter->addWidget(buildPreview());
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 2);

    m_buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    m_buttons->button(QDialogButtonBox::Ok)->setText(tr("Place"));
    connect(m_buttons, &QDialogButtonBox::accepted, this, &RockerDialog::accept);
    connect(m_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(splitter, 1);
    layout->addWidget(m_buttons);

    // Which corners are designed parts already.
    m_loading = true;
    for (Column& column : m_columns) column.used->setChecked(column.startsUsed);
    m_loading = false;
    // The sketch opens on the first rocker that is a part already.
    const auto first = std::find_if(m_columns.begin(), m_columns.end(),
                                    [](const Column& c) { return c.startsUsed; });
    showSketchOf(first == m_columns.end() ? 0 : static_cast<int>(first - m_columns.begin()));
    syncEnabled();
    rebuildPlan();
    resize(1100, 700);
}

QDoubleSpinBox* RockerDialog::number(const Field& field, int column)
{
    auto* spin = new QDoubleSpinBox(this);
    spin->setRange(field.minimum, field.maximum);
    spin->setDecimals(field.degrees ? 2 : 1);
    spin->setSuffix(field.degrees ? tr(" °") : tr(" mm"));
    spin->setToolTip(tr(field.tip));
    spin->setAccelerated(true);
    spin->setKeyboardTracking(false);
    spin->setMinimumWidth(96);
    connect(spin, &QDoubleSpinBox::valueChanged, this, [this, column] {
        showSketchOf(column);
        schedulePlan();
    });
    return spin;
}

void RockerDialog::addHeading(QGridLayout* grid, int* row, const QString& text)
{
    auto* heading = new QLabel(QStringLiteral("<b>%1</b>").arg(text.toHtmlEscaped()), this);
    heading->setContentsMargins(0, *row == 0 ? 0 : 10, 0, 2);
    grid->addWidget(heading, (*row)++, 0, 1, 1 + static_cast<int>(m_columns.size()));
}

QWidget* RockerDialog::buildDesigns()
{
    auto* host = new QWidget(this);
    auto* grid = new QGridLayout(host);
    int row = 0;

    // A column per corner, headed by the switch that makes it a part.
    for (std::size_t i = 0; i < m_columns.size(); ++i) {
        Column& column = m_columns[i];
        const CornerSpec& corner = m_templ.corners[i];
        column.used = new QCheckBox(corner.label.isEmpty() ? corner.token : corner.label, host);
        column.used->setToolTip(
            tr("Tick to design this corner's rocker here. It is then one part: dragging any of "
               "its points -- the pivot, its axis point, the arms' ends, either end of the "
               "damper -- moves all of them. Untick to leave its points where they are, as "
               "separate points again."));
        connect(column.used, &QCheckBox::toggled, this, [this, i] {
            syncEnabled();
            showSketchOf(static_cast<int>(i));
            schedulePlan();
        });
        grid->addWidget(column.used, row, 1 + static_cast<int>(i), Qt::AlignCenter);
    }
    ++row;

    for (std::size_t f = 0; f < m_fields.size(); ++f) {
        const Field& field = m_fields[f];
        if (field.heading) {
            addHeading(grid, &row, tr(field.heading));
            if (QLatin1String(field.heading) == QLatin1String("Anti-roll bar arm (L3)")) {
                grid->addWidget(new QLabel(tr("Drop link on the rocker"), host), row, 0);
                for (std::size_t i = 0; i < m_columns.size(); ++i) {
                    Column& column = m_columns[i];
                    column.antiRollArm = new QCheckBox(host);
                    column.antiRollArm->setChecked(column.seed.antiRollArmUsed);
                    column.antiRollArm->setToolTip(
                        tr("Tick if an anti-roll bar's drop link hangs off this rocker. Its "
                           "pickup is written only when the linkage template mounts the drop "
                           "link on the rocker."));
                    connect(column.antiRollArm, &QCheckBox::toggled, this, [this, i] {
                        syncEnabled();
                        showSketchOf(static_cast<int>(i));
                        schedulePlan();
                    });
                    grid->addWidget(column.antiRollArm, row, 1 + static_cast<int>(i),
                                    Qt::AlignCenter);
                }
                ++row;
            }
        }
        auto* label = new QLabel(tr(field.label), host);
        label->setToolTip(tr(field.tip));
        grid->addWidget(label, row, 0);
        for (std::size_t i = 0; i < m_columns.size(); ++i) {
            Column& column = m_columns[i];
            QDoubleSpinBox* spin = number(field, static_cast<int>(i));
            m_loading = true;
            spin->setValue(field.member(column.seed));
            m_loading = false;
            column.spins.push_back(spin);
            grid->addWidget(spin, row, 1 + static_cast<int>(i));
        }
        ++row;
    }
    grid->setRowStretch(row, 1);
    grid->setColumnStretch(0, 1);

    auto* scroll = new QScrollArea(this);
    scroll->setWidget(host);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    return scroll;
}

QWidget* RockerDialog::buildPreview()
{
    auto* host = new QWidget(this);
    auto* layout = new QVBoxLayout(host);
    layout->setContentsMargins(8, 0, 0, 0);
    // What the numbers mean, drawn with them.
    m_sketch = new RockerSketch(host);
    layout->addWidget(m_sketch, 2);
    layout->addWidget(new QLabel(tr("<b>What Place will do</b>"), host));

    m_summary = new QLabel(host);
    m_summary->setWordWrap(true);
    layout->addWidget(m_summary);

    m_preview = new QTreeWidget(host);
    m_preview->setRootIsDecorated(false);
    m_preview->setHeaderLabels({ tr("Point"), tr("Change"), tr("Note") });
    m_preview->header()->setStretchLastSection(true);
    layout->addWidget(m_preview, 3);

    m_notes = new QLabel(host);
    m_notes->setWordWrap(true);
    m_notes->setTextFormat(Qt::RichText);
    m_notes->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    layout->addWidget(m_notes, 1);
    return host;
}

RockerDesign RockerDialog::designOf(const Column& column) const
{
    RockerDesign design = column.seed;
    for (std::size_t f = 0; f < m_fields.size(); ++f)
        m_fields[f].member(design) = column.spins[f]->value();
    design.antiRollArmUsed = column.antiRollArm && column.antiRollArm->isChecked();
    return design;
}

std::vector<RockerDesign> RockerDialog::designs() const
{
    std::vector<RockerDesign> out;
    for (const Column& column : m_columns)
        if (column.used && column.used->isChecked()) out.push_back(designOf(column));
    return out;
}

void RockerDialog::showSketchOf(int column)
{
    if (m_loading || !m_sketch || column < 0 || column >= static_cast<int>(m_columns.size()))
        return;
    m_sketchColumn = column;
    const CornerSpec& corner = m_templ.corners[static_cast<std::size_t>(column)];
    m_sketch->setDesign(designOf(m_columns[static_cast<std::size_t>(column)]),
                        corner.label.isEmpty() ? corner.token : corner.label);
}

void RockerDialog::syncEnabled()
{
    // The last two fields are the anti-roll arm's.
    const std::size_t antiRollFields = 2;
    for (Column& column : m_columns) {
        const bool used = column.used->isChecked();
        column.antiRollArm->setEnabled(used);
        for (std::size_t f = 0; f < column.spins.size(); ++f) {
            const bool antiRoll = f >= column.spins.size() - antiRollFields;
            column.spins[f]->setEnabled(used && (!antiRoll || column.antiRollArm->isChecked()));
        }
    }
}

void RockerDialog::schedulePlan()
{
    if (m_loading) return;
    m_timer->start();
}

void RockerDialog::rebuildPlan()
{
    m_timer->stop();
    const std::vector<RockerDesign> ticked = designs();
    m_plan = ticked.empty() ? DesignPlan{}
                            : planRockers(ticked, m_templ, m_mirror, m_current, m_baseline);
    showPlan();
}

void RockerDialog::showPlan()
{
    const QPalette pal = palette();
    const QString errorColor = issueColor(ConfigIssueLevel::Error, pal).name();
    const QString warningColor = issueColor(ConfigIssueLevel::Warning, pal).name();
    m_preview->clear();

    if (designs().empty()) {
        m_buttons->button(QDialogButtonBox::Ok)->setEnabled(true);
        m_summary->setText(tr("No rocker is ticked. Place moves no point; any rocker that was a "
                              "part goes back to being separate points, where they are."));
        m_notes->clear();
        return;
    }
    m_buttons->button(QDialogButtonBox::Ok)->setEnabled(m_plan.ok());
    if (!m_plan.ok()) {
        m_summary->setText(QStringLiteral("<span style=\"color:%1\">%2</span>")
                               .arg(errorColor, m_plan.error.toHtmlEscaped()));
        m_notes->clear();
        return;
    }

    const int edited = m_plan.handEditedCount();
    QString summary = tr("%1 new, %2 moved, %3 unchanged.")
                          .arg(m_plan.count(DesignChange::Kind::Added))
                          .arg(m_plan.count(DesignChange::Kind::Moved))
                          .arg(m_plan.count(DesignChange::Kind::Unchanged));
    if (edited > 0) {
        summary += QStringLiteral(" <span style=\"color:%1\">%2</span>")
                       .arg(warningColor,
                            tr("%n point(s) you have edited by hand will be overwritten.", "", edited)
                                .toHtmlEscaped());
    }
    m_summary->setText(summary);

    const QColor warn(warningColor);
    for (const DesignChange& change : m_plan.changes) {
        QString what;
        switch (change.kind) {
        case DesignChange::Kind::Added: what = tr("new"); break;
        case DesignChange::Kind::Moved: what = tr("moves %1 mm").arg(change.distance, 0, 'f', 2); break;
        case DesignChange::Kind::Unchanged: what = tr("unchanged"); break;
        }
        QString note;
        if (change.handEdited) note = tr("your edit is overwritten");
        else if (change.mirrored) note = tr("far side, from the mirror rule");
        auto* item = new QTreeWidgetItem(m_preview, { change.name, what, note });
        if (change.handEdited)
            for (int c = 0; c < 3; ++c) item->setForeground(c, warn);
    }
    m_preview->resizeColumnToContents(0);
    m_preview->resizeColumnToContents(1);

    m_notes->setText(m_plan.warnings.isEmpty()
                         ? QString()
                         : tr("<b>Worth knowing</b>") + bulleted(m_plan.warnings));
}

void RockerDialog::accept()
{
    rebuildPlan();
    if (!designs().empty() && !m_plan.ok()) return;
    QDialog::accept();
}

} // namespace suspkin
