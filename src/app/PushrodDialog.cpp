#include "app/PushrodDialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QTreeWidget>
#include <QVBoxLayout>

namespace suspkin {
namespace {

constexpr PushrodLayout kLayouts[] = { PushrodLayout::Rocker, PushrodLayout::InlineDamper,
                                       PushrodLayout::DamperOnly };

QString layoutLabel(PushrodLayout layout)
{
    switch (layout) {
    case PushrodLayout::Rocker: return PushrodDialog::tr("Pushrod to rocker");
    case PushrodLayout::InlineDamper: return PushrodDialog::tr("Pushrod and damper in line");
    case PushrodLayout::DamperOnly: return PushrodDialog::tr("Damper only");
    }
    return QString();
}

int layoutIndex(PushrodLayout layout)
{
    for (int i = 0; i < int(std::size(kLayouts)); ++i)
        if (kLayouts[i] == layout) return i;
    return 0;
}

/// The stored design for @p corner, or a fresh one.
PushrodDesign seedFor(const std::vector<PushrodDesign>& stored, const QString& corner, bool* found)
{
    for (const PushrodDesign& design : stored) {
        if (design.corner != corner) continue;
        *found = true;
        return design;
    }
    *found = false;
    PushrodDesign fresh;
    fresh.corner = corner;
    return fresh;
}

} // namespace

PushrodDialog::PushrodDialog(const std::vector<PushrodDesign>& stored, const LinkageTemplate& templ,
                             const MirrorSpec& mirror, const HardpointTable& current,
                             const HardpointTable& baseline, ChassisSource chassis,
                             QWidget* parent)
    : QDialog(parent),
      m_templ(templ),
      m_mirror(mirror),
      m_current(current),
      m_baseline(baseline),
      m_chassis(std::move(chassis))
{
    setWindowTitle(tr("Pushrod and Damper"));

    auto* intro = new QLabel(
        tr("How each corner's spring is worked, and where its ends go. Without a rocker the "
           "chassis end is found on the chassis at the height given -- looking inboard from the "
           "ball joint -- and stood off it by the clearance; with no chassis imported it goes "
           "at the distance from the centreline given instead. The outboard end goes on the "
           "line from the ball joint to the chassis end, the offset inboard of the ball joint. "
           "With a rocker the line runs to the rocker's pushrod arm."),
        this);
    intro->setWordWrap(true);

    m_preview = new QTreeWidget(this);
    m_preview->setHeaderLabels({ tr("Point"), tr("Change") });
    m_preview->setRootIsDecorated(false);
    m_preview->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_summary = new QLabel(this);
    m_summary->setWordWrap(true);
    m_notes = new QLabel(this);
    m_notes->setWordWrap(true);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Cancel, this);
    m_place = buttons->addButton(tr("Place"), QDialogButtonBox::AcceptRole);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(intro);
    layout->addWidget(buildColumns(stored));
    layout->addWidget(m_summary);
    layout->addWidget(m_preview, 1);
    layout->addWidget(m_notes);
    layout->addWidget(buttons);

    m_replan = new QTimer(this);
    m_replan->setSingleShot(true);
    m_replan->setInterval(120);
    connect(m_replan, &QTimer::timeout, this, &PushrodDialog::replan);
    for (const Column& column : m_columns) enableFields(column);
    replan();
    resize(820, 720);
}

QDoubleSpinBox* PushrodDialog::spin(double value, double min, double max, const QString& tip)
{
    auto* box = new QDoubleSpinBox(this);
    box->setRange(min, max);
    box->setDecimals(1);
    box->setSuffix(tr(" mm"));
    box->setValue(value);
    box->setToolTip(tip);
    connect(box, &QDoubleSpinBox::valueChanged, this, [this] { m_replan->start(); });
    return box;
}

QWidget* PushrodDialog::buildColumns(const std::vector<PushrodDesign>& stored)
{
    auto* grid = new QWidget(this);
    auto* layout = new QGridLayout(grid);
    const QStringList rows = { tr("Layout"),        tr("Ball joint"),
                               tr("Chassis end height"), tr("Clearance to chassis"),
                               tr("Without a chassis, from centreline"),
                               tr("Outboard end, inboard of the ball joint"),
                               tr("Damper length, eye to eye") };
    for (int row = 0; row < rows.size(); ++row) layout->addWidget(new QLabel(rows[row], grid), row + 1, 0);

    std::vector<CornerSpec> corners = m_templ.corners;
    if (corners.empty()) corners.push_back(CornerSpec{});
    int col = 1;
    for (const CornerSpec& corner : corners) {
        bool found = false;
        const PushrodDesign seed = seedFor(stored, corner.token, &found);
        Column column;
        column.corner = corner.token;
        column.used = new QCheckBox(corner.label.isEmpty() ? corner.token : corner.label, grid);
        column.used->setChecked(found);
        column.layout = new QComboBox(grid);
        for (const PushrodLayout l : kLayouts) column.layout->addItem(layoutLabel(l));
        column.layout->setCurrentIndex(layoutIndex(seed.layout));
        column.arm = new QComboBox(grid);
        column.arm->addItem(tr("Upper wishbone"));
        column.arm->addItem(tr("Lower wishbone"));
        column.arm->setCurrentIndex(seed.arm == PickupArm::Lower ? 1 : 0);
        column.height = spin(seed.inboardHeight, -2000.0, 3000.0,
                             tr("The height the chassis end is looked for at."));
        column.clearance = spin(seed.clearance, 0.0, 500.0,
                                tr("How far the chassis end stands off the chassis surface."));
        column.inboardY = spin(seed.inboardY, 0.0, 2000.0,
                               tr("Used only when there is no chassis to find the end on."));
        column.offset = spin(seed.outboardOffset, 0.0, 1000.0,
                             tr("How far inboard of the ball joint, across the car, the outboard "
                                "end goes on its line."));
        column.damperLength = spin(seed.damperLength, 1.0, 2000.0,
                                   tr("The damper's length from eye to eye, laid off from the "
                                      "chassis end along the pushrod's line."));

        layout->addWidget(column.used, 0, col);
        QWidget* fields[] = { column.layout, column.arm, column.height, column.clearance,
                              column.inboardY, column.offset, column.damperLength };
        for (int row = 0; row < int(std::size(fields)); ++row) layout->addWidget(fields[row], row + 1, col);

        const std::size_t index = m_columns.size();
        connect(column.used, &QCheckBox::toggled, this, [this, index] {
            enableFields(m_columns[index]);
            m_replan->start();
        });
        connect(column.layout, &QComboBox::currentIndexChanged, this, [this, index] {
            enableFields(m_columns[index]);
            m_replan->start();
        });
        connect(column.arm, &QComboBox::currentIndexChanged, this, [this] { m_replan->start(); });
        m_columns.push_back(column);
        ++col;
    }
    return grid;
}

void PushrodDialog::enableFields(const Column& column)
{
    const bool used = column.used->isChecked();
    const PushrodLayout layout = kLayouts[column.layout->currentIndex()];
    const bool onChassis = layout != PushrodLayout::Rocker;
    column.layout->setEnabled(used);
    column.arm->setEnabled(used);
    column.height->setEnabled(used && onChassis);
    column.clearance->setEnabled(used && onChassis);
    column.inboardY->setEnabled(used && onChassis);
    column.offset->setEnabled(used);
    column.damperLength->setEnabled(used && layout == PushrodLayout::InlineDamper);
}

PushrodDesign PushrodDialog::designOf(const Column& column) const
{
    PushrodDesign design;
    design.corner = column.corner;
    design.layout = kLayouts[column.layout->currentIndex()];
    design.arm = column.arm->currentIndex() == 1 ? PickupArm::Lower : PickupArm::Upper;
    design.inboardHeight = column.height->value();
    design.clearance = column.clearance->value();
    design.inboardY = column.inboardY->value();
    design.outboardOffset = column.offset->value();
    design.damperLength = column.damperLength->value();
    return design;
}

std::vector<PushrodDesign> PushrodDialog::designs() const
{
    std::vector<PushrodDesign> out;
    for (const Column& column : m_columns)
        if (column.used->isChecked()) out.push_back(designOf(column));
    return out;
}

void PushrodDialog::replan()
{
    m_plan = planPushrods(designs(), m_templ, m_mirror, m_current, m_baseline,
                          m_chassis ? m_chassis() : nullptr);
    showPlan();
}

void PushrodDialog::showPlan()
{
    m_preview->clear();
    m_place->setEnabled(m_plan.ok());
    if (!m_plan.ok()) {
        m_summary->setText(m_plan.points.error);
        m_notes->clear();
        return;
    }

    for (const DesignChange& change : m_plan.points.changes) {
        QString what;
        switch (change.kind) {
        case DesignChange::Kind::Added: what = tr("new"); break;
        case DesignChange::Kind::Moved: what = tr("moves %1 mm").arg(change.distance, 0, 'f', 2); break;
        case DesignChange::Kind::Unchanged: what = tr("unchanged"); break;
        }
        if (change.handEdited) what += tr(" -- over your own edit");
        new QTreeWidgetItem(m_preview, { change.name, what });
    }
    for (const QString& name : m_plan.removed)
        new QTreeWidgetItem(m_preview, { name, tr("taken out: this layout has no use for it") });
    for (const QString& name : m_plan.unassigned)
        new QTreeWidgetItem(m_preview, { name, tr("kept, no longer solved") });

    m_summary->setText(tr("%1 point(s) placed, %2 taken out. Each point's settings are written "
                          "too, so the corner is solved the way it is laid out.")
                           .arg(m_plan.points.changes.size())
                           .arg(m_plan.removed.size()));
    m_notes->setText(m_plan.points.warnings.join(QLatin1Char('\n')));
}

} // namespace suspkin
