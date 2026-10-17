#include "app/CollisionDialog.h"

#include "model/Sweep.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGuiApplication>
#include <QHeaderView>
#include <QLabel>
#include <QPalette>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

#include <utility>

namespace suspkin {
namespace {

enum Column { MembersColumn, ClosestColumn, AtColumn, DesignColumn, ColumnCount };

/// The most pairs listed: past the closest few dozen nothing is near anything.
constexpr int kMaxRows = 60;

QDoubleSpinBox* millimetreBox(double minimum, double maximum, const QString& tip, QWidget* parent)
{
    auto* box = new QDoubleSpinBox(parent);
    box->setRange(minimum, maximum);
    box->setDecimals(1);
    box->setSuffix(QObject::tr(" mm"));
    box->setToolTip(tip);
    return box;
}

/// A colour mixed from the palette, the way the hardpoint table marks a row:
/// @p tint into the base colour, so it reads on a light desktop and a dark one.
QColor mixed(const QPalette& palette, const QColor& tint)
{
    const QColor base = palette.color(QPalette::Base);
    const double share = 0.35;
    return QColor::fromRgbF(base.redF() * (1 - share) + tint.redF() * share,
                            base.greenF() * (1 - share) + tint.greenF() * share,
                            base.blueF() * (1 - share) + tint.blueF() * share);
}

} // namespace

CollisionDialog::CollisionDialog(const CollisionSettings& settings, const QString& travel,
                                 Check check, QWidget* parent)
    : QDialog(parent), m_check(std::move(check))
{
    setWindowTitle(tr("Collision Check"));
    resize(720, 560);

    auto* intro = new QLabel(
        tr("Every member is taken as a round tube around the line drawn for it, and the "
           "suspension is walked through the travel the analysis panel sweeps. Listed is how "
           "close each two parts come, and each part to the chassis, closest first. Parts bolted "
           "together are not compared, and two that meet a third are not compared round that "
           "joint."),
        this);
    intro->setWordWrap(true);

    m_radius = millimetreBox(0.0, 100.0,
                             tr("Half the thickness of a member: the tube each line stands for."),
                             this);
    m_warn = millimetreBox(0.0, 100.0, tr("Clearance below this is marked as tight."), this);
    m_joint = millimetreBox(0.0, 200.0,
                            tr("How far round a joint two parts that meet there, or meet a third "
                               "part there, are not compared."),
                            this);
    m_chassis = new QCheckBox(tr("Compare with the imported chassis"), this);
    m_radius->setValue(settings.memberRadius);
    m_warn->setValue(settings.warnClearance);
    m_joint->setValue(settings.jointClearance);
    m_chassis->setChecked(settings.checkChassis);

    auto* form = new QFormLayout;
    form->addRow(tr("Member radius"), m_radius);
    form->addRow(tr("Tight below"), m_warn);
    form->addRow(tr("Joint clearance"), m_joint);
    form->addRow(QString(), m_chassis);

    auto* runButton = new QPushButton(tr("&Check"), this);
    runButton->setDefault(true);

    m_summary = new QLabel(this);
    m_summary->setWordWrap(true);

    m_table = new QTableWidget(0, ColumnCount, this);
    m_table->setHorizontalHeaderLabels(
        { tr("Members"), tr("Closest [mm]"), tr("At [%1]").arg(travel), tr("At design [mm]") });
    m_table->horizontalHeader()->setSectionResizeMode(MembersColumn, QHeaderView::Stretch);
    m_table->verticalHeader()->setVisible(false);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::accept);
    connect(runButton, &QPushButton::clicked, this, &CollisionDialog::run);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(intro);
    layout->addLayout(form);
    layout->addWidget(runButton, 0, Qt::AlignLeft);
    layout->addWidget(m_summary);
    layout->addWidget(m_table, 1);
    layout->addWidget(buttons);

    run();
}

CollisionSettings CollisionDialog::settings() const
{
    return CollisionSettings{ m_radius->value(), m_warn->value(), m_joint->value(),
                              m_chassis->isChecked() };
}

void CollisionDialog::run()
{
    QGuiApplication::setOverrideCursor(Qt::WaitCursor);
    const CollisionReport report = m_check(settings());
    QGuiApplication::restoreOverrideCursor();
    showReport(report);
}

void CollisionDialog::showReport(const CollisionReport& report)
{
    const double warn = m_warn->value();
    int touching = 0;
    int tight = 0;
    for (const CollisionPair& pair : report.pairs) {
        if (pair.touches()) ++touching;
        else if (pair.minClearance < warn) ++tight;
    }

    if (report.positions == 0) {
        m_summary->setText(tr("Nothing to check: no axle solves, or the analysis panel sweeps "
                              "nothing."));
    } else if (touching == 0 && tight == 0) {
        m_summary->setText(tr("Nothing comes closer than %1 mm over %2 positions.")
                               .arg(warn, 0, 'f', 1)
                               .arg(report.positions));
    } else {
        m_summary->setText(tr("<b>%1 touching, %2 tight</b> over %3 positions.")
                               .arg(touching)
                               .arg(tight)
                               .arg(report.positions));
    }

    const QPalette palette = this->palette();
    const int rows = std::min(static_cast<int>(report.pairs.size()), kMaxRows);
    m_table->setRowCount(rows);
    for (int row = 0; row < rows; ++row) {
        const CollisionPair& pair = report.pairs[static_cast<std::size_t>(row)];
        const QString design =
            pair.designKnown ? sweepValueText(pair.designClearance) : QStringLiteral("-");
        const QString cells[ColumnCount] = { pair.first + QStringLiteral(" / ") + pair.second,
                                             sweepValueText(pair.minClearance),
                                             sweepValueText(pair.atInput), design };
        const bool marked = pair.touches() || pair.minClearance < warn;
        const QColor tint = pair.touches() ? QColor(220, 50, 47) : QColor(240, 170, 30);
        for (int column = 0; column < ColumnCount; ++column) {
            auto* item = new QTableWidgetItem(cells[column]);
            if (column != MembersColumn) item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
            if (marked) item->setBackground(mixed(palette, tint));
            m_table->setItem(row, column, item);
        }
    }
    m_table->resizeColumnsToContents();
    m_table->horizontalHeader()->setSectionResizeMode(MembersColumn, QHeaderView::Stretch);
}

} // namespace suspkin
