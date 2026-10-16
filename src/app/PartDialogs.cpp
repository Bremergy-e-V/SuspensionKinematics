#include "app/PartDialogs.h"

#include "io/LinkageTemplate.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

namespace suspkin {
namespace {

constexpr PartKind kKinds[] = { PartKind::Other,   PartKind::Link,     PartKind::Wishbone,
                                PartKind::Upright, PartKind::Rocker,   PartKind::Damper,
                                PartKind::AntiRoll, PartKind::Wheel };

enum PartColumn { LabelColumn, KindColumn, DrawnColumn, PointsColumn, PartColumnCount };

} // namespace

QString partKindLabel(PartKind kind)
{
    switch (kind) {
    case PartKind::Wishbone: return QObject::tr("Wishbone");
    case PartKind::Link: return QObject::tr("Link");
    case PartKind::Upright: return QObject::tr("Upright");
    case PartKind::Rocker: return QObject::tr("Rocker");
    case PartKind::Damper: return QObject::tr("Damper");
    case PartKind::AntiRoll: return QObject::tr("Anti-roll bar");
    case PartKind::Wheel: return QObject::tr("Wheel");
    case PartKind::Other: return QObject::tr("Other");
    }
    return QObject::tr("Other");
}

// ---------------------------------------------------------------------------
// New part from the selection
// ---------------------------------------------------------------------------

NewPartDialog::NewPartDialog(const LinkageTemplate& templ, const QStringList& points,
                             QWidget* parent)
    : QDialog(parent), m_templ(templ)
{
    setWindowTitle(tr("New Part from Selection"));

    auto* layout = new QVBoxLayout(this);
    auto* intro = new QLabel(tr("The points are joined in this order. The part is drawn once, "
                                "through exactly these points -- it is not repeated for the "
                                "other corners or mirrored to the other side."),
                             this);
    intro->setWordWrap(true);
    layout->addWidget(intro);

    auto* form = new QFormLayout();
    m_label = new QLineEdit(tr("New part"), this);
    m_label->selectAll();
    m_label->setToolTip(tr("The part's name, shown in Edit Parts and used as a body name in the "
                           "hardpoint table's Part columns."));
    form->addRow(tr("Label"), m_label);

    m_kind = new QComboBox(this);
    for (const PartKind kind : kKinds) m_kind->addItem(partKindLabel(kind), partKindToString(kind));
    m_kind->setToolTip(tr("What the part is drawn as. A link and a wishbone are drawn in "
                          "different colours; nothing else about the solve changes."));
    form->addRow(tr("Kind"), m_kind);
    layout->addLayout(form);

    auto* order = new QHBoxLayout();
    m_points = new QListWidget(this);
    m_points->addItems(points);
    m_points->setDragDropMode(QAbstractItemView::InternalMove);
    m_points->setCurrentRow(0);
    m_points->setToolTip(tr("The points the part is drawn through, in order. Drag a point, or "
                            "use Up and Down, to change the order."));
    order->addWidget(m_points, 1);
    auto* buttons = new QVBoxLayout();
    auto* up = new QPushButton(tr("Up"), this);
    auto* down = new QPushButton(tr("Down"), this);
    buttons->addWidget(up);
    buttons->addWidget(down);
    buttons->addStretch(1);
    order->addLayout(buttons);
    layout->addLayout(order, 1);

    m_closed = new QCheckBox(tr("Close the chain, last point back to the first"), this);
    m_closed->setToolTip(tr("Tick to draw a closed outline, such as a wishbone's triangle, by "
                            "joining the last point back to the first."));
    layout->addWidget(m_closed);

    m_id = new QLabel(this);
    m_id->setWordWrap(true);
    layout->addWidget(m_id);

    m_buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    m_buttons->button(QDialogButtonBox::Ok)->setText(tr("Add Part"));
    connect(m_buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(m_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(m_buttons);

    connect(up, &QPushButton::clicked, this, [this] { move(-1); });
    connect(down, &QPushButton::clicked, this, [this] { move(+1); });
    connect(m_label, &QLineEdit::textChanged, this, &NewPartDialog::refresh);
    refresh();
    resize(420, 420);
}

void NewPartDialog::move(int step)
{
    const int row = m_points->currentRow();
    const int target = row + step;
    if (row < 0 || target < 0 || target >= m_points->count()) return;
    QListWidgetItem* item = m_points->takeItem(row);
    m_points->insertItem(target, item);
    m_points->setCurrentRow(target);
}

void NewPartDialog::refresh()
{
    // Three points make a closed chain a shape; two closed back on themselves
    // are the same segment twice.
    m_closed->setEnabled(m_points->count() >= 3);
    if (m_points->count() < 3) m_closed->setChecked(false);

    const bool named = !m_label->text().trimmed().isEmpty();
    m_buttons->button(QDialogButtonBox::Ok)->setEnabled(named && m_points->count() >= 2);
    m_id->setText(tr("Written into the project's linkage template as \"%1\".")
                      .arg(uniquePartId(m_templ, m_label->text())));
}

PartTemplate NewPartDialog::part() const
{
    PartTemplate part;
    part.label = m_label->text().trimmed();
    part.id = uniquePartId(m_templ, part.label);
    part.kind = partKindFromString(m_kind->currentData().toString());
    part.compliance = defaultCompliance(part.kind);
    part.perCorner = false;
    ChainTemplate chain;
    for (int i = 0; i < m_points->count(); ++i) chain.points << m_points->item(i)->text();
    chain.closed = m_closed->isEnabled() && m_closed->isChecked();
    part.chains.push_back(chain);
    return part;
}

} // namespace suspkin
