#include "app/TemplatePartsTab.h"

#include "app/PartDialogs.h"
#include "app/TemplateDraft.h"
#include "model/HardpointConfig.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <iterator>

namespace suspkin {
namespace {

enum Column { LabelColumn, KindColumn, ComplianceColumn, DrawnColumn, ColumnCount };

constexpr PartKind kKinds[] = { PartKind::Wishbone, PartKind::Link,     PartKind::Upright,
                                PartKind::Rocker,   PartKind::Damper,   PartKind::AntiRoll,
                                PartKind::Wheel,    PartKind::Other };

int kindIndex(PartKind kind)
{
    for (int i = 0; i < int(std::size(kKinds)); ++i)
        if (kKinds[i] == kind) return i;
    return int(std::size(kKinds)) - 1;
}

} // namespace

TemplatePartsTab::TemplatePartsTab(TemplateDraft* draft, QWidget* parent)
    : QWidget(parent), m_draft(draft)
{
    auto* intro = new QLabel(
        tr("The bodies the corner is made of. A rigid part keeps its shape; a compressible one "
           "is a strut -- a damper -- whose ends may come closer, and whose length is what is "
           "measured. {corner} and {side} in a label become the axle and the side."),
        this);
    intro->setWordWrap(true);

    m_table = new QTableWidget(this);
    m_table->setColumnCount(ColumnCount);
    m_table->setHorizontalHeaderLabels({ tr("Part Label"), tr("Kind"), tr("Shape"), tr("Drawn") });
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->verticalHeader()->setVisible(true);
    m_table->horizontalHeader()->setSectionResizeMode(LabelColumn, QHeaderView::Stretch);
    m_table->setColumnWidth(KindColumn, 130);
    m_table->setColumnWidth(ComplianceColumn, 130);
    m_table->setColumnWidth(DrawnColumn, 170);

    auto* add = new QPushButton(tr("+1"), this);
    add->setToolTip(tr("Add a part. It draws nothing until the Graphics tab connects points "
                       "through it."));
    auto* remove = new QPushButton(tr("−1"), this);
    remove->setToolTip(tr("Remove the selected part."));
    auto* buttons = new QHBoxLayout;
    buttons->addWidget(add);
    buttons->addWidget(remove);
    buttons->addStretch(1);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(intro);
    layout->addLayout(buttons);
    layout->addWidget(m_table, 1);

    connect(add, &QPushButton::clicked, this, &TemplatePartsTab::addPart);
    connect(remove, &QPushButton::clicked, this, &TemplatePartsTab::removeSelectedPart);
    connect(m_table, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item) {
        if (!m_rebuilding && item->column() == LabelColumn) commitLabel(item->row());
    });
    rebuild();
}

void TemplatePartsTab::rebuild()
{
    m_rebuilding = true;
    m_table->setRowCount(0);
    for (int i = 0; i < int(m_draft->linkageTemplate().parts.size()); ++i) addRow(i);
    m_rebuilding = false;
}

void TemplatePartsTab::addRow(int index)
{
    const PartTemplate& part = m_draft->linkageTemplate().parts[std::size_t(index)];
    m_table->insertRow(index);

    auto* label = new QTableWidgetItem(part.label);
    label->setToolTip(tr("Offered in the hardpoint settings as \"%1\".").arg(partBodyName(part)));
    m_table->setItem(index, LabelColumn, label);

    // A dozen rows at most, so a widget per cell costs nothing here.
    auto* kind = new QComboBox(m_table);
    for (const PartKind k : kKinds) kind->addItem(partKindLabel(k));
    kind->setCurrentIndex(kindIndex(part.kind));
    m_table->setCellWidget(index, KindColumn, kind);
    connect(kind, &QComboBox::currentIndexChanged, this, [this, kind](int choice) {
        commitKind(m_table->indexAt(kind->pos()).row(), choice);
    });

    auto* compliance = new QComboBox(m_table);
    compliance->addItem(tr("Rigid"));
    compliance->addItem(tr("Compressible"));
    compliance->setCurrentIndex(part.compliance == PartCompliance::Compressible ? 1 : 0);
    m_table->setCellWidget(index, ComplianceColumn, compliance);
    connect(compliance, &QComboBox::currentIndexChanged, this, [this, compliance](int choice) {
        commitCompliance(m_table->indexAt(compliance->pos()).row(), choice);
    });

    auto* drawn = new QTableWidgetItem(part.perCorner ? tr("Every corner, both sides")
                                                      : tr("Once, as named"));
    drawn->setFlags(drawn->flags() & ~Qt::ItemIsEditable);
    m_table->setItem(index, DrawnColumn, drawn);
}

void TemplatePartsTab::commitLabel(int row)
{
    if (row < 0 || row >= int(m_draft->linkageTemplate().parts.size())) return;
    PartTemplate part = m_draft->linkageTemplate().parts[std::size_t(row)];
    const QString label = m_table->item(row, LabelColumn)->text().trimmed();
    if (label.isEmpty()) {
        rebuild(); // a part has to be called something
        return;
    }
    part.label = label;
    m_draft->setPart(row, part);
    m_table->item(row, LabelColumn)
        ->setToolTip(tr("Offered in the hardpoint settings as \"%1\".").arg(partBodyName(part)));
}

void TemplatePartsTab::commitKind(int row, int kindChoice)
{
    if (row < 0 || row >= int(m_draft->linkageTemplate().parts.size())) return;
    PartTemplate part = m_draft->linkageTemplate().parts[std::size_t(row)];
    const PartKind kind = kKinds[std::clamp(kindChoice, 0, int(std::size(kKinds)) - 1)];
    // A part becoming a damper is a strut unless somebody says otherwise.
    if (part.compliance == defaultCompliance(part.kind)) part.compliance = defaultCompliance(kind);
    part.kind = kind;
    m_draft->setPart(row, part);
    if (auto* box = qobject_cast<QComboBox*>(m_table->cellWidget(row, ComplianceColumn))) {
        const QSignalBlocker quiet(box);
        box->setCurrentIndex(part.compliance == PartCompliance::Compressible ? 1 : 0);
    }
}

void TemplatePartsTab::commitCompliance(int row, int complianceChoice)
{
    if (row < 0 || row >= int(m_draft->linkageTemplate().parts.size())) return;
    PartTemplate part = m_draft->linkageTemplate().parts[std::size_t(row)];
    part.compliance = complianceChoice == 1 ? PartCompliance::Compressible : PartCompliance::Rigid;
    m_draft->setPart(row, part);
}

void TemplatePartsTab::addPart()
{
    m_draft->addPart(tr("New part"));
    rebuild();
    m_table->selectRow(m_table->rowCount() - 1);
    m_table->editItem(m_table->item(m_table->rowCount() - 1, LabelColumn));
}

void TemplatePartsTab::removeSelectedPart()
{
    const int row = m_table->currentRow();
    // A template with no parts is not a template the reader will take.
    if (row < 0 || m_draft->linkageTemplate().parts.size() <= 1) return;
    m_draft->removePart(row);
    rebuild();
}

} // namespace suspkin
