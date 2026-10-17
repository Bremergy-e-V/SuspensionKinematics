#include "app/TemplateGraphicsTab.h"

#include "app/TemplateDraft.h"

#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

#include <algorithm>

namespace suspkin {
namespace {

enum Column { PartColumn, AxlesColumn, StyleColumn, ClosedColumn, FirstPointColumn };

/// As many points as Lotus gives a graphic, or as the longest chain needs.
constexpr int kMinPointColumns = 4;

} // namespace

TemplateGraphicsTab::TemplateGraphicsTab(TemplateDraft* draft, QWidget* parent)
    : QWidget(parent), m_draft(draft)
{
    auto* intro = new QLabel(
        tr("What is drawn: lines through points, each belonging to a part, whose colour it "
           "takes. A <b>pivot</b> is drawn even between two chassis points, where a plain line "
           "is left out because nothing is there. <b>Closed</b> joins the last point back to the "
           "first. Names with {corner} draw on every axle <b>Axles</b> allows -- a line only the front "
           "or only the rear has, such as a pushrod that meets the damper where the other axle has "
           "a rocker; the far side is drawn by mirroring."),
        this);
    intro->setWordWrap(true);

    m_table = new QTableWidget(this);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);

    auto* add = new QPushButton(tr("+1"), this);
    add->setToolTip(tr("Add a line to the selected row's part."));
    auto* remove = new QPushButton(tr("−1"), this);
    remove->setToolTip(tr("Remove the selected line."));
    auto* buttons = new QHBoxLayout;
    buttons->addWidget(add);
    buttons->addWidget(remove);
    buttons->addStretch(1);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(intro);
    layout->addLayout(buttons);
    layout->addWidget(m_table, 1);

    connect(add, &QPushButton::clicked, this, &TemplateGraphicsTab::addRow);
    connect(remove, &QPushButton::clicked, this, &TemplateGraphicsTab::removeSelectedRow);
    // The parts tab can add, rename or take away the parts a line belongs to.
    connect(draft, &TemplateDraft::templateChanged, this, [this] {
        if (!m_rebuilding) rebuild();
    });
    rebuild();
}

int TemplateGraphicsTab::pointColumns() const
{
    int longest = kMinPointColumns;
    for (const Row& row : m_rows) longest = std::max(longest, int(row.chain.points.size()));
    return longest;
}

void TemplateGraphicsTab::rebuild()
{
    m_rows.clear();
    const std::vector<PartTemplate>& parts = m_draft->linkageTemplate().parts;
    for (int part = 0; part < int(parts.size()); ++part)
        for (const ChainTemplate& chain : parts[std::size_t(part)].chains)
            m_rows.push_back(Row{ part, chain });

    m_rebuilding = true;
    m_table->clear();
    m_table->setRowCount(0);
    m_table->setColumnCount(FirstPointColumn + pointColumns());
    QStringList headers = { tr("Part"), tr("Axles"), tr("Graphic"), tr("Closed") };
    for (int i = 0; i < pointColumns(); ++i) headers << tr("Point %1").arg(i + 1);
    m_table->setHorizontalHeaderLabels(headers);
    m_table->setColumnWidth(PartColumn, 190);
    m_table->setColumnWidth(AxlesColumn, 110);
    m_table->setColumnWidth(StyleColumn, 90);
    m_table->setColumnWidth(ClosedColumn, 60);
    for (int i = 0; i < pointColumns(); ++i) m_table->setColumnWidth(FirstPointColumn + i, 190);
    for (int i = 0; i < int(m_rows.size()); ++i) fillRow(i);
    m_rebuilding = false;
}

void TemplateGraphicsTab::fillRow(int index)
{
    const Row& row = m_rows[std::size_t(index)];
    m_table->insertRow(index);
    const auto changed = [this] {
        if (!m_rebuilding) commit();
    };

    auto* part = new QComboBox(m_table);
    for (const PartTemplate& p : m_draft->linkageTemplate().parts)
        part->addItem(partBodyName(p), p.id);
    part->setCurrentIndex(row.part);
    m_table->setCellWidget(index, PartColumn, part);
    connect(part, &QComboBox::currentIndexChanged, this, changed);

    m_table->setCellWidget(index, AxlesColumn, axlesBox(row.chain.corners));
    connect(qobject_cast<QComboBox*>(m_table->cellWidget(index, AxlesColumn)),
            &QComboBox::currentIndexChanged, this, changed);

    auto* style = new QComboBox(m_table);
    style->addItem(tr("Line"));
    style->addItem(tr("Pivot"));
    style->setCurrentIndex(row.chain.style == ChainStyle::Pivot ? 1 : 0);
    m_table->setCellWidget(index, StyleColumn, style);
    connect(style, &QComboBox::currentIndexChanged, this, changed);

    auto* closed = new QCheckBox(m_table);
    closed->setChecked(row.chain.closed);
    m_table->setCellWidget(index, ClosedColumn, closed);
    connect(closed, &QCheckBox::toggled, this, changed);

    const QStringList patterns = m_draft->pointPatterns();
    for (int i = 0; i < pointColumns(); ++i) {
        auto* point = new QComboBox(m_table);
        point->setEditable(true);
        point->addItem(QString());
        point->addItems(patterns);
        point->setCurrentText(row.chain.points.value(i));
        point->setToolTip(tr("Not used when empty."));
        m_table->setCellWidget(index, FirstPointColumn + i, point);
        connect(point, &QComboBox::currentTextChanged, this, changed);
    }
}

QComboBox* TemplateGraphicsTab::axlesBox(const QStringList& corners)
{
    auto* box = new QComboBox(m_table);
    box->setToolTip(tr("Which axles this line is drawn on."));
    box->addItem(tr("All"), QStringList());
    for (const CornerSpec& corner : m_draft->linkageTemplate().corners)
        box->addItem(corner.label.isEmpty() ? corner.token : corner.label,
                     QStringList{ corner.token });
    // A list the choices above cannot say -- two axles of three, written by
    // hand -- is offered as itself, so opening the editor does not change it.
    int current = box->findData(corners);
    if (current < 0) {
        box->addItem(corners.join(QStringLiteral(", ")), corners);
        current = box->count() - 1;
    }
    box->setCurrentIndex(current);
    return box;
}

void TemplateGraphicsTab::readRows()
{
    for (int i = 0; i < int(m_rows.size()) && i < m_table->rowCount(); ++i) {
        Row& row = m_rows[std::size_t(i)];
        row.part = qobject_cast<QComboBox*>(m_table->cellWidget(i, PartColumn))->currentIndex();
        row.chain.corners = qobject_cast<QComboBox*>(m_table->cellWidget(i, AxlesColumn))
                                ->currentData()
                                .toStringList();
        row.chain.style = qobject_cast<QComboBox*>(m_table->cellWidget(i, StyleColumn))
                                      ->currentIndex() == 1
                              ? ChainStyle::Pivot
                              : ChainStyle::Line;
        row.chain.closed = qobject_cast<QCheckBox*>(m_table->cellWidget(i, ClosedColumn))->isChecked();
        row.chain.points.clear();
        for (int p = 0; p < pointColumns(); ++p) {
            const QString name =
                qobject_cast<QComboBox*>(m_table->cellWidget(i, FirstPointColumn + p))
                    ->currentText()
                    .trimmed();
            if (!name.isEmpty()) row.chain.points << name;
        }
    }
}

void TemplateGraphicsTab::commit()
{
    readRows();
    std::vector<std::vector<ChainTemplate>> chains(m_draft->linkageTemplate().parts.size());
    // A line through fewer than two points draws nothing, and is kept here
    // rather than written until it has them.
    for (const Row& row : m_rows)
        if (row.part >= 0 && row.part < int(chains.size()) && row.chain.points.size() >= 2)
            chains[std::size_t(row.part)].push_back(row.chain);
    m_rebuilding = true; // our own change: the rows are already right
    m_draft->setChains(chains);
    m_rebuilding = false;
}

void TemplateGraphicsTab::addRow()
{
    readRows();
    const int current = m_table->currentRow();
    Row row;
    row.part = current >= 0 ? m_rows[std::size_t(current)].part : 0;
    m_rows.push_back(row);
    m_rebuilding = true;
    fillRow(int(m_rows.size()) - 1);
    m_rebuilding = false;
    m_table->selectRow(int(m_rows.size()) - 1);
}

void TemplateGraphicsTab::removeSelectedRow()
{
    const int current = m_table->currentRow();
    if (current < 0) return;
    readRows();
    m_rows.erase(m_rows.begin() + current);
    m_rebuilding = true;
    m_table->removeRow(current);
    m_rebuilding = false;
    commit();
}

} // namespace suspkin
