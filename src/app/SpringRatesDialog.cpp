#include "app/SpringRatesDialog.h"

#include "app/ExpressionSpinBox.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QGridLayout>
#include <QLabel>
#include <QVBoxLayout>

namespace suspkin {

SpringRatesDialog::SpringRatesDialog(const std::vector<SpringRatesAxle>& axles, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Springs and Tyres"));
    setMinimumWidth(520);

    auto* layout = new QVBoxLayout(this);
    auto* intro = new QLabel(
        tr("How stiff each axle's springs, anti-roll bar and tyres are. The spring is taken to "
           "sit on the damper, and the bar is stated at the bar: the linkage takes both to the "
           "wheel. Both sides of an axle take the same numbers.\n\n"
           "With these a sweep shows what the springs and the bar carry. Turn on Include tyre "
           "stiffness in the sweep parameters and the tyres give way under it as well."),
        this);
    intro->setWordWrap(true);
    layout->addWidget(intro);

    // Reserved first: each row's widgets are wired to its address.
    m_rows.reserve(axles.size());
    for (const SpringRatesAxle& axle : axles) addRow(axle);

    if (m_rows.empty()) {
        auto* none = new QLabel(tr("No axle in this table solves, so there is nothing to spring."),
                                this);
        none->setWordWrap(true);
        layout->addWidget(none);
    }

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

QDoubleSpinBox* SpringRatesDialog::addRate(double value, const QString& suffix,
                                           const QString& tip)
{
    auto* box = new ExpressionSpinBox(this);
    box->setRange(0.0, 100000.0);
    box->setDecimals(3);
    box->setSingleStep(1.0);
    box->setSuffix(suffix);
    box->setToolTip(tip);
    box->setValue(value);
    return box;
}

void SpringRatesDialog::addRow(const SpringRatesAxle& axle)
{
    auto* layout = static_cast<QVBoxLayout*>(this->layout());
    Row row;
    row.axle = axle;

    const QString heading = axle.label.isEmpty() ? axle.token : axle.label;
    layout->addWidget(new QLabel(QStringLiteral("<b>%1</b>").arg(heading.toHtmlEscaped()), this));

    const AxleRates shown = axle.stated.value_or(AxleRates{});
    row.stated = new QCheckBox(tr("State this axle's rates"), this);
    row.stated->setChecked(axle.stated.has_value());
    row.spring = addRate(shown.springRate, tr(" N/mm"),
                         tr("The spring's own rate, along the damper it sits on."));
    row.antiRoll = addRate(shown.antiRollRate, tr(" Nm/deg"),
                           tr("The bar's torsional rate, arm to arm. Zero for no bar."));
    row.tyre = addRate(shown.tyreRate, tr(" N/mm"),
                       tr("The tyre's vertical rate. Zero takes the tyre as rigid."));

    auto* grid = new QGridLayout;
    grid->setContentsMargins(12, 0, 0, 0);
    grid->addWidget(row.stated, 0, 0, 1, 2);
    grid->addWidget(new QLabel(tr("Spring"), this), 1, 0);
    grid->addWidget(row.spring, 1, 1);
    grid->addWidget(new QLabel(tr("Anti-roll bar"), this), 2, 0);
    grid->addWidget(row.antiRoll, 2, 1);
    grid->addWidget(new QLabel(tr("Tyre"), this), 3, 0);
    grid->addWidget(row.tyre, 3, 1);
    grid->setColumnStretch(1, 1);
    layout->addLayout(grid);

    // In the dialog's own column, where a wrapped label gets the height it
    // needs; across grid columns it is cut off after one line.
    row.note = new QLabel(this);
    row.note->setWordWrap(true);
    row.note->setContentsMargins(12, 0, 0, 6);
    layout->addWidget(row.note);

    m_rows.push_back(row);
    Row* placed = &m_rows.back();
    connect(row.stated, &QCheckBox::toggled, this, [this, placed] { refresh(*placed); });
    for (QDoubleSpinBox* box : { row.spring, row.antiRoll, row.tyre })
        connect(box, &QDoubleSpinBox::valueChanged, this, [this, placed] { refresh(*placed); });
    refresh(*placed);
}

QString SpringRatesDialog::noteFor(const Row& row) const
{
    if (!row.axle.hasDamper)
        return tr("This axle has no damper in the linkage template, so there is nothing for a "
                  "spring to sit on and its rates do nothing.");
    if (!row.stated->isChecked())
        return tr("No rates: this axle is swept as kinematics alone.");
    if (row.spring->value() <= 0.0)
        return tr("Without a spring rate there are no forces to work out.");

    QStringList notes;
    if (row.tyre->value() <= 0.0) notes << tr("The tyre is taken as rigid.");
    if (!row.axle.hasAntiRoll)
        notes << tr("The linkage template names no anti-roll bar on this axle.");
    return notes.join(QLatin1Char(' '));
}

void SpringRatesDialog::refresh(Row& row)
{
    const bool on = row.stated->isChecked();
    row.spring->setEnabled(on);
    row.tyre->setEnabled(on);
    row.antiRoll->setEnabled(on && row.axle.hasAntiRoll);
    const QString note = noteFor(row);
    row.note->setText(note);
    row.note->setVisible(!note.isEmpty());
}

QHash<QString, AxleRates> SpringRatesDialog::rates() const
{
    QHash<QString, AxleRates> rates;
    for (const Row& row : m_rows) {
        if (!row.stated->isChecked()) continue;
        rates.insert(row.axle.token,
                     AxleRates{ row.spring->value(), row.antiRoll->value(), row.tyre->value() });
    }
    return rates;
}

} // namespace suspkin
