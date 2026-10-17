#include "app/SpringsDialog.h"

#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QVBoxLayout>

namespace suspkin {
namespace {

/// A rate box: zero is "not given", and says so instead of "0".
QDoubleSpinBox* rateBox(double maximum, int decimals, const QString& suffix, const QString& tip,
                        QWidget* parent)
{
    auto* box = new QDoubleSpinBox(parent);
    box->setRange(0.0, maximum);
    box->setDecimals(decimals);
    box->setSuffix(suffix);
    box->setSpecialValueText(QObject::tr("none"));
    box->setToolTip(tip);
    return box;
}

} // namespace

SpringsDialog::SpringsDialog(const std::vector<SpringsAxle>& axles, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Springs and Tyres"));
    setMinimumWidth(560);

    auto* layout = new QVBoxLayout(this);
    auto* intro = new QLabel(
        tr("Rates for each axle. With them the analysis plots loads as well as angles: what the "
           "motion asks of the spring, what the anti-roll bar does in roll, and how much of it "
           "the tyre takes up -- Wheel load, Tyre deflection, Ride rate, Travel incl. tyre and "
           "Roll moment. The far side takes the same numbers. Leave a rate at none to leave it "
           "out.\n\nThe static load is what each wheel carries at the design position. With it "
           "the loads are absolute; without it they are changes from design."),
        this);
    intro->setWordWrap(true);
    layout->addWidget(intro);

    auto* grid = new QGridLayout;
    const QStringList headers = { tr("Axle"), tr("Spring"), tr("Anti-roll bar"), tr("Tyre"),
                                  tr("Static load") };
    for (int column = 0; column < headers.size(); ++column)
        grid->addWidget(new QLabel(QStringLiteral("<b>%1</b>").arg(headers[column]), this), 0,
                        column);

    m_rows.reserve(axles.size());
    int line = 1;
    for (const SpringsAxle& axle : axles) {
        Row row;
        row.axle = axle;
        row.spring = rateBox(10000.0, 1, tr(" N/mm"),
                             tr("The coil on the damper. Its rate at the wheel is this times "
                                "the installation ratio squared."),
                             this);
        row.antiRoll = rateBox(100000.0, 1, tr(" Nm/deg"),
                               tr("The bar's torsional rate: newton-metres per degree of twist "
                                  "between its two arms."),
                               this);
        row.tyre = rateBox(10000.0, 1, tr(" N/mm"), tr("The tyre's vertical stiffness."), this);
        row.staticLoad = rateBox(100000.0, 0, tr(" N"),
                                 tr("What one wheel of this axle carries at the design position: "
                                    "a quarter of the car's weight on that axle's share."),
                                 this);
        row.spring->setValue(axle.springs.springRate);
        row.antiRoll->setValue(axle.springs.antiRollRate);
        row.antiRoll->setEnabled(axle.hasAntiRoll || axle.springs.antiRollRate > 0.0);
        row.tyre->setValue(axle.springs.tyreRate);
        row.staticLoad->setValue(axle.springs.staticLoad);

        const QString heading = axle.label.isEmpty() ? axle.token : axle.label;
        grid->addWidget(new QLabel(heading.toHtmlEscaped(), this), line, 0);
        grid->addWidget(row.spring, line, 1);
        grid->addWidget(row.antiRoll, line, 2);
        grid->addWidget(row.tyre, line, 3);
        grid->addWidget(row.staticLoad, line, 4);
        ++line;

        row.note = new QLabel(this);
        row.note->setWordWrap(true);
        row.note->setContentsMargins(0, 0, 0, 6);
        grid->addWidget(row.note, line, 1, 1, 4);
        ++line;
        m_rows.push_back(row);
    }
    layout->addLayout(grid);

    // Wired once every row has its final address.
    for (Row& row : m_rows) {
        Row* at = &row;
        for (QDoubleSpinBox* box : { row.spring, row.antiRoll, row.tyre, row.staticLoad })
            connect(box, &QDoubleSpinBox::valueChanged, this, [this, at] { refresh(*at); });
        refresh(row);
    }

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

AxleSprings SpringsDialog::springsOf(const Row& row) const
{
    return AxleSprings{ row.spring->value(), row.antiRoll->value(), row.tyre->value(),
                        row.staticLoad->value() };
}

QHash<QString, AxleSprings> SpringsDialog::springs() const
{
    QHash<QString, AxleSprings> springs;
    for (const Row& row : m_rows) {
        const AxleSprings rates = springsOf(row);
        if (!rates.isEmpty()) springs.insert(row.axle.token, rates);
    }
    return springs;
}

void SpringsDialog::refresh(Row& row)
{
    const AxleSprings rates = springsOf(row);
    QStringList lines;
    if (!row.axle.installationRatio) {
        if (rates.springRate > 0.0)
            lines << tr("This axle has no damper in its mechanism, so the spring has nothing to "
                        "act through.");
    } else if (rates.springRate > 0.0) {
        const double ratio = *row.axle.installationRatio;
        const double wheelRate = rates.springRate * ratio * ratio;
        QString line = tr("Wheel rate at design %1 N/mm (installation ratio %2)")
                           .arg(wheelRate, 0, 'f', 1)
                           .arg(ratio, 0, 'f', 3);
        if (rates.tyreRate > 0.0)
            line += tr(", ride rate with the tyre %1 N/mm")
                        .arg(wheelRate * rates.tyreRate / (wheelRate + rates.tyreRate), 0, 'f', 1);
        lines << line + QLatin1Char('.');
    }
    if (rates.tyreRate > 0.0 && rates.staticLoad > 0.0)
        lines << tr("The tyre is squashed %1 mm at design.")
                     .arg(rates.staticLoad / rates.tyreRate, 0, 'f', 2);
    if (!row.axle.hasAntiRoll && rates.antiRollRate > 0.0)
        lines << tr("This axle has no anti-roll bar in its mechanism, so the rate is not used.");
    row.note->setText(lines.join(QLatin1Char(' ')));
    row.note->setVisible(!lines.isEmpty());
}

} // namespace suspkin
