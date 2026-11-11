#pragma once

#include "model/Compliance.h"

#include <QDialog>
#include <QHash>
#include <QString>

#include <optional>
#include <vector>

class QCheckBox;
class QDoubleSpinBox;
class QLabel;

namespace suspkin {

/// One axle as the spring rates dialog shows it.
struct SpringRatesAxle {
    QString token;
    QString label;

    /// What the project states for this axle, if anything.
    std::optional<AxleRates> stated;

    /// Whether the axle's mechanism has a damper for a spring to sit on, and
    /// an anti-roll bar on both sides. Without the first no rate does
    /// anything; without the second the bar's rate has nothing to act on.
    bool hasDamper = false;
    bool hasAntiRoll = false;
};

/// Spring, anti-roll bar and tyre rates, per axle.
///
/// They are what turns a movement into a force: with them a sweep reports
/// what the springs and the bar carry, and -- with the analysis panel's
/// "Include tyre stiffness" on -- how far that squashes the tyres. An axle left
/// unticked has no forces, and is swept as kinematics alone.
class SpringRatesDialog : public QDialog {
    Q_OBJECT

public:
    SpringRatesDialog(const std::vector<SpringRatesAxle>& axles, QWidget* parent = nullptr);

    /// The rates of every ticked axle, by corner token.
    QHash<QString, AxleRates> rates() const;

private:
    struct Row {
        SpringRatesAxle axle;
        QCheckBox* stated = nullptr;
        QDoubleSpinBox* spring = nullptr;
        QDoubleSpinBox* antiRoll = nullptr;
        QDoubleSpinBox* tyre = nullptr;
        QLabel* note = nullptr;
    };

    /// A rate field: zero upwards, three decimals, @p suffix after the number.
    QDoubleSpinBox* addRate(double value, const QString& suffix, const QString& tip);
    /// One axle's heading, tick box, three fields and note.
    void addRow(const SpringRatesAxle& axle);
    /// What @p row's numbers will do, in words.
    QString noteFor(const Row& row) const;
    void refresh(Row& row);

    std::vector<Row> m_rows;
};

} // namespace suspkin
