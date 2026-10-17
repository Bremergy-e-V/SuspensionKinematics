#pragma once

#include "model/SpringLoads.h"

#include <QDialog>
#include <QHash>
#include <QString>

#include <optional>
#include <vector>

class QDoubleSpinBox;
class QLabel;

namespace suspkin {

/// One axle as the springs dialog shows it.
struct SpringsAxle {
    QString token;
    QString label;
    AxleSprings springs;
    /// The installation ratio at design, when the axle has a damper, so the
    /// dialog can say what wheel rate a spring rate comes to.
    std::optional<double> installationRatio;
    bool hasAntiRoll = false;
};

/// Linkage > Springs and Tyres: per axle, the spring on the damper, the
/// anti-roll bar and the tyre, and the static load each wheel carries.
///
/// With them the sweep reads loads as well as angles: what the motion asks of
/// the spring, what the bar does in roll, and how much of it the tyre takes up.
class SpringsDialog : public QDialog {
    Q_OBJECT

public:
    SpringsDialog(const std::vector<SpringsAxle>& axles, QWidget* parent = nullptr);

    /// The rates of every axle shown, by corner token; one with nothing given
    /// is left out.
    QHash<QString, AxleSprings> springs() const;

private:
    struct Row {
        SpringsAxle axle;
        QDoubleSpinBox* spring = nullptr;
        QDoubleSpinBox* antiRoll = nullptr;
        QDoubleSpinBox* tyre = nullptr;
        QDoubleSpinBox* staticLoad = nullptr;
        QLabel* note = nullptr;
    };

    AxleSprings springsOf(const Row& row) const;
    void refresh(Row& row);

    std::vector<Row> m_rows;
};

} // namespace suspkin
