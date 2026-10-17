#pragma once

#include "model/Collision.h"

#include <QDialog>

#include <functional>

class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QTableWidget;

namespace suspkin {

/// Analysis > Collision Check: how close the members come to each other and
/// to the chassis over the travel the analysis panel sweeps.
///
/// The dialog holds the settings and shows the report; working the report out
/// is the caller's, handed in as @p check, because that is where the solve and
/// the chassis are.
class CollisionDialog : public QDialog {
    Q_OBJECT

public:
    using Check = std::function<CollisionReport(const CollisionSettings&)>;

    CollisionDialog(const CollisionSettings& settings, const QString& travel, Check check,
                    QWidget* parent = nullptr);

    CollisionSettings settings() const;

private:
    void run();
    void showReport(const CollisionReport& report);

    Check m_check;
    QDoubleSpinBox* m_radius = nullptr;
    QDoubleSpinBox* m_warn = nullptr;
    QDoubleSpinBox* m_joint = nullptr;
    QCheckBox* m_chassis = nullptr;
    QLabel* m_summary = nullptr;
    QTableWidget* m_table = nullptr;
};

} // namespace suspkin
