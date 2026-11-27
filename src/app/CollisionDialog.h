#pragma once

#include "model/Collision.h"

#include <QDialog>
#include <QString>

#include <functional>
#include <vector>

class QComboBox;
class QDoubleSpinBox;
class QLabel;

namespace suspkin {

/// One entry of the dialog's two dropdowns.
struct CollisionChoice {
    QString key;   ///< @ref collisionTargetKey
    QString label; ///< what the user reads
    bool isMesh = false;
};

/// Analysis > Collision Check: the least distance between two things the user
/// picks -- a part, a rim, a tyre or the chassis -- over the travel the
/// analysis panel sweeps.
///
/// The dialog holds the choice and the settings and shows the answer; working
/// it out is the caller's, handed in as @p check, because that is where the
/// solve and the meshes are.
class CollisionDialog : public QDialog {
    Q_OBJECT

public:
    using Check = std::function<CollisionResult(const CollisionSettings&)>;

    CollisionDialog(const CollisionSettings& settings, std::vector<CollisionChoice> choices,
                    const QString& travel, const QString& unit, Check check,
                    QWidget* parent = nullptr);

    CollisionSettings settings() const;

private:
    /// Put the saved choice back in @p box, or @p fallback when it is gone.
    void select(QComboBox* box, const QString& key, int fallback);
    void run();
    void showResult(const CollisionResult& result);

    Check m_check;
    QString m_travel;
    QString m_unit;
    std::vector<CollisionChoice> m_choices;
    QComboBox* m_first = nullptr;
    QComboBox* m_second = nullptr;
    QDoubleSpinBox* m_radius = nullptr;
    QDoubleSpinBox* m_joint = nullptr;
    QLabel* m_result = nullptr;
    QLabel* m_detail = nullptr;
};

} // namespace suspkin
