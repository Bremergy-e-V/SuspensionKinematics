#pragma once

#include "model/Collision.h"

#include <QDialog>
#include <QHash>
#include <QString>

#include <functional>
#include <utility>
#include <vector>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QTableWidget;

namespace suspkin {

/// One entry of the dialog's two dropdowns.
struct CollisionChoice {
    QString key;   ///< @ref collisionTargetKey
    QString label; ///< what the user reads
    bool isMesh = false;
    bool isTube = false; ///< one tube of the part above it, so indented
};

/// Analysis > Collision Check: the least distance between two things the user
/// picks -- a part or one of its tubes, a rim, a tyre or the chassis -- at any
/// steering position, at ride height and through the wheel travel.
///
/// The dialog holds the choice and the settings and shows the answer; working
/// it out is the caller's, handed in as @p check, because that is where the
/// solve and the meshes are. @p members names the tubes a target is made of,
/// for the table their radii are set in.
class CollisionDialog : public QDialog {
    Q_OBJECT

public:
    using Check = std::function<CollisionResult(const CollisionSettings&)>;
    using Members = std::function<std::vector<std::pair<QString, QString>>(const QString& key)>;

    CollisionDialog(const CollisionSettings& settings, std::vector<CollisionChoice> choices,
                    const CollisionTravel& travel, Check check, Members members,
                    QWidget* parent = nullptr);

    CollisionSettings settings() const;

private:
    QWidget* buildChoice();
    QWidget* buildResults();
    /// Put the saved choice back in @p box, or @p fallback when it is gone.
    void select(QComboBox* box, const QString& key, int fallback);
    /// List the tubes of the two chosen targets with their radii.
    void fillRadii();
    /// Keep the radius typed into the table's @p row.
    void takeRadius(int row);
    void run();
    void showResult(const CollisionResult& result);
    void showProblem(const QString& problem);
    /// "4.20 mm", red below zero, into @p label.
    void showDistance(QLabel* label, const CollisionExtreme& extreme) const;
    QString positionText(const CollisionPosition& position) const;
    QString whereText(const CollisionExtreme& extreme) const;
    bool bothMeshes() const;

    Check m_check;
    Members m_members;
    CollisionTravel m_travel;
    std::vector<CollisionChoice> m_choices;
    QHash<QString, double> m_radii;
    bool m_filling = false;

    QComboBox* m_first = nullptr;
    QComboBox* m_second = nullptr;
    QDoubleSpinBox* m_radius = nullptr;
    QDoubleSpinBox* m_joint = nullptr;
    QTableWidget* m_radiiTable = nullptr;
    QLabel* m_headline = nullptr;
    QLabel* m_rideHeight = nullptr;
    QLabel* m_rideHeightAt = nullptr;
    QLabel* m_overTravel = nullptr;
    QLabel* m_overTravelAt = nullptr;
    QLabel* m_design = nullptr;
    QLabel* m_where = nullptr;
    QWidget* m_results = nullptr;
    QLabel* m_note = nullptr;
};

} // namespace suspkin
