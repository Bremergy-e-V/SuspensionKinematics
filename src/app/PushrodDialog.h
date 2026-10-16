#pragma once

#include "model/Hardpoint.h"
#include "model/HardpointMirror.h"
#include "model/Linkage.h"
#include "model/PushrodDesign.h"

#include <QDialog>

#include <functional>
#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QTimer;
class QTreeWidget;

namespace suspkin {

class MeshQuery;

/// Linkage > Pushrod & Damper: per corner, how the spring is worked -- by the
/// rocker, by a pushrod in line with the damper, or by the damper alone -- and
/// where its ends go: the chassis end at a height on the chassis, the outboard
/// end a distance inboard of the chosen ball joint on the line between the two,
/// or at an offset of the user's own from that ball joint.
///
/// One column per corner of the template. Like Generate and Rocker, it
/// previews and nothing moves until Place.
class PushrodDialog : public QDialog {
    Q_OBJECT

public:
    using ChassisSource = std::function<const MeshQuery*()>;

    PushrodDialog(const std::vector<PushrodDesign>& stored, const LinkageTemplate& templ,
                  const MirrorSpec& mirror, const HardpointTable& current,
                  const HardpointTable& baseline, ChassisSource chassis,
                  QWidget* parent = nullptr);

    /// Every corner whose box is ticked, as it stands.
    std::vector<PushrodDesign> designs() const;
    /// What Place does, for the designs as they stand.
    const PushrodPlan& plan() const { return m_plan; }

private:
    /// The widgets of one corner's column.
    struct Column {
        QString corner;
        QCheckBox* used = nullptr;
        QComboBox* layout = nullptr;
        QComboBox* arm = nullptr;
        QDoubleSpinBox* height = nullptr;
        QDoubleSpinBox* clearance = nullptr;
        QDoubleSpinBox* inboardY = nullptr;
        QDoubleSpinBox* offset = nullptr;
        QComboBox* outboardMode = nullptr; ///< on the line, or at a custom offset
        QDoubleSpinBox* shiftX = nullptr;
        QDoubleSpinBox* shiftY = nullptr;  ///< inboard
        QDoubleSpinBox* shiftZ = nullptr;
        QDoubleSpinBox* damperLength = nullptr;
    };

    QWidget* buildColumns(const std::vector<PushrodDesign>& stored);
    QDoubleSpinBox* spin(double value, double min, double max, const QString& tip);
    PushrodDesign designOf(const Column& column) const;
    void enableFields(const Column& column);
    void replan();
    void showPlan();

    const LinkageTemplate& m_templ;
    MirrorSpec m_mirror;
    const HardpointTable& m_current;
    const HardpointTable& m_baseline;
    ChassisSource m_chassis;
    std::vector<Column> m_columns;
    PushrodPlan m_plan;

    QTimer* m_replan = nullptr;
    QTreeWidget* m_preview = nullptr;
    QLabel* m_summary = nullptr;
    QLabel* m_notes = nullptr;
    class QPushButton* m_place = nullptr;
};

} // namespace suspkin
