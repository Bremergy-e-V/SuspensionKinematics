#pragma once

#include "model/HardpointGenerator.h"
#include "model/RockerDesign.h"

#include <QDialog>

#include <functional>
#include <vector>

class QCheckBox;
class QDialogButtonBox;
class QDoubleSpinBox;
class QGridLayout;
class QLabel;
class QTimer;
class QTreeWidget;

namespace suspkin {

class RockerSketch;

/// Linkage > Rocker: each corner's rocker and damper as a part of its own --
/// the arms L1, L2 and L3 and their angles, the damper's angle and length --
/// with what placing it would do to the table beside it.
///
/// Like Generate from Design it is one-shot and previewed: nothing moves until
/// Place. A corner ticked here is a rigid part from then on, so dragging any
/// of its points drags the rocker; unticking it makes its points separate
/// points again, where they are.
class RockerDialog : public QDialog {
    Q_OBJECT

public:
    /// One column per corner of @p templ. A corner with a design in
    /// @p designs starts from it, its pivot where the table has the pivot now;
    /// one without starts from the rocker the table already has there, read
    /// back off its points, or from defaults.
    RockerDialog(const std::vector<RockerDesign>& designs, const LinkageTemplate& templ,
                 const MirrorSpec& mirror, const HardpointTable& current,
                 const HardpointTable& baseline, QWidget* parent = nullptr);

    /// The rockers ticked, as they stand in the dialog.
    std::vector<RockerDesign> designs() const;
    /// What Place would do. Empty, and not an error, when nothing is ticked.
    const DesignPlan& plan() const { return m_plan; }

public slots:
    void accept() override;

private:
    /// One number of a design, reached through an accessor because the pivot
    /// is a vector and the rest are plain members.
    struct Field {
        const char* heading; ///< starts a group when set
        const char* label;
        std::function<double&(RockerDesign&)> member;
        double minimum;
        double maximum;
        bool degrees;
        const char* tip;
    };
    struct Column {
        RockerDesign seed; ///< what the column opened with: its corner, its flags
        bool startsUsed = false; ///< the project has this rocker as a part already
        QCheckBox* used = nullptr;
        QCheckBox* antiRollArm = nullptr;
        std::vector<QDoubleSpinBox*> spins; ///< one per field, in order
    };

    QWidget* buildDesigns();
    QWidget* buildPreview();
    /// A field of column @p column: editing it plans again and draws that
    /// column's rocker.
    QDoubleSpinBox* number(const Field& field, int column);
    void addHeading(QGridLayout* grid, int* row, const QString& text);
    /// The column's design as its fields have it now.
    RockerDesign designOf(const Column& column) const;
    void schedulePlan();
    void rebuildPlan();
    void showPlan();
    /// A column's fields are only open while its rocker is ticked.
    void syncEnabled();
    /// Draw the rocker of @p column, the one last touched.
    void showSketchOf(int column);

    const LinkageTemplate& m_templ;
    MirrorSpec m_mirror;
    const HardpointTable& m_current;
    const HardpointTable& m_baseline;

    std::vector<Field> m_fields;
    std::vector<Column> m_columns;

    RockerSketch* m_sketch = nullptr;
    int m_sketchColumn = 0;
    QLabel* m_summary = nullptr;
    QTreeWidget* m_preview = nullptr;
    QLabel* m_notes = nullptr;
    QDialogButtonBox* m_buttons = nullptr;
    QTimer* m_timer = nullptr;
    DesignPlan m_plan;
    bool m_loading = false;
};

} // namespace suspkin
