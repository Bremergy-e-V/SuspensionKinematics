#pragma once

#include "model/PoseExport.h"

#include <QDialog>

#include <utility>
#include <vector>

class QCheckBox;
class QDialogButtonBox;
class QLabel;
class QLineEdit;

namespace suspkin {

/// Hardpoints > Export Posed Workbooks: which ends of the steering and the
/// wheel travel to write a workbook for, where to and under what name. The
/// writing is the caller's; this only asks.
class PoseExportDialog : public QDialog {
    Q_OBJECT

public:
    /// @p defaultBaseName is what a workbook is called when the user names
    /// nothing: the project's own workbook's.
    PoseExportDialog(const PoseExportSettings& settings, const CollisionTravel& travel,
                     const QString& defaultBaseName, QWidget* parent = nullptr);

    /// What was asked for. The name is empty when the user left it so.
    PoseExportSettings settings() const;

private:
    QWidget* buildGrid(const PoseExportSettings& settings);
    QWidget* buildFolder(const PoseExportSettings& settings);
    void browse();
    /// Say what the first file will be called, and allow Export only when
    /// there is something to write and somewhere to write it.
    void refresh();
    std::vector<ExportPose> tickedPoses() const;

    CollisionTravel m_travel;
    QString m_defaultBaseName;
    std::vector<std::pair<ExportPose, QCheckBox*>> m_boxes;
    QLineEdit* m_directory = nullptr;
    QLineEdit* m_baseName = nullptr;
    QLabel* m_example = nullptr;
    QDialogButtonBox* m_buttons = nullptr;
};

} // namespace suspkin
