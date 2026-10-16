#pragma once

#include "model/HardpointMirror.h"
#include "model/Wheels.h"

#include <QDialog>
#include <QString>

#include <array>

class QCheckBox;
class QComboBox;
class QDialogButtonBox;
class QFormLayout;
class QLabel;
class QLineEdit;

namespace suspkin {

/// Which hardpoints the wheels are centred on, and which two models are drawn
/// there.
///
/// One centre per axle: the wheel on the far side is centred on that point's
/// mirror image (mirrorImageName()), which the table is never asked to list.
///
/// The models are picked as files on disk; the project copies them in for
/// itself, as it does with everything else that is imported. A path handed to
/// the dialog and handed back unchanged means "keep the copy the project already
/// has", and an empty one means that model is not drawn at all.
class WheelDialog : public QDialog {
    Q_OBJECT

public:
    /// @p table is what the axle lists are filled from, @p mirror says where
    /// the far side is, @p spec seeds the
    /// controls with whatever the project last used, @p tyrePath and @p rimPath
    /// are the models it already holds, and @p browseDirectory is where the file
    /// dialog starts when there is nothing better to go on.
    WheelDialog(const HardpointTable& table, MirrorAxis mirror, const WheelSpec& spec,
                const QString& tyrePath, const QString& rimPath, const QString& browseDirectory,
                QWidget* parent = nullptr);

    WheelSpec spec() const;
    QString tyrePath() const;
    QString rimPath() const;

private:
    /// One model row: the path field and the button that fills it in.
    QLineEdit* buildModelRow(QFormLayout* form, const QString& label, const QString& path,
                             const QString& title);
    /// Show @p name as the centre of axle @p axle, the near-side point when
    /// @p name is a ghost.
    void fillAxle(int axle, const QString& name);
    /// The near-side point a spec names for @p axle, out of its two corners.
    QString axleCentre(const WheelSpec& spec, int axle) const;
    /// Name both corners of @p axle in @p spec: @p name and its mirror image,
    /// each on the side its y says.
    void setAxle(WheelSpec& spec, int axle, const QString& name) const;
    void applySpec(const WheelSpec& spec);
    void refreshSummary();

    const HardpointTable& m_table;
    MirrorAxis m_mirror = MirrorAxis::Y;
    QString m_browseDirectory;

    std::array<QComboBox*, 2> m_axles{}; ///< front, rear
    QLineEdit* m_tyrePath = nullptr;
    QLineEdit* m_rimPath = nullptr;
    QComboBox* m_modelSide = nullptr;
    QCheckBox* m_alignToCenter = nullptr;
    QLabel* m_summary = nullptr;
    QDialogButtonBox* m_buttons = nullptr;
};

} // namespace suspkin
