#include "app/WheelDialog.h"

#include "io/MeshImport.h"
#include "model/HardpointMirror.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

namespace suspkin {
namespace {

/// What a corner combo holds when the user has not picked a point for it.
const char kNoPoint[] = "";

} // namespace

WheelDialog::WheelDialog(const HardpointTable& table, MirrorAxis mirror, const WheelSpec& spec,
                         const QString& tyrePath, const QString& rimPath,
                         const QString& browseDirectory, QWidget* parent)
    : QDialog(parent), m_table(table), m_mirror(mirror), m_browseDirectory(browseDirectory)
{
    setWindowTitle(tr("Add wheels"));

    // --- one centre per axle -----------------------------------------------
    auto* cornerBox = new QGroupBox(tr("Wheel centres"), this);
    auto* cornerForm = new QFormLayout(cornerBox);
    const QString axleLabels[] = { tr("Front axle"), tr("Rear axle") };
    for (int axle = 0; axle < int(m_axles.size()); ++axle) {
        auto* combo = new QComboBox(cornerBox);
        combo->addItem(tr("- not used -"), QString::fromLatin1(kNoPoint));
        // The points the table lists: a ghost is the far wheel, not a choice.
        for (int row = 0; row < static_cast<int>(m_table.points.size()); ++row)
            if (!isGhostAt(m_table, row))
                combo->addItem(m_table.points[std::size_t(row)].name,
                               m_table.points[std::size_t(row)].name);
        combo->setToolTip(tr("The hardpoint at the centre of this axle's wheel. The wheel on the "
                             "other side goes on its mirror image. The models are placed there, "
                             "and follow the point when it moves."));
        m_axles[std::size_t(axle)] = combo;
        cornerForm->addRow(axleLabels[axle], combo);
        connect(combo, &QComboBox::currentIndexChanged, this, [this] { refreshSummary(); });
    }

    auto* guess = new QPushButton(tr("Guess from the hardpoint names"), cornerBox);
    guess->setToolTip(tr("Look for points named like a wheel centre and sort them by where they "
                         "are: forward is +X and left is +Y."));
    connect(guess, &QPushButton::clicked, this, [this] {
        WheelSpec guessed = guessWheelSpec(m_table);
        // Only the four points are a guess; how the models are placed is the
        // user's setting and is left alone.
        for (int axle = 0; axle < int(m_axles.size()); ++axle)
            fillAxle(axle, axleCentre(guessed, axle));
        refreshSummary();
    });
    cornerForm->addRow(QString(), guess);

    // --- the two models ----------------------------------------------------
    auto* modelBox = new QGroupBox(tr("Models"), this);
    auto* modelForm = new QFormLayout(modelBox);
    // A wheel is the two of them together: the tyre, and the rim it sits on.
    m_tyrePath = buildModelRow(modelForm, tr("Tyre:"), tyrePath, tr("Choose the tyre model"));
    m_rimPath = buildModelRow(modelForm, tr("Rim:"), rimPath, tr("Choose the rim model"));

    auto* modelHint = new QLabel(
        tr("One model each, drawn at every wheel centre. Leave a field empty to draw only the "
           "other one. The files are copied into the project."),
        modelBox);
    modelHint->setWordWrap(true);
    modelForm->addRow(modelHint);

    // --- how they are placed ----------------------------------------------
    auto* placementBox = new QGroupBox(tr("Placement"), this);
    auto* placementForm = new QFormLayout(placementBox);

    m_modelSide = new QComboBox(placementBox);
    m_modelSide->addItem(tr("The left side - mirror them for the right"), int(WheelModelSide::Left));
    m_modelSide->addItem(tr("The right side - mirror them for the left"), int(WheelModelSide::Right));
    m_modelSide->addItem(tr("Neither - never mirror them"), int(WheelModelSide::Symmetric));
    m_modelSide->setToolTip(tr("A rim is dished, so the same model cannot simply be dropped onto "
                               "all four corners. The far side is drawn as its mirror image."));
    placementForm->addRow(tr("The models are drawn for:"), m_modelSide);

    m_alignToCenter = new QCheckBox(tr("Put the centre of each model on its hardpoint"),
                                    placementBox);
    m_alignToCenter->setToolTip(tr("On for a wheel modelled on its own, wherever its origin sits. "
                                   "Off for one already positioned in vehicle coordinates, which "
                                   "is then placed by its own origin."));
    placementForm->addRow(m_alignToCenter);

    m_summary = new QLabel(this);
    m_summary->setWordWrap(true);

    m_buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    m_buttons->button(QDialogButtonBox::Ok)->setText(tr("Add wheels"));
    connect(m_buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(m_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(cornerBox);
    layout->addWidget(modelBox);
    layout->addWidget(placementBox);
    layout->addWidget(m_summary);
    layout->addStretch(1);
    layout->addWidget(m_buttons);

    connect(m_modelSide, &QComboBox::currentIndexChanged, this, [this] { refreshSummary(); });
    connect(m_alignToCenter, &QCheckBox::toggled, this, [this] { refreshSummary(); });

    // A project that has never had wheels gets the guess as its starting point,
    // which is right often enough to be worth the one glance it costs to check.
    applySpec(spec.isEmpty() ? [&] {
        WheelSpec guessed = guessWheelSpec(table);
        guessed.modelSide = spec.modelSide;
        guessed.alignToCenter = spec.alignToCenter;
        return guessed;
    }() : spec);

    refreshSummary();
    resize(560, 520);
}

QLineEdit* WheelDialog::buildModelRow(QFormLayout* form, const QString& label, const QString& path,
                                      const QString& title)
{
    auto* row = new QWidget(this);
    auto* rowLayout = new QHBoxLayout(row);
    rowLayout->setContentsMargins(0, 0, 0, 0);

    auto* edit = new QLineEdit(path, row);
    edit->setClearButtonEnabled(true);
    edit->setPlaceholderText(tr("no model"));
    edit->setToolTip(tr("The STEP or STL file of the model. It is copied into the project."));
    rowLayout->addWidget(edit, 1);

    auto* browse = new QPushButton(tr("Browse..."), row);
    rowLayout->addWidget(browse);
    connect(browse, &QPushButton::clicked, this, [this, edit, title] {
        // Start where the field already points, so replacing a model lands in
        // the folder the last one came from.
        const QString current = edit->text().trimmed();
        const QString start =
            current.isEmpty() ? m_browseDirectory : QFileInfo(current).absolutePath();
        const QString chosen = QFileDialog::getOpenFileName(this, title, start, importFileFilter());
        if (!chosen.isEmpty()) edit->setText(QDir::toNativeSeparators(chosen));
    });
    connect(edit, &QLineEdit::textChanged, this, [this] { refreshSummary(); });

    form->addRow(label, row);
    return edit;
}

void WheelDialog::fillAxle(int axle, const QString& name)
{
    QComboBox* combo = m_axles[std::size_t(axle)];
    int index = combo->findData(name);
    if (index < 0 && !name.isEmpty()) {
        // A point the project remembers that this workbook does not have. Kept
        // as a choice rather than dropped, so importing the right workbook again
        // restores the wheel instead of losing which point it was on.
        combo->addItem(tr("%1 (not in this table)").arg(name), name);
        index = combo->count() - 1;
    }
    combo->setCurrentIndex(index < 0 ? 0 : index);
}

QString WheelDialog::axleCentre(const WheelSpec& spec, int axle) const
{
    // The left corner first, as kWheelCorners lists it.
    const bool front = axle == 0;
    for (const WheelCorner corner : kWheelCorners) {
        const QString& name = spec.point(corner);
        if (isFrontWheel(corner) != front || name.isEmpty()) continue;
        // A wheel on a ghost is the far wheel of the ghost's source.
        const int row = m_table.indexOf(name);
        return isGhostAt(m_table, row) ? m_table.points[std::size_t(row)].mirrorOf : name;
    }
    return {};
}

void WheelDialog::setAxle(WheelSpec& spec, int axle, const QString& name) const
{
    const bool front = axle == 0;
    const WheelCorner left = front ? WheelCorner::FrontLeft : WheelCorner::RearLeft;
    const WheelCorner right = front ? WheelCorner::FrontRight : WheelCorner::RearRight;
    const Hardpoint* point = m_table.find(name);
    if (!point) {
        spec.setPoint(left, name); // not in this table: kept as the project had it
        return;
    }
    const QString image = mirrorImageName(m_table, name, m_mirror);
    // Left is +y, whichever of the two the user picked.
    const bool pickedLeft = point->coord[1] >= 0.0;
    spec.setPoint(pickedLeft ? left : right, name);
    spec.setPoint(pickedLeft ? right : left, image);
}

void WheelDialog::applySpec(const WheelSpec& spec)
{
    for (int axle = 0; axle < int(m_axles.size()); ++axle) fillAxle(axle, axleCentre(spec, axle));
    m_modelSide->setCurrentIndex(m_modelSide->findData(int(spec.modelSide)));
    m_alignToCenter->setChecked(spec.alignToCenter);
}

WheelSpec WheelDialog::spec() const
{
    WheelSpec spec;
    for (int axle = 0; axle < int(m_axles.size()); ++axle) {
        const QString name = m_axles[std::size_t(axle)]->currentData().toString();
        if (!name.isEmpty()) setAxle(spec, axle, name);
    }
    spec.modelSide = static_cast<WheelModelSide>(m_modelSide->currentData().toInt());
    spec.alignToCenter = m_alignToCenter->isChecked();
    return spec;
}

QString WheelDialog::tyrePath() const { return m_tyrePath->text().trimmed(); }
QString WheelDialog::rimPath() const { return m_rimPath->text().trimmed(); }

void WheelDialog::refreshSummary()
{
    QStringList warnings;
    const std::vector<WheelPlacement> placements = resolveWheels(spec(), m_table, &warnings);
    const int models = int(!tyrePath().isEmpty()) + int(!rimPath().isEmpty());

    // Both halves are needed for anything to appear: a model with nowhere to go
    // and a centre with nothing to draw are equally invisible.
    const bool usable = !placements.empty() && models > 0;
    m_buttons->button(QDialogButtonBox::Ok)->setEnabled(usable);

    QStringList lines;
    if (placements.empty()) {
        lines << tr("Pick the hardpoint each axle's wheel is centred on.");
    } else if (models == 0) {
        lines << tr("Pick a tyre model, a rim model, or both.");
    } else {
        int mirrored = 0;
        for (const WheelPlacement& placement : placements) mirrored += placement.mirrored ? 1 : 0;
        lines << tr("%1 wheel(s), %2 of them mirrored, from %3 model(s).")
                     .arg(placements.size())
                     .arg(mirrored)
                     .arg(models);
    }
    lines += warnings;
    m_summary->setText(lines.join(QStringLiteral("\n")));
}

} // namespace suspkin
