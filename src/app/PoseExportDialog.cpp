#include "app/PoseExportDialog.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>

namespace suspkin {
namespace {

/// Bump on top, as a car is drawn; full right lock on the left, as a driver
/// sees the wheel turned that way.
const TravelEnd kRows[] = { TravelEnd::Positive, TravelEnd::Centre, TravelEnd::Negative };
const TravelEnd kColumns[] = { TravelEnd::Negative, TravelEnd::Centre, TravelEnd::Positive };

QString mm(double value) { return QString::number(value, 'g', 6); }

} // namespace

PoseExportDialog::PoseExportDialog(const PoseExportSettings& settings,
                                   const CollisionTravel& travel, const QString& defaultBaseName,
                                   QWidget* parent)
    : QDialog(parent), m_travel(travel), m_defaultBaseName(defaultBaseName)
{
    setWindowTitle(tr("Export Posed Workbooks"));

    auto* intro = new QLabel(
        tr("One workbook per pose ticked, laid out like the project's own, with every point "
           "where the solve puts it: every axle at that wheel travel, and the rack moving "
           "every axle it steers. The far side is written too, because steering does not "
           "mirror.\n\nThe travel is the sweep parameters': rack %1 mm each way, bump %2 mm, "
           "rebound %3 mm.")
            .arg(mm(travel.rack), mm(travel.bump), mm(travel.rebound)),
        this);
    intro->setWordWrap(true);

    m_example = new QLabel(this);
    m_example->setWordWrap(true);

    m_buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    m_buttons->button(QDialogButtonBox::Ok)->setText(tr("Export"));
    connect(m_buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(m_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(intro);
    layout->addWidget(buildGrid(settings));
    layout->addWidget(buildFolder(settings));
    layout->addWidget(m_example);
    layout->addWidget(m_buttons);

    refresh();
}

QWidget* PoseExportDialog::buildGrid(const PoseExportSettings& settings)
{
    auto* grid = new QWidget(this);
    auto* layout = new QGridLayout(grid);
    layout->setContentsMargins(0, 0, 0, 0);

    const QString columnLabels[] = { tr("Rack -%1 mm\n(full right)").arg(mm(m_travel.rack)),
                                     tr("Straight ahead"),
                                     tr("Rack +%1 mm\n(full left)").arg(mm(m_travel.rack)) };
    const QString rowLabels[] = { tr("Full bump (%1 mm)").arg(mm(m_travel.bump)),
                                  tr("Ride height"),
                                  tr("Full rebound (%1 mm)").arg(mm(m_travel.rebound)) };
    for (int column = 0; column < 3; ++column) {
        auto* header = new QLabel(columnLabels[column], grid);
        header->setAlignment(Qt::AlignCenter);
        layout->addWidget(header, 0, column + 1);
    }

    for (int row = 0; row < 3; ++row) {
        layout->addWidget(new QLabel(rowLabels[row], grid), row + 1, 0);
        for (int column = 0; column < 3; ++column) {
            const ExportPose pose{ kColumns[column], kRows[row] };
            auto* box = new QCheckBox(grid);
            box->setChecked(std::find(settings.poses.begin(), settings.poses.end(), pose)
                            != settings.poses.end());
            connect(box, &QCheckBox::toggled, this, &PoseExportDialog::refresh);
            layout->addWidget(box, row + 1, column + 1, Qt::AlignCenter);
            m_boxes.emplace_back(pose, box);
        }
    }
    return grid;
}

QWidget* PoseExportDialog::buildFolder(const PoseExportSettings& settings)
{
    auto* panel = new QWidget(this);
    auto* layout = new QGridLayout(panel);
    layout->setContentsMargins(0, 0, 0, 0);

    m_directory = new QLineEdit(QDir::toNativeSeparators(settings.directory), panel);
    auto* browse = new QPushButton(tr("Browse..."), panel);
    connect(browse, &QPushButton::clicked, this, &PoseExportDialog::browse);
    connect(m_directory, &QLineEdit::textChanged, this, &PoseExportDialog::refresh);

    m_baseName = new QLineEdit(settings.baseName, panel);
    m_baseName->setPlaceholderText(m_defaultBaseName);
    connect(m_baseName, &QLineEdit::textChanged, this, &PoseExportDialog::refresh);

    layout->addWidget(new QLabel(tr("Folder:"), panel), 0, 0);
    layout->addWidget(m_directory, 0, 1);
    layout->addWidget(browse, 0, 2);
    layout->addWidget(new QLabel(tr("Name:"), panel), 1, 0);
    layout->addWidget(m_baseName, 1, 1, 1, 2);
    return panel;
}

void PoseExportDialog::browse()
{
    const QString directory = QFileDialog::getExistingDirectory(
        this, tr("Export posed workbooks to"), QDir::fromNativeSeparators(m_directory->text()));
    if (!directory.isEmpty()) m_directory->setText(QDir::toNativeSeparators(directory));
}

void PoseExportDialog::refresh()
{
    const std::vector<ExportPose> poses = tickedPoses();
    const PoseExportSettings chosen = settings();
    m_buttons->button(QDialogButtonBox::Ok)
        ->setEnabled(!poses.empty() && !chosen.directory.isEmpty());

    if (poses.empty()) {
        m_example->setText(tr("Tick at least one pose."));
        return;
    }
    m_example->setText(tr("%n workbook(s), named like %1", nullptr, int(poses.size()))
                           .arg((chosen.baseName.isEmpty() ? m_defaultBaseName : chosen.baseName)
                                + QLatin1Char('_')
                                + exportPoseSuffix(poses.front(), m_travel)
                                + QStringLiteral(".xlsx")));
}

std::vector<ExportPose> PoseExportDialog::tickedPoses() const
{
    // In allExportPoses()'s order, not the grid's, so the manifest does not
    // change with the layout of this dialog.
    std::vector<ExportPose> poses;
    for (const ExportPose& pose : allExportPoses())
        for (const auto& [boxPose, box] : m_boxes)
            if (boxPose == pose && box->isChecked()) poses.push_back(pose);
    return poses;
}

PoseExportSettings PoseExportDialog::settings() const
{
    PoseExportSettings settings;
    settings.poses = tickedPoses();
    settings.directory = QDir::fromNativeSeparators(m_directory->text().trimmed());
    settings.baseName = m_baseName->text().trimmed();
    return settings;
}

} // namespace suspkin
