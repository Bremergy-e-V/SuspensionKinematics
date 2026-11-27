#include "app/HardpointModel.h"
#include "app/PoseExportDialog.h"
#include "app/features/SharedCommands.h"
#include "app/framework/AppContext.h"
#include "app/framework/CommandRegistry.h"
#include "app/framework/Feature.h"
#include "app/framework/FeatureRegistry.h"
#include "app/session/ProjectSession.h"
#include "model/PoseExport.h"
#include "model/Simulation.h"
#include "project/Project.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QMainWindow>
#include <QMessageBox>
#include <QStringList>

#include <optional>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("PoseExportFeature", text); }

/// Hardpoints > Export Posed Workbooks: the table as it stands at the ends of
/// the steering and the wheel travel, one workbook per pose, so a CAD model
/// can be checked against the car at full lock and full bump as well as at
/// ride height.
class PoseExportFeature : public Feature {
public:
    explicit PoseExportFeature(AppContext& context) : m_context(context) {}

    QString id() const override { return QStringLiteral("poseExport"); }

    void registerCommands(CommandRegistry& commands) override
    {
        commands.add({
            .id = QStringLiteral("hardpoints.exportPosed"),
            .text = tr("Export &Posed Workbooks..."),
            .icon = Icon::SteeringWheel,
            .iconText = tr("Export Posed"),
            .statusTip = tr("Write the hardpoints at full lock, full bump and full rebound, "
                            "one workbook per pose, for checking against the CAD."),
            .ribbon = { { QStringLiteral("hardpoints"), tr("Workbook"), RibbonButton::Small,
                          nullptr, 35 } },
            .run = [this] { exportPosedWorkbooks(); },
            .enabledWhen = [this] {
                return !m_context.session().simulation().simulation().isEmpty()
                       && m_context.session().hardpoints().workbookWritable();
            },
        });
    }

private:
    void exportPosedWorkbooks()
    {
        const CollisionTravel travel = sweepTravel(m_context);
        const std::optional<PoseExportSettings> settings = askForPoses(travel);
        if (!settings) return;
        const QString baseName = settings->baseName.isEmpty() ? workbookBaseName()
                                                              : settings->baseName;
        const QStringList paths = posedPaths(*settings, baseName, travel);
        if (!QDir().mkpath(settings->directory) || !confirmReplace(paths)) return;
        writePoses(settings->poses, paths, travel);
    }

    /// What the user ticks, saved into the project once they say Export.
    std::optional<PoseExportSettings> askForPoses(const CollisionTravel& travel)
    {
        Project& project = m_context.project();
        PoseExportSettings saved = project.poseExport();
        if (saved.directory.isEmpty()) saved.directory = hardpointDialogDirectory(project);

        PoseExportDialog dialog(saved, travel, workbookBaseName(), m_context.window());
        if (dialog.exec() != QDialog::Accepted) return std::nullopt;

        const PoseExportSettings chosen = dialog.settings();
        if (chosen != project.poseExport()) {
            project.setPoseExport(chosen);
            m_context.markDirty();
        }
        return chosen;
    }

    /// The project's workbook's name without its extension: what a posed one
    /// is called after unless the user says otherwise.
    QString workbookBaseName() const
    {
        const QString name =
            QFileInfo(m_context.project().hardpoints().workbook.relativePath).completeBaseName();
        return name.isEmpty() ? QStringLiteral("hardpoints") : name;
    }

    static QStringList posedPaths(const PoseExportSettings& settings, const QString& baseName,
                                  const CollisionTravel& travel)
    {
        const QDir directory(settings.directory);
        QStringList paths;
        for (const ExportPose& pose : settings.poses)
            paths << directory.filePath(baseName + QLatin1Char('_')
                                        + exportPoseSuffix(pose, travel) + QStringLiteral(".xlsx"));
        return paths;
    }

    /// Ask once before replacing any of @p paths that is already there.
    bool confirmReplace(const QStringList& paths) const
    {
        QStringList existing;
        for (const QString& path : paths)
            if (QFileInfo::exists(path)) existing << QFileInfo(path).fileName();
        if (existing.isEmpty()) return true;
        return QMessageBox::question(m_context.window(), tr("Replace the workbooks?"),
                                     tr("These are there already and will be replaced:\n\n%1")
                                         .arg(existing.join(QLatin1Char('\n'))),
                                     QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel)
               == QMessageBox::Yes;
    }

    /// Solve and write each of @p poses into its path, then say what came of it.
    void writePoses(const std::vector<ExportPose>& poses, const QStringList& paths,
                    const CollisionTravel& travel)
    {
        QStringList unsolved;
        int written = 0;
        for (std::size_t i = 0; i < poses.size(); ++i) {
            const std::optional<HardpointTable> table = posedTable(exportPosition(poses[i], travel));
            if (!table) {
                unsolved << exportPoseSuffix(poses[i], travel);
                continue;
            }
            SessionMessage problem;
            if (!m_context.session().hardpoints().writeTableTo(paths[int(i)], *table, &problem)) {
                m_context.showProblem(problem);
                return;
            }
            ++written;
        }
        reportWritten(written, paths.isEmpty() ? QString() : QFileInfo(paths.front()).absolutePath(),
                      unsolved);
    }

    /// The table with every point where the car puts it at @p position, the
    /// far side included. Nothing where a corner does not assemble there.
    std::optional<HardpointTable> posedTable(const CollisionPosition& position) const
    {
        const std::optional<SimulationPose> pose =
            m_context.session().simulation().simulation().poseAtWheelTravel(position.travel,
                                                                            position.rack);
        if (!pose) return std::nullopt;
        return pose->layOver(m_context.hardpoints()->table());
    }

    void reportWritten(int written, const QString& directory, const QStringList& unsolved) const
    {
        m_context.showStatus(tr("Wrote %1 posed workbook(s) to %2")
                                 .arg(written)
                                 .arg(QDir::toNativeSeparators(directory)),
                             8000);
        if (unsolved.isEmpty()) return;
        QMessageBox::information(
            m_context.window(), tr("Some poses were not written"),
            tr("The suspension does not assemble at these, so there is nothing to write:\n\n%1\n\n"
               "Less travel in the sweep parameters may reach them.")
                .arg(unsolved.join(QLatin1Char('\n'))));
    }

    AppContext& m_context;
};

} // namespace

SUSPKIN_FEATURE(PoseExportFeature)

} // namespace suspkin
