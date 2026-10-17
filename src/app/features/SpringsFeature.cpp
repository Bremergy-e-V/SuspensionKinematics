#include "app/SpringsDialog.h"
#include "app/framework/AppContext.h"
#include "app/framework/CommandRegistry.h"
#include "app/framework/Feature.h"
#include "app/framework/FeatureRegistry.h"
#include "app/session/ProjectSession.h"
#include "model/Simulation.h"
#include "project/Project.h"

#include <QCoreApplication>
#include <QMainWindow>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("SpringsFeature", text); }

/// What the car is sprung with: the spring, the anti-roll bar and the tyre of
/// each axle, which turn the solved motion into loads.
class SpringsFeature : public Feature {
public:
    explicit SpringsFeature(AppContext& context) : m_context(context) {}

    QString id() const override { return QStringLiteral("springs"); }

    void registerCommands(CommandRegistry& commands) override
    {
        commands.add({
            .id = QStringLiteral("linkage.springs"),
            .text = tr("Springs and &Tyres..."),
            .icon = Icon::WaveSine,
            .iconText = tr("Springs\nand Tyres"),
            .statusTip = tr("Give each axle a spring rate, an anti-roll bar rate and a tyre "
                            "rate, so the analysis shows wheel loads, tyre deflection, ride rate "
                            "and roll moment as well as angles."),
            .ribbon = { { QStringLiteral("linkage"), tr("Alignment"), RibbonButton::Large,
                          nullptr, 55 } },
            .run = [this] { springsDialog(); },
            // Loads are read off a solved motion, so there has to be one.
            .enabledWhen = [this] {
                return !m_context.session().simulation().simulation().isEmpty();
            },
        });
    }

private:
    void springsDialog()
    {
        const std::vector<SpringsAxle> rows = springsAxles();
        if (rows.empty()) return;

        SpringsDialog dialog(rows, m_context.window());
        if (dialog.exec() != QDialog::Accepted) return;

        // Every axle shown is replaced; one that does not solve today keeps
        // what the project said about it.
        Project& project = m_context.project();
        QHash<QString, AxleSprings> springs = project.springs();
        for (const SpringsAxle& row : rows) springs.remove(row.token);
        const QHash<QString, AxleSprings> chosen = dialog.springs();
        for (auto it = chosen.begin(); it != chosen.end(); ++it) springs.insert(it.key(), it.value());
        if (springs == project.springs()) return;

        // Not an edit of the points, so not a step: like the design targets,
        // the rates are saved when the dialog is accepted.
        project.setSprings(springs);
        m_context.session().resolveMechanism();
        m_context.markDirty();
        m_context.showStatus(tr("Spring, bar and tyre rates saved with the project."), 5000);
    }

    /// Every axle that solves, with what the project gives it and what the
    /// dialog needs to say about it.
    std::vector<SpringsAxle> springsAxles() const
    {
        const Project& project = m_context.project();
        std::vector<SpringsAxle> rows;
        for (const AxleSolver& axle : m_context.session().simulation().simulation().axles()) {
            SpringsAxle row;
            row.token = axle.cornerToken();
            row.label = axle.label().isEmpty() ? tr("Suspension") : axle.label();
            row.springs = project.springs().value(axle.cornerToken());

            const std::optional<CornerSolver>& near = axle.left() ? axle.left() : axle.right();
            if (near && near->hasDamper()) {
                const AxleSample design = sampleAxleAt(axle, SweepKind::Bump, 0.0, 0.0);
                row.installationRatio = axle.left() ? design.leftInstallationRatio
                                                    : design.rightInstallationRatio;
            }
            row.hasAntiRoll = near && near->hasAntiRoll();
            rows.push_back(row);
        }
        return rows;
    }

    AppContext& m_context;
};

} // namespace

SUSPKIN_FEATURE(SpringsFeature)

} // namespace suspkin
