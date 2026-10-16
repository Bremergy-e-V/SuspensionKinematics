#include "app/HardpointModel.h"
#include "app/RockerDialog.h"
#include "app/features/SharedCommands.h"
#include "app/framework/AppContext.h"
#include "app/framework/CommandRegistry.h"
#include "app/framework/Feature.h"
#include "app/framework/FeatureRegistry.h"
#include "app/session/ProjectSession.h"
#include "model/Linkage.h"
#include "project/Project.h"

#include <QCoreApplication>
#include <QMainWindow>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("RockerFeature", text); }

/// Linkage > Rocker: the rocker and its damper as a part of their own.
///
/// The design -- arms and angles -- is project state from the moment it is
/// placed, and is what makes the rocker's points one rigid part. The points
/// themselves go in as one edit, the way Generate's do.
class RockerFeature : public Feature {
public:
    explicit RockerFeature(AppContext& context) : m_context(context) {}

    QString id() const override { return QStringLiteral("rocker"); }

    void registerCommands(CommandRegistry& commands) override
    {
        commands.add({
            .id = QStringLiteral("linkage.rocker"),
            .text = tr("&Rocker..."),
            .icon = Icon::Triangle,
            .iconText = tr("Rocker"),
            .statusTip = tr("Design each corner's rocker and damper by its arms and angles, and "
                            "make it one part that moves as a whole."),
            .ribbon = { { QStringLiteral("linkage"), tr("Rocker"), RibbonButton::Large, nullptr,
                          45 } },
            .run = [this] { rockerDialog(); },
            .enabledWhen = [this] { return canDesign(); },
        });
    }

private:
    /// The rocker's points are named through the template's mechanism, so it
    /// needs one that has a rocker, and corners to name it in.
    bool canDesign() const
    {
        const LinkageTemplate& templ = m_context.session().linkage().linkageTemplate();
        return templ.mechanism.hasRocker() && !templ.corners.empty();
    }

    void rockerDialog()
    {
        if (!canDesign()) return; // the action is disabled
        ProjectSession& session = m_context.session();
        Project& project = m_context.project();

        RockerDialog dialog(project.rockers(), session.linkage().linkageTemplate(),
                            project.mirror(), m_context.hardpoints()->table(),
                            session.hardpoints().baseline(), m_context.window());
        if (dialog.exec() != QDialog::Accepted) return;

        // Before the points: placing them resolves the table again, and that
        // is where the parts are told which points are one rocker.
        project.setRockers(dialog.designs());
        m_context.markDirty();

        const DesignPlan& plan = dialog.plan();
        if (!plan.ok() || plan.count(DesignChange::Kind::Added) + plan.count(DesignChange::Kind::Moved) == 0) {
            session.applyRigidGroups();
            m_context.showStatus(tr("Rockers saved; no point moved."), 6000);
            return;
        }

        if (project.hardpoints().isEmpty()) {
            if (!adoptNewWorkbook(m_context, plan.table)) return;
        } else {
            m_context.setHardpointTable(plan.table, false);
            session.hardpoints().captureMirrorProvenance();
        }
        session.recordEdit(tr("Place rocker"));
        m_context.showStatus(tr("Placed %1 rocker point(s): %2 new, %3 moved.")
                                 .arg(plan.changes.size())
                                 .arg(plan.count(DesignChange::Kind::Added))
                                 .arg(plan.count(DesignChange::Kind::Moved)),
                             8000);
    }

    AppContext& m_context;
};

} // namespace

SUSPKIN_FEATURE(RockerFeature)

} // namespace suspkin
