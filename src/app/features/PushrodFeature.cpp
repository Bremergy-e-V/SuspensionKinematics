#include "app/HardpointModel.h"
#include "app/PushrodDialog.h"
#include "app/features/SharedCommands.h"
#include "app/framework/AppContext.h"
#include "app/framework/CommandRegistry.h"
#include "app/framework/Feature.h"
#include "app/framework/FeatureRegistry.h"
#include "app/session/ProjectSession.h"
#include "io/LinkageTemplate.h"
#include "project/Project.h"
#include "render/ViewportWidget.h"

#include <QCoreApplication>
#include <QMainWindow>

#include <algorithm>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("PushrodFeature", text); }

/// Linkage > Pushrod & Damper: how each corner's spring is worked, with its
/// ends laid out on the chassis and the wishbone, placed as one edit.
class PushrodFeature : public Feature {
public:
    explicit PushrodFeature(AppContext& context) : m_context(context) {}

    QString id() const override { return QStringLiteral("pushrod"); }

    void registerCommands(CommandRegistry& commands) override
    {
        commands.add({
            .id = QStringLiteral("linkage.pushrod"),
            .text = tr("&Pushrod and Damper..."),
            .icon = Icon::Line,
            .iconText = tr("Pushrod &&\nDamper"),
            .statusTip = tr("Lay out each corner's pushrod and damper -- to a rocker, in line, "
                            "or the damper alone -- from a ball joint and the chassis."),
            .ribbon = { { QStringLiteral("linkage"), tr("Rocker"), RibbonButton::Large, nullptr,
                          47 } },
            .run = [this] { pushrodDialog(); },
            .enabledWhen = [this] { return canDesign(); },
        });
    }

private:
    bool canDesign() const
    {
        const LinkageTemplate& templ = m_context.session().linkage().linkageTemplate();
        return !templ.isEmpty() && !templ.mechanism.isEmpty()
               && !m_context.hardpoints()->table().isEmpty();
    }

    void pushrodDialog()
    {
        if (!canDesign()) return; // the action is disabled
        ProjectSession& session = m_context.session();
        Project& project = m_context.project();

        PushrodDialog dialog(
            project.pushrods(), session.linkage().linkageTemplate(), project.mirror(),
            m_context.hardpoints()->table(), session.hardpoints().baseline(),
            [this] { return m_context.session().chassis().query(m_context.viewport()->mesh()); },
            m_context.window());
        if (dialog.exec() != QDialog::Accepted || !dialog.plan().ok()) return;

        saveDesigns(dialog.designs());
        if (!makePartsOptional(dialog.plan().optionalParts)) return;
        placePoints(dialog.plan());
    }

    /// The designs, and no rocker design for a corner that no longer has one.
    void saveDesigns(const std::vector<PushrodDesign>& designs)
    {
        Project& project = m_context.project();
        project.setPushrods(designs);
        std::vector<RockerDesign> rockers = project.rockers();
        std::erase_if(rockers, [&designs](const RockerDesign& rocker) {
            return std::any_of(designs.begin(), designs.end(), [&](const PushrodDesign& d) {
                return d.corner == rocker.corner && d.layout != PushrodLayout::Rocker;
            });
        });
        project.setRockers(rockers);
        m_context.markDirty();
    }

    /// A part some corner no longer has says nothing when it is missing there.
    bool makePartsOptional(const QStringList& ids)
    {
        const LinkageTemplate& templ = m_context.session().linkage().linkageTemplate();
        std::vector<PartTemplate> changed;
        for (const PartTemplate& part : templ.parts) {
            if (!ids.contains(part.id) || part.optional) continue;
            PartTemplate optional = part;
            optional.optional = true;
            changed.push_back(optional);
        }
        if (changed.empty()) return true;

        SessionMessage problem;
        const bool written = m_context.session().linkage().patch(
            [&changed](const QByteArray& bytes, QString* error) {
                QByteArray out = bytes;
                for (const PartTemplate& part : changed) {
                    out = setTemplatePart(out, part, error);
                    if (out.isEmpty()) break;
                }
                return out;
            },
            tr("The template could not be changed."), &problem);
        m_context.showProblem(problem);
        return written;
    }

    /// The settings first, then the points: taking the points in resolves the
    /// linkage again and refills the settings from the project's copy.
    void placePoints(const PushrodPlan& plan)
    {
        ProjectSession& session = m_context.session();
        HardpointModel* model = m_context.hardpoints();
        HardpointConfigMap config = model->config();
        for (auto it = plan.config.constBegin(); it != plan.config.constEnd(); ++it)
            config.insert(it.key(), it.value());
        model->setConfig(config);
        session.hardpoints().captureConfig();

        if (m_context.project().hardpoints().isEmpty()) {
            if (!adoptNewWorkbook(m_context, plan.points.table)) return;
        } else {
            m_context.setHardpointTable(plan.points.table, false);
            session.hardpoints().captureMirrorProvenance();
        }
        session.recordEdit(tr("Place pushrod"));
        m_context.showStatus(tr("Placed %1 point(s), took out %2.")
                                 .arg(plan.points.changes.size())
                                 .arg(plan.removed.size()),
                             8000);
    }

    AppContext& m_context;
};

} // namespace

SUSPKIN_FEATURE(PushrodFeature)

} // namespace suspkin
