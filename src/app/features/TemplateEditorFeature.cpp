#include "app/HardpointModel.h"
#include "app/TemplateDraft.h"
#include "app/TemplateEditorDialog.h"
#include "app/features/SharedCommands.h"
#include "app/framework/AppContext.h"
#include "app/framework/CommandRegistry.h"
#include "app/framework/Feature.h"
#include "app/framework/FeatureRegistry.h"
#include "app/session/ProjectSession.h"
#include "io/LinkageTemplate.h"
#include "model/EditHistory.h"
#include "project/Project.h"

#include <QCoreApplication>
#include <QMainWindow>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("TemplateEditorFeature", text); }

/// Whether @p a and @p b hold the same points, in the same order, at the same
/// places and with the same provenance.
bool sameTable(const HardpointTable& a, const HardpointTable& b)
{
    if (a.points.size() != b.points.size()) return false;
    for (std::size_t i = 0; i < a.points.size(); ++i) {
        const Hardpoint& p = a.points[i];
        const Hardpoint& q = b.points[i];
        if (p.name != q.name || p.mirrorOf != q.mirrorOf || p.coord[0] != q.coord[0]
            || p.coord[1] != q.coord[1] || p.coord[2] != q.coord[2])
            return false;
    }
    return true;
}

const PartTemplate* partById(const LinkageTemplate& templ, const QString& id)
{
    for (const PartTemplate& part : templ.parts)
        if (part.id == id) return &part;
    return nullptr;
}

/// @p bytes with every part @p before has and @p after has not taken out, every
/// part @p after has added or replaced, and nothing else touched.
QByteArray patchParts(QByteArray bytes, const LinkageTemplate& before,
                      const LinkageTemplate& after, QString* error)
{
    for (const PartTemplate& part : before.parts) {
        if (partById(after, part.id)) continue;
        bytes = removeTemplatePart(bytes, part.id, error);
        if (bytes.isEmpty()) return bytes;
    }
    for (const PartTemplate& part : after.parts) {
        const PartTemplate* was = partById(before, part.id);
        if (was && *was == part) continue;
        bytes = was ? setTemplatePart(bytes, part, error) : addTemplatePart(bytes, part, error);
        if (bytes.isEmpty()) return bytes;
    }
    return bytes;
}

/// Linkage > Template Editor: parts, points, settings and graphics in one
/// window, Lotus's way, applied to the project as one step.
class TemplateEditorFeature : public Feature {
public:
    explicit TemplateEditorFeature(AppContext& context) : m_context(context) {}

    QString id() const override { return QStringLiteral("templateEditor"); }

    void registerCommands(CommandRegistry& commands) override
    {
        commands.add({
            .id = QStringLiteral("linkage.editTemplate"),
            .text = tr("&Template Editor..."),
            .icon = Icon::Template,
            .iconText = tr("Template\nEditor"),
            .statusTip = tr("Describe the suspension the way Lotus does: its parts, its points, "
                            "what each point is, and what is drawn."),
            .ribbon = { { QStringLiteral("linkage"), tr("Parts"), RibbonButton::Large, nullptr,
                          15 } },
            .run = [this] { editTemplate(); },
            .enabledWhen = [this] {
                return !m_context.session().linkage().linkageTemplate().isEmpty();
            },
        });
    }

    void collectViewState(ViewState& view) const override
    {
        view.templateEditorTab = m_tab;
        view.templateEditorFarSide = m_farSide;
    }

    void applyViewState(const ViewState& view) override
    {
        m_tab = view.templateEditorTab;
        m_farSide = view.templateEditorFarSide;
    }

private:
    void editTemplate()
    {
        ProjectSession& session = m_context.session();
        LinkageTemplate applied = session.linkage().linkageTemplate();
        TemplateEditorDialog dialog(applied, m_context.hardpoints()->table(),
                                    m_context.hardpoints()->config(), m_context.project().mirror(),
                                    m_context.window());
        dialog.setCurrentTab(m_tab);
        dialog.setFarSideShown(m_farSide);
        QObject::connect(&dialog, &TemplateEditorDialog::applyRequested, &dialog,
                         [&] { apply(dialog.draft(), &applied); });
        dialog.exec();

        // Where it was left is the project's, like every other view.
        if (m_tab != dialog.currentTab() || m_farSide != dialog.farSideShown()) {
            m_tab = dialog.currentTab();
            m_farSide = dialog.farSideShown();
            m_context.markDirty();
        }
    }

    /// Make @p draft the project's: the template's parts patched into its file,
    /// then the settings, then the points -- in that order, because taking the
    /// points in resolves the linkage again and refills the settings from the
    /// project's copy (CLAUDE.md, "adding, deleting, renaming").
    void apply(TemplateDraft& draft, LinkageTemplate* applied)
    {
        ProjectSession& session = m_context.session();
        if (!applyTemplate(draft, *applied)) return;
        for (const auto& [from, to] : draft.renamedBodies()) session.history().renameBody(from, to);
        *applied = draft.linkageTemplate();

        HardpointModel* model = m_context.hardpoints();
        const HardpointConfigMap config = draft.configWithMirrors();
        const bool configChanged = config != model->config();
        const HardpointTable& table = draft.model()->table();
        const bool tableChanged = !sameTable(table, model->table());

        model->setConfig(config);
        session.hardpoints().captureConfig();
        if (tableChanged && m_context.project().hardpoints().isEmpty()) {
            if (!adoptNewWorkbook(m_context, table)) return;
        } else if (tableChanged) {
            m_context.setHardpointTable(table, false);
            session.hardpoints().captureMirrorProvenance();
        } else {
            m_context.syncTableToViewport(false);
        }
        if (tableChanged || configChanged) session.recordEdit(tr("Edit template"));
        m_context.markDirty();
        draft.markApplied();
        m_context.showStatus(tr("Template applied"), 4000);
    }

    /// The template's parts, written into the project's file if they changed.
    bool applyTemplate(const TemplateDraft& draft, const LinkageTemplate& applied)
    {
        const LinkageTemplate& edited = draft.linkageTemplate();
        if (edited.parts == applied.parts) return true;
        SessionMessage problem;
        const bool written = m_context.session().linkage().patch(
            [&](const QByteArray& bytes, QString* error) {
                return patchParts(bytes, applied, edited, error);
            },
            tr("The template could not be changed."), &problem);
        m_context.showProblem(problem);
        return written;
    }

    AppContext& m_context;
    int m_tab = 0;
    bool m_farSide = false;
};

} // namespace

SUSPKIN_FEATURE(TemplateEditorFeature)

} // namespace suspkin
