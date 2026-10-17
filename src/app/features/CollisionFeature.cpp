#include "app/CollisionDialog.h"
#include "app/HardpointModel.h"
#include "app/framework/AppContext.h"
#include "app/framework/CommandRegistry.h"
#include "app/framework/Feature.h"
#include "app/framework/FeatureRegistry.h"
#include "app/session/ProjectSession.h"
#include "model/Collision.h"
#include "model/HardpointConfig.h"
#include "model/Simulation.h"
#include "project/Project.h"
#include "render/ViewportWidget.h"

#include <QCoreApplication>
#include <QMainWindow>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("CollisionFeature", text); }

/// Analysis > Collision Check: whether anything runs into anything else over
/// the travel the analysis panel sweeps.
class CollisionFeature : public Feature {
public:
    explicit CollisionFeature(AppContext& context) : m_context(context) {}

    QString id() const override { return QStringLiteral("collision"); }

    void registerCommands(CommandRegistry& commands) override
    {
        commands.add({
            .id = QStringLiteral("analysis.collision"),
            .text = tr("&Collision Check..."),
            .icon = Icon::CarCrash,
            .iconText = tr("Collision\nCheck"),
            .statusTip = tr("Walk the suspension through the sweep and list how close its "
                            "members come to each other and to the chassis."),
            .ribbon = { { .page = QStringLiteral("analysis"),
                          .group = tr("Check"),
                          .order = 40 } },
            .run = [this] { collisionDialog(); },
            .enabledWhen = [this] {
                return !m_context.session().simulation().simulation().isEmpty();
            },
        });
    }

private:
    void collisionDialog()
    {
        const SimulationRequest request = m_context.session().request();
        CollisionDialog dialog(
            m_context.project().collision(),
            sweepInputLabel(request.spec.kind) + QStringLiteral(", ")
                + sweepInputUnit(request.spec.kind),
            [this](const CollisionSettings& settings) { return check(settings); },
            m_context.window());
        dialog.exec();

        // The settings are the user's work whether or not anything was found.
        const CollisionSettings settings = dialog.settings();
        if (settings == m_context.project().collision()) return;
        m_context.project().setCollision(settings);
        m_context.markDirty();
    }

    CollisionReport check(const CollisionSettings& settings)
    {
        ProjectSession& session = m_context.session();
        const HardpointTable& design = m_context.hardpoints()->table();
        const Linkage& linkage = session.linkage().parts();
        const MeshQuery* chassis =
            settings.checkChassis ? session.chassis().query(m_context.viewport()->mesh()) : nullptr;
        return checkCollisions(linkage, frames(design), groundedPoints(design, m_context.hardpoints()->config()),
                               chassis, settings);
    }

    /// The table at every position of the analysis panel's sweep, in the car's
    /// own coordinates -- where the chassis is -- with every axle moving.
    std::vector<CollisionFrame> frames(const HardpointTable& design) const
    {
        const SimulationRequest request = m_context.session().request();
        const Simulation& simulation = m_context.session().simulation().simulation();
        std::vector<CollisionFrame> frames;
        if (simulation.isEmpty()) return frames;

        const SweepSpec& spec = request.spec;
        for (int i = 0; i < std::max(2, spec.steps); ++i) {
            const double input = spec.inputAt(i);
            SimulationPose pose =
                simulation.poseAt(request.axle, spec.kind, input, spec.rackTravel, true);
            // A roll is solved with the road tilted under a body that stays
            // put, which is the frame the chassis mesh is in.
            pose.bodyMotion.reset();
            frames.push_back(CollisionFrame{ input, pose.layOver(design) });
        }
        return frames;
    }

    AppContext& m_context;
};

} // namespace

SUSPKIN_FEATURE(CollisionFeature)

} // namespace suspkin
