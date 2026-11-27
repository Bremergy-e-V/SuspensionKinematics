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

#include <optional>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("CollisionFeature", text); }

/// Analysis > Collision Check: how close two things the user picks -- a part,
/// a rim, a tyre, the chassis -- come over the travel the analysis panel
/// sweeps.
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
            .statusTip = tr("Walk the suspension through the sweep and report the least "
                            "distance between two parts, a part and a rim or tyre, or "
                            "either and the chassis."),
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
        const Linkage& linkage = m_context.session().linkage().parts();
        const CollisionMeshes meshes = collisionMeshes();
        // The sweep does not change while the dialog is open, so it is walked
        // once, the first time something is measured.
        std::optional<std::vector<CollisionFrame>> walked;
        const auto check = [&](const CollisionSettings& settings) {
            if (!walked) walked = frames();
            return measure(settings, *walked, meshes);
        };

        CollisionDialog dialog(m_context.project().collision(), choices(linkage, meshes),
                               sweepInputLabel(request.spec.kind),
                               sweepInputUnit(request.spec.kind), check, m_context.window());
        dialog.exec();

        // The choice is the user's work whether or not anything was found.
        const CollisionSettings settings = dialog.settings();
        if (settings == m_context.project().collision()) return;
        m_context.project().setCollision(settings);
        m_context.markDirty();
    }

    /// The meshes the project has, each with the query a distance is
    /// measured through.
    CollisionMeshes collisionMeshes() const
    {
        ProjectSession& session = m_context.session();
        const ViewportWidget* viewport = m_context.viewport();
        CollisionMeshes meshes;
        meshes.chassis = { &viewport->mesh(), session.chassis().query(viewport->mesh()) };
        meshes.rim = { &viewport->rimMesh(), session.wheels().rimQuery(viewport->rimMesh()) };
        meshes.tyre = { &viewport->tyreMesh(), session.wheels().tyreQuery(viewport->tyreMesh()) };
        meshes.alignToCenter = m_context.project().wheels().spec.alignToCenter;
        return meshes;
    }

    /// Every target the dropdowns offer, labelled.
    std::vector<CollisionChoice> choices(const Linkage& linkage, const CollisionMeshes& meshes) const
    {
        std::vector<CollisionChoice> out;
        for (const CollisionTarget& target :
             collisionTargets(linkage, m_context.project().wheels().spec, meshes))
            out.push_back(CollisionChoice{ collisionTargetKey(target),
                                           collisionTargetLabel(target, linkage), !target.isPart() });
        return out;
    }

    CollisionResult measure(const CollisionSettings& settings,
                            const std::vector<CollisionFrame>& frames,
                            const CollisionMeshes& meshes) const
    {
        const std::optional<CollisionTarget> first = collisionTargetFromKey(settings.first);
        const std::optional<CollisionTarget> second = collisionTargetFromKey(settings.second);
        if (!first || !second) return CollisionResult{ false, tr("Choose two things to measure between."),
                                                       0.0, 0.0, false, 0.0, 0 };
        const HardpointTable& design = m_context.hardpoints()->table();
        return minimumDistance(m_context.session().linkage().parts(), frames,
                               groundedPoints(design, m_context.hardpoints()->config()), meshes,
                               settings, *first, *second);
    }

    /// The car at every position of the analysis panel's sweep, in its own
    /// coordinates -- where the chassis is -- with every axle moving, and
    /// where the wheel models are at each.
    std::vector<CollisionFrame> frames() const
    {
        ProjectSession& session = m_context.session();
        const HardpointTable& design = m_context.hardpoints()->table();
        const SimulationRequest request = session.request();
        const Simulation& simulation = session.simulation().simulation();
        const WheelsRef& wheels = m_context.project().wheels();
        const WheelRotations attitudes = simulation.designWheelAttitudes();
        std::vector<CollisionFrame> frames;
        if (simulation.isEmpty()) return frames;

        const SweepSpec& spec = request.spec;
        for (int i = 0; i < std::max(2, spec.steps); ++i) {
            const double input = spec.inputAt(i);
            SimulationPose pose =
                simulation.poseAt(request.axle, spec.kind, input, spec.rackTravel, true,
                                  spec.tyreCompliance);
            // A roll is solved with the road tilted under a body that stays
            // put, which is the frame the chassis mesh is in.
            pose.bodyMotion.reset();
            frames.push_back(CollisionFrame{
                input, pose.layOver(design),
                wheels.isEmpty() ? std::vector<WheelPlacement>{}
                                 : placeWheels(wheels.spec, design, attitudes, pose) });
        }
        return frames;
    }

    AppContext& m_context;
};

} // namespace

SUSPKIN_FEATURE(CollisionFeature)

} // namespace suspkin
