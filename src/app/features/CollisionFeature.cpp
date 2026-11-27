#include "app/AnalysisPanel.h"
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
#include <QHash>
#include <QPair>
#include <QMainWindow>

#include <cmath>
#include <optional>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("CollisionFeature", text); }

/// Analysis > Collision Check: how close two things the user picks -- a part
/// or one tube of it, a rim, a tyre, the chassis -- come, with the rack
/// anywhere in its travel and the wheels anywhere between rebound and bump.
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
            .statusTip = tr("The least distance between two parts, a part and a rim or "
                            "tyre, or either and the chassis, at any steering position and "
                            "any wheel travel."),
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
    /// The car at every position the search asks for, each solved once.
    using FrameCache = QHash<QPair<qint64, qint64>, std::optional<CollisionFrame>>;

    void collisionDialog()
    {
        const Linkage& linkage = m_context.session().linkage().parts();
        const HardpointTable& design = m_context.hardpoints()->table();
        const std::vector<bool> grounded = groundedPoints(design, m_context.hardpoints()->config());
        const CollisionMeshes meshes = collisionMeshes();
        const CollisionTravel travel = collisionTravel();
        // The car does not change while the dialog is open, so a position is
        // solved the first time something is measured there.
        FrameCache cache;
        const CollisionFrameSource source = [this, &cache](const CollisionPosition& position) {
            return frameAt(position, &cache);
        };
        const auto check = [&](const CollisionSettings& settings) {
            return measure(settings, grounded, meshes, travel, source);
        };
        const auto members = [&](const QString& key) {
            const std::optional<CollisionTarget> target = collisionTargetFromKey(key);
            return target ? collisionMembers(*target, linkage, design, grounded)
                          : std::vector<std::pair<QString, QString>>{};
        };

        CollisionDialog dialog(m_context.project().collision(),
                               choices(linkage, design, grounded, meshes), travel, check, members,
                               m_context.window());
        dialog.exec();

        // The choice is the user's work whether or not anything was found.
        const CollisionSettings settings = dialog.settings();
        if (settings == m_context.project().collision()) return;
        m_context.project().setCollision(settings);
        m_context.markDirty();
    }

    /// How far the analysis panel's sweep parameters take the rack and the
    /// wheels, whichever kind of sweep it is showing.
    CollisionTravel collisionTravel() const
    {
        const SweepSettings sweep = m_context.analysisPanel()->settings();
        return CollisionTravel{ std::abs(sweep.steerTravel), std::abs(sweep.bumpTravel),
                                std::abs(sweep.reboundTravel) };
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
    std::vector<CollisionChoice> choices(const Linkage& linkage, const HardpointTable& design,
                                         const std::vector<bool>& grounded,
                                         const CollisionMeshes& meshes) const
    {
        std::vector<CollisionChoice> out;
        for (const CollisionTarget& target :
             collisionTargets(linkage, design, grounded, m_context.project().wheels().spec, meshes))
            out.push_back(CollisionChoice{ collisionTargetKey(target),
                                           collisionTargetLabel(target, linkage),
                                           !target.isLinkage(),
                                           target.kind == CollisionKind::Member });
        return out;
    }

    CollisionResult measure(const CollisionSettings& settings, const std::vector<bool>& grounded,
                            const CollisionMeshes& meshes, const CollisionTravel& travel,
                            const CollisionFrameSource& source) const
    {
        const std::optional<CollisionTarget> first = collisionTargetFromKey(settings.first);
        const std::optional<CollisionTarget> second = collisionTargetFromKey(settings.second);
        if (!first || !second) {
            CollisionResult result;
            result.problem = tr("Choose two things to measure between.");
            return result;
        }
        return minimumDistance(m_context.session().linkage().parts(), grounded, meshes, settings,
                               *first, *second, travel, source);
    }

    /// The car at @p position, solved the first time it is asked for.
    std::optional<CollisionFrame> frameAt(const CollisionPosition& position, FrameCache* cache) const
    {
        // A thousandth of a millimetre tells two positions apart.
        const QPair<qint64, qint64> key(qRound64(position.rack * 1000.0),
                                        qRound64(position.travel * 1000.0));
        const auto cached = cache->constFind(key);
        if (cached != cache->constEnd()) return *cached;
        return *cache->insert(key, solveFrame(position));
    }

    /// The car at @p position, in its own coordinates -- where the chassis
    /// is -- with every axle at that wheel travel and the rack moving every
    /// axle it steers, and where the wheel models are.
    std::optional<CollisionFrame> solveFrame(const CollisionPosition& position) const
    {
        const Simulation& simulation = m_context.session().simulation().simulation();
        const std::optional<SimulationPose> pose =
            simulation.poseAtWheelTravel(position.travel, position.rack);
        if (!pose) return std::nullopt;
        const HardpointTable& design = m_context.hardpoints()->table();
        const WheelsRef& wheels = m_context.project().wheels();
        return CollisionFrame{
            position, pose->layOver(design),
            wheels.isEmpty() ? std::vector<WheelPlacement>{}
                             : placeWheels(wheels.spec, design, simulation.designWheelAttitudes(),
                                           *pose) };
    }

    AppContext& m_context;
};

} // namespace

SUSPKIN_FEATURE(CollisionFeature)

} // namespace suspkin
