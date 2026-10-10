#include "app/framework/AppContext.h"
#include "app/framework/CommandRegistry.h"
#include "app/framework/Feature.h"
#include "app/framework/FeatureRegistry.h"
#include "render/ViewportWidget.h"

#include <QCoreApplication>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("ChassisDistanceFeature", text); }

/// Hardpoints > Distance to Chassis: hold the selected point a given distance
/// off the imported chassis while it is dragged -- what C does in the viewport.
///
/// The work is PointEditController's, where the other ways of moving a point
/// in the viewport are; this only puts the gesture on the ribbon, where it can
/// be found. Every move it makes goes through HardpointModel::movePoint(), so
/// it is an ordinary edit of the point, one step of undo each.
class ChassisDistanceFeature : public Feature {
public:
    explicit ChassisDistanceFeature(AppContext& context) : m_context(context) {}

    QString id() const override { return QStringLiteral("chassisDistance"); }

    void registerCommands(CommandRegistry& commands) override
    {
        commands.add({
            .id = QStringLiteral("hardpoints.distanceToChassis"),
            .text = tr("Distance to &Chassis..."),
            .icon = Icon::RulerMeasure,
            .iconText = tr("To Chassis"),
            .statusTip = tr("Type how far the selected point is to be from the chassis, then "
                            "drag its arrows: it slides along the chassis at that distance until "
                            "Enter. C in the viewport does the same."),
            .ribbon = { { QStringLiteral("hardpoints"), tr("Points"), RibbonButton::Small, nullptr,
                          95 } },
            .run = [this] { holdDistanceToChassis(); },
            .enabledWhen = [this] { return canMove(); },
        });
    }

private:
    /// One point, a chassis to measure from, and the points where the table
    /// has them -- not where a simulation has posed them.
    bool canMove() const
    {
        const ViewportWidget* viewport = m_context.viewport();
        return viewport->selectedHardpoints().size() == 1 && viewport->hasMesh()
               && viewport->selectedPointMovable();
    }

    void holdDistanceToChassis()
    {
        // The same field C opens over the viewport, so the ribbon and the key
        // are one gesture: type the distance, drag, Enter.
        m_context.viewport()->requestChassisDistance();
        m_context.viewport()->setFocus(Qt::OtherFocusReason);
    }

    AppContext& m_context;
};

} // namespace

SUSPKIN_FEATURE(ChassisDistanceFeature)

} // namespace suspkin
