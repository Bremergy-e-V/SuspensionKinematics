#include "app/ExpressionSpinBox.h"
#include "app/HardpointModel.h"
#include "app/framework/AppContext.h"
#include "app/framework/CommandRegistry.h"
#include "app/framework/Feature.h"
#include "app/framework/FeatureRegistry.h"
#include "app/session/ProjectSession.h"
#include "geom/MeshQuery.h"
#include "model/ChassisOffset.h"
#include "render/ViewportWidget.h"

#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QVBoxLayout>

#include <optional>

namespace suspkin {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("ChassisDistanceFeature", text); }

/// Asks for the distance one point is to be from the chassis. Opens on the
/// distance it is now, so a correction is typed onto what is there.
class ChassisDistanceDialog : public QDialog {
public:
    ChassisDistanceDialog(const QString& name, double now, QWidget* parent) : QDialog(parent)
    {
        setWindowTitle(tr("Distance to Chassis"));
        auto* layout = new QVBoxLayout(this);
        auto* intro = new QLabel(
            tr("%1 is %2 mm from the chassis. It is moved square to the chassis surface, the "
               "shortest way, until it is the distance below from it.")
                .arg(name)
                .arg(now, 0, 'f', 2),
            this);
        intro->setWordWrap(true);
        layout->addWidget(intro);

        auto* form = new QFormLayout;
        m_distance = new ExpressionSpinBox(this);
        m_distance->setRange(0.0, 10000.0);
        m_distance->setDecimals(2);
        m_distance->setSuffix(tr(" mm"));
        m_distance->setValue(now);
        m_distance->setToolTip(tr("How far the point is to be from the nearest point of the "
                                  "chassis surface. Arithmetic works: \"%1 + 5\".")
                                   .arg(now, 0, 'f', 2));
        form->addRow(tr("Distance"), m_distance);
        layout->addLayout(form);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        layout->addWidget(buttons);

        m_distance->selectAll();
        m_distance->setFocus();
    }

    double distance() const { return m_distance->value(); }

private:
    ExpressionSpinBox* m_distance = nullptr;
};

/// Hardpoints > Distance to Chassis: put the selected point a given distance
/// off the imported chassis.
///
/// The move goes through HardpointModel::movePoint(), so it is an ordinary
/// edit of the point -- the same coordinateChanged(), the same edits file, one
/// step of undo -- and a point of a rigid part takes the part with it.
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
            .statusTip = tr("Move the selected point square to the chassis surface until it is "
                            "a given distance from it."),
            .ribbon = { { QStringLiteral("hardpoints"), tr("Points"), RibbonButton::Small, nullptr,
                          95 } },
            .run = [this] { distanceToChassisDialog(); },
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
               && viewport->pointEditingEnabled();
    }

    void distanceToChassisDialog()
    {
        if (!canMove()) return; // the action is disabled
        const int row = m_context.viewport()->selectedHardpoint();
        const HardpointTable& table = m_context.hardpoints()->table();
        if (row < 0 || row >= static_cast<int>(table.points.size())) return;
        const Hardpoint& point = table.points[static_cast<std::size_t>(row)];
        const QString name = point.name;
        const Vec3 at(point.coord[0], point.coord[1], point.coord[2]);

        const MeshQuery* chassis =
            m_context.session().chassis().query(m_context.viewport()->mesh());
        if (!chassis || chassis->isEmpty()) return;

        ChassisDistanceDialog dialog(name, chassis->distanceTo(at), m_context.window());
        if (dialog.exec() != QDialog::Accepted) return;

        const ChassisOffset offset = offsetFromChassis(*chassis, at, dialog.distance());
        if (!offset.position) {
            QMessageBox::warning(m_context.window(), tr("Distance to Chassis"), offset.error);
            return;
        }
        if (m_context.hardpoints()->movePoint(row, *offset.position))
            m_context.showStatus(tr("%1 is now %2 mm from the chassis")
                                     .arg(name)
                                     .arg(dialog.distance(), 0, 'f', 2),
                                 6000);
    }

    AppContext& m_context;
};

} // namespace

SUSPKIN_FEATURE(ChassisDistanceFeature)

} // namespace suspkin
