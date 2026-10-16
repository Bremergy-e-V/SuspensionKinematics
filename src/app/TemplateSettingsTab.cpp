#include "app/TemplateSettingsTab.h"

#include "app/HardpointModel.h"
#include "app/TemplateDraft.h"
#include "app/TemplatePointsTab.h"
#include "model/GraphSolver.h"
#include "model/MechanismGraph.h"
#include "model/SuspensionSolver.h"

#include <QCoreApplication>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QLabel>
#include <QTableView>
#include <QTimer>
#include <QVBoxLayout>

namespace suspkin {
namespace {

struct RoleName {
    QString MechanismTemplate::*field;
    const char* label;
};

/// The roles a point can play, in the order a corner is read: what the solve is
/// driven by and what the measures are read off.
const RoleName kRoles[] = {
    { &MechanismTemplate::lowerFront, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Lower wishbone, front pivot") },
    { &MechanismTemplate::lowerRear, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Lower wishbone, rear pivot") },
    { &MechanismTemplate::lowerOuter, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Lower ball joint") },
    { &MechanismTemplate::upperFront, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Upper wishbone, front pivot") },
    { &MechanismTemplate::upperRear, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Upper wishbone, rear pivot") },
    { &MechanismTemplate::upperOuter, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Upper ball joint") },
    { &MechanismTemplate::tieRodInboard, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Inner tie rod end (steering)") },
    { &MechanismTemplate::tieRodOutboard, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Outer tie rod end") },
    { &MechanismTemplate::wheelCenter, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Wheel centre") },
    { &MechanismTemplate::wheelAxis, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Point on the wheel axis") },
    { &MechanismTemplate::contactPatch, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Contact patch") },
    { &MechanismTemplate::pushrodOuter, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Pushrod, outer end") },
    { &MechanismTemplate::pushrodInner, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Pushrod, inner end") },
    { &MechanismTemplate::rockerPivot, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Rocker pivot") },
    { &MechanismTemplate::rockerAxis, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Rocker axis point") },
    { &MechanismTemplate::damperOutboard, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Damper, suspension end") },
    { &MechanismTemplate::damperInboard, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Damper, body end") },
    { &MechanismTemplate::antiRollDropLinkOuter, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Anti-roll drop link, upper end") },
    { &MechanismTemplate::antiRollArmEnd, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Anti-roll bar arm end") },
    { &MechanismTemplate::antiRollArmRoot, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Anti-roll bar, on its axis") },
    { &MechanismTemplate::antiRollBearing, QT_TRANSLATE_NOOP("TemplateSettingsTab", "Anti-roll bar bearing") },
};

std::vector<CornerSpec> cornersOf(const LinkageTemplate& templ)
{
    std::vector<CornerSpec> corners = templ.corners;
    if (corners.empty()) corners.push_back(CornerSpec{});
    return corners;
}

} // namespace

QString mechanismRoleOf(const TemplateDraft& draft, const QString& name)
{
    const LinkageTemplate& templ = draft.linkageTemplate();
    const HardpointTable& table = draft.model()->table();
    for (const CornerSpec& corner : cornersOf(templ)) {
        for (const bool mirrored : { false, true }) {
            const MechanismTemplate named =
                instantiateMechanism(templ.mechanism, corner.token, mirrored, draft.mirror(), table);
            for (const RoleName& role : kRoles)
                if (named.*(role.field) == name)
                    return QCoreApplication::translate("TemplateSettingsTab", role.label);
            if (named.carried.contains(name))
                return QCoreApplication::translate("TemplateSettingsTab", "Carried by the upright");
        }
    }
    return QString();
}

QStringList describeCornerMobility(const TemplateDraft& draft)
{
    const LinkageTemplate& templ = draft.linkageTemplate();
    const HardpointTable& table = draft.model()->table();
    const HardpointConfigMap config = draft.configWithMirrors();
    QStringList lines;
    for (const CornerSpec& corner : cornersOf(templ)) {
        MechanismTemplate roles = templ.mechanism;
        roles.steeringRack = templ.steeringDeclared() ? corner.steeringRack : roles.tieRodInboard;
        const MechanismTemplate named =
            instantiateMechanism(roles, corner.token, false, draft.mirror(), table);
        if (named.isEmpty() || coverMechanism(named, table).absent) continue;

        const QString axle = corner.label.isEmpty() ? corner.token : corner.label;
        const MechanismGraph graph = configGraph(
            cornerPointNames(templ, corner.token, false, draft.mirror(), table), config, templ, table);
        QString error;
        const std::optional<CornerSolver> solver =
            CornerSolver::bind(named, table, &error, std::nullopt, &graph);
        if (!solver) {
            lines << TemplateSettingsTab::tr("%1: %2").arg(axle, error);
            continue;
        }
        const QString problem = describeMobilityProblem(solver->mobility());
        lines << (problem.isEmpty()
                      ? TemplateSettingsTab::tr("%1: moves as one corner -- the lower wishbone's "
                                                "angle decides everything else.")
                            .arg(axle)
                      : TemplateSettingsTab::tr("%1: %2. Until that is put right it is solved "
                                                "from the template's roles.")
                            .arg(axle, problem));
    }
    if (lines.isEmpty()) lines << TemplateSettingsTab::tr("No axle has enough points to solve yet.");
    return lines;
}

TemplateSettingsTab::TemplateSettingsTab(TemplateDraft* draft, QWidget* parent)
    : QWidget(parent), m_draft(draft)
{
    auto* intro = new QLabel(
        tr("What each point is. <b>To Body/Ground</b> is fixed to the chassis. <b>Solved</b> is "
           "a ball joint: Part 1 and Part 2 meet there and may turn. <b>Solved, rigid</b> holds "
           "them together without turning -- a pushrod on a damper's shaft. <b>Dependent</b> "
           "rides Part 1. <b>Contact Patch</b> is under the wheel on Part 1. The far side takes "
           "the same settings as its mirror."),
        this);
    intro->setWordWrap(true);

    m_proxy = new NearSideProxy(draft->model(), this);
    m_view = new QTableView(this);
    setUpPointView(m_view, m_proxy,
                   { HardpointModel::IndexColumn, HardpointModel::NameColumn,
                     HardpointModel::TypeColumn, HardpointModel::Part1Column,
                     HardpointModel::Part2Column, HardpointModel::BushingColumn });

    m_role = new QLabel(this);
    m_role->setWordWrap(true);
    m_mobility = new QLabel(this);
    m_mobility->setWordWrap(true);
    m_mobility->setTextInteractionFlags(Qt::TextSelectableByMouse);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(intro);
    layout->addWidget(m_view, 1);
    layout->addWidget(m_role);
    layout->addWidget(m_mobility);

    // Checked after a pause: a solve per axle is quick, but not per keystroke.
    m_recheck = new QTimer(this);
    m_recheck->setSingleShot(true);
    m_recheck->setInterval(150);
    connect(m_recheck, &QTimer::timeout, this, &TemplateSettingsTab::checkMobility);
    const auto later = [this] { m_recheck->start(); };
    HardpointModel* model = draft->model();
    connect(model, &QAbstractItemModel::dataChanged, this, later);
    connect(model, &QAbstractItemModel::rowsInserted, this, later);
    connect(model, &QAbstractItemModel::rowsRemoved, this, later);
    connect(model, &QAbstractItemModel::modelReset, this, later);
    connect(draft, &TemplateDraft::templateChanged, this, later);
    connect(m_view->selectionModel(), &QItemSelectionModel::currentRowChanged, this,
            &TemplateSettingsTab::showRoleOfCurrent);

    showRoleOfCurrent();
    checkMobility();
}

void TemplateSettingsTab::setFarSideShown(bool shown) { m_proxy->setFarSideShown(shown); }

void TemplateSettingsTab::showRoleOfCurrent()
{
    const QModelIndex current = m_view->currentIndex();
    if (!current.isValid()) {
        m_role->setText(tr("Select a point to see what it does in the mechanism."));
        return;
    }
    const int row = m_proxy->mapToSource(current).row();
    const QString name = m_draft->model()->table().points[std::size_t(row)].name;
    const QString role = mechanismRoleOf(*m_draft, name);
    m_role->setText(role.isEmpty()
                        ? tr("%1 plays no named role: it is solved from its settings and nothing "
                             "is measured off it.")
                              .arg(name)
                        : tr("%1 is the %2 -- what the solve is driven by or measured off.")
                              .arg(name, role.toLower()));
}

void TemplateSettingsTab::checkMobility()
{
    m_mobility->setText(describeCornerMobility(*m_draft).join(QLatin1Char('\n')));
}

} // namespace suspkin
