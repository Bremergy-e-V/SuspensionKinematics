#include "app/CollisionDialog.h"

#include "model/Sweep.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGuiApplication>
#include <QHeaderView>
#include <QLabel>
#include <QTableWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <utility>

namespace suspkin {
namespace {

/// What a distance below zero is written in: the two overlap.
const QColor kOverlap(220, 50, 47);
/// How much a tube indents under its part in the dropdowns.
const QString kTubeIndent = QStringLiteral("      ");

enum RadiiColumn { kTubeColumn, kRadiusColumn };
/// Where a row of the radii table keeps its tube's key.
constexpr int kMemberKeyRole = Qt::UserRole;

QDoubleSpinBox* millimetreBox(double minimum, double maximum, const QString& tip, QWidget* parent)
{
    auto* box = new QDoubleSpinBox(parent);
    box->setRange(minimum, maximum);
    box->setDecimals(1);
    box->setSuffix(QObject::tr(" mm"));
    box->setToolTip(tip);
    // A value typed digit by digit would otherwise measure at every keystroke.
    box->setKeyboardTracking(false);
    return box;
}

QString millimetres(double value) { return QObject::tr("%1 mm").arg(sweepValueText(value)); }

QString signedMillimetres(double value)
{
    const QString text = sweepValueText(value);
    return QObject::tr("%1 mm").arg(value > 0.0 ? QLatin1Char('+') + text : text);
}

} // namespace

CollisionDialog::CollisionDialog(const CollisionSettings& settings,
                                 std::vector<CollisionChoice> choices,
                                 const CollisionTravel& travel, Check check, Members members,
                                 QWidget* parent)
    : QDialog(parent),
      m_check(std::move(check)),
      m_members(std::move(members)),
      m_travel(travel),
      m_choices(std::move(choices)),
      m_radii(settings.memberRadii)
{
    setWindowTitle(tr("Collision Check"));
    resize(560, 0);

    auto* intro = new QLabel(
        tr("The least distance between two things, surface to surface, with the rack anywhere "
           "within %1 and the wheels anywhere from %2 rebound to %3 bump (the analysis panel's "
           "Parameters). Every tube is taken as a round bar round the line drawn for it.")
            .arg(QStringLiteral("±") + millimetres(m_travel.rack), millimetres(m_travel.rebound),
                 millimetres(m_travel.bump)),
        this);
    intro->setWordWrap(true);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::accept);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(intro);
    layout->addWidget(buildChoice());
    layout->addSpacing(8);
    layout->addWidget(buildResults());
    layout->addStretch(1);
    layout->addWidget(buttons);

    // A part against the first rim is what the check is mostly for.
    const auto firstMesh = std::find_if(m_choices.begin(), m_choices.end(),
                                        [](const CollisionChoice& c) { return c.isMesh; });
    const int meshIndex = firstMesh == m_choices.end()
                              ? std::min(1, static_cast<int>(m_choices.size()) - 1)
                              : static_cast<int>(firstMesh - m_choices.begin());
    select(m_first, settings.first, 0);
    select(m_second, settings.second, meshIndex);
    m_radius->setValue(settings.memberRadius);
    m_joint->setValue(settings.jointClearance);
    fillRadii();

    for (QComboBox* box : { m_first, m_second })
        connect(box, &QComboBox::currentIndexChanged, this, [this] {
            fillRadii();
            run();
        });
    connect(m_radius, &QDoubleSpinBox::valueChanged, this, [this] {
        fillRadii();
        run();
    });
    connect(m_joint, &QDoubleSpinBox::valueChanged, this, &CollisionDialog::run);
    connect(m_radiiTable, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item) {
        if (m_filling || item->column() != kRadiusColumn) return;
        takeRadius(item->row());
        run();
    });

    run();
}

QWidget* CollisionDialog::buildChoice()
{
    auto* box = new QWidget(this);
    m_first = new QComboBox(box);
    m_second = new QComboBox(box);
    for (QComboBox* combo : { m_first, m_second }) {
        for (const CollisionChoice& choice : m_choices)
            combo->addItem(choice.isTube ? kTubeIndent + choice.label : choice.label, choice.key);
        combo->setSizeAdjustPolicy(QComboBox::AdjustToContents);
        combo->setMaxVisibleItems(25);
    }

    m_radius = millimetreBox(0.0, 100.0,
                             tr("Half the thickness of every tube the table below does not give "
                                "a radius of its own."),
                             box);
    m_joint = millimetreBox(
        0.0, 200.0,
        tr("Two parts bolted together touch at their joint: a tie rod touches the upright at "
           "its ball joint. So that the joint itself does not count as a collision, the two "
           "are not compared within this distance of it -- nor of a part both are bolted to, "
           "such as the upright for a tie rod and a wishbone."),
        box);

    m_radiiTable = new QTableWidget(0, 2, box);
    m_radiiTable->setHorizontalHeaderLabels({ tr("Tube"), tr("Radius [mm]") });
    m_radiiTable->horizontalHeader()->setSectionResizeMode(kTubeColumn, QHeaderView::Stretch);
    m_radiiTable->horizontalHeader()->setSectionResizeMode(kRadiusColumn,
                                                           QHeaderView::ResizeToContents);
    m_radiiTable->verticalHeader()->hide();
    m_radiiTable->setSelectionMode(QAbstractItemView::NoSelection);
    m_radiiTable->setToolTip(tr("Half the thickness of each tube of the two chosen parts. A "
                                "value is kept per tube, by the names of its two ends."));
    m_radiiTable->setMaximumHeight(160);

    auto* form = new QFormLayout(box);
    form->setContentsMargins(0, 0, 0, 0);
    form->addRow(tr("Between"), m_first);
    form->addRow(tr("and"), m_second);
    form->addRow(tr("Tube radius"), m_radius);
    form->addRow(tr("Joint clearance"), m_joint);
    form->addRow(m_radiiTable);
    return box;
}

QWidget* CollisionDialog::buildResults()
{
    m_headline = new QLabel(this);
    QFont large = m_headline->font();
    large.setPointSizeF(large.pointSizeF() * 1.4);
    m_headline->setFont(large);
    m_headline->setWordWrap(true);

    m_results = new QWidget(this);
    const auto value = [this] {
        auto* label = new QLabel(m_results);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
        return label;
    };
    m_rideHeight = value();
    m_rideHeightAt = value();
    m_overTravel = value();
    m_overTravelAt = value();
    m_design = value();
    auto* grid = new QGridLayout(m_results);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->addWidget(new QLabel(tr("At ride height, any steering:"), m_results), 0, 0);
    grid->addWidget(m_rideHeight, 0, 1);
    grid->addWidget(m_rideHeightAt, 0, 2);
    grid->addWidget(new QLabel(tr("Any steering, any wheel travel:"), m_results), 1, 0);
    grid->addWidget(m_overTravel, 1, 1);
    grid->addWidget(m_overTravelAt, 1, 2);
    grid->addWidget(new QLabel(tr("Straight ahead at ride height:"), m_results), 2, 0);
    grid->addWidget(m_design, 2, 1);
    grid->setColumnStretch(2, 1);

    m_where = new QLabel(this);
    m_where->setWordWrap(true);
    m_note = new QLabel(this);
    m_note->setWordWrap(true);

    auto* box = new QWidget(this);
    auto* layout = new QVBoxLayout(box);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_headline);
    layout->addWidget(m_results);
    layout->addWidget(m_where);
    layout->addWidget(m_note);
    return box;
}

CollisionSettings CollisionDialog::settings() const
{
    CollisionSettings settings;
    settings.memberRadius = m_radius->value();
    settings.memberRadii = m_radii;
    settings.jointClearance = m_joint->value();
    settings.first = m_first->currentData().toString();
    settings.second = m_second->currentData().toString();
    return settings;
}

void CollisionDialog::select(QComboBox* box, const QString& key, int fallback)
{
    const int saved = key.isEmpty() ? -1 : box->findData(key);
    box->setCurrentIndex(saved >= 0 ? saved : fallback);
}

void CollisionDialog::fillRadii()
{
    const CollisionSettings current = settings();
    std::vector<std::pair<QString, QString>> tubes;
    for (const QComboBox* box : { m_first, m_second }) {
        const auto members = m_members(box->currentData().toString());
        tubes.insert(tubes.end(), members.begin(), members.end());
    }

    m_filling = true;
    m_radiiTable->setRowCount(static_cast<int>(tubes.size()));
    for (int row = 0; row < static_cast<int>(tubes.size()); ++row) {
        const auto& [from, to] = tubes[static_cast<std::size_t>(row)];
        auto* name = new QTableWidgetItem(from + QStringLiteral(" - ") + to);
        name->setFlags(Qt::ItemIsEnabled);
        name->setData(kMemberKeyRole, collisionMemberKey(from, to));
        auto* radius = new QTableWidgetItem;
        radius->setData(Qt::EditRole, current.radiusOf(from, to));
        m_radiiTable->setItem(row, kTubeColumn, name);
        m_radiiTable->setItem(row, kRadiusColumn, radius);
    }
    m_filling = false;
    m_radiiTable->setVisible(!tubes.empty());
}

void CollisionDialog::takeRadius(int row)
{
    const QString key = m_radiiTable->item(row, kTubeColumn)->data(kMemberKeyRole).toString();
    const double radius = std::max(0.0, m_radiiTable->item(row, kRadiusColumn)->data(Qt::EditRole).toDouble());
    // A tube set back to the common radius follows it again.
    if (std::abs(radius - m_radius->value()) < 1e-9) m_radii.remove(key);
    else m_radii.insert(key, radius);
}

void CollisionDialog::run()
{
    if (m_choices.empty()) {
        showProblem(tr("There is nothing to measure: the linkage has no parts and there are no "
                       "wheels or chassis."));
        return;
    }
    QGuiApplication::setOverrideCursor(Qt::WaitCursor);
    const CollisionResult result = m_check(settings());
    QGuiApplication::restoreOverrideCursor();
    showResult(result);
}

void CollisionDialog::showProblem(const QString& problem)
{
    m_headline->setText(tr("No distance"));
    m_headline->setStyleSheet(QString());
    m_results->hide();
    m_where->setText(problem);
    m_note->clear();
}

void CollisionDialog::showResult(const CollisionResult& result)
{
    if (!result.valid) {
        showProblem(result.problem);
        return;
    }
    const CollisionExtreme& worst = result.overTravel;
    m_headline->setText(worst.touches()
                            ? tr("<b>Collision</b> at %1").arg(positionText(worst.at))
                            : tr("Minimum distance: <b>%1</b>").arg(millimetres(worst.distance)));
    m_headline->setStyleSheet(worst.touches() ? QStringLiteral("color: %1;").arg(kOverlap.name())
                                              : QString());

    m_results->show();
    showDistance(m_rideHeight, result.rideHeight);
    m_rideHeightAt->setText(result.rideHeight.valid
                                ? tr("at rack %1").arg(signedMillimetres(result.rideHeight.at.rack))
                                : QString());
    showDistance(m_overTravel, worst);
    m_overTravelAt->setText(tr("at %1").arg(positionText(worst.at)));
    showDistance(m_design, result.design);
    m_where->setText(whereText(worst));

    QStringList note;
    if (worst.touches()) note << tr("Below zero the two overlap by that much.");
    if (bothMeshes())
        note << tr("Between two models the distance is measured from the vertices of the "
                   "smaller one, so it is as fine as that model's tessellation.");
    note << tr("%1 positions solved.").arg(result.positions);
    m_note->setText(note.join(QLatin1Char(' ')));
}

void CollisionDialog::showDistance(QLabel* label, const CollisionExtreme& extreme) const
{
    if (!extreme.valid) {
        label->setText(QStringLiteral("--"));
        label->setStyleSheet(QString());
        return;
    }
    label->setText(QStringLiteral("<b>%1</b>").arg(millimetres(extreme.distance)));
    label->setStyleSheet(extreme.touches() ? QStringLiteral("color: %1;").arg(kOverlap.name())
                                           : QString());
}

QString CollisionDialog::positionText(const CollisionPosition& position) const
{
    const QString travel = position.travel == 0.0
                               ? tr("ride height")
                               : tr("wheel travel %1 (%2)")
                                     .arg(signedMillimetres(position.travel),
                                          position.travel > 0.0 ? tr("bump") : tr("rebound"));
    return tr("rack %1, %2").arg(signedMillimetres(position.rack), travel);
}

QString CollisionDialog::whereText(const CollisionExtreme& extreme) const
{
    const auto side = [](const QString& member, double along, const QString& fallback) {
        if (member.isEmpty()) return fallback;
        const QString from = member.section(QStringLiteral(" - "), 0, 0);
        return tr("tube %1, %2 from %3").arg(member, millimetres(along), from);
    };
    const QString first = side(extreme.firstMember, extreme.firstAlong, m_first->currentText().trimmed());
    const QString second =
        side(extreme.secondMember, extreme.secondAlong, m_second->currentText().trimmed());
    return tr("Closest between %1 and %2.").arg(first, second);
}

bool CollisionDialog::bothMeshes() const
{
    const auto isMesh = [&](const QComboBox* box) {
        const int index = box->currentIndex();
        return index >= 0 && m_choices[static_cast<std::size_t>(index)].isMesh;
    };
    return isMesh(m_first) && isMesh(m_second);
}

} // namespace suspkin
