#include "app/CollisionDialog.h"

#include "model/Sweep.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGuiApplication>
#include <QLabel>
#include <QVBoxLayout>

#include <algorithm>
#include <utility>

namespace suspkin {
namespace {

/// What a distance below zero is written in: the two overlap.
const QColor kOverlap(220, 50, 47);

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

} // namespace

CollisionDialog::CollisionDialog(const CollisionSettings& settings,
                                 std::vector<CollisionChoice> choices, const QString& travel,
                                 const QString& unit, Check check, QWidget* parent)
    : QDialog(parent),
      m_check(std::move(check)),
      m_travel(travel),
      m_unit(unit),
      m_choices(std::move(choices))
{
    setWindowTitle(tr("Collision Check"));
    resize(520, 0);

    auto* intro = new QLabel(
        tr("The least distance between two things over the travel the analysis panel sweeps. "
           "A member is taken as a round tube around the line drawn for it; two parts that "
           "meet, or meet a third, are not compared round that joint."),
        this);
    intro->setWordWrap(true);

    m_first = new QComboBox(this);
    m_second = new QComboBox(this);
    for (QComboBox* box : { m_first, m_second }) {
        for (const CollisionChoice& choice : m_choices) box->addItem(choice.label, choice.key);
        box->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    }
    // A part against the first rim is what the check is mostly for.
    const auto firstMesh = std::find_if(m_choices.begin(), m_choices.end(),
                                        [](const CollisionChoice& c) { return c.isMesh; });
    const int meshIndex = firstMesh == m_choices.end()
                              ? std::min(1, static_cast<int>(m_choices.size()) - 1)
                              : static_cast<int>(firstMesh - m_choices.begin());
    select(m_first, settings.first, 0);
    select(m_second, settings.second, meshIndex);

    m_radius = millimetreBox(0.0, 100.0,
                             tr("Half the thickness of a member: the tube each line stands for."),
                             this);
    m_joint = millimetreBox(0.0, 200.0,
                            tr("How far round a joint two parts that meet there, or meet a third "
                               "part there, are not compared."),
                            this);
    m_radius->setValue(settings.memberRadius);
    m_joint->setValue(settings.jointClearance);

    auto* form = new QFormLayout;
    form->addRow(tr("Between"), m_first);
    form->addRow(tr("and"), m_second);
    form->addRow(tr("Member radius"), m_radius);
    form->addRow(tr("Joint clearance"), m_joint);

    m_result = new QLabel(this);
    QFont large = m_result->font();
    large.setPointSizeF(large.pointSizeF() * 1.4);
    m_result->setFont(large);
    m_result->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_detail = new QLabel(this);
    m_detail->setWordWrap(true);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::accept);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(intro);
    layout->addLayout(form);
    layout->addSpacing(8);
    layout->addWidget(m_result);
    layout->addWidget(m_detail);
    layout->addStretch(1);
    layout->addWidget(buttons);

    for (QComboBox* box : { m_first, m_second })
        connect(box, &QComboBox::currentIndexChanged, this, &CollisionDialog::run);
    for (QDoubleSpinBox* box : { m_radius, m_joint })
        connect(box, &QDoubleSpinBox::valueChanged, this, &CollisionDialog::run);

    run();
}

CollisionSettings CollisionDialog::settings() const
{
    CollisionSettings settings;
    settings.memberRadius = m_radius->value();
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

void CollisionDialog::run()
{
    if (m_choices.empty()) {
        showResult(CollisionResult{ false, tr("There is nothing to measure: the linkage has no "
                                              "parts and there are no wheels or chassis."),
                                    0.0, 0.0, false, 0.0, 0 });
        return;
    }
    QGuiApplication::setOverrideCursor(Qt::WaitCursor);
    const CollisionResult result = m_check(settings());
    QGuiApplication::restoreOverrideCursor();
    showResult(result);
}

void CollisionDialog::showResult(const CollisionResult& result)
{
    if (!result.valid) {
        m_result->setText(tr("No distance"));
        m_result->setStyleSheet(QString());
        m_detail->setText(result.problem);
        return;
    }

    m_result->setText(tr("Minimum distance: <b>%1 mm</b>").arg(sweepValueText(result.minDistance)));
    m_result->setStyleSheet(result.touches()
                                ? QStringLiteral("color: %1;").arg(kOverlap.name())
                                : QString());

    QStringList detail;
    detail << tr("Closest at %1 %2 %3, over %4 positions.")
                  .arg(m_travel.toLower(), sweepValueText(result.atInput), m_unit)
                  .arg(result.positions);
    if (result.designKnown)
        detail << tr("At design: %1 mm.").arg(sweepValueText(result.designDistance));
    if (result.touches()) detail << tr("Below zero the two overlap.");
    const auto isMesh = [&](const QComboBox* box) {
        return m_choices[static_cast<std::size_t>(box->currentIndex())].isMesh;
    };
    if (isMesh(m_first) && isMesh(m_second))
        detail << tr("Between two models the distance is measured from the vertices of the "
                     "smaller one, so it is as fine as that model's tessellation.");
    m_detail->setText(detail.join(QLatin1Char(' ')));
}

} // namespace suspkin
