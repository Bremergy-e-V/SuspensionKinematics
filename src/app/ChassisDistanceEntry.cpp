#include "app/ChassisDistanceEntry.h"

#include "app/HardpointDelegates.h"
#include "model/Expression.h"

#include <QEvent>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QLocale>
#include <QVBoxLayout>

#include <algorithm>

namespace suspkin {
namespace {

constexpr int kBoxWidth = 236;
/// How far the box sits from the marker, so the hub and the arrows stay visible.
constexpr int kAnchorGap = 18;
/// What the distance is shown to. A distance is measured, not stored, so it is
/// written out readably rather than to every digit.
constexpr int kDecimals = 2;

} // namespace

ChassisDistanceEntry::ChassisDistanceEntry(QWidget* parent) : QFrame(parent)
{
    setAutoFillBackground(true);
    setFixedWidth(kBoxWidth);
    hide();

    m_caption = new QLabel(this);
    m_edit = new QLineEdit(this);
    m_edit->setToolTip(tr("How far the point is to be from the chassis, in mm. Drag an arrow and "
                          "the point slides along the chassis at this distance. Arithmetic works "
                          "on what is there, e.g. type + 5 after the value. Enter finishes; "
                          "Escape leaves the point where the table has it."));
    m_result = new QLabel(this);
    m_result->setWordWrap(true);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 8, 10, 8);
    layout->setSpacing(4);
    layout->addWidget(m_caption);
    layout->addWidget(m_edit);
    layout->addWidget(m_result);

    connect(m_edit, &QLineEdit::textChanged, this, [this] {
        showEvaluation();
        if (const std::optional<double> value = distance()) emit distanceEdited(*value);
    });
    connect(m_edit, &QLineEdit::returnPressed, this, [this] {
        const std::optional<double> value = distance();
        if (!value) return; // the field already says why
        hide();
        emit finished(*value);
        emit closed();
    });

    applyTheme();
}

void ChassisDistanceEntry::openAt(const QPointF& anchor, const QString& pointName, double distance)
{
    m_caption->setText(tr("%1  ·  to chassis").arg(pointName));
    // Blocked: opening on the distance the point is at is not an edit of it.
    {
        const QSignalBlocker quiet(m_edit);
        m_edit->setText(QLocale().toString(distance, 'f', kDecimals));
    }
    m_edit->setCursorPosition(m_edit->text().size());
    showEvaluation();
    placeNear(anchor);
    show();
    raise();
    focusField();
}

void ChassisDistanceEntry::followAnchor(const QPointF& anchor)
{
    if (isOpen()) placeNear(anchor);
}

void ChassisDistanceEntry::focusField()
{
    m_edit->setFocus(Qt::OtherFocusReason);
}

void ChassisDistanceEntry::dismiss()
{
    if (!isVisible()) return;
    hide();
    emit closed();
}

std::optional<double> ChassisDistanceEntry::distance() const
{
    double value = 0.0;
    if (!evaluateExpression(m_edit->text(), &value) || value < 0.0) return std::nullopt;
    return value;
}

void ChassisDistanceEntry::showEvaluation()
{
    double value = 0.0;
    const bool readable = evaluateExpression(m_edit->text(), &value);
    const bool usable = readable && value >= 0.0;
    m_result->setText(usable ? tr("= %1 mm  ·  drag an arrow, Enter to finish")
                                   .arg(QLocale().toString(value, 'f', 3))
                             : readable ? tr("A distance cannot be negative")
                                        : tr("Not a number yet"));
    const QColor colour = usable ? palette().color(QPalette::WindowText)
                                 : issueColor(ConfigIssueLevel::Error, palette());
    m_result->setStyleSheet(QStringLiteral("color: %1; border: none;").arg(colour.name()));
}

void ChassisDistanceEntry::placeNear(const QPointF& anchor)
{
    if (!parentWidget()) return;
    const QSize size = sizeHint();
    const QRect room = parentWidget()->rect();
    const int x = static_cast<int>(anchor.x()) + kAnchorGap;
    const int y = static_cast<int>(anchor.y()) + kAnchorGap;
    move(std::clamp(x, room.left() + 4, std::max(room.left() + 4, room.right() - size.width() - 4)),
         std::clamp(y, room.top() + 4, std::max(room.top() + 4, room.bottom() - size.height() - 4)));
    resize(size);
}

void ChassisDistanceEntry::applyTheme()
{
    // The coordinate field's look: colours out of the palette, a line mixed
    // half way between the background and the text.
    m_theming = true;
    const QPalette pal = palette();
    const QColor window = pal.color(QPalette::Window);
    const QColor text = pal.color(QPalette::WindowText);
    const QColor line = QColor::fromRgbF(0.5 * window.redF() + 0.5 * text.redF(),
                                         0.5 * window.greenF() + 0.5 * text.greenF(),
                                         0.5 * window.blueF() + 0.5 * text.blueF());
    setStyleSheet(QStringLiteral("QFrame { background-color: %1; border: 1px solid %2;"
                                 " border-radius: 6px; }"
                                 "QLineEdit { border: 1px solid %2; border-radius: 4px;"
                                 " padding: 4px 6px; }")
                      .arg(window.name(), line.name()));

    QFont small = font();
    small.setPointSizeF(std::max(7.5, small.pointSizeF() - 1.0));
    m_caption->setFont(small);
    m_result->setFont(small);
    m_caption->setStyleSheet(QStringLiteral("color: %1; border: none;").arg(text.name()));
    showEvaluation();
    m_theming = false;
}

void ChassisDistanceEntry::changeEvent(QEvent* event)
{
    QFrame::changeEvent(event);
    if (m_theming) return;
    if (event->type() == QEvent::PaletteChange || event->type() == QEvent::ApplicationPaletteChange
        || event->type() == QEvent::ThemeChange)
        applyTheme();
}

void ChassisDistanceEntry::keyPressEvent(QKeyEvent* event)
{
    // The line edit does not use Escape, so it arrives here.
    if (event->key() == Qt::Key_Escape) {
        dismiss();
        event->accept();
        return;
    }
    QFrame::keyPressEvent(event);
}

} // namespace suspkin
