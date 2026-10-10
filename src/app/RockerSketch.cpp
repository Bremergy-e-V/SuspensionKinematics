#include "app/RockerSketch.h"

#include <QLocale>
#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>

namespace suspkin {
namespace {

constexpr double kDegToRad = 0.017453292519943295769236907684886;
constexpr int kMargin = 42;
/// Room for the caption above the drawing.
constexpr int kCaptionHeight = 20;

/// A direction in the rocker's plane, seen along its axis from in front of
/// the car, with y up.
struct V2 {
    double x = 0.0;
    double y = 0.0;
    V2 operator+(const V2& o) const { return { x + o.x, y + o.y }; }
    V2 operator-(const V2& o) const { return { x - o.x, y - o.y }; }
    V2 operator*(double s) const { return { x * s, y * s }; }
};

/// The rocker's points in its own plane, laid out the way layOutRocker() lays
/// them out in space: the same arms, the same angles, positive inboard.
struct Flat {
    double inboard = 1.0; ///< which way positive turns on screen: +1 counter-clockwise
    V2 up{ 0.0, 1.0 };
    V2 toPushrod, toDamper, alongDamper, toAntiRoll;
    V2 pushrod, eye, mount, antiRoll;
};

V2 turn(const V2& v, double degrees, double inboard)
{
    const double a = inboard * degrees * kDegToRad;
    return { v.x * std::cos(a) - v.y * std::sin(a), v.x * std::sin(a) + v.y * std::cos(a) };
}

Flat flatten(const RockerDesign& d)
{
    Flat f;
    // From in front, the left side's inboard is to the viewer's left: a
    // counter-clockwise turn. The right side's is the other way.
    f.inboard = d.pivot.y >= 0.0 ? 1.0 : -1.0;
    f.toPushrod = turn(f.up, d.pushrodArmAngle, f.inboard);
    f.toDamper = turn(f.toPushrod, d.damperArmAngle, f.inboard);
    f.alongDamper = turn(f.toDamper * -1.0, d.damperAngle, f.inboard);
    f.toAntiRoll = turn(f.toPushrod, d.antiRollArmAngle, f.inboard);
    f.pushrod = f.toPushrod * d.pushrodArm;
    f.eye = f.toDamper * d.damperArm;
    f.mount = f.eye + f.alongDamper * d.damperLength;
    f.antiRoll = f.toAntiRoll * d.antiRollArm;
    return f;
}

/// Plane coordinates onto the widget: one scale for both axes, y flipped.
struct Mapping {
    QPointF origin;
    double scale = 1.0;
    QPointF operator()(const V2& p) const { return origin + QPointF(p.x * scale, -p.y * scale); }
};

/// An arc about @p centre from direction @p from through @p sweep degrees,
/// positive counter-clockwise, @p radius pixels out -- the angle as the dialog
/// has it, not the short way round. Where its middle is goes in @p middle, for
/// the label.
QPainterPath arc(const Mapping& map, const V2& centre, const V2& from, double sweep, double radius,
                 QPointF* middle)
{
    QPainterPath path;
    constexpr int kSteps = 32;
    for (int i = 0; i <= kSteps; ++i) {
        const V2 dir = turn(from, sweep * i / kSteps, 1.0);
        const QPointF p = map(centre) + QPointF(dir.x * radius, -dir.y * radius);
        if (i == 0) path.moveTo(p);
        else path.lineTo(p);
    }
    const V2 half = turn(from, sweep / 2.0, 1.0);
    *middle = map(centre) + QPointF(half.x * (radius + 16.0), -half.y * (radius + 12.0));
    return path;
}

QString degrees(double value) { return QLocale().toString(value, 'f', 0) + QStringLiteral("°"); }

} // namespace

RockerSketch::RockerSketch(QWidget* parent) : QWidget(parent)
{
    setMinimumHeight(220);
    setToolTip(tr("The rocker as the numbers make it, seen along its axis from in front of the "
                  "car. Arms are L1 to the pushrod, L2 to the damper's eye and L3 to the drop "
                  "link; the angles are marked where they are measured, positive turning "
                  "inboard."));
}

void RockerSketch::setDesign(const RockerDesign& design, const QString& corner)
{
    m_design = design;
    m_corner = corner;
    update();
}

QSize RockerSketch::sizeHint() const
{
    return { 360, 260 };
}

void RockerSketch::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QPalette pal = palette();
    const QColor text = pal.color(QPalette::WindowText);
    const QColor arm = pal.color(QPalette::Highlight);
    QColor faint = text;
    faint.setAlphaF(0.45f);
    QColor body = arm;
    body.setAlphaF(0.18f);

    const RockerDesign& d = m_design;
    const Flat f = flatten(d);

    // The caption: which rocker, from where, and which way is inboard.
    QFont small = font();
    small.setPointSizeF(std::max(7.5, small.pointSizeF() - 1.0));
    painter.setFont(small);
    painter.setPen(text);
    const QString inboard = f.inboard > 0.0 ? tr("inboard ←") : tr("inboard →");
    painter.drawText(QRectF(0, 0, width(), kCaptionHeight), Qt::AlignLeft | Qt::AlignVCenter,
                     tr("%1, seen along the axis from the front  ·  %2").arg(m_corner, inboard));

    // Fit everything drawn into the room left, the same scale both ways.
    std::vector<V2> extent{ V2{}, f.pushrod, f.eye, f.mount, f.up * (0.8 * d.pushrodArm) };
    if (d.antiRollArmUsed) extent.push_back(f.antiRoll);
    double minX = 0, maxX = 0, minY = 0, maxY = 0;
    for (const V2& p : extent) {
        minX = std::min(minX, p.x);
        maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
    }
    const QRectF room(kMargin, kCaptionHeight + kMargin, width() - 2 * kMargin,
                      height() - kCaptionHeight - 2 * kMargin);
    const double spanX = std::max(1.0, maxX - minX);
    const double spanY = std::max(1.0, maxY - minY);
    Mapping map;
    map.scale = std::min(room.width() / spanX, room.height() / spanY);
    map.origin = room.center() - QPointF((minX + maxX) / 2.0 * map.scale,
                                         -(minY + maxY) / 2.0 * map.scale);

    const auto label = [&](const QPointF& at, const QString& what) {
        const QRectF box(at.x() - 40, at.y() - 9, 80, 18);
        painter.setPen(text);
        painter.drawText(box, Qt::AlignCenter, what);
    };
    // A length's name sits beside its line, toward the far end, where the
    // angles marked at the near end are not.
    const auto besideLine = [&](const V2& a, const V2& b, double off) {
        const V2 along = b - a;
        const double length = std::hypot(along.x, along.y);
        const V2 side = length > 0 ? V2{ -along.y / length, along.x / length } : V2{};
        return map(a + along * 0.72) + QPointF(side.x * off, -side.y * off);
    };
    // An end's name sits past the end, along the line it ends.
    const auto pastEnd = [&](const V2& from, const V2& end) {
        const V2 along = end - from;
        const double length = std::hypot(along.x, along.y);
        const V2 unit = length > 0 ? along * (1.0 / length) : V2{ 0.0, 1.0 };
        return map(end) + QPointF(unit.x * 30.0, -unit.y * 18.0);
    };

    // The plate, then the arms on it.
    QPolygonF plate{ map(V2{}), map(f.pushrod), map(f.eye) };
    if (d.antiRollArmUsed) plate.insert(1, map(f.antiRoll));
    painter.setPen(Qt::NoPen);
    painter.setBrush(body);
    painter.drawPolygon(plate);
    painter.setBrush(Qt::NoBrush);

    // Up: what L1's angle is measured from.
    painter.setPen(QPen(faint, 1.0, Qt::DashLine));
    painter.drawLine(map(V2{}), map(f.up * (0.8 * d.pushrodArm)));

    painter.setPen(QPen(arm, 3.0, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(map(V2{}), map(f.pushrod));
    painter.drawLine(map(V2{}), map(f.eye));
    if (d.antiRollArmUsed) {
        painter.setPen(QPen(arm, 2.0, Qt::DashLine, Qt::RoundCap));
        painter.drawLine(map(V2{}), map(f.antiRoll));
    }

    // The damper: a rod from the eye, a body toward the chassis end.
    painter.setPen(QPen(text, 2.0, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(map(f.eye), map(f.mount));
    painter.setPen(QPen(text, 7.0, Qt::SolidLine, Qt::FlatCap));
    painter.drawLine(map(f.eye + f.alongDamper * (0.55 * d.damperLength)),
                     map(f.eye + f.alongDamper * (0.95 * d.damperLength)));

    // The joints, and the pivot with its axis coming out of the page.
    painter.setPen(QPen(text, 1.5));
    painter.setBrush(pal.color(QPalette::Window));
    for (const V2& joint : { f.pushrod, f.eye, f.mount })
        painter.drawEllipse(map(joint), 4.0, 4.0);
    if (d.antiRollArmUsed) painter.drawEllipse(map(f.antiRoll), 4.0, 4.0);
    painter.drawEllipse(map(V2{}), 7.0, 7.0);
    painter.setBrush(text);
    painter.drawEllipse(map(V2{}), 2.0, 2.0);
    painter.setBrush(Qt::NoBrush);

    // The lengths.
    label(besideLine(V2{}, f.pushrod, 14), tr("L1"));
    label(besideLine(V2{}, f.eye, -14), tr("L2"));
    label(besideLine(f.eye, f.mount, 16), tr("Ld"));
    if (d.antiRollArmUsed) label(besideLine(V2{}, f.antiRoll, 14), tr("L3"));
    label(pastEnd(V2{}, f.pushrod), tr("pushrod"));
    label(pastEnd(f.eye, f.mount), tr("chassis"));
    if (d.antiRollArmUsed) label(pastEnd(V2{}, f.antiRoll), tr("drop link"));

    // The angles, each where the dialog measures it.
    painter.setPen(QPen(faint, 1.2));
    QPointF at;
    painter.drawPath(arc(map, V2{}, f.up, f.inboard * d.pushrodArmAngle, 26, &at));
    label(at, degrees(d.pushrodArmAngle));
    painter.setPen(QPen(faint, 1.2));
    painter.drawPath(arc(map, V2{}, f.toPushrod, f.inboard * d.damperArmAngle, 40, &at));
    label(at, degrees(d.damperArmAngle));
    if (d.antiRollArmUsed) {
        painter.setPen(QPen(faint, 1.2));
        painter.drawPath(arc(map, V2{}, f.toPushrod, f.inboard * d.antiRollArmAngle, 54, &at));
        label(at, degrees(d.antiRollArmAngle));
    }
    painter.setPen(QPen(faint, 1.2));
    painter.drawPath(arc(map, f.eye, f.toDamper * -1.0, f.inboard * d.damperAngle, 22, &at));
    label(at, tr("β %1").arg(degrees(d.damperAngle)));
}

} // namespace suspkin
