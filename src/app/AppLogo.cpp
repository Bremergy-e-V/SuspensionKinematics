#include "app/AppLogo.h"

#include <QPainter>
#include <QSvgRenderer>

namespace suspkin {

QPixmap appLogoPixmap(int size, qreal devicePixelRatio)
{
    const int pixels = qRound(size * devicePixelRatio);
    QPixmap pixmap(pixels, pixels);
    pixmap.fill(Qt::transparent);
    QSvgRenderer renderer(QStringLiteral(":/logo/suspkin.svg"));
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    renderer.render(&painter);
    painter.end();
    pixmap.setDevicePixelRatio(devicePixelRatio);
    return pixmap;
}

QIcon appLogo()
{
    // Rendered once per size rather than handed to Qt's SVG icon engine,
    // which is a plugin a deployment can leave out.
    QIcon icon;
    for (const int size : { 16, 20, 24, 32, 40, 48, 64, 128, 256 })
        icon.addPixmap(appLogoPixmap(size));
    return icon;
}

} // namespace suspkin
