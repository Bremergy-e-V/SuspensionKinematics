#pragma once

#include <QIcon>
#include <QPixmap>

namespace suspkin {

/// The application's own logo, drawn from :/logo/suspkin.svg -- a double
/// wishbone corner seen from the front. The exe carries the same drawing as
/// packaging/windows/suspkin.ico, for Explorer and the installer's shortcuts.
QPixmap appLogoPixmap(int size, qreal devicePixelRatio = 1.0);

/// The logo at every size a window title, a taskbar and Alt+Tab ask for.
QIcon appLogo();

} // namespace suspkin
