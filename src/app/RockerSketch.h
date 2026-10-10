#pragma once

#include "model/RockerDesign.h"

#include <QWidget>

namespace suspkin {

/// The rocker drawn as the dialog's numbers make it, seen along its own axis:
/// L1 to the pushrod, L2 to the damper's eye, the damper Ld, and L3 when there
/// is one, with every angle the dialog asks for marked where it is measured.
///
/// The 2025 car's chalk-board sketch, but live: what a number means is shown
/// by what it does to the drawing. Colours come from the palette, so it reads
/// on a light desktop and a dark one alike.
class RockerSketch : public QWidget {
    Q_OBJECT

public:
    explicit RockerSketch(QWidget* parent = nullptr);

    /// Draw @p design. @p corner is the column's name, for the caption.
    void setDesign(const RockerDesign& design, const QString& corner);

    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    RockerDesign m_design;
    QString m_corner;
};

} // namespace suspkin
