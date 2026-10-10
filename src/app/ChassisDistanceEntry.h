#pragma once

#include <QFrame>
#include <QPointF>
#include <QString>

#include <optional>

class QLabel;
class QLineEdit;

namespace suspkin {

/// The field C opens over the viewport: how far the selected point is to be
/// from the chassis, held while the point is dragged.
///
/// Unlike the X, Y and Z field it does not close when the viewport is clicked:
/// the whole point of it is that the arrows are dragged while it is open, and
/// the point slides along the chassis at the distance it holds. Enter finishes,
/// Escape leaves. It opens on the distance the point is at now, so a point
/// dragged without anything typed keeps the distance it already had.
///
/// A view, like the coordinate field: it reports the distance, and the
/// controller moves the point.
class ChassisDistanceEntry : public QFrame {
    Q_OBJECT

public:
    explicit ChassisDistanceEntry(QWidget* parent);

    /// Open for @p pointName, holding @p distance, beside @p anchor -- the
    /// marker's own position in the parent's coordinates.
    void openAt(const QPointF& anchor, const QString& pointName, double distance);
    /// Put the box beside where the marker is now, after it has moved.
    void followAnchor(const QPointF& anchor);
    /// Give the keyboard back to the field, so Enter after a drag lands here.
    void focusField();
    /// Close; reports closed() and nothing else.
    void dismiss();

    bool isOpen() const { return isVisible(); }
    /// What the field reads as, or nothing while it is not a number.
    std::optional<double> distance() const;

signals:
    /// The field reads as a new distance, keystroke by keystroke.
    void distanceEdited(double distance);
    /// Enter was pressed on a field that reads as a distance.
    void finished(double distance);
    /// The field closed, however it closed.
    void closed();

protected:
    void changeEvent(QEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    void applyTheme();
    /// Show what the field comes to, or why it comes to nothing.
    void showEvaluation();
    void placeNear(const QPointF& anchor);

    bool m_theming = false;
    QLabel* m_caption = nullptr;
    QLineEdit* m_edit = nullptr;
    QLabel* m_result = nullptr;
};

} // namespace suspkin
