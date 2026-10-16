#pragma once

#include <QList>
#include <QWidget>

class QAbstractItemModel;
class QCheckBox;
class QTableView;

namespace suspkin {

class NearSideProxy;
class TemplateDraft;

/// The template editor's second tab: the points, by name and X, Y, Z, for the
/// near side only. The far side is the mirror of it and is made by mirroring,
/// so there is nothing of it to type.
class TemplatePointsTab : public QWidget {
    Q_OBJECT

public:
    explicit TemplatePointsTab(TemplateDraft* draft, QWidget* parent = nullptr);

    bool farSideShown() const;
    void setFarSideShown(bool shown);

signals:
    void farSideShownChanged(bool shown);

private:
    void addPoint();
    void removeSelectedPoints();

    TemplateDraft* m_draft;
    NearSideProxy* m_proxy = nullptr;
    QTableView* m_view = nullptr;
    QCheckBox* m_farSide = nullptr;
};

/// The same view setup the hardpoint dock has, for a table over @p model's
/// rows in @p proxy that shows only @p columns: fixed rows, the dock's
/// delegates, one click to edit.
void setUpPointView(QTableView* view, QAbstractItemModel* proxy, const QList<int>& columns);

} // namespace suspkin
