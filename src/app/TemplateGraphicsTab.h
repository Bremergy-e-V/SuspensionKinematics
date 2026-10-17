#pragma once

#include "model/Linkage.h"

#include <QWidget>

#include <vector>

class QComboBox;
class QTableWidget;

namespace suspkin {

class TemplateDraft;

/// The template editor's fourth tab: what is drawn. One row per line drawn
/// through points -- which part it belongs to, and so its colour, whether it
/// is a plain line or a pivot axis, whether it closes, and the points in the
/// order it runs through them. Names carry {corner}, so one row draws every
/// axle -- or the one its Axles cell names.
class TemplateGraphicsTab : public QWidget {
    Q_OBJECT

public:
    explicit TemplateGraphicsTab(TemplateDraft* draft, QWidget* parent = nullptr);

private:
    /// One row: the part it belongs to and the chain itself.
    struct Row {
        int part = 0;
        ChainTemplate chain;
    };

    void rebuild();
    void readRows();
    void fillRow(int index);
    /// The Axles cell: every axle, or one of them.
    QComboBox* axlesBox(const QStringList& corners);
    void commit();
    void addRow();
    void removeSelectedRow();
    int pointColumns() const;

    TemplateDraft* m_draft;
    QTableWidget* m_table = nullptr;
    std::vector<Row> m_rows;
    bool m_rebuilding = false;
};

} // namespace suspkin
