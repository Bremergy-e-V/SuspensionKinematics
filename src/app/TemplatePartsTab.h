#pragma once

#include <QWidget>

class QTableWidget;

namespace suspkin {

class TemplateDraft;

/// The template editor's first tab: what the car is made of. One row per part
/// -- its name, what kind of thing it is, and whether it keeps its shape --
/// and a part added here is a body the settings can name straight away.
class TemplatePartsTab : public QWidget {
    Q_OBJECT

public:
    explicit TemplatePartsTab(TemplateDraft* draft, QWidget* parent = nullptr);

private:
    void rebuild();
    void addRow(int index);
    void commitLabel(int row);
    void commitKind(int row, int kindIndex);
    void commitCompliance(int row, int complianceIndex);
    void addPart();
    void removeSelectedPart();

    TemplateDraft* m_draft;
    QTableWidget* m_table = nullptr;
    bool m_rebuilding = false;
};

} // namespace suspkin
