#pragma once

#include "model/Hardpoint.h"
#include "model/HardpointConfig.h"
#include "model/HardpointMirror.h"
#include "model/Linkage.h"

#include <QDialog>

class QTabWidget;

namespace suspkin {

class TemplateDraft;
class TemplatePointsTab;
class TemplateSettingsTab;

/// Linkage > Template Editor: the Lotus way of describing a suspension, in four
/// tabs -- the parts it is made of, the points, what each point is, and what is
/// drawn -- over a draft of the project's own template, points and settings.
///
/// Nothing reaches the project until Apply or OK, which hand the draft over
/// through applyRequested(); Cancel leaves the project exactly as it was.
class TemplateEditorDialog : public QDialog {
    Q_OBJECT

public:
    TemplateEditorDialog(const LinkageTemplate& templ, const HardpointTable& table,
                         const HardpointConfigMap& config, const MirrorSpec& mirror,
                         QWidget* parent = nullptr);

    TemplateDraft& draft() { return *m_draft; }

    int currentTab() const;
    void setCurrentTab(int tab);
    bool farSideShown() const;
    void setFarSideShown(bool shown);

signals:
    /// Apply or OK: the draft is to become the project's.
    void applyRequested();

private:
    TemplateDraft* m_draft = nullptr;
    QTabWidget* m_tabs = nullptr;
    TemplatePointsTab* m_points = nullptr;
    TemplateSettingsTab* m_settings = nullptr;
};

} // namespace suspkin
