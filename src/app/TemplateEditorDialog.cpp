#include "app/TemplateEditorDialog.h"

#include "app/TemplateDraft.h"
#include "app/TemplateGraphicsTab.h"
#include "app/TemplatePartsTab.h"
#include "app/TemplatePointsTab.h"
#include "app/TemplateSettingsTab.h"

#include <QDialogButtonBox>
#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>

namespace suspkin {

TemplateEditorDialog::TemplateEditorDialog(const LinkageTemplate& templ,
                                           const HardpointTable& table,
                                           const HardpointConfigMap& config,
                                           const MirrorSpec& mirror, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Template Editor"));
    m_draft = new TemplateDraft(templ, table, config, mirror, this);

    m_tabs = new QTabWidget(this);
    m_tabs->addTab(new TemplatePartsTab(m_draft, m_tabs), tr("Parts"));
    m_points = new TemplatePointsTab(m_draft, m_tabs);
    m_tabs->addTab(m_points, tr("Points"));
    m_settings = new TemplateSettingsTab(m_draft, m_tabs);
    m_tabs->addTab(m_settings, tr("Settings"));
    m_tabs->addTab(new TemplateGraphicsTab(m_draft, m_tabs), tr("Graphics"));
    // The far side is shown on both point tables, or on neither.
    connect(m_points, &TemplatePointsTab::farSideShownChanged, m_settings,
            &TemplateSettingsTab::setFarSideShown);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel | QDialogButtonBox::Apply, this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this] {
        emit applyRequested();
        accept();
    });
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(buttons->button(QDialogButtonBox::Apply), &QPushButton::clicked, this,
            &TemplateEditorDialog::applyRequested);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(m_tabs, 1);
    layout->addWidget(buttons);
    resize(1100, 700);
}

int TemplateEditorDialog::currentTab() const { return m_tabs->currentIndex(); }

void TemplateEditorDialog::setCurrentTab(int tab)
{
    if (tab >= 0 && tab < m_tabs->count()) m_tabs->setCurrentIndex(tab);
}

bool TemplateEditorDialog::farSideShown() const { return m_points->farSideShown(); }

void TemplateEditorDialog::setFarSideShown(bool shown) { m_points->setFarSideShown(shown); }

} // namespace suspkin
