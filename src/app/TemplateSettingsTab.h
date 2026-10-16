#pragma once

#include <QWidget>

class QLabel;
class QTableView;
class QTimer;

namespace suspkin {

class NearSideProxy;
class TemplateDraft;

/// The template editor's third tab: what each point is. Its type says what
/// the joint there allows -- fixed to the chassis, a ball joint, a rigid joint,
/// carried, or the contact patch -- and Part 1 and Part 2 say which bodies meet
/// there. This is what the corner is solved from, so under the table it says,
/// per axle, whether what is described moves as one corner.
class TemplateSettingsTab : public QWidget {
    Q_OBJECT

public:
    explicit TemplateSettingsTab(TemplateDraft* draft, QWidget* parent = nullptr);

    void setFarSideShown(bool shown);

private:
    void showRoleOfCurrent();
    void checkMobility();

    TemplateDraft* m_draft;
    NearSideProxy* m_proxy = nullptr;
    QTableView* m_view = nullptr;
    QLabel* m_role = nullptr;
    QLabel* m_mobility = nullptr;
    QTimer* m_recheck = nullptr;
};

/// What the point called @p name does in @p draft's mechanism -- "Lower ball
/// joint", "Wheel centre" -- or empty when the mechanism block gives it no role.
QString mechanismRoleOf(const TemplateDraft& draft, const QString& name);

/// One line per axle of @p draft: whether its near side, as the settings
/// describe it, moves as one corner, and what is wrong when it does not.
QStringList describeCornerMobility(const TemplateDraft& draft);

} // namespace suspkin
