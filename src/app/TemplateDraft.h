#pragma once

#include "app/HardpointModel.h"
#include "model/HardpointMirror.h"
#include "model/Linkage.h"

#include <QObject>
#include <QPair>
#include <QSortFilterProxyModel>
#include <QString>
#include <QStringList>

#include <vector>

namespace suspkin {

/// What the template editor is changing, before any of it is the project's: a
/// copy of the template and a model of its own over a copy of the points and
/// their settings. Cancel throws it away; Apply hands it over whole.
///
/// The four tabs all work on this one object, so a part made on the first is a
/// body the third can name before anything is written anywhere.
class TemplateDraft : public QObject {
    Q_OBJECT

public:
    TemplateDraft(const LinkageTemplate& templ, const HardpointTable& table,
                  const HardpointConfigMap& config, const MirrorSpec& mirror,
                  QObject* parent = nullptr);

    const LinkageTemplate& linkageTemplate() const { return m_template; }
    HardpointModel* model() { return &m_model; }
    const HardpointModel* model() const { return &m_model; }
    const MirrorSpec& mirror() const { return m_mirror; }

    /// Change part @p index to @p part, keeping every row that named its body
    /// pointed at it when the label changes.
    void setPart(int index, const PartTemplate& part);
    /// A new part with nothing drawn yet: a body points can be joined to.
    void addPart(const QString& label);
    void removePart(int index);
    /// Every chain of every part, replaced. What the graphics tab edits.
    void setChains(const std::vector<std::vector<ChainTemplate>>& chainsByPart);

    /// The bodies relabelling has renamed, oldest first: what an undo history
    /// has to be told about.
    const QList<QPair<QString, QString>>& renamedBodies() const { return m_renamedBodies; }
    /// The settings with every mirrored point given what its source has, so the
    /// far side is described by the near side alone.
    HardpointConfigMap configWithMirrors() const;
    /// Forget what has been renamed: it has been applied.
    void markApplied() { m_renamedBodies.clear(); }

    /// A point name as a template writes it: the corner's token at the front
    /// becomes {corner}, so one name describes every axle.
    QString patternOf(const QString& name) const;
    /// Every name the graphics tab can offer: the near side's points as
    /// patterns, and whatever the template names already.
    QStringList pointPatterns() const;

signals:
    /// Anything about the template changed: its parts, their bodies, what is drawn.
    void templateChanged();

private:
    void refreshCatalog();

    LinkageTemplate m_template;
    HardpointModel m_model;
    MirrorSpec m_mirror;
    QList<QPair<QString, QString>> m_renamedBodies;
};

/// The rows of a hardpoint model a template is written for: the near side.
/// A ghost -- a mirror following its source -- is never shown, and the far
/// side is shown only when asked for.
class NearSideProxy : public QSortFilterProxyModel {
    Q_OBJECT

public:
    explicit NearSideProxy(HardpointModel* model, QObject* parent = nullptr);

    void setFarSideShown(bool shown);
    bool farSideShown() const { return m_farSide; }
    /// The model row behind proxy row @p row.
    int sourceRowOf(int row) const;

protected:
    bool filterAcceptsRow(int sourceRow, const QModelIndex& sourceParent) const override;

private:
    HardpointModel* m_model;
    bool m_farSide = false;
};

} // namespace suspkin
