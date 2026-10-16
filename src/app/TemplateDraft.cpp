#include "app/TemplateDraft.h"

#include "io/LinkageTemplate.h"
#include "model/HardpointConfig.h"

namespace suspkin {
namespace {

const QString kCornerToken = QStringLiteral("{corner}");

} // namespace

TemplateDraft::TemplateDraft(const LinkageTemplate& templ, const HardpointTable& table,
                             const HardpointConfigMap& config, const MirrorSpec& mirror,
                             QObject* parent)
    : QObject(parent), m_template(templ), m_model(this), m_mirror(mirror)
{
    m_model.setMirrorAxis(mirror.axis);
    m_model.setTable(table);
    m_model.setConfig(config);
    refreshCatalog();
}

void TemplateDraft::refreshCatalog() { m_model.setBodyCatalog(bodyCatalog(m_template)); }

void TemplateDraft::setPart(int index, const PartTemplate& part)
{
    if (index < 0 || index >= int(m_template.parts.size())) return;
    PartTemplate& current = m_template.parts[std::size_t(index)];
    if (current == part) return;

    const QString from = partBodyName(current);
    current = part;
    const QString to = partBodyName(current);
    // Rows naming the body follow it, unless another part still answers to
    // the old name -- an upright drawn as two parts is still one body.
    if (from != to && !bodyCatalog(m_template).contains(from)) {
        HardpointConfigMap config = m_model.config();
        if (renameBody(config, from, to) > 0) m_model.setConfig(config);
        m_renamedBodies.append({ from, to });
    }
    refreshCatalog();
    emit templateChanged();
}

void TemplateDraft::addPart(const QString& label)
{
    PartTemplate part;
    part.id = uniquePartId(m_template, label);
    part.label = QStringLiteral("{corner} {side} ") + label.toLower();
    part.kind = PartKind::Other;
    part.compliance = defaultCompliance(part.kind);
    m_template.parts.push_back(part);
    refreshCatalog();
    emit templateChanged();
}

void TemplateDraft::removePart(int index)
{
    if (index < 0 || index >= int(m_template.parts.size())) return;
    m_template.parts.erase(m_template.parts.begin() + index);
    refreshCatalog();
    emit templateChanged();
}

void TemplateDraft::setChains(const std::vector<std::vector<ChainTemplate>>& chainsByPart)
{
    bool changed = false;
    for (std::size_t i = 0; i < m_template.parts.size() && i < chainsByPart.size(); ++i) {
        if (m_template.parts[i].chains == chainsByPart[i]) continue;
        m_template.parts[i].chains = chainsByPart[i];
        changed = true;
    }
    if (changed) emit templateChanged();
}

HardpointConfigMap TemplateDraft::configWithMirrors() const
{
    HardpointConfigMap config = m_model.config();
    const HardpointTable& table = m_model.table();
    for (int row = 0; row < int(table.points.size()); ++row) {
        if (!m_model.isGhost(row)) continue;
        const Hardpoint& point = table.points[std::size_t(row)];
        const auto source = config.constFind(point.mirrorOf);
        if (source == config.constEnd()) continue;
        config.insert(point.name, *source);
    }
    return config;
}

QString TemplateDraft::patternOf(const QString& name) const
{
    for (const CornerSpec& corner : m_template.corners) {
        if (corner.token.isEmpty() || !name.startsWith(corner.token)) continue;
        return kCornerToken + name.mid(corner.token.size());
    }
    return name;
}

QStringList TemplateDraft::pointPatterns() const
{
    QStringList patterns;
    const auto take = [&patterns](const QString& name) {
        if (!name.isEmpty() && !patterns.contains(name)) patterns << name;
    };
    for (int row = 0; row < m_model.rowCount(); ++row) {
        const Hardpoint& point = m_model.table().points[std::size_t(row)];
        if (m_model.isGhost(row) || point.coord[1] < 0.0) continue;
        take(patternOf(point.name));
    }
    for (const PartTemplate& part : m_template.parts)
        for (const ChainTemplate& chain : part.chains)
            for (const QString& name : chain.points) take(name);
    patterns.sort(Qt::CaseInsensitive);
    return patterns;
}

NearSideProxy::NearSideProxy(HardpointModel* model, QObject* parent)
    : QSortFilterProxyModel(parent), m_model(model)
{
    setSourceModel(model);
    setDynamicSortFilter(true);
}

void NearSideProxy::setFarSideShown(bool shown)
{
    if (m_farSide == shown) return;
    m_farSide = shown;
    invalidateFilter();
}

int NearSideProxy::sourceRowOf(int row) const { return mapToSource(index(row, 0)).row(); }

bool NearSideProxy::filterAcceptsRow(int sourceRow, const QModelIndex&) const
{
    if (m_model->isGhost(sourceRow)) return false;
    // +y is left in ISO 8855, and the near side is the one a template is
    // written for.
    return m_farSide || m_model->table().points[std::size_t(sourceRow)].coord[1] >= 0.0;
}

} // namespace suspkin
