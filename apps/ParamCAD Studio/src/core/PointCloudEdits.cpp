#include "core/PointCloudSelection.h"

#include "io/PlyWriter.h"
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkStaticPointLocator.h>

#include <algorithm>
#include <set>
#include <unordered_set>

bool PointCloudSelection::active(vtkIdType id) const
{
    return id >= 0 && static_cast<size_t>(id) < m_deleted.size() && !m_deleted[static_cast<size_t>(id)];
}
int PointCloudSelection::activePointCount() const { return static_cast<int>(m_activeIds.size()); }

const PointCloudRegion* PointCloudSelection::region(int id) const
{
    const auto found = std::find_if(m_regions.begin(), m_regions.end(), [id](const auto& r) { return r.id == id; });
    return found == m_regions.end() ? nullptr : &*found;
}
const PointCloudRelation* PointCloudSelection::relation(int id) const
{
    const auto found = std::find_if(m_relations.begin(), m_relations.end(), [id](const auto& r) { return r.id == id; });
    return found == m_relations.end() ? nullptr : &*found;
}

void PointCloudSelection::checkpoint(bool recordUndo, Edit edit)
{
    if (recordUndo) {
        if (m_undo.size() >= 20) m_undo.erase(m_undo.begin());
        m_undo.push_back({m_memberships, m_deleted, m_regions, m_relations, m_suppressed,
                          m_selected, m_nextRegionId, m_nextRelationId, m_surfaceRevision});
    }
    ++m_revision;
    if (edit == Edit::Surface) ++m_surfaceRevision;
}

bool PointCloudSelection::undo()
{
    if (m_undo.empty()) return false;
    auto previous = std::move(m_undo.back());
    m_undo.pop_back();
    m_memberships = std::move(previous.memberships);
    m_deleted = std::move(previous.deleted);
    m_regions = std::move(previous.regions);
    m_relations = std::move(previous.relations);
    m_suppressed = std::move(previous.suppressed);
    m_selected = std::move(previous.selected);
    m_nextRegionId = previous.nextRegionId;
    m_nextRelationId = previous.nextRelationId;
    ++m_revision;
    if (previous.surfaceRevision != m_surfaceRevision) ++m_surfaceRevision;
    rebuildLocator();
    return true;
}

void PointCloudSelection::rebuildLocator()
{
    vtkNew<vtkPoints> points;
    m_activeIds.clear();
    for (size_t i = 0; i < m_deleted.size(); ++i) {
        if (!m_deleted[i]) {
            points->InsertNextPoint(m_sourceCloud.positions[i].data());
            m_activeIds.push_back(static_cast<vtkIdType>(i));
        }
    }
    m_activeData->SetPoints(points);
    m_locator->Initialize();
    m_locator->SetDataSet(m_activeData);
    if (!m_activeIds.empty()) m_locator->BuildLocator();
}

void PointCloudSelection::invalidateRegion(int id)
{
    if (id <= 0) return;
    for (auto& r : m_regions) if (r.id == id) {r.supportValidation={};if(!r.parameters.isEmpty())r.fitStale=true;}
    // Inferred relationships no longer have valid evidence after membership
    // edits. User-authored relationships are kept as explicit annotations.
    m_relations.erase(std::remove_if(m_relations.begin(), m_relations.end(),
        [id](const auto& r) { return r.automatic && (r.first == id || r.second == id); }), m_relations.end());
}

bool PointCloudSelection::editRegion(int id, const QString& name, const QString& geometry)
{
    const auto* original = region(id);
    if (!original || name.trimmed().isEmpty() || !validGeometry(geometry)) return false;
    checkpoint(true, original->geometry == geometry ? Edit::Annotation : Edit::Surface);
    for (auto& r : m_regions) if (r.id == id) {
        if (r.geometry != geometry) { r.parameters = {}; r.source = "manual"; r.fitStale = false; invalidateRegion(id); }
        r.geometry = geometry;
        r.templateLocked = true;
        r.source = "manual";
        r.name = name.trimmed();
    }
    return true;
}

std::vector<vtkIdType> PointCloudSelection::regionPoints(int id) const
{
    std::vector<vtkIdType> points;
    for (size_t i = 0; i < m_memberships.size(); ++i) {
        if (!m_deleted[i] && m_memberships[i] == id) points.push_back(static_cast<vtkIdType>(i));
    }
    return points;
}
void PointCloudSelection::replaceRegionPoints(int id)
{
    if (!region(id) || m_selected.empty()) return;
    checkpoint();
    invalidateRegion(id);
    for (int& membership : m_memberships) if (membership == id) membership = 0;
    for (vtkIdType point : m_selected) {
        invalidateRegion(m_memberships[static_cast<size_t>(point)]);
        m_memberships[static_cast<size_t>(point)] = id;
    }
}
void PointCloudSelection::updateRegionFit(int id, const PointCloudRegion& fitted, bool recordUndo)
{
    const auto* original = region(id);
    if (!original || original->geometry != fitted.geometry || fitted.parameters.isEmpty()) return;
    checkpoint(recordUndo);
    invalidateRegion(id);
    for (auto& r : m_regions) if (r.id == id) {
        r.parameters = fitted.parameters; r.residual = fitted.residual; r.supportValidation=fitted.supportValidation;
        r.confidence = fitted.confidence; r.source = r.templateLocked ? "manual" : "detected"; r.fitStale = false;
    }
}
void PointCloudSelection::selectUnassigned() { select(regionPoints(0)); }

void PointCloudSelection::lockTemplate(int id)
{
    const auto* original = region(id);
    if (!original || original->templateLocked || original->geometry=="part") return;
    checkpoint(true, Edit::Annotation);
    for (auto& r : m_regions) if (r.id == id) { r.templateLocked = true; r.source = "manual"; }
}

void PointCloudSelection::selectRelation(int id)
{
    const auto* r = relation(id);
    if (r) selectRegions({r->first, r->second});
}

void PointCloudSelection::selectRegions(const std::vector<int>& regionIds)
{
    const std::unordered_set<int> regions(regionIds.begin(), regionIds.end());
    m_selected.clear();
    for (const vtkIdType id : m_activeIds)
        if (regions.count(m_memberships[static_cast<std::size_t>(id)])) m_selected.push_back(id);
}

void PointCloudSelection::deletePoints(const std::vector<vtkIdType>& ids)
{
    if (ids.empty()) return;
    checkpoint();
    std::set<int> affected;
    for (vtkIdType id : ids) if (active(id)) {
        const size_t index = static_cast<size_t>(id);
        affected.insert(m_memberships[index]);
        m_memberships[index] = 0;
        m_deleted[index] = 1;
    }
    for (int id : affected) invalidateRegion(id);
    m_relations.erase(std::remove_if(m_relations.begin(), m_relations.end(), [this](const auto& r) {
        return regionPointCount(r.first) == 0 || regionPointCount(r.second) == 0;
    }), m_relations.end());
    const auto remaining = m_selected;
    select(remaining);
    rebuildLocator();
}
void PointCloudSelection::deleteSelectedPoints() { const auto ids = m_selected; deletePoints(ids); }
void PointCloudSelection::deleteRegionPoints(int id) { deletePoints(regionPoints(id)); }

void PointCloudSelection::addSurfaces(const std::vector<DetectedPointCloudSurface>& surfaces, bool recordUndo)
{
    if (surfaces.empty()) return;
    checkpoint(recordUndo);
    for (const auto& detected : surfaces) {
        if (!validGeometry(detected.surface.geometry)) continue;
        std::vector<vtkIdType> ids;
        for (vtkIdType id : detected.points) {
            if (active(id) && m_memberships[static_cast<size_t>(id)] == 0) ids.push_back(id);
        }
        if (ids.empty()) continue;
        auto surface = detected.surface;
        surface.id = m_nextRegionId++;
        surface.source = "detected";
        m_regions.push_back(surface);
        for (vtkIdType id : ids) m_memberships[static_cast<size_t>(id)] = surface.id;
    }
}

bool PointCloudSelection::samePartCandidates(const std::vector<DetectedPointCloudSurface>& parts,
                                              const std::set<int>& replaceable) const
{
    std::vector<const PointCloudRegion*> current;
    for (const auto& region : m_regions)
        if (replaceable.count(region.id)) current.push_back(&region);
    auto memberships = m_memberships;
    for (auto& id : memberships)
        if (replaceable.count(id)) id = 0;
    std::size_t index = 0;
    for (const auto& part : parts) {
        const auto& next = part.surface;
        if (!validGeometry(next.geometry)) continue;
        const auto available = [&](vtkIdType id) { return active(id) && memberships[static_cast<std::size_t>(id)] == 0; };
        if (std::none_of(part.points.begin(), part.points.end(), available)) continue;
        if (index == current.size()) return false;
        const auto& prior = *current[index++];
        if (prior.name != next.name || prior.geometry != next.geometry ||
            prior.parameters != next.parameters || prior.supportValidation != next.supportValidation ||
            prior.residual != next.residual || prior.confidence != next.confidence ||
            prior.fitStale != next.fitStale || prior.templateLocked != next.templateLocked) return false;
        for (const auto id : part.points)
            if (available(id)) memberships[static_cast<std::size_t>(id)] = prior.id;
    }
    return index == current.size() && memberships == m_memberships;
}

bool PointCloudSelection::replacePartCandidates(const std::vector<DetectedPointCloudSurface>& parts,bool recordUndo)
{
    std::set<int> removed;
    for(const auto& region:m_regions)if(region.source=="detected"&&!region.templateLocked)removed.insert(region.id);
    if (samePartCandidates(parts, removed)) return false;
    checkpoint(recordUndo);
    for(auto& membership:m_memberships)if(removed.count(membership))membership=0;
    m_regions.erase(std::remove_if(m_regions.begin(),m_regions.end(),[&](const auto& r){return removed.count(r.id)!=0;}),m_regions.end());
    m_relations.erase(std::remove_if(m_relations.begin(),m_relations.end(),[&](const auto& r){return removed.count(r.first)||removed.count(r.second);}),m_relations.end());
    addSurfaces(parts,false);
    return true;
}

PlyCloud PointCloudSelection::workingCloud(std::vector<vtkIdType>* originalIds, bool unassignedOnly, int regionId) const
{
    PlyCloud result;
    if (originalIds) originalIds->clear();
    const bool normals = m_sourceCloud.normals.size() == m_sourceCloud.positions.size();
    const bool colors = m_sourceCloud.colors.size() == m_sourceCloud.positions.size();
    for (vtkIdType id : m_activeIds) {
        const size_t i = static_cast<size_t>(id);
        if (unassignedOnly && m_memberships[i] != 0) continue;
        if (regionId >= 0 && m_memberships[i] != regionId) continue;
        result.positions.push_back(m_sourceCloud.positions[i]);
        result.sourceRows.push_back(i);
        if (normals) result.normals.push_back(m_sourceCloud.normals[i]);
        if (colors) result.colors.push_back(m_sourceCloud.colors[i]);
        if (originalIds) originalIds->push_back(id);
    }
    return result;
}

bool PointCloudSelection::exportCloud(const QString& path, QString& error) const
{
    return PlyWriter::write(workingCloud(), path, error);
}
