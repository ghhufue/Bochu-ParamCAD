#include "detail/AnalysisInternal.h"

namespace reconstruction::detail::analysis {
const std::vector<PointId>& ids(const PrimitiveEvidence& e) {
    return std::visit([](const auto& x) -> const std::vector<PointId>& { return x.supportPointIds; }, e);
}

double confidence(const PrimitiveEvidence& e) {
    return std::visit([](const auto& x) { return x.confidence; }, e);
}

std::unordered_map<PointId, const PointSample*> pointMap(const PointStore& points) {
    std::unordered_map<PointId, const PointSample*> result;
    result.reserve(points.size());
    for (const auto& p : points.points()) result.emplace(p.id, &p);
    return result;
}

Vec3 axis(const PrimitiveEvidence& e){return primitiveAxis(e);}
Vec3 center(const PrimitiveEvidence& e){return primitiveCenter(e);}

double radius(const PrimitiveEvidence& e) {
    if (const auto* c = std::get_if<CylinderEvidence>(&e)) return c->cylinder.radius;
    if (const auto* s = std::get_if<SphereEvidence>(&e)) return s->sphere.radius;
    return -1;
}

bool axial(PrimitiveKind k) { return k == PrimitiveKind::Cylinder || k == PrimitiveKind::Cone || k == PrimitiveKind::Torus; }

bool valid(const SurfaceAnalysisOptions& o) {
    return o.familyDetectionWorkerCount <= 5 && validSurfaceSupportOptions(o.supportValidation) &&
        std::isfinite(o.connectivityDistance) && o.connectivityDistance >= 0 &&
        std::isfinite(o.relations.distanceTolerance) && o.relations.distanceTolerance > 0 &&
        std::isfinite(o.relations.angularToleranceRadians) && o.relations.angularToleranceRadians > 0 &&
        o.relations.angularToleranceRadians <= 0.5235987755982988;
}
} // namespace reconstruction::detail::analysis
