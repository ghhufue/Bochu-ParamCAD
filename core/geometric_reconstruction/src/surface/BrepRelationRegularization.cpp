#include "detail/AnalysisInternal.h"

#include <Eigen/Cholesky>
#include <Eigen/Core>

namespace reconstruction {
namespace {
using namespace detail::analysis;

struct Groups {
    explicit Groups(std::size_t size) : parent(size) { std::iota(parent.begin(), parent.end(), 0); }
    std::size_t root(std::size_t i) {
        while (i != parent[i]) {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    }
    void join(std::size_t a, std::size_t b) { parent[root(b)] = root(a); }
    std::vector<std::size_t> parent;
};

struct Link {
    std::size_t a, b;
    SurfaceRelation relation;
};

bool parallel(SurfaceRelationKind kind) {
    return kind == SurfaceRelationKind::Parallel || kind == SurfaceRelationKind::Coplanar ||
        kind == SurfaceRelationKind::Coaxial || kind == SurfaceRelationKind::AxisNormalToPlane;
}

bool perpendicular(SurfaceRelationKind kind) {
    return kind == SurfaceRelationKind::Perpendicular || kind == SurfaceRelationKind::AxisParallelToPlane;
}

Vec3 align(Vec3 n, Vec3 reference) { return dot(n, reference) < 0 ? scale(n, -1) : n; }

struct ObservationFit {
    double mean = 0, max = 0;
    std::size_t inliers = 0;
};

ObservationFit measure(const PrimitiveEvidence& e, const PointStore& points,
                       const std::unordered_map<PointId, const PointSample*>& samples, double tolerance) {
    ObservationFit fit;
    for (const auto id : ids(e)) {
        const double d = surfaceResidual(e, samples.at(id)->position);
        fit.mean += d;
        fit.max = std::max(fit.max, d);
        fit.inliers += d <= tolerance;
    }
    fit.mean /= std::max<std::size_t>(1, std::min(ids(e).size(), points.size()));
    return fit;
}

std::array<Vec3, 3> dominantFrame(const std::vector<Vec3>& directions, const std::vector<double>& weights,
                                  double sine, double cosine) {
    std::array<Vec3, 3> frame{};
    double best = 0;
    for (std::size_t i = 0; i < directions.size(); ++i) {
        if (weights[i] == 0) continue;
        for (std::size_t j = 0; j < i; ++j) {
            if (weights[j] == 0 || std::abs(dot(directions[i], directions[j])) > sine) continue;
            const auto n = directions[i];
            const auto t = normalized(subtract(directions[j], scale(n, dot(n, directions[j]))));
            const std::array<Vec3, 3> candidate{n, t, cross(n, t)};
            double score = 0;
            for (std::size_t k = 0; k < directions.size(); ++k) {
                const double alignment = std::max({std::abs(dot(candidate[0], directions[k])),
                    std::abs(dot(candidate[1], directions[k])), std::abs(dot(candidate[2], directions[k]))});
                if (alignment >= cosine) score += weights[k] * alignment;
            }
            if (score > best) { best = score; frame = candidate; }
        }
    }
    return frame;
}

// One small least-squares block per coaxial component. Radius (or cone radial
// intercept) is recovered independently per surface after eliminating its mean.
bool fitAxisGroup(const std::vector<std::size_t>& group, const std::vector<Vec3>& directions,
    std::vector<PrimitiveEvidence>& fitted, const std::unordered_map<PointId, const PointSample*>& samples) {
    const auto n = directions[group.front()];
    const auto x = normalized(cross(n, std::abs(n.z) < .9 ? Vec3{0,0,1} : Vec3{0,1,0}));
    const auto y = cross(n, x);
    auto origin = center(fitted[group.front()]);
    Eigen::Vector2d shift = Eigen::Vector2d::Zero();
    std::vector<double> radii(group.size());
    for (int iteration = 0; iteration < 8; ++iteration) {
        Eigen::Matrix2d normal = Eigen::Matrix2d::Zero();
        Eigen::Vector2d rhs = Eigen::Vector2d::Zero();
        for (std::size_t k = 0; k < group.size(); ++k) {
            const auto i = group[k];
            const auto& evidence = fitted[i];
            const auto* cone = std::get_if<ConeEvidence>(&evidence);
            const double slope = cone ? std::tan(cone->cone.openingAngleRadians) : 0;
            double mean = 0;
            Eigen::Vector2d meanJacobian = Eigen::Vector2d::Zero(), crossTerm = Eigen::Vector2d::Zero();
            for (const auto id : ids(evidence)) {
                const auto p = subtract(samples.at(id)->position, origin);
                const Eigen::Vector2d radial = shift - Eigen::Vector2d(dot(p, x), dot(p, y));
                const double distance = radial.norm();
                if (distance <= 1e-12) return false;
                const Eigen::Vector2d jacobian = radial / distance;
                const double residual = distance - slope * dot(p, directions[i]);
                normal += jacobian * jacobian.transpose();
                crossTerm += jacobian * residual;
                meanJacobian += jacobian;
                mean += residual;
            }
            const double count = static_cast<double>(ids(evidence).size());
            mean /= count;
            normal -= meanJacobian * meanJacobian.transpose() / count;
            rhs += meanJacobian * mean - crossTerm;
            radii[k] = mean;
        }
        if (iteration == 7) break;
        normal.diagonal().array() += 1e-7;
        const Eigen::Vector2d step = normal.ldlt().solve(rhs);
        if (!step.allFinite()) return false;
        shift += step;
    }
    const auto fittedOrigin = add(origin, add(scale(x, shift.x()), scale(y, shift.y())));
    for (std::size_t k = 0; k < group.size(); ++k) {
        const auto i = group[k];
        const auto u = directions[i];
        if (auto* cylinder = std::get_if<CylinderEvidence>(&fitted[i])) {
            if (radii[k] <= 0) return false;
            cylinder->cylinder = {subtract(fittedOrigin, scale(u, dot(fittedOrigin, u))), u, radii[k]};
        } else {
            auto& cone = std::get<ConeEvidence>(fitted[i]).cone;
            cone.apex = subtract(fittedOrigin, scale(u, radii[k] / std::tan(cone.openingAngleRadians)));
            cone.axisDirection = u;
        }
    }
    return true;
}
} // namespace

bool regularizeBrepRelations(const PointStore& points, SurfaceAnalysisResult& analysis,
    RelationOptimizationReport& report, std::string& error) {
    report = {};
    error.clear();
    if (!analysisMatches(points, analysis) || !valid(analysis.options)) {
        error = "Stale or invalid B-Rep regularization snapshot";
        return false;
    }
    const auto samples = pointMap(points);
    const auto& settings = analysis.options.relations;
    const double tolerance = analysis.options.detection.plane.distanceThreshold;
    const double snap = std::min(settings.angularToleranceRadians, 0.03490658503988659);
    const double sine = std::sin(snap), cosine = std::cos(snap);
    const double inlierTolerance = tolerance * analysis.options.recoveryHaloToleranceMultiplier;
    const auto original = analysis.surfaces;
    const auto size = original.size();
    std::vector<Vec3> originalDirections(size), directions(size), centroids(size);
    std::vector<double> weights(size);
    std::vector<ObservationFit> before(size);
    for (std::size_t i = 0; i < size; ++i) {
        const auto kind = primitiveKind(original[i]);
        if (ids(original[i]).empty() || (kind != PrimitiveKind::Plane &&
            kind != PrimitiveKind::Cylinder && kind != PrimitiveKind::Cone)) continue;
        for (const auto id : ids(original[i])) {
            if (!samples.count(id)) { error = "Unknown B-Rep support point"; return false; }
            centroids[i] = add(centroids[i], samples.at(id)->position);
        }
        weights[i] = static_cast<double>(ids(original[i]).size());
        centroids[i] = scale(centroids[i], 1 / weights[i]);
        originalDirections[i] = directions[i] = axis(original[i]);
        before[i] = measure(original[i], points, samples, inlierTolerance);
        report.beforeError += before[i].mean;
    }
    std::unordered_map<PatchId, std::size_t> surfaces;
    for (const auto& patch : analysis.patches) {
        if (!patch.surfaceId || patch.surfaceId > size ||
            std::any_of(patch.supportPointIds.begin(), patch.supportPointIds.end(),
                        [&](auto id) { return !samples.count(id); })) {
            error = "Invalid B-Rep patch support";
            return false;
        }
        surfaces.emplace(patch.id, patch.surfaceId - 1);
    }
    std::vector<Link> links;
    for (const auto& relation : analysis.relations) {
        const auto kind = relation.kind;
        if (!parallel(kind) && !perpendicular(kind) && kind != SurfaceRelationKind::SameRadius) continue;
        const auto a = surfaces.find(relation.first), b = surfaces.find(relation.second);
        if (a == surfaces.end() || b == surfaces.end() || !weights[a->second] || !weights[b->second]) {
            report.rejected.push_back(relation);
            continue;
        }
        if (a->second != b->second) links.push_back({a->second, b->second, relation});
    }
    const auto isPlane = [&](std::size_t i) { return std::holds_alternative<PlaneEvidence>(original[i]); };
    const auto fitsObservations = [&](std::size_t i, const PrimitiveEvidence& candidate) {
        const auto fit = measure(candidate, points, samples, inlierTolerance);
        return std::isfinite(fit.mean) && fit.inliers >= weights[i] * settings.optimizationInlierFraction &&
            fit.mean <= before[i].mean + tolerance * settings.maximumMeanErrorIncreaseMultiplier;
    };

    // Fit unit directions with anchored projected gradient steps, then enforce
    // exact relationships on the support-weighted frame and parallel groups.
    for (std::size_t step = 0; step < settings.optimizationIterations; ++step) {
        std::vector<Vec3> gradient(size);
        std::vector<double> stiffness = weights;
        for (std::size_t i = 0; i < size; ++i)
            gradient[i] = scale(subtract(directions[i], originalDirections[i]), weights[i]);
        for (const auto& link : links) {
            const auto [a, b, relation] = link;
            if (!parallel(relation.kind) && !perpendicular(relation.kind)) continue;
            const double alignment = dot(directions[a], directions[b]);
            const double originalAlignment = std::abs(dot(originalDirections[a], originalDirections[b]));
            if (perpendicular(relation.kind) ? originalAlignment > 2 * sine : originalAlignment < 2 * cosine - 1) continue;
            const double w = settings.constraintWeight * std::min(weights[a], weights[b]);
            const double multiplier = perpendicular(relation.kind) ? w * alignment : -w * alignment;
            gradient[a] = add(gradient[a], scale(directions[b], multiplier));
            gradient[b] = add(gradient[b], scale(directions[a], multiplier));
            stiffness[a] += w;
            stiffness[b] += w;
        }
        for (std::size_t i = 0; i < size; ++i)
            if (weights[i] > 0) directions[i] = normalized(subtract(directions[i], scale(gradient[i], .3 / stiffness[i])));
    }
    const auto frame = dominantFrame(directions, weights, sine, cosine);
    Groups parallelGroups(size);
    for (const auto& link : links) {
        if (parallel(link.relation.kind) &&
            std::abs(dot(originalDirections[link.a], originalDirections[link.b])) >= cosine)
            parallelGroups.join(link.a, link.b);
    }
    std::vector<Vec3> groupDirections(size);
    std::vector<double> groupWeights(size);
    for (std::size_t i = 0; i < size; ++i) {
        const auto group = parallelGroups.root(i);
        groupDirections[group] = add(groupDirections[group], scale(align(directions[i], directions[group]), weights[i]));
        groupWeights[group] += weights[i];
    }
    std::vector<bool> onFrame(size), locked(size);
    std::vector<std::size_t> priority;
    for (std::size_t i = 0; i < size; ++i) {
        if (!groupWeights[i]) continue;
        priority.push_back(i);
        auto& n = groupDirections[i];
        n = normalized(n);
        for (const auto candidate : frame)
            if (std::abs(dot(n, candidate)) >= cosine) { n = align(candidate, n); onFrame[i] = true; break; }
    }
    std::stable_sort(priority.begin(), priority.end(), [&](auto a, auto b) {
        if (onFrame[a] != onFrame[b]) return bool(onFrame[a]);
        return groupWeights[a] > groupWeights[b];
    });
    std::vector<std::vector<std::size_t>> orthogonal(size);
    for (const auto& [a, b, relation] : links) {
        if (!perpendicular(relation.kind) || std::abs(dot(originalDirections[a], originalDirections[b])) > sine) continue;
        const auto first = parallelGroups.root(a), second = parallelGroups.root(b);
        if (first == second) continue;
        orthogonal[first].push_back(second);
        orthogonal[second].push_back(first);
    }
    for (const auto group : priority) {
        locked[group] = true;
        if (onFrame[group]) continue;
        const auto n = groupDirections[group];
        std::vector<Vec3> basis;
        for (const auto peer : orthogonal[group]) {
            if (!locked[peer] || std::abs(dot(n, groupDirections[peer])) > sine) continue;
            auto u = groupDirections[peer];
            for (const auto previous : basis) u = subtract(u, scale(previous, dot(previous, u)));
            if (length(u) > 1e-6) basis.push_back(normalized(u));
        }
        auto projected = n;
        for (const auto u : basis) projected = subtract(projected, scale(u, dot(u, projected)));
        if (length(projected) >= cosine) groupDirections[group] = normalized(projected);
    }
    for (std::size_t i = 0; i < size; ++i) {
        if (!weights[i]) continue;
        const auto n = groupDirections[parallelGroups.root(i)];
        directions[i] = std::abs(dot(n, originalDirections[i])) >= cosine ? align(n, originalDirections[i]) : originalDirections[i];
    }

    Groups positionalGroups(size);
    std::vector<std::vector<std::size_t>> components(size);
    for (const auto& [a, b, relation] : links) {
        if (std::abs(dot(directions[a], directions[b])) < 1 - 1e-12) continue;
        if ((relation.kind == SurfaceRelationKind::Coplanar && isPlane(a) && isPlane(b)) ||
            (relation.kind == SurfaceRelationKind::Coaxial && !isPlane(a) && !isPlane(b)))
            positionalGroups.join(a, b);
    }
    for (std::size_t i = 0; i < size; ++i)
        if (weights[i]) components[positionalGroups.root(i)].push_back(i);
    auto fitted = original;
    for (const auto& component : components) {
        if (component.empty()) continue;
        bool success = true;
        if (isPlane(component.front())) {
            const auto n = directions[component.front()];
            double offset = 0, totalWeight = 0;
            for (const auto i : component) {
                offset -= dot(n, centroids[i]) * weights[i];
                totalWeight += weights[i];
            }
            for (const auto i : component)
                std::get<PlaneEvidence>(fitted[i]).plane = {directions[i], dot(n, directions[i]) * offset / totalWeight};
        } else success = fitAxisGroup(component, directions, fitted, samples);
        for (const auto i : component) success &= fitsObservations(i, fitted[i]);
        if (!success) for (const auto i : component) fitted[i] = original[i];
    }
    Groups radiusGroups(size);
    for (const auto& [a, b, relation] : links)
        if (relation.kind == SurfaceRelationKind::SameRadius &&
            std::holds_alternative<CylinderEvidence>(fitted[a]) && std::holds_alternative<CylinderEvidence>(fitted[b]))
            radiusGroups.join(a, b);
    std::vector<std::vector<std::size_t>> radii(size);
    for (std::size_t i = 0; i < size; ++i)
        if (weights[i] && std::holds_alternative<CylinderEvidence>(fitted[i])) radii[radiusGroups.root(i)].push_back(i);
    for (const auto& group : radii) {
        if (group.size() < 2) continue;
        double mean = 0, sum = 0;
        for (const auto i : group) { mean += radius(fitted[i]) * weights[i]; sum += weights[i]; }
        mean /= sum;
        const auto verify = [&](std::size_t i) {
            auto candidate = fitted[i];
            std::get<CylinderEvidence>(candidate).cylinder.radius = mean;
            return fitsObservations(i, candidate);
        };
        if (std::all_of(group.begin(), group.end(), verify))
            for (const auto i : group) std::get<CylinderEvidence>(fitted[i]).cylinder.radius = mean;
    }
    for (const auto& [a, b, relation] : links) {
        const auto u = axis(fitted[a]), v = axis(fitted[b]);
        const double alignment = dot(u, v);
        const auto kind = relation.kind;
        bool accepted = perpendicular(kind) ? std::abs(alignment) <= 1e-6 : length(cross(u, v)) <= 1e-6;
        if (kind == SurfaceRelationKind::Coaxial)
            accepted &= length(cross(subtract(center(fitted[a]), center(fitted[b])), u)) <= 1e-6;
        if (kind == SurfaceRelationKind::Coplanar && isPlane(a) && isPlane(b))
            accepted &= std::abs(std::get<PlaneEvidence>(fitted[a]).plane.offset -
                std::copysign(1., alignment) * std::get<PlaneEvidence>(fitted[b]).plane.offset) <= 1e-6;
        if (kind == SurfaceRelationKind::SameRadius)
            accepted = std::holds_alternative<CylinderEvidence>(fitted[a]) &&
                std::holds_alternative<CylinderEvidence>(fitted[b]) && std::abs(radius(fitted[a]) - radius(fitted[b])) <= 1e-6;
        (accepted ? report.accepted : report.rejected).push_back(relation);
    }
    for (std::size_t i = 0; i < size; ++i) {
        if (!weights[i]) continue;
        const auto fit = measure(fitted[i], points, samples, inlierTolerance);
        report.afterError += fit.mean;
        std::visit([&](auto& surface) {
            surface.meanAbsoluteResidual = fit.mean;
            surface.maxAbsoluteResidual = fit.max;
            using Type = std::decay_t<decltype(surface)>;
            if constexpr (std::is_same_v<Type, ConeEvidence> || std::is_same_v<Type, CylinderEvidence>) {
                surface.axialMinimum = INFINITY;
                surface.axialMaximum = -INFINITY;
                for (const auto id : surface.supportPointIds) {
                    const auto height = dot(subtract(samples.at(id)->position, center(fitted[i])), axis(fitted[i]));
                    surface.axialMinimum = std::min(surface.axialMinimum, height);
                    surface.axialMaximum = std::max(surface.axialMaximum, height);
                }
            }
        }, fitted[i]);
    }
    analysis.surfaces = std::move(fitted);
    return true;
}
} // namespace reconstruction
