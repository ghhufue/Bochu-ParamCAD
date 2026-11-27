#include "ClosureInternal.h"
#include "reconstruction/data/StageProfile.h"
#include "reconstruction/surface/SurfaceOwnership.h"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepCheck.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Result.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Vertex.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>

#include <set>
#include <sstream>

namespace reconstruction {
namespace {
using namespace detail::closure;

TopoDS_Solid singleSolid(const TopoDS_Shape& shape) {
    TopExp_Explorer explorer(shape, TopAbs_SOLID);
    if (!explorer.More()) return {};
    const auto solid = TopoDS::Solid(explorer.Current());
    explorer.Next();
    return explorer.More() ? TopoDS_Solid{} : solid;
}

double volume(const TopoDS_Shape& shape) {
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties, true);
    return properties.Mass();
}

bool finishSolid(TopoDS_Solid& solid, const CuttingGeometry& geometry, double fuzzy,
                 const BrepClosureOptions& options, std::string& error, std::vector<std::string>& diagnostics) {
    // General Fuse may retain fingerprints of dangling split tools as
    // INTERNAL wires or vertices. The exported body only needs its boundary.
    TopoDS_Shape cleaned = BRepBuilderAPI_Copy(solid, false, false).Shape();
    BRepTools::RemoveInternals(cleaned, true);
    solid = singleSolid(cleaned);
    for (const auto& tool : geometry.holeTools) {
        NCollection_List<TopoDS_Shape> arguments, tools;
        arguments.Append(solid);
        tools.Append(tool);
        BRepAlgoAPI_Cut cut;
        cut.SetArguments(arguments);
        cut.SetTools(tools);
        cut.SetNonDestructive(true);
        cut.SetFuzzyValue(fuzzy);
        cut.SetRunParallel(options.enableParallelBop);
        cut.Build();
        if (!cut.IsDone() || cut.HasErrors()) {
            error = "Regular hole boolean subtraction failed";
            return false;
        }
        auto drilled = singleSolid(cut.Shape());
        if (drilled.IsNull() || !(volume(drilled) < volume(solid))) {
            error = "Regular hole did not remove material from a single solid";
            return false;
        }
        solid = drilled;
    }
    ShapeUpgrade_UnifySameDomain unify(solid, true, true, false);
    unify.SetSafeInputMode(true);
    unify.SetLinearTolerance(fuzzy);
    unify.SetAngularTolerance(1e-6);
    unify.Build();
    const auto reduced = singleSolid(unify.Shape());
    if (!reduced.IsNull() && BRepCheck_Analyzer(reduced).IsValid()) {
        solid = reduced;
    } else if (BRepCheck_Analyzer(solid).IsValid()) {
        diagnostics.push_back("Same-domain unification was rolled back by the topology check");
    }
    return true;
}

std::string topologyErrors(const TopoDS_Solid& solid, const BRepCheck_Analyzer& analyzer) {
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> shapes;
    TopExp::MapShapes(solid, shapes);
    std::set<BRepCheck_Status> statuses;
    const auto record = [&](const NCollection_List<BRepCheck_Status>& checks) {
        for (const auto status : checks)
            if (status != BRepCheck_NoError) statuses.insert(status);
    };
    for (const auto& shape : shapes) {
        const auto& result = analyzer.Result(shape);
        if (result.IsNull()) continue;
        record(result->Status());
        for (result->InitContextIterator(); result->MoreShapeInContext(); result->NextShapeInContext())
            record(result->StatusOnShape());
    }
    std::ostringstream message;
    for (const auto status : statuses) {
        message << ' ';
        BRepCheck::Print(status, message);
    }
    return message.str();
}

bool checkSolid(const TopoDS_Solid& solid, const PointStore& observations, double tolerance,
                const BrepClosureOptions& options, BrepClosureQualityReport& quality,
                std::string& error, const std::function<bool()>& cancelled) {
    const BRepCheck_Analyzer analyzer(solid);
    quality.isValidTopology = analyzer.IsValid();
    BRep_Builder builder;
    TopoDS_Compound shells;
    builder.MakeCompound(shells);
    for (TopExp_Explorer shell(solid, TopAbs_SHELL); shell.More(); shell.Next()) {
        builder.Add(shells, shell.Current());
        quality.isValidTopology &= BRep_Tool::IsClosed(shell.Current());
    }
    ShapeAnalysis_FreeBounds freeBounds(shells, false, true, false);
    quality.freeEdgeCount = 0;
    for (const auto& wires : {freeBounds.GetOpenWires(), freeBounds.GetClosedWires()})
        for (TopExp_Explorer edge(wires, TopAbs_EDGE); edge.More(); edge.Next()) ++quality.freeEdgeCount;
    quality.volumeMm3 = volume(solid);
    if (!quality.isValidTopology || quality.freeEdgeCount != 0 ||
        !std::isfinite(quality.volumeMm3) || quality.volumeMm3 <= 0) {
        error = "B-Rep failed topology, free-edge or positive-volume validation";
        if (!analyzer.IsValid()) error += topologyErrors(solid, analyzer);
        return false;
    }
    // Query the boundary compound: distance to a solid is zero for interior points.
    TopoDS_Compound faces;
    builder.MakeCompound(faces);
    for (TopExp_Explorer face(solid, TopAbs_FACE); face.More(); face.Next()) builder.Add(faces, face.Current());
    BRepExtrema_DistShapeShape distance;
    distance.LoadS2(faces);
    std::vector<double> distances;
    double squaredError = 0;
    constexpr std::size_t sampleLimit = 4096;
    const std::size_t count = std::min(sampleLimit, observations.size());
    distances.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        if (cancelled && cancelled()) return false;
        TopoDS_Vertex vertex;
        builder.MakeVertex(vertex, point(observations.points()[i * observations.size() / count].position),
                           Precision::Confusion());
        distance.LoadS1(vertex);
        if (!distance.Perform() || !std::isfinite(distance.Value())) {
            error = "Could not measure an observation against the B-Rep boundary";
            return false;
        }
        distances.push_back(distance.Value());
        squaredError += distance.Value() * distance.Value();
    }
    quality.scoredPointCount = distances.size();
    quality.rmsErrorMm = std::sqrt(squaredError / distances.size());
    const auto p95 = distances.begin() + static_cast<std::ptrdiff_t>(std::ceil(0.95 * distances.size()) - 1);
    std::nth_element(distances.begin(), p95, distances.end());
    quality.p95ErrorMm = *p95;
    if (quality.p95ErrorMm > options.maxAllowedP95Multiplier * tolerance) {
        error = "B-Rep P95 distance " + std::to_string(*p95) + " mm exceeds " +
            std::to_string(options.maxAllowedP95Multiplier * tolerance) + " mm";
        return false;
    }
    return true;
}

double minimumWall(const SurfaceAnalysisResult& analysis, double fallback) {
    double thickness = fallback;
    const auto record = [&](double gap) {
        if (gap > 10 * Precision::Confusion()) thickness = std::min(thickness, gap);
    };
    for (std::size_t i = 0; i < analysis.surfaces.size(); ++i) {
        const auto* a = std::get_if<PlaneEvidence>(&analysis.surfaces[i]);
        const auto* ca = std::get_if<CylinderEvidence>(&analysis.surfaces[i]);
        for (std::size_t j = 0; j < i; ++j) {
            if (const auto* b = std::get_if<PlaneEvidence>(&analysis.surfaces[j]); a && b) {
                const auto alignment = dot(a->plane.normal, b->plane.normal);
                if (std::abs(alignment) >= 1 - 1e-8)
                    record(std::abs(a->plane.offset - std::copysign(1.0, alignment) * b->plane.offset));
            }
            if (const auto* cb = std::get_if<CylinderEvidence>(&analysis.surfaces[j]); ca && cb) {
                const auto& first = ca->cylinder;
                const auto& second = cb->cylinder;
                if (std::abs(dot(first.axisDirection, second.axisDirection)) < 1 - 1e-8) continue;
                const auto centers = subtract(first.axisPoint, second.axisPoint);
                const auto separation = length(cross(centers, first.axisDirection));
                const auto radiusGap = std::abs(first.radius - second.radius);
                if (separation < radiusGap) record(radiusGap - separation);
            }
        }
    }
    return thickness;
}

bool reindex(const PointStore& points, const SurfaceAnalysisResult& source,
    const std::vector<PrimitiveEvidence>& evidence, SurfaceAnalysisResult& refined, std::string& error) {
    if (!indexSurfacePatches(points, evidence, source.options, refined, error) ||
        !inferPatchRelations(points, refined, refined.relations, error)) return false;
    std::unordered_map<PatchId, const SurfacePatch*> patches;
    for (const auto& patch : source.patches) patches.emplace(patch.id, &patch);
    std::vector<SurfaceRelationHint> hints;
    for (const auto& relation : source.relations) {
        if (relation.automatic) continue;
        const auto a = patches.find(relation.first), b = patches.find(relation.second);
        if (a != patches.end() && b != patches.end())
            hints.push_back({a->second->supportPointIds, b->second->supportPointIds, relation.kind});
    }
    return hints.empty() || applySurfaceRelationHints(points, hints, refined, error);
}

void releaseUnreliablePatches(const PointStore& points, const PointIndex& samples, SurfaceAnalysisResult& analysis,
                              ClosedBrepResult& result) {
    std::vector<PrimitiveEvidence> supported;
    for (const auto& patch : analysis.patches) {
        const auto& surface = analysis.surfaces.at(patch.surfaceId - 1);
        if (!supportedAnalyticPatch(surface, patch.supportPointIds, samples)) continue;
        auto observation = surface;
        std::visit([&](auto& entry) { entry.supportPointIds = patch.supportPointIds; }, observation);
        supported.push_back(std::move(observation));
    }
    if (supported.size() == analysis.patches.size()) return;
    SurfaceAnalysisResult refined;
    std::string error;
    if (reindex(points, analysis, supported, refined, error)) {
        result.diagnostics.push_back("Released observations from " +
            std::to_string(analysis.patches.size() - supported.size()) + " weak analytic patches");
        analysis = std::move(refined);
    } else {
        result.diagnostics.push_back("Analytic patch filtering: " + error);
    }
}

void recoverResidualAnalyticSurfaces(const PointStore& points, const PointIndex& samples, SurfaceAnalysisResult& analysis,
    double spacing, double tolerance, ClosedBrepResult& result, const std::function<bool()>& cancelled) {
    const auto clusters = connectedPointComponents(points, analysis.unassignedPointIds,
                                                    std::max(2.8 * spacing, 4 * tolerance));
    auto evidence = analysis.surfaces;
    for (const auto& cluster : clusters) {
        if (cancelled && cancelled()) return;
        if (cluster.size() < 24) continue;
        std::vector<PointSample> local;
        local.reserve(cluster.size());
        for (const auto id : cluster) local.push_back(*samples.at(id));
        const PointStore residual(std::move(local), points.unit());
        auto options = analysis.options.detection;
        options.detectSpheres = options.detectTori = false;
        const auto minimum = std::clamp<std::size_t>(cluster.size() / 12, 16, 30);
        options.plane.minimumSupportPoints = minimum;
        options.cylinder.minimumSupportPoints = options.cone.minimumSupportPoints = minimum;
        options.plane.clusterEpsilon = std::max(2.8 * spacing, 4 * tolerance);
        options.cylinder.clusterEpsilon = options.cone.clusterEpsilon = std::max(2.8 * spacing, 4 * tolerance);
        options.plane.maximumPlanes = 4;
        options.cylinder.maximumCylinders = options.cone.maximumCones = 4;
        PrimitiveDetectionResult detected;
        std::string ignored;
        if (!detectPrimitiveEvidence(residual, options, detected, ignored)) continue;
        for (auto& candidate : detected.evidence) {
            const auto supported = std::visit([&](const auto& entry) {
                using Type = std::decay_t<decltype(entry)>;
                if constexpr (std::is_same_v<Type, CylinderEvidence> || std::is_same_v<Type, ConeEvidence>) {
                    return entry.angularCoverage >= .78 && entry.axialMaximum - entry.axialMinimum >= 1.5 * spacing &&
                        entry.meanAbsoluteResidual <= .5 * tolerance;
                } else if constexpr (std::is_same_v<Type, PlaneEvidence>) {
                    if (entry.meanAbsoluteResidual > .35 * tolerance) return false;
                    const auto n = entry.plane.normal;
                    const auto u = normalized(cross(n, std::abs(n.z) < .9 ? Vec3{0,0,1} : Vec3{0,1,0}));
                    const auto v = cross(n, u);
                    double x0 = INFINITY, x1 = -INFINITY, y0 = INFINITY, y1 = -INFINITY;
                    for (const auto id : entry.supportPointIds) {
                        const auto p = samples.at(id)->position;
                        const auto x = dot(p, u), y = dot(p, v);
                        x0 = std::min(x0, x); x1 = std::max(x1, x);
                        y0 = std::min(y0, y); y1 = std::max(y1, y);
                    }
                    return std::min(x1 - x0, y1 - y0) >= 3 * spacing;
                } else {
                    return false;
                }
            }, candidate);
            const auto validNormals = std::visit([&](const auto& entry) {
                return supportedAnalyticPatch(candidate, entry.supportPointIds, samples);
            }, candidate);
            if (supported && validNormals) evidence.push_back(std::move(candidate));
        }
    }
    if (evidence.size() == analysis.surfaces.size()) return;
    SurfaceAnalysisResult refined;
    std::string error;
    if (reindex(points, analysis, evidence, refined, error)) {
        result.diagnostics.push_back("Recovered " + std::to_string(evidence.size() - analysis.surfaces.size()) +
                                      " analytic surfaces from residual observations");
        analysis = std::move(refined);
    } else {
        result.diagnostics.push_back("Residual analytic surface indexing: " + error);
    }
}
} // namespace

bool reconstructClosedBrep(const PointStore& points, const SurfaceAnalysisResult& analysis,
    const BrepClosureOptions& options, ClosedBrepResult& output, std::string& error,
    const PointStore* originalObservations, const std::function<bool()>& cancelled) {
    output = {};
    error.clear();
    const auto& observations = originalObservations ? *originalObservations : points;
    const auto tolerance = analysis.options.detection.plane.distanceThreshold;
    const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
    if (points.size() < 4 || observations.empty() || points.unit() != LengthUnit::Millimeter ||
        observations.unit() != points.unit() || !analysisMatches(points, analysis) || !positive(tolerance) ||
        !positive(options.planeMarginFraction) || !positive(options.freeformExtendFraction) ||
        !positive(options.baseFuzzyToleranceFraction) || !positive(options.maxAllowedP95Multiplier)) {
        error = "Closed B-Rep requires a matching millimetre analysis and positive closure options";
        return false;
    }
    if (cancelled && cancelled()) return false;
    try {
        PointIndex samples;
        samples.reserve(points.size());
        Bnd_Box envelope;
        for (const auto& sample : points.points()) {
            samples.emplace(sample.id, &sample);
            envelope.Add(point(sample.position));
        }
        const auto [lo, hi] = bounds(envelope);
        const double diagonal = length(subtract(hi, lo));
        if (diagonal <= Precision::Confusion()) {
            error = "Point cloud has no measurable extent";
            return false;
        }
        const KdTree tree(points.points());
        std::vector<double> spacings;
        const auto stride = std::max<std::size_t>(1, points.size() / 512);
        for (std::size_t i = 0; i < points.size(); i += stride) {
            const auto near = tree.nearest(i, 4);
            if (!near.empty() && near.back().first > 0) spacings.push_back(std::sqrt(near.back().first) / 1.5);
        }
        std::sort(spacings.begin(), spacings.end());
        const double spacing = spacings.empty() ? tolerance : spacings[spacings.size() / 2];
        auto regularized = analysis;
        const bool hasValidatedSupport = analysis.supportValidation.pointFingerprint == analysis.pointFingerprint;
        if (hasValidatedSupport) {
            SurfaceAnalysisResult refined;
            SurfaceOwnershipResult ownership;
            std::string ownershipError;
            if (diagnostics::profileStage("brep.ownership", [&] {
                return resolveSurfaceOwnership(points, analysis, {tolerance, std::max(4 * tolerance, 2 * spacing), true},
                    ownership, ownershipError, cancelled) &&
                    reindex(points, analysis, ownership.surfaces, refined, ownershipError);
            })) {
                output.diagnostics.push_back("Surface ownership assigned " + std::to_string(analysis.patches.size()) +
                    " proposals to " + std::to_string(refined.patches.size()) + " observed patches");
                regularized = std::move(refined);
            } else if (!ownershipError.empty()) {
                output.diagnostics.push_back("Surface ownership: " + ownershipError);
            }
            releaseUnreliablePatches(points, samples, regularized, output);
            diagnostics::profileStage("brep.residual_analytic_surfaces", [&] {
                recoverResidualAnalyticSurfaces(points, samples, regularized, spacing, tolerance, output, cancelled);
            });
        }
        if (cancelled && cancelled()) return false;
        RelationOptimizationReport relations;
        if (!diagnostics::profileStage("brep.regularization", [&] {
            return regularizeBrepRelations(points, regularized, relations, error);
        })) return false;
        output.diagnostics.push_back("Regularization accepted " + std::to_string(relations.accepted.size()) +
            " relations and rejected " + std::to_string(relations.rejected.size()));
        const auto extent = subtract(hi, lo);
        const Vec3 margin{std::max(.15 * extent.x, 8 * tolerance), std::max(.15 * extent.y, 8 * tolerance),
                          std::max(.15 * extent.z, 8 * tolerance)};
        envelope.Update(lo.x - margin.x, lo.y - margin.y, lo.z - margin.z,
                        hi.x + margin.x, hi.y + margin.y, hi.z + margin.z);
        auto geometry = diagnostics::profileStage("brep.cutting_faces", [&] {
            return makeCuttingGeometry(points, regularized, samples, tree, spacing, tolerance, options, output, cancelled);
        });
        if (cancelled && cancelled()) return false;
        if (geometry.faces.empty()) {
            error = "No supported cutting surfaces";
            return false;
        }
        const double fuzzyLimit = std::max(Precision::Confusion(), .1 * minimumWall(regularized, diagonal));
        const double base = std::min(options.baseFuzzyToleranceFraction * tolerance, diagonal * 1e-4);
        for (int mode = 0; mode < 2; ++mode) {
            double lastFuzzy = 0;
            for (const double multiplier : {1.0, 5.0, 10.0}) {
                if (cancelled && cancelled()) return false;
                const double fuzzy = std::clamp(multiplier * base, Precision::Confusion(), fuzzyLimit);
                if (fuzzy == lastFuzzy) break;
                lastFuzzy = fuzzy;
                const auto freeformCount = output.quality.freeformFaceCount;
                output.quality = {};
                output.quality.freeformFaceCount = freeformCount;
                output.quality.regularHoleToolCount = geometry.holeTools.size();
                error.clear();
                try {
                    auto solid = diagnostics::profileStage("brep.partition_and_cut", [&] {
                        return partitionAndSelect(points, tree, envelope, geometry, spacing, tolerance,
                                                  fuzzy, options, output, error, cancelled);
                    });
                    if (cancelled && cancelled()) return false;
                    if (!solid.IsNull() && finishSolid(solid, geometry, fuzzy, options, error, output.diagnostics) &&
                        checkSolid(solid, observations, tolerance, options, output.quality, error, cancelled)) {
                        output.solid = solid;
                        error.clear();
                        return true;
                    }
                } catch (const Standard_Failure& failure) {
                    error = failure.what();
                }
                if (cancelled && cancelled()) return false;
                if (error.empty()) error = "Could not produce a single closed B-Rep solid";
                output.diagnostics.push_back("Boolean tolerance " + std::to_string(fuzzy) + " mm: " + error);
                if (output.quality.scoredPointCount) break;
            }
            if (geometry.holeTools.empty()) break;
            auto partitionOptions = options;
            partitionOptions.separateRegularHoles = false;
            output.diagnostics.push_back("Retrying with the regular hole walls included in CellsBuilder");
            geometry = makeCuttingGeometry(points, regularized, samples, tree, spacing, tolerance,
                                           partitionOptions, output, cancelled);
        }
    } catch (const Standard_Failure& failure) {
        error = failure.what();
    }
    return false;
}
} // namespace reconstruction
