#include "reconstruction/recognition/CompoundRecognizer.h"
#include "common/Math3.h"
#include "reconstruction/recognition/ThroughHoleRecognizer.h"
#include "reconstruction/recognition/MixedFeatureRecognizer.h"
#include "reconstruction/settings/ModuleConfig.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <unordered_map>

namespace reconstruction {
using math3::add;
using math3::dot;
using math3::scale;
using math3::subtract;
using modeling::Vec3;
using Coord = std::array<double, 3>;

namespace {
bool recognizeOrientedComponents(const PointStore& points, const SurfaceAnalysisResult& analysis,
    CompoundCandidate& output, std::vector<LocalAnalysisFailure>* failures,
    const RecognitionSettings* settings) {
    // Splitting must not silently discard a user's cross-component constraint.
    if (std::any_of(analysis.relations.begin(),analysis.relations.end(),
            [](const auto& relation) { return !relation.automatic; })) return false;
    std::vector<PointId> ids;
    std::unordered_map<PointId, const PointSample*> observations;
    Vec3 lower = points.points().front().position, upper = lower;
    for (const auto& p : points.points()) {
        ids.push_back(p.id); observations.emplace(p.id, &p);
        lower = {std::min(lower.x,p.position.x),std::min(lower.y,p.position.y),std::min(lower.z,p.position.z)};
        upper = {std::max(upper.x,p.position.x),std::max(upper.y,p.position.y),std::max(upper.z,p.position.z)};
    }
    const double distance = analysis.options.detection.plane.distanceThreshold * 2;
    const auto groups = connectedPointComponents(points, ids, distance,
        std::max(distance * 8, math3::length(subtract(upper, lower)) * .02));
    if (groups.size() < 2) return false;
    double covered = 0;
    bool recognized = false;
    for (const auto& group : groups) {
        std::vector<PointSample> samples;
        bool oriented = true;
        for (auto id : group) {
            const auto& p = *observations.at(id);
            samples.push_back(p);
            oriented &= p.normal.has_value() && p.normalOrientationKnown;
        }
        PointStore local(std::move(samples), points.unit());
        CompoundCandidate candidate;
        std::string failure = "Surface component lacks material normal orientation";
        bool accepted = false;
        if (oriented) {
            // Refit finite parameters on this component. Merely slicing IDs
            // would retain another component's cylinder/cone axial bounds.
            SurfaceAnalysisResult indexed;
            accepted = analyzeSurfaces(local, analysis.options, indexed, failure)
                && recognizeParametricSolidFromAnalysis(local, indexed, candidate, failure, nullptr, &local, settings);
        }
        if (!accepted) {
            output.unresolvedPointIds.insert(output.unresolvedPointIds.end(), group.begin(), group.end());
            if (failures) {
                LocalAnalysisFailure diagnostic;
                diagnostic.reason = LocalFailureReason::InsufficientSupport;
                diagnostic.pointIds = group; diagnostic.message = failure;
                failures->push_back(std::move(diagnostic));
            }
            continue;
        }
        recognized = true;
        covered += candidate.surfaceCoverage * group.size();
        output.boxes.insert(output.boxes.end(), candidate.boxes.begin(), candidate.boxes.end());
        output.prisms.insert(output.prisms.end(), candidate.prisms.begin(), candidate.prisms.end());
        output.throughHoles.insert(output.throughHoles.end(), candidate.throughHoles.begin(), candidate.throughHoles.end());
        output.curvedSolids.insert(output.curvedSolids.end(), candidate.curvedSolids.begin(), candidate.curvedSolids.end());
        output.profileBodies.insert(output.profileBodies.end(), candidate.profileBodies.begin(), candidate.profileBodies.end());
        output.projectionPrograms.insert(output.projectionPrograms.end(), candidate.projectionPrograms.begin(), candidate.projectionPrograms.end());
        output.gridCells += candidate.gridCells; output.searchNodes += candidate.searchNodes;
        output.occupancyError = std::max(output.occupancyError, candidate.occupancyError);
    }
    output.surfaceCoverage = output.confidence = covered / points.size();
    return recognized;
}
} // namespace

bool reconstructParametricSolid(const PointStore &points, const PlaneDetectionOptions &planes,
                                const CylinderDetectionOptions &cylinders,
                                CompoundCandidate &output, std::string &error,
                                const std::vector<PrimitiveEvidence>& seeds) {
    output={};
    SurfaceAnalysisResult analysis;
    if (!analyzeSurfaces(points, recognitionAnalysisOptions(points, planes, cylinders), analysis, error, seeds)) return false;
    return recognizeAndRepairParametricSolid(points, analysis, output, error);
}

bool recognizeParametricSolidFromAnalysis(const PointStore& points, const SurfaceAnalysisResult& analysis,
                                        CompoundCandidate& output, std::string& error,
                                        std::vector<LocalAnalysisFailure>* failures, const PointStore* source,
                                        const RecognitionSettings* settings) {
    output = {};
    if (!analysisMatches(points, analysis)) { error = "Stale surface analysis"; return false; }
    const auto& planes = analysis.options.detection.plane;
    if (std::any_of(points.points().begin(), points.points().end(), [](const PointSample& p) {
            return p.normal && !p.normalOrientationKnown;
        })) {
        if (recognizeOrientedComponents(points, analysis, output, failures, settings)) {
            error.clear();
            return true;
        }
        output = {};
        error = "Surface normal directions lack material orientation; provide oriented normals "
                "or additional observations for disconnected surface patches";
        return false;
    }
    CompoundRecognitionOptions options = settings ? settings->compound : CompoundRecognitionOptions{};
    options.distanceTolerance = planes.distanceThreshold;
    for(const auto& e : analysis.surfaces)
        if(const auto* plane=std::get_if<PlaneEvidence>(&e)) options.boundaryEvidence.push_back(*plane);
    std::string mixedError;
    CompoundCandidate mixed;
    const bool hasMixed = recognizeMixedFeatures(points, analysis, mixed, mixedError, failures, source,
        settings);
    if(!hasMixed && mixedError!="No observed thin rectangular host") {
        error=mixedError;output={};return false;
    }
    const bool hasCuts = std::any_of(mixed.prisms.begin(),mixed.prisms.end(),[](const auto& p){return p.subtract;});
    if(hasMixed && (!mixed.curvedSolids.empty() || !mixed.throughHoles.empty() || hasCuts)) {
        output=std::move(mixed);return true;
    }
    if (reconstructCompound(points, options, output, error)) {
        if(hasMixed && output.surfaceCoverage<0.995) output=std::move(mixed);
        return true;
    }
    const auto compoundError = error;
    if(hasMixed) {output=std::move(mixed);error.clear();return true;}
    auto tryCurved = [&](std::string previous) {
        CurvedSolidRecognitionOptions curvedOptions = settings ? settings->curved : CurvedSolidRecognitionOptions{};
        curvedOptions.distanceTolerance = planes.distanceThreshold;
        curvedOptions.randomSeed = planes.randomSeed;

        CurvedSolidCandidate curved;
        output = {};
        if (!recognizeCurvedSolidFromAnalysis(points, analysis, curvedOptions, curved, error)) {
            error = compoundError + "; mixed: " + mixedError + "; box/hole: " + previous + "; curved solid: " + error;
            return false;
        }
        output.curvedSolids.push_back(curved);
        output.surfaceCoverage = curved.surfaceCoverage;
        output.confidence = curved.surfaceCoverage;
        error.clear();
        return true;
    };
    BoxWithThroughHolesCandidate legacy;
    if (!recognizeBoxWithThroughHolesFromAnalysis(points, analysis,
                                        settings ? settings->box : BoxRecognitionOptions{},
                                        settings ? settings->throughHole : ThroughHoleRecognitionOptions{},
                                        legacy, error)) {
        return tryCurved(error);
    }
    const auto &box = legacy.box.primitive;
    const auto x = math3::normalized(box.pose.xDirection),
               z = math3::normalized(box.pose.zDirection), y = math3::cross(z, x);
    auto contains = [&](const Vec3 &point) {
        auto delta = subtract(point, box.pose.origin);
        const Coord local{{dot(delta, x), dot(delta, y), dot(delta, z)}};
        if (local[0] < 0 || local[0] > box.sizeX || local[1] < 0 || local[1] > box.sizeY ||
            local[2] < 0 || local[2] > box.sizeZ)
            return false;
        for (const auto &hole : legacy.throughHoles) {
            const auto direction = math3::normalized(hole.primitive.axisDirection);
            auto d = subtract(point, hole.primitive.axisPoint);
            auto radial = subtract(d, scale(direction, dot(d, direction)));
            if (dot(radial, radial) < hole.primitive.radius * hole.primitive.radius)
                return false;
        }
        return true;
    };
    std::size_t support = 0;
    for (const auto &p : points.points())
        if (p.normal) {
            const auto shift = scale(math3::normalized(*p.normal), planes.distanceThreshold * 4);
            support += contains(subtract(p.position, shift)) && !contains(add(p.position, shift));
        }
    const double coverage = double(support) / points.size();
    // Curved fused scan normals have larger boundary uncertainty than planar
    // normals. Still require the whole-body hypothesis to explain most points.
    if (coverage < 0.80) {
        return tryCurved("single-box boundary coverage=" + std::to_string(coverage));
    }
    output = {};
    output.boxes.push_back(legacy.box);
    output.throughHoles = legacy.throughHoles;
    output.surfaceCoverage = coverage;
    output.confidence = legacy.confidence;
    output.minimumBoxCountProven = true;
    error.clear();
    return true;
}
} // namespace reconstruction
