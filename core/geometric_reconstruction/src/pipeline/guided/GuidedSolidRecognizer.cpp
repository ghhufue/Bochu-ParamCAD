#include "reconstruction/pipeline/ParametricReconstruction.h"
#include "reconstruction/recognition/RecognitionAnalysisOptions.h"
#include "reconstruction/surface/SurfaceSupportValidation.h"
#include "reconstruction/solid_inference/SolidInference.h"
#include "detail/GuidedInternal.h"
#include "reconstruction/settings/ModuleConfig.h"
#include "reconstruction/data/StageProfile.h"
#include <algorithm>
#include <map>
#include <set>

namespace reconstruction {
using namespace detail::guided;

namespace detail::guided {
bool fitTemplateRegions(const PointStore& points, const PlaneDetectionOptions& planes,
    const CylinderDetectionOptions& cylinders, const std::vector<TemplateHint>& hints,
    TemplateRegions& output, std::string& error, const std::function<bool()>& cancelled,
    const GuidedRecognitionOptions& options) {
    error.clear();
    output = {};
    output.adjustedPoints = points.points();
    auto& samples = output.adjustedPoints;
    std::map<PointId, std::size_t> indices;
    for (std::size_t i = 0; i < samples.size(); ++i)
        indices[samples[i].id] = i;
    std::set<PointId> assigned;
    auto& seeds = output.seeds;
    auto& closed = output.closed;
    for (const auto& hint : hints) {
        if (cancelled && cancelled()) {
            error = "Cancelled";
            return false;
        }
        output.failedRegionId = hint.regionId;
        std::vector<PointSample> selected;
        for (auto id : hint.points) {
            if (!assigned.insert(id).second) {
                error = "Overlapping template groups";
                return false;
            }
            const auto found = indices.find(id);
            if (found != indices.end())
                selected.push_back(samples[found->second]);
        }
        if (selected.size() < options.minimumTemplatePoints) {
            error = "At least " + std::to_string(options.minimumTemplatePoints) +
                    " valid retained points are required for a template";
            return false;
        }
        const PointStore subset(selected, points.unit());
        TemplateFit fit;
        fit.regionId = hint.regionId;
        if (hint.kind == TemplateKind::Box) {
            auto settings = planes;
            settings.minimumSupportPoints = options.templateBoxMinimumSupportPoints;
            settings.maximumPlanes = 6;
            BoxCandidate box;
            if (!reconstructBox(subset, settings, {}, box, error)) {
                // A local patch cannot determine all dimensions by itself.
                // Complete it only from actual observations of the whole cloud.
                CompoundCandidate context;
                if (!reconstructParametricSolid(points, planes, cylinders, context, error))
                    return false;
                const auto found =
                    std::find_if(context.boxes.begin(), context.boxes.end(), [&](const auto& c) {
                        return std::count_if(selected.begin(), selected.end(), [&](const auto& p) {
                                   return onBox(c.primitive, p,
                                       planes.distanceThreshold * options.boundaryToleranceMultiplier);
                               }) >= selected.size() * options.minimumSupportFraction;
                    });
                if (found == context.boxes.end()) {
                    error = "No observed box boundary supports this template patch";
                    return false;
                }
                box = *found;
            }
            fit.box = box;
            for (const auto& plane : box.sourceSurfaces)
                seeds.push_back(plane);
        } else {
            SurfaceAnalysisOptions localOptions;
            const auto supportSettings=loadAnalysisSettings();localOptions.supportValidation=supportSettings.supportValidation;localOptions.supportMeshing=supportSettings.meshing;
            localOptions.familyDetectionWorkerCount = supportSettings.familyDetectionWorkerCount;
            localOptions.detection = detectionFor(hint.kind, planes.distanceThreshold, options);
            localOptions.relations.distanceTolerance =
                planes.distanceThreshold * options.relationToleranceMultiplier;
            SurfaceAnalysisResult proposals;
            auto proposalOptions=localOptions;
            if(proposalOptions.supportValidation.mode==SupportValidationMode::Enforce)
                proposalOptions.supportValidation.mode=SupportValidationMode::Report;
            if (!analyzeSurfaces(subset, proposalOptions, proposals, error, {}, cancelled))
                return false;
            if (!proposals.surfaces.empty()) {
                fit.surface = *std::max_element(
                    proposals.surfaces.begin(), proposals.surfaces.end(),
                    [](const auto& a, const auto& b) { return supportSize(a) < supportSize(b); });
            }
            if (hint.kind != TemplateKind::Plane) {
                CurvedSolidRecognitionOptions settings;
                settings.distanceTolerance = planes.distanceThreshold;
                settings.requiredKind = static_cast<PrimitiveKind>(hint.kind);
                if (fit.surface)
                    settings.seeds.push_back(*fit.surface);
                CurvedSolidCandidate solid;
                std::string incomplete;
                SurfaceAnalysisResult validatedSubset;
                if(!indexSurfaceEvidence(subset,proposals.surfaces,localOptions,validatedSubset,error,{},cancelled))return false;
                bool complete = recognizeCurvedSolidFromAnalysis(subset, validatedSubset, settings, solid, incomplete);
                if (!complete && fit.surface) {
                    const auto expanded =
                        expandCurvedSupport(*fit.surface, samples, planes.distanceThreshold, options);
                    if (expanded.size() >= options.minimumExpandedPoints &&
                        expanded.size() > supportSize(*fit.surface)) {
                        PointStore local(expanded, points.unit());
                        SurfaceAnalysisResult expandedAnalysis;
                        if (analyzeSurfaces(local, localOptions, expandedAnalysis, incomplete,
                                            {*fit.surface}, cancelled))
                            complete = recognizeCurvedSolidFromAnalysis(local, expandedAnalysis, settings, solid, incomplete);
                    }
                }
                if (complete) {
                    const auto same =
                        std::find_if(closed.begin(), closed.end(), [&](const auto& known) {
                            return std::count_if(
                                       selected.begin(), selected.end(), [&](const auto& p) {
                                           return matchesCurvedBoundary(
                                               known.primitive, p,
                                               planes.distanceThreshold * options.boundaryToleranceMultiplier,
                                               options.boundaryNormalCosine);
                                       }) >= selected.size() * options.minimumSupportFraction;
                        });
                    if (same != closed.end())
                        solid = *same;
                    else
                        closed.push_back(solid);
                    fit.solid = solid;
                }
            }
            if (!fit.solid && (!fit.surface ||
                supportSize(*fit.surface) < selected.size() * options.minimumSurfaceSupportFraction)) {
                error = "Selected points do not support the requested template; correct membership "
                        "or add observations (region=" + std::to_string(hint.regionId) +
                        ", kind=" + std::to_string(static_cast<int>(hint.kind)) +
                        ", selected=" + std::to_string(selected.size()) + ")";
                return false;
            }
            if (fit.surface)
                seeds.push_back(*fit.surface);
        }
        // The user's planar equation guides normal/frame recovery. Preserve
        // outward orientation; the original document's observations stay intact.
        std::vector<PlaneEvidence> fixed;
        if (fit.box)
            fixed = fit.box->sourceSurfaces;
        if (fit.surface)
            if (const auto* p = std::get_if<PlaneEvidence>(&*fit.surface))
                fixed.push_back(*p);
        for (const auto& p : fixed)
            for (auto id : p.supportPointIds) {
                auto& sample = samples[indices.at(id)];
                auto normal = p.plane.normal;
                if (sample.normal && math3::dot(normal, *sample.normal) < 0)
                    normal = math3::scale(normal, -1);
                sample.normal = normal;
                const double offset = math3::dot(p.plane.normal, sample.position) + p.plane.offset;
                sample.position =
                    math3::subtract(sample.position, math3::scale(p.plane.normal, offset));
            }
        output.fits.push_back(std::move(fit));
    }
    output.failedRegionId = 0;
    if (cancelled && cancelled()) {
        error = "Cancelled";
        return false;
    }
    return true;
}
} // namespace detail::guided

bool reconstructGuidedSolid(const PointStore& points, const PlaneDetectionOptions& planes,
                            const CylinderDetectionOptions& cylinders,
                            const std::vector<TemplateHint>& hints, GuidedSolidResult& output,
                            std::string& error, const std::function<bool()>& cancelled,
                            const SurfaceAnalysisResult* sharedAnalysis,
                            const std::vector<SurfaceRelationHint>& relationHints, const PointStore* source,
                            const RecognitionSettings* settings, const SurfaceAnalysisOptions* conventionalOptions,
                            const GuidedRecognitionProgress& progress)
{
    output = {};
    const GuidedRecognitionOptions& guided = settings
        ? settings->guided : GuidedRecognitionOptions{};
    if (hints.empty()) {
        const auto options = conventionalOptions ? *conventionalOptions : recognitionAnalysisOptions(points, planes, cylinders,
            settings ? settings->analysis : RecognitionAnalysisSettings{});
        if (sharedAnalysis && analysisInputMatches(points, *sharedAnalysis) &&
            analysisSettingsMatch(options,sharedAnalysis->options) &&
            (!source || !sharedAnalysis->baseFingerprint || sharedAnalysis->recoverySourceFingerprint==pointStoreFingerprint(*source)) &&
            (!source || !sharedAnalysis->sourceSupportValidation.pointFingerprint ||
             sharedAnalysis->sourceSupportValidation.pointFingerprint==pointStoreFingerprint(*source))) {
            output.analysis = *sharedAnalysis;
            output.analysisReused = true;
            if (!conventionalOptions) reportSurfaceAnalysis(output.analysis, progress.surfaces);
        }
        else if (!diagnostics::profileStage("recognition.surface_analysis", [&] {
            return analyzeSurfaces(points, options, output.analysis, error, {}, cancelled, progress.surfaces);
        })) return false;
        if (source && !diagnostics::profileStage("recognition.source_revalidation", [&] {
            return revalidateSurfaceSupportOnSource(points, *source, output.analysis, error, cancelled, nullptr, progress.surfaces);
        })) return false;
        return diagnostics::profileStage("recognition.solid_inference", [&] {
            PointStore materialized({}, points.unit());
            const PointStore* working = &points;
            if (output.analysis.baseFingerprint) {
                if (!materializeAnalysisPoints(points, output.analysis, materialized, error)) return false;
                working = &materialized;
            }
            if (!relationHints.empty() &&
                !applySurfaceRelationHints(*working, relationHints, output.analysis, error)) return false;
            SurfaceScene scene;
            if (!exportSurfaceScene(*working, output.analysis, scene, error, source)) return false;
            SolidInferenceResult inferred;
            if (!inferSolids(scene, inferred, error, {}, working, progress.solid) || inferred.candidates.empty()) return false;
            std::vector<PointSample> screeningPoints;
            const auto screeningCount=std::min<std::size_t>(256,working->size());
            screeningPoints.reserve(screeningCount);
            for(std::size_t i=0;i<screeningCount;++i)
                screeningPoints.push_back(working->points()[i*working->size()/screeningCount]);
            const PointStore screeningStore(std::move(screeningPoints),working->unit());
            auto screeningScene=scene;
            screeningScene.header.sourceFingerprint=0;
            std::vector<std::pair<SolidCandidate,SolidCandidate>> screened;
            std::vector<std::string> screeningFailures;
            const auto describeProposal = [](const SolidCandidate& proposal) {
                std::string label = "candidate patches=";
                for (std::size_t i = 0; i < proposal.supportingPatches.size(); ++i) {
                    if (i) label += ",";
                    label += std::to_string(proposal.supportingPatches[i]);
                }
                label += "; operations=" + std::to_string(proposal.featureProgram.size());
                if (proposal.curvedSolid) label += "; curved=1";
                if (proposal.boundaryBody) label += "; boundary=1";
                return label;
            };
            const auto scoringTolerance = [&](const SolidCandidate& candidate) {
                double value = output.analysis.options.detection.plane.distanceThreshold;
                for (const auto& patch : scene.patches)
                    if (std::find(candidate.supportingPatches.begin(), candidate.supportingPatches.end(), patch.id) !=
                        candidate.supportingPatches.end() && std::isfinite(patch.spacingMm))
                        value = std::max(value, patch.spacingMm);
                return value;
            };
            SolidScoringOptions screeningOptions;
            for (const auto& proposal : inferred.candidates) {
                if (cancelled && cancelled()) { error = "Cancelled"; return false; }
                if (proposal.status == SolidCandidateStatus::Rejected) continue;
                SolidCandidate candidate;
                std::string scoringError;
                if (scoreSolidCandidate(proposal, screeningScene, screeningStore,
                        scoringTolerance(proposal),
                        candidate, scoringError, screeningOptions, progress.solid) && candidate.status == SolidCandidateStatus::Accepted)
                    screened.emplace_back(proposal,std::move(candidate));
                else if (!scoringError.empty()) screeningFailures.push_back(scoringError);
                else if (!candidate.diagnostics.empty()) {
                    auto failure = describeProposal(proposal) + ": " + candidate.diagnostics.back();
                    if (candidate.occupancyConflictCount)
                        failure += " (occupancy_conflicts=" +
                            std::to_string(candidate.occupancyConflictCount) + ")";
                    screeningFailures.push_back(std::move(failure));
                }
                else screeningFailures.push_back("candidate rejected");
            }
            std::stable_sort(screened.begin(),screened.end(),[](const auto& a,const auto& b){
                return a.second.score<b.second.score;
            });
            if(screened.empty()) {
                error="No candidate passed working-observation screening";
                for(const auto& failure:screeningFailures) error += ": " + failure;
                return false;
            }
            std::vector<std::pair<SolidCandidate,SolidCandidate>> shortlisted;
            for(auto& item:screened) {
                if(shortlisted.empty()||item.second.score-screened.front().second.score<.05)
                    shortlisted.push_back(std::move(item));
                if(shortlisted.size()==3)break;
            }
            const auto& evidence = source ? *source : *working;
            std::vector<SolidCandidate> scored;
            SolidScoringOptions finalOptions;
            for(const auto& item:shortlisted) {
                if(cancelled && cancelled()) { error="Cancelled"; return false; }
                SolidCandidate candidate;
                std::string scoringError;
                if(scoreSolidCandidate(item.first,scene,evidence,
                    scoringTolerance(item.first),candidate,scoringError,finalOptions,progress.solid)&&
                    candidate.status==SolidCandidateStatus::Accepted)
                    scored.push_back(std::move(candidate));
            }
            const auto* best = selectSolidCandidate(scored, error);
            if (!best && progress.solid) progress.solid({});
            if (!best) {
                if (error == "No candidate passed B-Rep and evidence scoring") {
                    error += ": ";
                    for (const auto& item : shortlisted) {
                        SolidCandidate diagnostics;
                        std::string detail;
                        scoreSolidCandidate(item.first, scene, evidence,
                            scoringTolerance(item.first),
                            diagnostics, detail, finalOptions);
                        if (!diagnostics.diagnostics.empty()) error += diagnostics.diagnostics.back() + "; ";
                    }
                }
                return false;
            }
            const auto& selected = *best;
            if (selected.curvedSolid) {
                output.candidate.curvedSolids.push_back(*selected.curvedSolid);
            } else if (!selected.featureProgram.empty()) {
                output.candidate.projectionPrograms.push_back(selected.featureProgram);
            } else {
                output.candidate.profileBodies.push_back(*selected.boundaryBody);
            }
            output.candidate.unresolvedPointIds=selected.unresolvedPoints;
            output.candidate.surfaceCoverage=selected.observationCoverage;
            output.candidate.confidence=std::clamp(1-selected.score,0.0,1.0);
            error.clear();
            return true;
        });
    }
    TemplateRegions templates;
    const bool fitted = fitTemplateRegions(points, planes, cylinders, hints, templates, error,
                                           cancelled, guided);
    output.fits = std::move(templates.fits);
    output.failedRegionId = templates.failedRegionId;
    if (!fitted) return false;
    auto& seeds = templates.seeds;
    const auto& closed = templates.closed;
    // A complete manually identified curved component is fixed first. Its
    // remaining observed boundary is consumed, and automatic reconstruction
    // operates on the remaining material rather than refitting that component.
    std::vector<PointSample> remaining;
    for (const auto& p : templates.adjustedPoints) {
        const bool consumed = std::any_of(closed.begin(), closed.end(), [&](const auto& solid) {
            return matchesCurvedBoundary(solid.primitive, p,
                planes.distanceThreshold * guided.boundaryToleranceMultiplier,
                guided.boundaryNormalCosine);
        });
        if (!consumed)
            remaining.push_back(p);
    }
    if (!remaining.empty()) {
        std::set<PointId> remainingIds;
        for (const auto& p : remaining)
            remainingIds.insert(p.id);
        seeds.erase(
            std::remove_if(seeds.begin(), seeds.end(),
                           [&](auto& seed) {
                               return std::visit(
                                   [&](auto& evidence) {
                                       auto& ids = evidence.supportPointIds;
                                       ids.erase(std::remove_if(ids.begin(), ids.end(),
                                                                [&](auto id) {
                                                                    return !remainingIds.count(id);
                                                                }),
                                                 ids.end());
                                       return ids.size() < guided.minimumSeedPoints;
                                   },
                                   seed);
                           }),
            seeds.end());
        PointStore local(remaining, points.unit());
        if (!analyzeSurfaces(local, recognitionAnalysisOptions(local, planes, cylinders,
                                settings ? settings->analysis : RecognitionAnalysisSettings{}),
                             output.analysis, error, seeds, cancelled, progress.surfaces) ||
            !recognizeParametricSolidFromAnalysis(local, output.analysis, output.candidate, error, nullptr, nullptr, settings)) return false;
    }
    output.candidate.curvedSolids.insert(output.candidate.curvedSolids.end(), closed.begin(),
                                         closed.end());
    if (output.candidate.boxes.empty() && output.candidate.prisms.empty() &&
        output.candidate.curvedSolids.empty()) {
        error = "No supported material";
        return false;
    }
    if (progress.guidedSolid) progress.guidedSolid(output.candidate);
    if (!verifyHints(points, hints, output.candidate, output.fits, planes.distanceThreshold,
                     output.failedRegionId, error, cancelled, guided)) {
        if (progress.solid) progress.solid({});
        return false;
    }
    output.candidate.surfaceCoverage =
        (remaining.size() * output.candidate.surfaceCoverage + (points.size() - remaining.size())) /
        points.size();
    output.candidate.confidence = output.candidate.surfaceCoverage;
    if(!relationHints.empty()) {
        // Guided completion may remove a fully observed independent component.
        // Relationship endpoints still refer to the original full snapshot.
        if(!analyzeSurfaces(points,recognitionAnalysisOptions(points,planes,cylinders,
                settings ? settings->analysis : RecognitionAnalysisSettings{}),
                output.analysis,error,{},cancelled,progress.surfaces) ||
           !applySurfaceRelationHints(points,relationHints,output.analysis,error)) return false;
    }
    error.clear();
    return true;
}
} // namespace reconstruction
