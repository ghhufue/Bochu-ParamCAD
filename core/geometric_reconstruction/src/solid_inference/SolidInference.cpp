#include "reconstruction/solid_inference/SolidInference.h"

#include <BRepGProp.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopAbs_Orientation.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <set>

namespace reconstruction {
namespace {
bool scoreCandidateShape(const SolidCandidate& candidate,const SurfaceScene& scene,
                         const PointStore& observations,double tolerance,const TopoDS_Shape& shape,
                         SolidCandidate& scored,std::string& error,const SolidScoringOptions& options);

double median(std::vector<double> values) {
    if(values.empty())return 0;
    const auto middle=values.begin()+values.size()/2;
    std::nth_element(values.begin(),middle,values.end());
    return *middle;
}

bool near(modeling::Vec3 first, modeling::Vec3 second, double tolerance) {
    return std::hypot(first.x-second.x, first.y-second.y, first.z-second.z) <= tolerance;
}

bool shareObservedEdge(const ScenePatch& first, const ScenePatch& second, double tolerance) {
    for (const auto& firstLoop : first.loops) {
        if (!firstLoop.closed) continue;
        for (const auto& secondLoop : second.loops) {
            if (!secondLoop.closed) continue;
            for (std::size_t i = 0; i < firstLoop.vertices.size(); ++i) {
                const auto firstA = firstLoop.vertices[i];
                const auto firstB = firstLoop.vertices[(i+1)%firstLoop.vertices.size()];
                if (near(firstA, firstB, tolerance)) continue;
                for (std::size_t j = 0; j < secondLoop.vertices.size(); ++j) {
                    const auto secondA = secondLoop.vertices[j];
                    const auto secondB = secondLoop.vertices[(j+1)%secondLoop.vertices.size()];
                    if ((near(firstA, secondA, tolerance) && near(firstB, secondB, tolerance)) ||
                        (near(firstA, secondB, tolerance) && near(firstB, secondA, tolerance))) return true;
                }
            }
        }
    }
    return false;
}

double dot(modeling::Vec3 first, modeling::Vec3 second) {
    return first.x*second.x+first.y*second.y+first.z*second.z;
}

modeling::Vec3 add(modeling::Vec3 first, modeling::Vec3 second) {
    return {first.x+second.x,first.y+second.y,first.z+second.z};
}

double length(modeling::Vec3 value) {
    return std::sqrt(dot(value,value));
}

bool rebuildFeatureProgram(const std::vector<SolidFeatureOperation>& program,
                           modeling::ModelingCore& output,std::string& error,
                           double minimumVolumeChange=1e-9) {
    if(program.empty()||program.front().operation!=modeling::ExtrudeOperation::NewBody) {
        error="Feature program must begin with NewBody";
        return false;
    }
    auto staged=output;
    double previousVolume=0;
    modeling::FeatureId targetBody=modeling::kInvalidFeatureId;
    for(const auto& operation:program) {
        if(operation.entities.empty()||!std::isfinite(operation.depth)||operation.depth<=0) {
            error="Feature program contains an invalid sketch or extrusion depth";
            return false;
        }
        modeling::SketchFeatureParams sketch;
        sketch.plane=operation.plane;
        sketch.entities=operation.entities;
        modeling::AddFeatureCommand addSketch{sketch,true};
        const auto sketchResult=staged.execute(addSketch);
        if(!sketchResult.success) {error=sketchResult.error;return false;}
        modeling::ExtrudeFeatureParams extrusion;
        extrusion.sketchId=sketchResult.featureId;
        extrusion.depth=operation.depth;
        extrusion.reverse=operation.reverse;
        extrusion.operation=operation.operation;
        if (operation.operation != modeling::ExtrudeOperation::NewBody)
            extrusion.targetBody=targetBody;
        modeling::AddFeatureCommand addExtrusion{extrusion,true};
        const auto extrusionResult=staged.execute(addExtrusion);
        if(!extrusionResult.success) {error=extrusionResult.error;return false;}
        if(!staged.rebuild()||staged.bodyShape().IsNull()) {
            error=staged.lastError().empty()?"Feature program produced no valid body":staged.lastError();
            return false;
        }
        GProp_GProps properties;
        BRepGProp::VolumeProperties(staged.bodyShape(),properties);
        const double volume=properties.Mass();
        if(!std::isfinite(volume)||volume<=0) {
            error="Feature program produced a degenerate body";
            return false;
        }
        if(operation.operation==modeling::ExtrudeOperation::Join||
           operation.operation==modeling::ExtrudeOperation::Cut) {
            const double change=volume-previousVolume;
            if((operation.operation==modeling::ExtrudeOperation::Join&&change<=minimumVolumeChange)||
               (operation.operation==modeling::ExtrudeOperation::Cut&&change>=-minimumVolumeChange)) {
                error=operation.operation==modeling::ExtrudeOperation::Join
                    ?"Join operation does not add material to the host body"
                    :"Cut operation does not remove material from the host body";
                return false;
            }
        }
        previousVolume=volume;
        targetBody=extrusionResult.featureId;
    }
    output.adoptPrepared(std::move(staged));
    error.clear();
    return true;
}

bool rebuildCandidate(const SolidCandidate& candidate, modeling::ModelingCore& model,
                      std::string& error, double minimumVolumeChange) {
    if (!candidate.featureProgram.empty())
        return rebuildFeatureProgram(candidate.featureProgram, model, error, minimumVolumeChange);
    if (candidate.curvedSolid) {
        modeling::ModelPatch patch;
        std::visit([&](const auto& primitive) {
            patch.commands.push_back(modeling::AddFeatureCommand{primitive, true});
        }, candidate.curvedSolid->primitive);
        patch.rebuild = true;
        const auto result = model.apply(patch);
        error = result.error;
        if (result.success && !model.bodyShape().IsNull()) return true;
        if (error.empty()) error = "Curved primitive failed OCCT body validation";
        return false;
    }
    if (!candidate.boundaryBody) {
        error = "Candidate has no rebuildable body or feature program";
        return false;
    }
    modeling::ModelPatch patch;
    patch.commands.push_back(modeling::AddFeatureCommand{*candidate.boundaryBody, true});
    patch.rebuild = true;
    const auto result = model.apply(patch);
    error = result.error;
    if (result.success && !model.bodyShape().IsNull()) return true;
    if (error.empty()) error = "Candidate failed OCCT body validation";
    return false;
}

double shapeVolume(const TopoDS_Shape& shape) {
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return properties.Mass();
}

bool equivalentMaterial(const TopoDS_Shape& first, const TopoDS_Shape& second) {
    try {
        const double firstVolume = shapeVolume(first);
        const double secondVolume = shapeVolume(second);
        const double tolerance = 1e-7 * std::max(firstVolume, secondVolume);
        if (tolerance <= 0 || std::abs(firstVolume - secondVolume) > tolerance) return false;
        BRepAlgoAPI_Common overlap(first, second);
        return overlap.IsDone() && !overlap.Shape().IsNull() &&
            std::max(firstVolume, secondVolume) - shapeVolume(overlap.Shape()) <= tolerance;
    } catch (const Standard_Failure&) {
        return false;
    }
}

bool independentSolidsDoNotOverlap(const TopoDS_Shape& shape, double volumeTolerance) {
    try {
        std::vector<TopoDS_Shape> solids;
        for (TopExp_Explorer solid(shape, TopAbs_SOLID); solid.More(); solid.Next())
            solids.push_back(solid.Current());
        for (std::size_t i = 0; i < solids.size(); ++i)
            for (std::size_t j = i + 1; j < solids.size(); ++j) {
                BRepAlgoAPI_Common overlap(solids[i], solids[j]);
                overlap.SetRunParallel(false);
                overlap.Build();
                if (!overlap.IsDone()) return false;
                if (!overlap.Shape().IsNull() && shapeVolume(overlap.Shape()) > volumeTolerance)
                    return false;
            }
        return true;
    } catch (const Standard_Failure&) {
        return false;
    }
}

bool sharesSupportingPatch(const SolidCandidate& first,const SolidCandidate& second) {
    for(const auto id:first.supportingPatches)
        if(std::find(second.supportingPatches.begin(),second.supportingPatches.end(),id)!=second.supportingPatches.end())
            return true;
    return false;
}

bool sameHypothesis(const SolidCandidate& first,const SolidCandidate& second) {
    if(first.supportingPatches!=second.supportingPatches||first.featureProgram.size()!=second.featureProgram.size())
        return false;
    for(std::size_t i=0;i<first.featureProgram.size();++i)
        if(first.featureProgram[i].operation!=second.featureProgram[i].operation)return false;
    return true;
}
}

bool scoreSolidCandidate(const SolidCandidate& candidate,const SurfaceScene& scene,
                         const PointStore& observations,double tolerance,
                         SolidCandidate& scored,std::string& error,
                         const SolidScoringOptions& options,const SolidPreview& preview) {
    scored=candidate;
    if(candidate.status==SolidCandidateStatus::Rejected) {
        error="Rejected solid candidates cannot enter observation scoring";
        return false;
    }
    if(candidate.analysisFingerprint!=scene.header.analysisFingerprint) {
        error="SnapshotMismatch: candidate belongs to another analysis";return false;
    }
    if(scene.header.sourceFingerprint &&
       pointStoreFingerprint(observations)!=scene.header.sourceFingerprint) {
        error="SnapshotMismatch: scoring observations differ from the SurfaceScene source snapshot";
        return false;
    }
    if(observations.unit()!=LengthUnit::Millimeter||!std::isfinite(tolerance)||tolerance<=0) {
        error="Scoring requires millimetre observations and positive tolerance";return false;
    }
    if(!std::isfinite(options.minimumObservationCoverage)||options.minimumObservationCoverage<0||
       options.minimumObservationCoverage>1||!std::isfinite(options.maximumP95ToleranceMultiplier)||
       options.maximumP95ToleranceMultiplier<=0||!std::isfinite(options.maximumOccupancyConflictFraction)||
       options.maximumOccupancyConflictFraction<0||options.maximumOccupancyConflictFraction>1) {
        error="Invalid solid candidate scoring options";return false;
    }
    modeling::ModelingCore core;
    if (!rebuildCandidate(candidate, core, error, tolerance*tolerance*tolerance)) {
        scored.status = SolidCandidateStatus::Rejected;
        scored.diagnostics.push_back(error);
        return false;
    }
    if (preview) preview(core.bodyShape());
    const bool valid = scoreCandidateShape(candidate,scene,observations,tolerance,core.bodyShape(),scored,error,options);
    if (preview && (!valid || scored.status != SolidCandidateStatus::Accepted)) preview({});
    return valid;
}

const SolidCandidate* selectSolidCandidate(const std::vector<SolidCandidate>& scored,
                                          std::string& error) {
    error.clear();
    std::vector<const SolidCandidate*> ranked;
    for (const auto& candidate : scored)
        if (candidate.status == SolidCandidateStatus::Accepted) ranked.push_back(&candidate);
    if (ranked.empty()) {
        error = "No candidate passed B-Rep and evidence scoring";
        return nullptr;
    }
    std::stable_sort(ranked.begin(), ranked.end(), [](const auto* first, const auto* second) {
        return first->score < second->score;
    });
    constexpr double ambiguityMargin = .05;
    const auto* best = ranked.front();
    const auto* selected = best;
    modeling::ModelingCore reference;
    for (std::size_t i = 1; i < ranked.size(); ++i) {
        const auto* other = ranked[i];
        if (other->score - best->score >= ambiguityMargin) break;
        if (reference.bodyShape().IsNull() && !rebuildCandidate(*best, reference, error, 0)) return nullptr;
        modeling::ModelingCore alternative;
        if (!rebuildCandidate(*other, alternative, error, 0)) return nullptr;
        if (!equivalentMaterial(reference.bodyShape(), alternative.bodyShape())) {
            const auto describe = [](const SolidCandidate& candidate) {
                std::string value="[patches=";
                for(std::size_t index=0;index<candidate.supportingPatches.size();++index) {
                    if(index)value+=',';
                    value+=std::to_string(candidate.supportingPatches[index]);
                }
                value+="; operations=";
                for(std::size_t index=0;index<candidate.featureProgram.size();++index) {
                    if(index)value+=',';
                    value+=std::to_string(static_cast<int>(candidate.featureProgram[index].operation));
                }
                if(candidate.curvedSolid)
                    value += "; curved=" + std::to_string(candidate.curvedSolid->primitive.index());
                return value+"]";
            };
            error = "Competing candidates explain the observations with distinct material (scores="+
                std::to_string(best->score)+"/"+std::to_string(other->score)+
                ", coverage="+std::to_string(best->observationCoverage)+"/"+
                std::to_string(other->observationCoverage)+", features="+
                std::to_string(best->featureProgram.size())+"/"+
                std::to_string(other->featureProgram.size())+", candidates="+
                describe(*best)+"/"+describe(*other)+")";
            return nullptr;
        }
        if (!other->featureProgram.empty() && (selected->featureProgram.empty() ||
            other->featureProgram.size() < selected->featureProgram.size())) selected = other;
    }
    return selected;
}

namespace {
bool scoreCandidateShape(const SolidCandidate& candidate,const SurfaceScene& scene,
                         const PointStore& observations,double tolerance,const TopoDS_Shape& shape,
                         SolidCandidate& scored,std::string& error,const SolidScoringOptions& options) {
    std::vector<TopoDS_Shape> solids;
    for (TopExp_Explorer solid(shape, TopAbs_SOLID); solid.More(); solid.Next())
        solids.push_back(solid.Current());
    std::vector<double> distances;
    scored.unresolvedPoints.clear();
    scored.relationComparisons=0;
    std::size_t conflicts=0,normalComparisons=0,inliers=0;
    double normalLoss=0;
    std::unordered_map<PointId,std::vector<PatchId>> patchesByPoint;
    std::set<PatchId> candidatePatches(candidate.supportingPatches.begin(),candidate.supportingPatches.end());
    for(const auto& scenePatch:scene.patches)if(candidatePatches.count(scenePatch.id))
        for(const auto id:scenePatch.supportPointIds)patchesByPoint[id].push_back(scenePatch.id);
    std::unordered_map<PatchId,modeling::Vec3> modelNormalSums;
    std::unordered_map<PatchId,std::size_t> modelNormalCounts;
    TopoDS_Compound boundary;
    BRep_Builder builder;
    builder.MakeCompound(boundary);
    for(TopExp_Explorer face(shape,TopAbs_FACE);face.More();face.Next())
        builder.Add(boundary,face.Current());
    BRepExtrema_DistShapeShape distance;
    distance.LoadS2(boundary);
    for(const auto& point:observations.points()) {
        const auto vertex=BRepBuilderAPI_MakeVertex(gp_Pnt(point.position.x,point.position.y,point.position.z)).Vertex();
        distance.LoadS1(vertex);distance.Perform();
        if(!distance.IsDone()||!distance.NbSolution()) {error="B-Rep distance evaluation failed";return false;}
        const auto d=distance.Value();distances.push_back(d);
        if(d>tolerance*2)scored.unresolvedPoints.push_back(point.id);
        else ++inliers;
        if(point.normal&&point.normalOrientationKnown&&d<=tolerance*2&&distance.SupportTypeShape2(1)==BRepExtrema_IsInFace) {
            const auto face=TopoDS::Face(distance.SupportOnShape2(1));double u=0,v=0;
            distance.ParOnFaceS2(1,u,v);BRepAdaptor_Surface surface(face);gp_Pnt p;gp_Vec du,dv;
            surface.D1(u,v,p,du,dv);auto n=du.Crossed(dv);
            if(n.SquareMagnitude()>1e-24) {
                const auto modelNormal=modeling::Vec3{n.Normalized().X(),n.Normalized().Y(),n.Normalized().Z()};
                double alignment=dot(modelNormal,*point.normal);
                if(face.Orientation()==TopAbs_REVERSED)alignment=-alignment;
                normalLoss+=1-std::clamp(alignment,-1.0,1.0);++normalComparisons;
                const auto matched=patchesByPoint.find(point.id);
                if(matched!=patchesByPoint.end())for(const auto patchId:matched->second) {
                    modelNormalSums[patchId]=add(modelNormalSums[patchId],modelNormal);
                    ++modelNormalCounts[patchId];
                }
            }
        }
        // OCC classifiers can report a point just outside a fitted face as
        // inside when the face tolerance is comparable to the fitting error.
        // Reserve occupancy conflicts for points clearly deep in material.
        if(d>tolerance*10) {
            const gp_Pnt position(point.position.x,point.position.y,point.position.z);
            for (const auto& solid : solids) {
                BRepClass3d_SolidClassifier classifier(solid);
                classifier.Perform(position,tolerance);
                if(classifier.State()==TopAbs_IN) {
                    ++conflicts;
                    break;
                }
            }
        }
    }
    if(distances.empty()) {error="No observations available for candidate scoring";return false;}
    std::sort(distances.begin(),distances.end());
    scored.distanceMedianMm=median(distances);
    scored.distanceP95Mm=distances[std::size_t(.95*(distances.size()-1))];
    scored.distanceError=(scored.distanceMedianMm+scored.distanceP95Mm*2)/(3*tolerance);
    scored.observationCoverage=double(inliers)/distances.size();
    scored.normalError=normalComparisons?normalLoss/normalComparisons:0;
    scored.normalComparisons=normalComparisons;
    scored.occupancyConflictCount=conflicts;
    double relationLoss=0,relationWeight=0;
    for(const auto& relation:scene.relations) {
        if(!candidatePatches.count(relation.first)||!candidatePatches.count(relation.second))continue;
        const auto firstCount=modelNormalCounts[relation.first],secondCount=modelNormalCounts[relation.second];
        if(!firstCount||!secondCount)continue;
        const auto firstNormal=modelNormalSums[relation.first];
        const auto secondNormal=modelNormalSums[relation.second];
        const auto firstLength=length(firstNormal),secondLength=length(secondNormal);
        if(firstLength<1e-9||secondLength<1e-9)continue;
        const double alignment=std::clamp(std::abs(dot(firstNormal,secondNormal)/(firstLength*secondLength)),0.0,1.0);
        double loss=0;
        if(relation.kind==SurfaceRelationKind::Parallel||relation.kind==SurfaceRelationKind::Coplanar)
            loss=1-alignment;
        else if(relation.kind==SurfaceRelationKind::Perpendicular)
            loss=alignment;
        else continue;
        const double weight=relation.manualConfirmed?1.0:relation.confidence;
        if(weight<=0)continue;
        relationLoss+=weight*loss;
        relationWeight+=weight;
        ++scored.relationComparisons;
    }
    scored.relationError=relationWeight?relationLoss/relationWeight:0;
    scored.complexityPenalty=.05*std::max<std::size_t>(1,scored.supportingPatches.size());
    const double inferredSurfacePenalty=candidate.inferredSideWalls?.2:0;
    scored.score=scored.distanceError+scored.normalError+scored.relationError+scored.complexityPenalty+
        inferredSurfacePenalty+4.0*conflicts/observations.size();
    bool rejected=false;
    if(scored.observationCoverage<options.minimumObservationCoverage) {
        scored.status=SolidCandidateStatus::Rejected;
        scored.diagnostics.push_back("Observation coverage is below the configured acceptance threshold");
        rejected=true;
    }
    if(scored.distanceP95Mm>options.maximumP95ToleranceMultiplier*tolerance) {
        scored.status=SolidCandidateStatus::Rejected;
        scored.diagnostics.push_back("P95 model distance exceeds the configured tolerance bound (p95="+
            std::to_string(scored.distanceP95Mm)+", tolerance="+std::to_string(tolerance)+")");
        rejected=true;
    }
    const auto maximumConflicts=std::max(options.maximumOccupancyConflicts,
        std::size_t(std::ceil(options.maximumOccupancyConflictFraction*observations.size())));
    if(conflicts>maximumConflicts) {
        scored.status=SolidCandidateStatus::Rejected;
        scored.diagnostics.push_back("Observed points contradict candidate interior occupancy");
        rejected=true;
    }
    if(!rejected) {
        scored.status=SolidCandidateStatus::Accepted;
        scored.diagnostics.push_back("Scoring metrics: coverage="+std::to_string(scored.observationCoverage)+
            ", p50_mm="+std::to_string(scored.distanceMedianMm)+
            ", p95_mm="+std::to_string(scored.distanceP95Mm)+
            ", normal_error="+std::to_string(scored.normalError)+
            ", normal_comparisons="+std::to_string(scored.normalComparisons)+
            ", relation_error="+std::to_string(scored.relationError)+
            ", relation_comparisons="+std::to_string(scored.relationComparisons)+
            ", occupancy_conflicts="+std::to_string(scored.occupancyConflictCount)+
            ", complexity_penalty="+std::to_string(scored.complexityPenalty));
    }
    error.clear();
    return true;
}
} // namespace

bool inferSolids(const SurfaceScene& scene,SolidInferenceResult& output,std::string& error,
                 const SolidInferenceOptions& options,const PointStore* workingObservations,const SolidPreview& preview) {
    output={};
    if(const auto valid=validateSurfaceScene(scene);!valid){error=valid.message;return false;}
    if(options.beamWidth==0||options.maximumFeatureOperations==0||options.maximumFeatureOperations>8||
       options.maximumCandidates==0||options.maximumSearchSamples==0) {
        error="Invalid solid inference search limits";
        return false;
    }
    std::optional<PointStore> searchSamples;
    if(workingObservations) {
        if(workingObservations->unit()!=scene.header.unit||workingObservations->empty()||
           pointStoreFingerprint(*workingObservations)!=scene.header.analysisFingerprint) {
            error="SnapshotMismatch: search observations differ from the SurfaceScene working snapshot";
            return false;
        }
        const auto sampleCount=std::min(options.maximumSearchSamples,workingObservations->size());
        std::vector<PointSample> samples;
        samples.reserve(sampleCount);
        for(std::size_t i=0;i<sampleCount;++i)
            samples.push_back(workingObservations->points()[i*workingObservations->size()/sampleCount]);
        searchSamples.emplace(std::move(samples),scene.header.unit);
    }
    std::vector<PatchId> planar;
    for(const auto& patch:scene.patches) {
        if(std::holds_alternative<PlaneEvidence>(scene.surfaces[std::size_t(patch.surfaceId-1)]))planar.push_back(patch.id);
    }
    if (workingObservations && scene.header.unit == LengthUnit::Millimeter &&
        output.candidates.size() < options.maximumCandidates) {
        SurfaceAnalysisResult analysis;
        analysis.pointFingerprint = scene.header.analysisFingerprint;
        analysis.unit = scene.header.unit;
        analysis.surfaces = scene.surfaces;
        CurvedSolidRecognitionOptions curvedOptions;
        curvedOptions.distanceTolerance = scene.header.distanceToleranceMm;
        for (const auto& patch : scene.patches)
            if (!std::holds_alternative<PlaneEvidence>(
                    scene.surfaces[std::size_t(patch.surfaceId - 1)]) &&
                std::isfinite(patch.spacingMm))
                curvedOptions.distanceTolerance = std::max(curvedOptions.distanceTolerance,
                                                           patch.spacingMm);
        CurvedSolidCandidate curved;
        std::string curvedError;
        if (recognizeCurvedSolidFromAnalysis(*workingObservations, analysis, curvedOptions,
                                             curved, curvedError)) {
            SolidCandidate candidate;
            candidate.analysisFingerprint = scene.header.analysisFingerprint;
            candidate.curvedSolid = std::move(curved);
            const PrimitiveKind kind = std::visit([](const auto& primitive) {
                using Primitive = std::decay_t<decltype(primitive)>;
                if constexpr (std::is_same_v<Primitive, modeling::SpherePrimitiveParams>)
                    return PrimitiveKind::Sphere;
                else if constexpr (std::is_same_v<Primitive, modeling::CylinderPrimitiveParams>)
                    return PrimitiveKind::Cylinder;
                else if constexpr (std::is_same_v<Primitive, modeling::ConePrimitiveParams>)
                    return PrimitiveKind::Cone;
                else
                    return PrimitiveKind::Torus;
            }, candidate.curvedSolid->primitive);
            for (const auto& patch : scene.patches)
                if (primitiveKind(scene.surfaces[std::size_t(patch.surfaceId - 1)]) == kind)
                    candidate.supportingPatches.push_back(patch.id);
            candidate.status = SolidCandidateStatus::Proposed;
            candidate.diagnostics.push_back(
                "Complete analytic curved solid proposed from validated surface evidence");
            output.candidates.push_back(std::move(candidate));
        }
    }
    std::vector<const ScenePatch*> shellPatches;
    for (const auto& patch : scene.patches) {
        if (patch.boundaryStatus == SceneBoundaryStatus::ObservedClosed && patch.orientationKnown &&
            std::holds_alternative<PlaneEvidence>(scene.surfaces[std::size_t(patch.surfaceId-1)]))
            shellPatches.push_back(&patch);
    }
    std::vector<std::vector<std::size_t>> adjacency(shellPatches.size());
    for (std::size_t i = 0; i < shellPatches.size(); ++i)
        for (std::size_t j = i+1; j < shellPatches.size(); ++j)
            if (shareObservedEdge(*shellPatches[i], *shellPatches[j], scene.header.distanceToleranceMm*2)) {
                adjacency[i].push_back(j);
                adjacency[j].push_back(i);
            }
    std::vector<bool> visited(shellPatches.size());
    for (std::size_t start = 0; start < shellPatches.size(); ++start) {
        if (visited[start]) continue;
        std::vector<std::size_t> pending{start};
        visited[start] = true;
        std::vector<PatchId> component;
        for (std::size_t head = 0; head < pending.size(); ++head) {
            const auto index = pending[head];
            component.push_back(shellPatches[index]->id);
            for (const auto neighbor : adjacency[index]) if (!visited[neighbor]) {
                visited[neighbor] = true;
                pending.push_back(neighbor);
            }
        }
        if (component.size() < 4||output.candidates.size()>=options.maximumCandidates) continue;
        SolidCandidate shell;
        std::string shellError;
        if (proposeObservedShell(scene, component, shell, shellError, preview)) {
            shell.status = SolidCandidateStatus::Proposed;
            shell.diagnostics.push_back("Observed closed planar shell proposed from shared boundary edges");
            output.candidates.push_back(std::move(shell));
        }
    }
    const std::size_t maximumPairHypotheses=std::max<std::size_t>(1,
        std::min<std::size_t>(64,options.maximumCandidates/2));
    std::vector<std::string> pairFailures;
    for(std::size_t i=0;i<planar.size()&&output.candidates.size()<maximumPairHypotheses&&
        output.candidates.size()<options.maximumCandidates;++i)
      for(std::size_t j=i+1;j<planar.size()&&output.candidates.size()<maximumPairHypotheses&&
          output.candidates.size()<options.maximumCandidates;++j) {
        SolidCandidate candidate;std::string reason;
        if(proposeParallelCapExtrusion(scene,planar[i],planar[j],candidate,reason,preview))
            output.candidates.push_back(std::move(candidate));
        else pairFailures.push_back(std::to_string(planar[i])+"/"+std::to_string(planar[j])+": "+reason);
    }
    const auto attachEvidenceTrace = [&](SolidCandidate& candidate) {
        std::sort(candidate.supportingPatches.begin(), candidate.supportingPatches.end());
        candidate.supportingPatches.erase(std::unique(candidate.supportingPatches.begin(),
            candidate.supportingPatches.end()), candidate.supportingPatches.end());
        candidate.supportingPoints.clear();
        candidate.unresolvedPatches.clear();
        for (const auto& patch : scene.patches) {
            if (std::binary_search(candidate.supportingPatches.begin(),
                                   candidate.supportingPatches.end(), patch.id)) {
                candidate.supportingPoints.insert(candidate.supportingPoints.end(),
                    patch.supportPointIds.begin(), patch.supportPointIds.end());
            } else {
                candidate.unresolvedPatches.push_back(patch.id);
            }
        }
        std::sort(candidate.supportingPoints.begin(), candidate.supportingPoints.end());
        candidate.supportingPoints.erase(std::unique(candidate.supportingPoints.begin(),
            candidate.supportingPoints.end()), candidate.supportingPoints.end());
    };
    std::set<PatchId> explainedPatches;
    for (auto& candidate : output.candidates) {
        attachEvidenceTrace(candidate);
        explainedPatches.insert(candidate.supportingPatches.begin(), candidate.supportingPatches.end());
    }
    for (const auto& patch : scene.patches)
        if (!explainedPatches.count(patch.id)) output.unresolvedPatches.push_back(patch.id);
    const auto generatorCandidates=output.candidates;
    const double tolerance=scene.header.distanceToleranceMm;
    const double minimumVolumeChange=tolerance*tolerance*tolerance;
    const auto searchScore=[&](SolidCandidate& candidate,const TopoDS_Shape& shape) {
        if(!searchSamples)return true;
        auto evaluated=candidate;
        std::string scoringError;
        if(!scoreCandidateShape(candidate,scene,*searchSamples,tolerance,shape,evaluated,scoringError,{}))
            return false;
        // A partial model can gain evidence in subsequent search steps.
        candidate.score=evaluated.score+1-evaluated.observationCoverage;
        return true;
    };
    const auto trimBeam=[&](auto& states) {
        std::stable_sort(states.begin(),states.end(),[](const auto& first,const auto& second) {
            return first.score<second.score;
        });
        if(states.size()>options.beamWidth)states.resize(options.beamWidth);
    };
    std::vector<SolidCandidate> beam;
    for(const auto& candidate:generatorCandidates)
        if(candidate.featureProgram.size()==1&&candidate.featureProgram.front().operation==modeling::ExtrudeOperation::NewBody) {
            auto state=candidate;
            if(searchSamples) {
                modeling::ModelingCore verification;
                std::string verificationError;
                if(!rebuildFeatureProgram(state.featureProgram,verification,verificationError,minimumVolumeChange)||
                   !searchScore(state,verification.bodyShape()))continue;
            }
            beam.push_back(std::move(state));
        }
    trimBeam(beam);
    for(std::size_t operationCount=2;operationCount<=options.maximumFeatureOperations&&
        output.candidates.size()<options.maximumCandidates&&!beam.empty();++operationCount) {
        std::vector<SolidCandidate> expanded;
        for(const auto& state:beam)for(const auto& tool:generatorCandidates) {
            if(tool.featureProgram.size()!=1||tool.featureProgram.front().operation!=modeling::ExtrudeOperation::NewBody||
               sharesSupportingPatch(state,tool))continue;
            for(const auto operation:{modeling::ExtrudeOperation::NewBody,
                                      modeling::ExtrudeOperation::Join,
                                      modeling::ExtrudeOperation::Cut}) {
                if(output.candidates.size()>=options.maximumCandidates)break;
                SolidCandidate combined=state;
                combined.featureProgram.push_back(tool.featureProgram.front());
                combined.featureProgram.back().operation=operation;
                combined.supportingPatches.insert(combined.supportingPatches.end(),
                    tool.supportingPatches.begin(),tool.supportingPatches.end());
                std::sort(combined.supportingPatches.begin(),combined.supportingPatches.end());
                combined.supportingPatches.erase(std::unique(combined.supportingPatches.begin(),
                    combined.supportingPatches.end()),combined.supportingPatches.end());
                combined.inferredSurfaces.insert(combined.inferredSurfaces.end(),
                    tool.inferredSurfaces.begin(), tool.inferredSurfaces.end());
                attachEvidenceTrace(combined);
                combined.inferredSideWalls|=tool.inferredSideWalls;
                combined.score=state.score+tool.score+.1*combined.featureProgram.size();
                combined.status=SolidCandidateStatus::Proposed;
                combined.boundaryBody.reset();
                combined.diagnostics.push_back(operation==modeling::ExtrudeOperation::Join
                    ?"Beam search proposes joining disjoint evidence groups":"Beam search proposes cutting a supported host body");
                if(std::any_of(output.candidates.begin(),output.candidates.end(),
                    [&](const auto& existing){return sameHypothesis(existing,combined);}))continue;
                modeling::ModelingCore verification;
                std::string verificationError;
                if(!rebuildFeatureProgram(combined.featureProgram,verification,verificationError,minimumVolumeChange)) {
                    combined.status=SolidCandidateStatus::Rejected;
                    combined.diagnostics.push_back("OCCT rejected feature combination: "+verificationError);
                    output.candidates.push_back(std::move(combined));
                    continue;
                }
                if (std::any_of(combined.featureProgram.begin() + 1, combined.featureProgram.end(),
                        [](const auto& feature) {
                            return feature.operation == modeling::ExtrudeOperation::NewBody;
                        }) && !independentSolidsDoNotOverlap(verification.bodyShape(), minimumVolumeChange)) {
                    combined.status=SolidCandidateStatus::Rejected;
                    combined.diagnostics.push_back(
                        "OCCT rejected independent NewBody operations with overlapping material");
                    output.candidates.push_back(std::move(combined));
                    continue;
                }
                if(!searchScore(combined,verification.bodyShape())) {
                    combined.status=SolidCandidateStatus::Rejected;
                    combined.diagnostics.push_back("Observation screening failed for feature combination");
                    output.candidates.push_back(std::move(combined));
                    continue;
                }
                if (preview) preview(verification.bodyShape());
                output.candidates.push_back(combined);
                expanded.push_back(std::move(combined));
            }
        }
        trimBeam(expanded);
        beam=std::move(expanded);
    }
    error.clear();
    if(output.candidates.empty()) {
        error="No supported solid candidates; finite planar patches may remain ambiguous";
        for(std::size_t index=0;index<std::min<std::size_t>(pairFailures.size(),24);++index)
            error+="; "+pairFailures[index];
    }
    return true;
}
} // namespace reconstruction
