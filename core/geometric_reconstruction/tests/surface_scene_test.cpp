#include "reconstruction/solid_inference/SurfaceScene.h"
#include "reconstruction/solid_inference/SolidInference.h"
#include "reconstruction/modeling/ModelingAdapter.h"
#include "reconstruction/surface/SurfaceFootprint.h"
#include "reconstruction/surface/SurfaceSupportValidation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <utility>
#include <vector>

int main() {
    reconstruction::SurfaceScene scene;
    scene.header.unit = reconstruction::LengthUnit::Millimeter;
    scene.header.analysisFingerprint = 17;
    scene.header.distanceToleranceMm = .1;
    scene.header.angularToleranceRad = .02;
    reconstruction::PlaneEvidence plane;
    scene.surfaces.emplace_back(plane);
    reconstruction::ScenePatch patch;
    patch.id = 1;
    patch.surfaceId = 1;
    patch.boundaryStatus = reconstruction::SceneBoundaryStatus::Ambiguous;
    patch.loops.push_back({reconstruction::SceneLoopRole::Outer,
        {{0,0,0},{1,0,0},{1,1,0},{0,1,0}}, true, .5});
    scene.patches.push_back(patch);
    if (!reconstruction::validateSurfaceScene(scene, 17)) return 1;
    if (reconstruction::validateSurfaceScene(scene, 18).code != reconstruction::SurfaceSceneError::SnapshotMismatch) return 2;
    scene.header.unit = reconstruction::LengthUnit::Meter;
    if (reconstruction::validateSurfaceScene(scene).code != reconstruction::SurfaceSceneError::WrongUnit) return 3;
    scene.header.unit = reconstruction::LengthUnit::Millimeter;
    auto observedOpen = scene;
    observedOpen.relations.clear();
    observedOpen.patches.front().boundaryStatus=reconstruction::SceneBoundaryStatus::ObservedOpen;
    observedOpen.patches.front().loops.front().closed=false;
    observedOpen.patches.front().loops.front().vertices.pop_back();
    if(!reconstruction::validateSurfaceScene(observedOpen))return 75;
    auto falselyClosed = observedOpen;
    falselyClosed.patches.front().boundaryStatus=reconstruction::SceneBoundaryStatus::ObservedClosed;
    if(reconstruction::validateSurfaceScene(falselyClosed).code!=reconstruction::SurfaceSceneError::InvalidLoop)return 76;
    scene.relations.push_back({1, 2, reconstruction::SurfaceRelationKind::Adjacent, 0, .8, false});
    if (reconstruction::validateSurfaceScene(scene).code != reconstruction::SurfaceSceneError::InvalidSurfaceReference) return 4;

    reconstruction::SurfaceScene cube;
    cube.header.unit = reconstruction::LengthUnit::Millimeter;
    cube.header.analysisFingerprint = 42;
    cube.header.distanceToleranceMm = 1e-5;
    cube.header.angularToleranceRad = .01;
    const auto face = [&](modeling::Vec3 normal, double offset,
                          std::vector<modeling::Vec3> points) {
        reconstruction::PlaneEvidence equation;
        equation.plane.normal = normal;
        equation.plane.offset = offset;
        cube.surfaces.emplace_back(equation);
        reconstruction::ScenePatch patch;
        patch.id = cube.patches.size() + 1;
        patch.surfaceId = cube.surfaces.size();
        patch.orientationKnown = true;
        patch.materialNormal = normal;
        patch.boundaryStatus = reconstruction::SceneBoundaryStatus::ObservedClosed;
        patch.loops.push_back({reconstruction::SceneLoopRole::Outer,std::move(points),true,1});
        cube.patches.push_back(std::move(patch));
    };
    face({-1,0,0},0,{{0,0,0},{0,0,1},{0,1,1},{0,1,0}});
    face({1,0,0},-1,{{1,0,0},{1,1,0},{1,1,1},{1,0,1}});
    face({0,-1,0},0,{{0,0,0},{1,0,0},{1,0,1},{0,0,1}});
    face({0,1,0},-1,{{0,1,0},{0,1,1},{1,1,1},{1,1,0}});
    face({0,0,-1},0,{{0,0,0},{0,1,0},{1,1,0},{1,0,0}});
    face({0,0,1},-1,{{0,0,1},{1,0,1},{1,1,1},{0,1,1}});
    reconstruction::SolidCandidate candidate;
    std::string error;
    if (!reconstruction::proposeObservedShell(cube,{1,2,3,4,5,6},candidate,error)) {
        std::cerr << error << '\n';
        return 5;
    }
    if (candidate.status != reconstruction::SolidCandidateStatus::Accepted || !candidate.boundaryBody) return 6;
    modeling::ModelingCore model;
    const auto committed = reconstruction::commitSolidCandidate(candidate,model);
    if (!committed.success || model.features().size()!=1 || model.bodyShape().IsNull()) return 8;
    std::vector<reconstruction::PointSample> unsupportedSamples(8);
    for(std::size_t i=0;i<unsupportedSamples.size();++i) {
        unsupportedSamples[i].id=i+1;
        unsupportedSamples[i].position={20+double(i),20,20};
    }
    reconstruction::SolidCandidate unsupportedScore;
    if(!reconstruction::scoreSolidCandidate(candidate,cube,
        reconstruction::PointStore(std::move(unsupportedSamples)),.01,unsupportedScore,error))return 40;
    if(unsupportedScore.status!=reconstruction::SolidCandidateStatus::Rejected||
       unsupportedScore.observationCoverage!=0||unsupportedScore.distanceP95Mm<=.04)return 41;
    const reconstruction::PointStore interiorObservations({
        {1,{.4,.4,.4}}, {2,{.5,.5,.5}}, {3,{.6,.6,.6}}
    });
    reconstruction::SolidCandidate interiorScore;
    if(!reconstruction::scoreSolidCandidate(candidate,cube,interiorObservations,.01,interiorScore,error)||
       interiorScore.status!=reconstruction::SolidCandidateStatus::Rejected||
       interiorScore.occupancyConflictCount!=3||interiorScore.distanceP95Mm<.39)return 58;
    auto open = cube;
    open.patches.front().boundaryStatus = reconstruction::SceneBoundaryStatus::ObservedOpen;
    if (reconstruction::proposeObservedShell(open,{1,2,3,4,5,6},candidate,error) ||
        candidate.status != reconstruction::SolidCandidateStatus::Rejected) return 7;

    reconstruction::SurfaceScene prism;
    prism.header.unit=reconstruction::LengthUnit::Millimeter;
    prism.header.analysisFingerprint=71;
    prism.header.distanceToleranceMm=.01;
    prism.header.angularToleranceRad=.01;
    for(int cap=0;cap<2;++cap) {
        reconstruction::PlaneEvidence equation;
        equation.plane.normal={0,0,cap?1.0:-1.0};
        equation.plane.offset=cap?-1:0;
        prism.surfaces.emplace_back(equation);
        reconstruction::ScenePatch patch;
        patch.id=cap+1;patch.surfaceId=cap+1;
        patch.supportPointIds=cap?std::vector<reconstruction::PointId>{9,10,11,12}
            :std::vector<reconstruction::PointId>{1,2,3,4};
        patch.boundaryStatus=reconstruction::SceneBoundaryStatus::Ambiguous;
        patch.loops.push_back({reconstruction::SceneLoopRole::Outer,
            {{0,0,double(cap)},{2,0,double(cap)},{2,1,double(cap)},{0,1,double(cap)}},true,.8});
        prism.patches.push_back(std::move(patch));
    }
    if(!reconstruction::proposeParallelCapExtrusion(prism,1,2,candidate,error)) {
        std::cerr<<error<<'\n';return 9;
    }
    if(candidate.status!=reconstruction::SolidCandidateStatus::Proposed||!candidate.inferredSideWalls||
       candidate.depthEvidence.empty())return 10;
    std::vector<reconstruction::PointSample> samples;
    for(int i=0;i<=4;++i)for(int j=0;j<=2;++j) {
        reconstruction::PointSample point;point.id=samples.size()+1;
        point.position={i*.5,j*.5,0};point.normal=modeling::Vec3{0,0,-1};samples.push_back(point);
        point.id=samples.size()+1;point.position.z=1;point.normal=modeling::Vec3{0,0,1};samples.push_back(point);
    }
    reconstruction::PointStore observations(std::move(samples));
    reconstruction::SolidCandidate scored;
    if(!reconstruction::scoreSolidCandidate(candidate,prism,observations,.02,scored,error)||
       scored.status!=reconstruction::SolidCandidateStatus::Accepted) {std::cerr<<error<<" status="<<int(scored.status)<<'\n';return 11;}
    if(scored.observationCoverage<.99||!scored.unresolvedPoints.empty()) {
        std::cerr<<"coverage="<<scored.observationCoverage<<" unresolved="<<scored.unresolvedPoints.size()<<'\n';return 13;
    }
    auto mismatched=prism;
    mismatched.patches[1].loops[0].vertices[2].x+=.1;
    if(reconstruction::proposeParallelCapExtrusion(mismatched,1,2,candidate,error))return 14;
    reconstruction::SolidInferenceResult inferred;
    if(!reconstruction::inferSolids(prism,inferred,error)||inferred.candidates.size()!=1)return 12;
    if(inferred.candidates.front().supportingPoints.size()!=8||
       !inferred.candidates.front().unresolvedPatches.empty())return 69;
    auto differentlySampledCaps=prism;
    const auto& rectangularLoop=prism.patches[1].loops.front().vertices;
    auto& subdividedLoop=differentlySampledCaps.patches[1].loops.front().vertices;
    subdividedLoop.clear();
    for(std::size_t index=0;index<rectangularLoop.size();++index) {
        const auto first=rectangularLoop[index],second=rectangularLoop[(index+1)%rectangularLoop.size()];
        subdividedLoop.push_back(first);
        subdividedLoop.push_back({(first.x+second.x)*.5,(first.y+second.y)*.5,(first.z+second.z)*.5});
    }
    if(!reconstruction::proposeParallelCapExtrusion(differentlySampledCaps,1,2,candidate,error)) {
        std::cerr<<"resampled parallel caps: "<<error<<'\n';
        return 73;
    }

    // A concave L-shaped cap must stay a polygonal prism and must not be
    // silently replaced by its bounding rectangle.
    auto lPrism = prism;
    for (std::size_t cap = 0; cap < lPrism.patches.size(); ++cap) {
        const double z = cap == 0 ? 0.0 : 1.0;
        lPrism.patches[cap].loops.front().vertices = {
            {0,0,z},{2,0,z},{2,.7,z},{.7,.7,z},{.7,2,z},{0,2,z}};
    }
    reconstruction::SolidCandidate lCandidate;
    if (!reconstruction::proposeParallelCapExtrusion(lPrism,1,2,lCandidate,error) ||
        lCandidate.featureProgram.size()!=1 || !lCandidate.boundaryBody ||
        lCandidate.boundaryBody->outline.size()!=6) return 77;
    std::vector<reconstruction::PointSample> lSamples;
    for (const double z : {0.0,1.0})
        for (const auto point : lPrism.patches[0].loops.front().vertices)
            lSamples.push_back({lSamples.size()+1, {point.x,point.y,z}});
    reconstruction::SolidCandidate lScored;
    if (!reconstruction::scoreSolidCandidate(lCandidate,lPrism,
        reconstruction::PointStore(std::move(lSamples)),.02,lScored,error) ||
        lScored.status!=reconstruction::SolidCandidateStatus::Accepted) return 78;
    modeling::ModelingCore lModel;
    const auto lCommit = reconstruction::commitSolidCandidate(lScored,lModel);
    if (!lCommit.success || lModel.bodyShape().IsNull()) return 79;
    reconstruction::SolidInferenceResult lInference;
    if (!reconstruction::inferSolids(lPrism,lInference,error) ||
        std::none_of(lInference.candidates.begin(),lInference.candidates.end(),[](const auto& item) {
            return item.featureProgram.size()==1 && item.boundaryBody &&
                   item.boundaryBody->outline.size()==6;
        })) return 80;

    // Two levels with a smaller upper footprint must yield a Join candidate
    // through the shared beam-search interface.
    reconstruction::SurfaceScene stair;
    stair.header.unit = reconstruction::LengthUnit::Millimeter;
    stair.header.analysisFingerprint = 740;
    stair.header.distanceToleranceMm = .01;
    stair.header.angularToleranceRad = .02;
    const auto addStairCap = [&](double z, double x0, double x1, int id, bool top) {
        reconstruction::PlaneEvidence equation;
        equation.plane.normal = {0,0,top ? 1.0 : -1.0};
        equation.plane.offset = top ? -z : z;
        stair.surfaces.emplace_back(equation);
        reconstruction::ScenePatch cap;
        cap.id = id; cap.surfaceId = stair.surfaces.size();
        cap.supportPointIds = {static_cast<reconstruction::PointId>(id*10+1),
                               static_cast<reconstruction::PointId>(id*10+2)};
        cap.boundaryStatus = reconstruction::SceneBoundaryStatus::Ambiguous;
        cap.loops.push_back({reconstruction::SceneLoopRole::Outer,
            {{x0,0,z},{x1,0,z},{x1,2,z},{x0,2,z}},true,.9});
        stair.patches.push_back(std::move(cap));
    };
    addStairCap(0,0,2,1,false); addStairCap(1,0,2,2,true);
    addStairCap(1,.5,1.5,3,false); addStairCap(2,.5,1.5,4,true);
    reconstruction::SolidInferenceResult stairInference;
    reconstruction::SolidInferenceOptions stairOptions;
    stairOptions.maximumFeatureOperations = 3;
    stairOptions.maximumCandidates = 32;
    if (!reconstruction::inferSolids(stair,stairInference,error,stairOptions) ||
        std::none_of(stairInference.candidates.begin(),stairInference.candidates.end(),[](const auto& item) {
            return item.featureProgram.size()==2 &&
                   item.featureProgram[0].operation==modeling::ExtrudeOperation::NewBody &&
                   item.featureProgram[1].operation==modeling::ExtrudeOperation::Join;
    })) return 81;

    auto separatePrisms=prism;
    for(std::size_t i=0;i<prism.surfaces.size();++i) {
        separatePrisms.surfaces.push_back(prism.surfaces[i]);
        auto separateCap=prism.patches[i];
        separateCap.id+=2;
        separateCap.surfaceId+=2;
        for(auto& pointId:separateCap.supportPointIds)pointId+=100;
        for(auto& loop:separateCap.loops)for(auto& vertex:loop.vertices)vertex.x+=5;
        separatePrisms.patches.push_back(std::move(separateCap));
    }
    reconstruction::SolidInferenceResult separateInference;
    if(!reconstruction::inferSolids(separatePrisms,separateInference,error))return 70;
    const auto separateBodies=std::find_if(separateInference.candidates.begin(),separateInference.candidates.end(),
        [](const auto& item) {
            return item.featureProgram.size()==2&&
                item.featureProgram[0].operation==modeling::ExtrudeOperation::NewBody&&
                item.featureProgram[1].operation==modeling::ExtrudeOperation::NewBody;
        });
    if(separateBodies==separateInference.candidates.end()) {
        std::cerr<<"separate-body candidates="<<separateInference.candidates.size()<<'\n';
        for(const auto& item:separateInference.candidates) {
            std::cerr<<"  features="<<item.featureProgram.size()<<" ops=";
            for(const auto& feature:item.featureProgram)std::cerr<<int(feature.operation)<<',';
            std::cerr<<" supports="<<item.supportingPatches.size()<<" status="<<int(item.status)<<'\n';
            for(const auto& diagnostic:item.diagnostics)std::cerr<<"  "<<diagnostic<<'\n';
        }
        return 71;
    }
    if(separateBodies->supportingPatches.size()!=4||separateBodies->supportingPoints.size()!=16||
       !separateBodies->unresolvedPatches.empty())return 71;
    const auto multiBodyObservations=reconstruction::PointStore({
        {1,{0,0,0}},{2,{2,0,0}},{3,{2,1,0}},{4,{0,1,0}},
        {5,{0,0,1}},{6,{2,0,1}},{7,{2,1,1}},{8,{0,1,1}},
        {9,{5,0,0}},{10,{7,0,0}},{11,{7,1,0}},{12,{5,1,0}},
        {13,{5,0,1}},{14,{7,0,1}},{15,{7,1,1}},{16,{5,1,1}}
    });
    reconstruction::SolidCandidate scoredSeparateBodies;
    if(!reconstruction::scoreSolidCandidate(*separateBodies,separatePrisms,
        multiBodyObservations,.01,scoredSeparateBodies,error)||
       scoredSeparateBodies.status!=reconstruction::SolidCandidateStatus::Accepted||
       scoredSeparateBodies.observationCoverage<.99) {
        std::cerr<<"multi-body score status="<<int(scoredSeparateBodies.status)
                 <<" coverage="<<scoredSeparateBodies.observationCoverage
                 <<" p95="<<scoredSeparateBodies.distanceP95Mm
                 <<" conflicts="<<scoredSeparateBodies.occupancyConflictCount
                 <<" error="<<error<<'\n';
        for(const auto& diagnostic:scoredSeparateBodies.diagnostics)std::cerr<<diagnostic<<'\n';
        return 72;
    }
    modeling::ModelingCore multiBodyModel;
    const auto multiBodyCommit=reconstruction::commitSolidCandidate(scoredSeparateBodies,multiBodyModel);
    if(!multiBodyCommit.success)return 73;
    std::size_t committedSolidCount=0;
    for(TopExp_Explorer solid(multiBodyModel.bodyShape(),TopAbs_SOLID);solid.More();solid.Next())
        ++committedSolidCount;
    if(committedSolidCount!=2)return 74;

    reconstruction::SurfaceScene holePrism;
    holePrism.header.unit=reconstruction::LengthUnit::Millimeter;
    holePrism.header.analysisFingerprint=72;
    holePrism.header.distanceToleranceMm=.01;
    holePrism.header.angularToleranceRad=.02;
    const auto addCap=[&](double z,bool top) {
        reconstruction::PlaneEvidence equation;
        equation.plane.normal={0,0,top?1.0:-1.0};
        equation.plane.offset=top?-1:0;
        equation.confidence=.99;
        holePrism.surfaces.emplace_back(equation);
        reconstruction::ScenePatch cap;
        cap.id=holePrism.patches.size()+1;
        cap.surfaceId=holePrism.surfaces.size();
        cap.orientationKnown=true;
        cap.materialNormal=equation.plane.normal;
        cap.boundaryStatus=reconstruction::SceneBoundaryStatus::ObservedClosed;
        cap.loops.push_back({reconstruction::SceneLoopRole::Outer,
            {{0,0,z},{2,0,z},{2,2,z},{0,2,z}},true,.9});
        reconstruction::BoundaryLoop3d hole;
        hole.role=reconstruction::SceneLoopRole::Hole;
        hole.closed=true;
        hole.confidence=.9;
        constexpr double pi=3.14159265358979323846;
        for(int i=0;i<16;++i) {
            const double angle=2*pi*i/16;
            hole.vertices.push_back({1+.3*std::cos(angle),1+.3*std::sin(angle),z});
        }
        cap.loops.push_back(std::move(hole));
        holePrism.patches.push_back(std::move(cap));
    };
    addCap(0,false);
    addCap(1,true);
    reconstruction::SolidCandidate holeCandidate;
    if(reconstruction::proposeParallelCapExtrusion(holePrism,1,2,holeCandidate,error)||
       error.find("cylinder wall")==std::string::npos)return 46;
    reconstruction::CylinderEvidence cylinder;
    cylinder.cylinder.axisPoint={1,1,0};
    cylinder.cylinder.axisDirection={0,0,1};
    cylinder.cylinder.radius=.3;
    cylinder.axialMinimum=0;
    cylinder.axialMaximum=1;
    cylinder.angularCoverage=1;
    cylinder.confidence=.99;
    for(reconstruction::PointId id=1;id<=32;++id)cylinder.supportPointIds.push_back(id);
    holePrism.surfaces.emplace_back(cylinder);
    reconstruction::ScenePatch cylinderPatch;
    cylinderPatch.id=3;
    cylinderPatch.surfaceId=3;
    cylinderPatch.supportPointIds=cylinder.supportPointIds;
    cylinderPatch.supportConfidence=.99;
    cylinderPatch.boundaryStatus=reconstruction::SceneBoundaryStatus::ObservedOpen;
    holePrism.patches.push_back(cylinderPatch);
    if(!reconstruction::proposeParallelCapExtrusion(holePrism,1,2,holeCandidate,error)) {
        std::cerr<<"supported through-hole candidate: "<<error<<'\n';return 47;
    }
    if(!holeCandidate.boundaryBody||holeCandidate.boundaryBody->profileVoids.size()!=1||
       std::find(holeCandidate.supportingPatches.begin(),holeCandidate.supportingPatches.end(),3)==
           holeCandidate.supportingPatches.end()||holeCandidate.featureProgram.size()!=2||
       holeCandidate.featureProgram.front().operation!=modeling::ExtrudeOperation::NewBody||
       holeCandidate.featureProgram.back().operation!=modeling::ExtrudeOperation::Cut)return 48;
    std::vector<reconstruction::PointSample> holeSamples;
    for(const double z:{0.0,1.0})for(int x=0;x<=4;++x)for(int y=0;y<=4;++y) {
        const double px=x*.5,py=y*.5;
        if(std::hypot(px-1,py-1)<.31)continue;
        reconstruction::PointSample sample;
        sample.id=holeSamples.size()+1;
        sample.position={px,py,z};
        holeSamples.push_back(sample);
    }
    constexpr double pi=3.14159265358979323846;
    for(int level=0;level<=4;++level)for(int angle=0;angle<16;++angle) {
        const double theta=2*pi*angle/16;
        reconstruction::PointSample sample;
        sample.id=holeSamples.size()+1;
        sample.position={1+.3*std::cos(theta),1+.3*std::sin(theta),level*.25};
        holeSamples.push_back(sample);
    }
    reconstruction::SolidCandidate scoredHole;
    if(!reconstruction::scoreSolidCandidate(holeCandidate,holePrism,
        reconstruction::PointStore(std::move(holeSamples)),.02,scoredHole,error)) {
        std::cerr<<"through-hole score: "<<error<<'\n';return 49;
    }
    if(scoredHole.status!=reconstruction::SolidCandidateStatus::Accepted)return 50;
    modeling::ModelingCore holeModel;
    const auto holePatch=reconstruction::planSolidCandidate(scoredHole,holeModel,error);
    if(!holePatch||holePatch->commands.size()!=4)return 51;
    const auto holeCommit=reconstruction::commitSolidCandidate(scoredHole,holeModel);
    if(!holeCommit.success||holeModel.features().size()!=4||holeModel.bodyShape().IsNull())return 52;
    reconstruction::SolidInferenceResult inferredHole;
    if(!reconstruction::inferSolids(holePrism,inferredHole,error))return 53;
    if(std::none_of(inferredHole.candidates.begin(),inferredHole.candidates.end(),[](const auto& item) {
        return item.featureProgram.size()==2&&item.supportingPatches.size()==3;
    }))return 54;

    auto blindPrism=holePrism;
    blindPrism.patches.front().loops.pop_back();
    std::get<reconstruction::CylinderEvidence>(blindPrism.surfaces[2]).axialMinimum=.4;
    reconstruction::SolidCandidate blindCandidate;
    if(reconstruction::proposeParallelCapExtrusion(blindPrism,1,2,blindCandidate,error)||
       error.find("blind-hole floor")==std::string::npos)return 66;
    reconstruction::PlaneEvidence floorEquation;
    floorEquation.plane={{0,0,1},-.4};
    blindPrism.surfaces.emplace_back(floorEquation);
    reconstruction::ScenePatch floor;
    floor.id=4;
    floor.surfaceId=4;
    floor.supportPointIds={1000};
    floor.orientationKnown=true;
    floor.materialNormal={0,0,1};
    floor.boundaryStatus=reconstruction::SceneBoundaryStatus::ObservedClosed;
    auto floorLoop=blindPrism.patches[1].loops.back();
    floorLoop.role=reconstruction::SceneLoopRole::Outer;
    for(auto& vertex:floorLoop.vertices)vertex.z=.4;
    floor.loops.push_back(std::move(floorLoop));
    blindPrism.patches.push_back(std::move(floor));
    if(!reconstruction::proposeParallelCapExtrusion(blindPrism,1,2,blindCandidate,error)) {
        std::cerr<<"supported blind-hole candidate: "<<error<<'\n';return 67;
    }
    if(blindCandidate.featureProgram.size()!=2||blindCandidate.supportingPatches.size()!=4||
       !blindCandidate.featureProgram.back().reverse||
       std::abs(blindCandidate.featureProgram.back().depth-.6)>1e-9)return 68;
    const reconstruction::PointStore blindObservations({
        {1,{1,1,0}}, {2,{1,1,.4}}, {3,{1.3,1,.7}}, {4,{.7,1,.7}}, {5,{0,0,1}}
    });
    reconstruction::SolidCandidate blindScore;
    if(!reconstruction::scoreSolidCandidate(blindCandidate,blindPrism,
        blindObservations,.01,blindScore,error)||
       blindScore.status!=reconstruction::SolidCandidateStatus::Accepted||
       blindScore.observationCoverage!=1)return 69;

    reconstruction::SurfaceScene bossScene;
    bossScene.header.unit=reconstruction::LengthUnit::Millimeter;
    bossScene.header.analysisFingerprint=73;
    bossScene.header.distanceToleranceMm=.01;
    bossScene.header.angularToleranceRad=.02;
    const auto addRectangularCap=[&](double z,double x0,double x1,double y0,double y1,bool top) {
        reconstruction::PlaneEvidence equation;
        equation.plane.normal={0,0,top?1.0:-1.0};
        equation.plane.offset=top?-z:z;
        equation.confidence=.9;
        bossScene.surfaces.emplace_back(equation);
        reconstruction::ScenePatch cap;
        cap.id=bossScene.patches.size()+1;
        cap.surfaceId=bossScene.surfaces.size();
        cap.orientationKnown=true;
        cap.materialNormal=equation.plane.normal;
        cap.boundaryStatus=reconstruction::SceneBoundaryStatus::Ambiguous;
        cap.loops.push_back({reconstruction::SceneLoopRole::Outer,
            {{x0,y0,z},{x1,y0,z},{x1,y1,z},{x0,y1,z}},true,.9});
        bossScene.patches.push_back(std::move(cap));
    };
    addRectangularCap(0,0,2,0,1,false);
    addRectangularCap(1,0,2,0,1,true);
    addRectangularCap(1,.5,1.5,0,1,false);
    addRectangularCap(1.8,.5,1.5,0,1,true);
    const reconstruction::PointStore bossObservations({
        {1,{.25,.5,0}}, {2,{1.75,.5,0}},
        {3,{.25,.5,1}}, {4,{1.75,.5,1}},
        {5,{.75,.5,1.8}}, {6,{1.25,.5,1.8}},
        {7,{.5,.5,1.4}}, {8,{1.5,.5,1.4}}
    });
    bossScene.header.analysisFingerprint=reconstruction::pointStoreFingerprint(bossObservations);
    reconstruction::SolidInferenceOptions beamOptions;
    beamOptions.beamWidth=4;
    beamOptions.maximumFeatureOperations=3;
    beamOptions.maximumCandidates=32;
    reconstruction::SolidInferenceResult bossInference;
    if(!reconstruction::inferSolids(bossScene,bossInference,error,beamOptions,&bossObservations))return 55;
    const auto joined=std::find_if(bossInference.candidates.begin(),bossInference.candidates.end(),[](const auto& item) {
        return item.featureProgram.size()==2&&item.featureProgram[0].operation==modeling::ExtrudeOperation::NewBody&&
            item.featureProgram[1].operation==modeling::ExtrudeOperation::Join;
    });
    if(joined==bossInference.candidates.end())return 56;
    reconstruction::SolidCandidate scoredBoss;
    if(!reconstruction::scoreSolidCandidate(*joined,bossScene,bossObservations,.01,scoredBoss,error)||
       scoredBoss.status!=reconstruction::SolidCandidateStatus::Accepted||
       scoredBoss.observationCoverage<.99)return 59;
    const auto rejectedCut=std::find_if(bossInference.candidates.begin(),bossInference.candidates.end(),[](const auto& item) {
        return item.featureProgram.size()==2&&item.featureProgram[1].operation==modeling::ExtrudeOperation::Cut;
    });
    if(rejectedCut==bossInference.candidates.end()||
       rejectedCut->status!=reconstruction::SolidCandidateStatus::Rejected||
       rejectedCut->diagnostics.empty()||
       rejectedCut->diagnostics.back().find("OCCT rejected feature combination")==std::string::npos)return 57;

    constexpr double sizeX=20,sizeY=30,sizeZ=10;
    constexpr std::size_t samplesPerEdge=6;
    const std::array<std::pair<modeling::Vec3,double>,6> boxPlanes{{
        {{-1,0,0},0},{{1,0,0},-sizeX},
        {{0,-1,0},0},{{0,1,0},-sizeY},
        {{0,0,-1},0},{{0,0,1},-sizeZ}}};
    const std::array<modeling::Vec3,6> planeOrigins{{
        {0,0,0},{sizeX,0,0},{0,0,0},{0,sizeY,0},{0,0,0},{0,0,sizeZ}}};
    const std::array<modeling::Vec3,6> planeAxesU{{
        {0,1,0},{0,1,0},{1,0,0},{1,0,0},{1,0,0},{1,0,0}}};
    const std::array<modeling::Vec3,6> planeAxesV{{
        {0,0,1},{0,0,1},{0,0,1},{0,0,1},{0,1,0},{0,1,0}}};
    const std::array<double,6> planeWidths{{
        sizeY,sizeY,sizeX,sizeX,sizeX,sizeX}};
    const std::array<double,6> planeHeights{{
        sizeZ,sizeZ,sizeZ,sizeZ,sizeY,sizeY}};
    std::vector<reconstruction::PointSample> boxSamples;
    for(std::size_t face=0;face<boxPlanes.size();++face)
        for(std::size_t row=0;row<samplesPerEdge;++row)
            for(std::size_t column=0;column<samplesPerEdge;++column) {
                const double u=double(column)/(samplesPerEdge-1),v=double(row)/(samplesPerEdge-1);
                const auto origin=planeOrigins[face],axisU=planeAxesU[face],axisV=planeAxesV[face];
                boxSamples.push_back({boxSamples.size()+1,
                    {origin.x+axisU.x*planeWidths[face]*u+axisV.x*planeHeights[face]*v,
                     origin.y+axisU.y*planeWidths[face]*u+axisV.y*planeHeights[face]*v,
                     origin.z+axisU.z*planeWidths[face]*u+axisV.z*planeHeights[face]*v},
                    boxPlanes[face].first});
            }
    const reconstruction::PointStore boxPoints(std::move(boxSamples));
    reconstruction::SurfaceAnalysisResult analysis;
    analysis.pointFingerprint=reconstruction::pointStoreFingerprint(boxPoints);
    analysis.unit=boxPoints.unit();
    analysis.options.relations.distanceTolerance=.05;
    analysis.options.relations.angularToleranceRadians=.01;
    analysis.options.supportValidation.mode=reconstruction::SupportValidationMode::Report;
    analysis.supportValidation.pointFingerprint=analysis.pointFingerprint;
    analysis.supportValidation.unit=boxPoints.unit();
    analysis.supportValidation.mode=reconstruction::SupportValidationMode::Report;
    analysis.supportValidation.settingsSignature=
        reconstruction::surfaceSupportSettingsSignature(analysis.options);
    const auto samplesPerFace=samplesPerEdge*samplesPerEdge;
    for(std::size_t face=0;face<boxPlanes.size();++face) {
        reconstruction::PlaneEvidence equation;
        equation.plane.normal=boxPlanes[face].first;
        equation.plane.offset=boxPlanes[face].second;
        equation.confidence=1;
        reconstruction::ScenePatch patch;
        patch.id=face+1;
        patch.surfaceId=face+1;
        patch.orientationKnown=true;
        patch.materialNormal=boxPlanes[face].first;
        reconstruction::SurfacePatch indexedPatch;
        indexedPatch.id=patch.id;
        indexedPatch.surfaceId=patch.surfaceId;
        indexedPatch.orientationKnown=true;
        indexedPatch.materialNormal=patch.materialNormal;
        for(std::size_t point=face*samplesPerFace;point<(face+1)*samplesPerFace;++point) {
            const auto& sample=boxPoints.points()[point];
            equation.supportPointIds.push_back(sample.id);
            patch.supportPointIds.push_back(sample.id);
            indexedPatch.supportPointIds.push_back(sample.id);
        }
        analysis.surfaces.emplace_back(std::move(equation));
        analysis.patches.push_back(std::move(indexedPatch));
        auto footprint=std::make_shared<reconstruction::SurfaceFootprint>();
        footprint->origin=planeOrigins[face];
        footprint->axisU=planeAxesU[face];
        footprint->axisV=planeAxesV[face];
        footprint->positions={{{0,0,0}},{{planeWidths[face],0,0}},
                              {{planeWidths[face],planeHeights[face],0}},
                              {{0,planeHeights[face],0}}};
        footprint->triangles={{{0,1,2}},{{0,2,3}}};
        reconstruction::SurfaceSupportDecision decision;
        decision.patchId=patch.id;
        decision.verdict=reconstruction::SupportVerdict::Accepted;
        decision.supportPointIds=patch.supportPointIds;
        decision.footprint=std::move(footprint);
        analysis.supportValidation.decisions.push_back(std::move(decision));
    }
    analysis.sourceSupportValidation=analysis.supportValidation;
    for(std::size_t i=0;i<analysis.sourceSupportValidation.decisions.size();++i) {
        auto& decision=analysis.sourceSupportValidation.decisions[i];
        decision.patchId=100+i;
        decision.supportPointIds=analysis.patches[i].supportPointIds;
    }
    if(analysis.patches.size()!=6)return 16;
    reconstruction::SurfaceScene exported,exportedAgain;
    if(!reconstruction::exportSurfaceScene(boxPoints,analysis,exported,error,&boxPoints)) {
        std::cerr<<"box export: "<<error<<'\n';return 17;
    }
    if(!reconstruction::exportSurfaceScene(boxPoints,analysis,exportedAgain,error,&boxPoints))return 18;
    auto incompleteAnalysis=analysis;
    incompleteAnalysis.sourceSupportValidation.decisions.front().footprint=
        std::make_shared<reconstruction::SurfaceFootprint>();
    reconstruction::SurfaceScene incompleteScene;
    if(!reconstruction::exportSurfaceScene(boxPoints,incompleteAnalysis,incompleteScene,error,&boxPoints))return 25;
    if(incompleteScene.patches.front().boundaryStatus!=reconstruction::SceneBoundaryStatus::Ambiguous)return 26;
    auto workingOnlyAnalysis=analysis;
    workingOnlyAnalysis.sourceSupportValidation={};
    reconstruction::SurfaceScene workingOnlyScene;
    if(!reconstruction::exportSurfaceScene(boxPoints,workingOnlyAnalysis,workingOnlyScene,error))return 42;
    if(workingOnlyScene.patches.front().boundaryStatus!=reconstruction::SceneBoundaryStatus::ObservedClosed)return 43;
    auto mismatchedSupport=analysis;
    mismatchedSupport.supportValidation.settingsSignature.front()+=1;
    mismatchedSupport.sourceSupportValidation.settingsSignature.front()+=1;
    reconstruction::SurfaceScene mismatchedSupportScene;
    if(!reconstruction::exportSurfaceScene(boxPoints,mismatchedSupport,mismatchedSupportScene,error,&boxPoints))return 29;
    if(mismatchedSupportScene.patches.front().boundaryStatus!=reconstruction::SceneBoundaryStatus::Ambiguous)return 30;
    auto ambiguousSource=analysis;
    auto duplicateDecision=ambiguousSource.sourceSupportValidation.decisions.front();
    duplicateDecision.patchId=999;
    ambiguousSource.sourceSupportValidation.decisions.push_back(std::move(duplicateDecision));
    reconstruction::SurfaceScene ambiguousSourceScene;
    if(!reconstruction::exportSurfaceScene(boxPoints,ambiguousSource,ambiguousSourceScene,error,&boxPoints))return 34;
    if(ambiguousSourceScene.patches.front().boundaryStatus!=reconstruction::SceneBoundaryStatus::Ambiguous)return 35;
    auto malformedFootprint=analysis;
    auto badMesh=std::make_shared<reconstruction::SurfaceFootprint>(
        *malformedFootprint.sourceSupportValidation.decisions.front().footprint);
    badMesh->triangles.front()[2]=999;
    malformedFootprint.sourceSupportValidation.decisions.front().footprint=std::move(badMesh);
    reconstruction::SurfaceScene malformedScene;
    if(!reconstruction::exportSurfaceScene(boxPoints,malformedFootprint,malformedScene,error,&boxPoints))return 31;
    if(malformedScene.patches.front().boundaryStatus!=reconstruction::SceneBoundaryStatus::Ambiguous)return 32;
    auto foreignSupport=analysis;
    foreignSupport.patches.front().supportPointIds.front()=999999;
    reconstruction::SurfaceScene rejectedScene;
    if(reconstruction::exportSurfaceScene(boxPoints,foreignSupport,rejectedScene,error,&boxPoints)||
       error.find("MappingAmbiguous")==std::string::npos)return 33;
    if(exported.patches.size()!=exportedAgain.patches.size())return 19;
    for(std::size_t i=0;i<exported.patches.size();++i) {
        const auto& actual=exported.patches[i];
        const auto& repeated=exportedAgain.patches[i];
        if(actual.id!=repeated.id||actual.boundaryStatus!=repeated.boundaryStatus||
           actual.supportPointIds!=repeated.supportPointIds||actual.loops.size()!=repeated.loops.size())return 20;
        if(actual.boundaryStatus!=reconstruction::SceneBoundaryStatus::ObservedClosed||
           actual.loops.size()!=1||actual.loops.front().role!=reconstruction::SceneLoopRole::Outer)return 21;
        for(std::size_t loop=0;loop<actual.loops.size();++loop) {
            const auto& actualLoop=actual.loops[loop];
            const auto& repeatedLoop=repeated.loops[loop];
            if(actualLoop.role!=repeatedLoop.role||actualLoop.closed!=repeatedLoop.closed||
               actualLoop.vertices.size()!=repeatedLoop.vertices.size())return 27;
            for(std::size_t vertex=0;vertex<actualLoop.vertices.size();++vertex) {
                const auto a=actualLoop.vertices[vertex],b=repeatedLoop.vertices[vertex];
                if(a.x!=b.x||a.y!=b.y||a.z!=b.z)return 28;
            }
        }
    }
    std::vector<reconstruction::PatchId> boxPatches;
    for(const auto& exportedPatch:exported.patches)boxPatches.push_back(exportedPatch.id);
    reconstruction::SolidCandidate observedBox;
    if(!reconstruction::proposeObservedShell(exported,boxPatches,observedBox,error)) {
        std::cerr<<"box shell: "<<error<<'\n';return 22;
    }
    reconstruction::SolidCandidate scoredBox;
    if(!reconstruction::scoreSolidCandidate(observedBox,exported,boxPoints,.05,scoredBox,error)) {
        std::cerr<<"box score: "<<error<<'\n';return 23;
    }
    if(scoredBox.status!=reconstruction::SolidCandidateStatus::Accepted||
       scoredBox.observationCoverage<.99||!scoredBox.unresolvedPoints.empty())return 24;
    auto inconsistentRelationScene=exported;
    inconsistentRelationScene.relations.push_back({1,2,
        reconstruction::SurfaceRelationKind::Perpendicular,0,1,false});
    reconstruction::SolidCandidate relationScore;
    if(!reconstruction::scoreSolidCandidate(observedBox,inconsistentRelationScene,
        boxPoints,.05,relationScore,error))return 44;
    if(relationScore.relationComparisons!=1||relationScore.relationError<.99)return 45;
    reconstruction::SolidInferenceResult inferredBox;
    if(!reconstruction::inferSolids(exported,inferredBox,error))return 36;
    const auto observedShellCount=std::count_if(inferredBox.candidates.begin(),inferredBox.candidates.end(),
        [](const auto& item) {
            return item.supportingPatches.size()==6&&!item.inferredSideWalls&&
                item.status==reconstruction::SolidCandidateStatus::Proposed;
        });
    if(observedShellCount!=1)return 37;
    auto twoBoxes=exported;
    for(std::size_t i=0;i<exported.surfaces.size();++i) {
        auto translated=exported.surfaces[i];
        auto& equation=std::get<reconstruction::PlaneEvidence>(translated);
        equation.plane.offset-=equation.plane.normal.x*100;
        twoBoxes.surfaces.push_back(std::move(translated));
        auto patch=exported.patches[i];
        patch.id+=100;
        patch.surfaceId+=exported.surfaces.size();
        for(auto& id:patch.supportPointIds)id+=10000;
        patch.centroid.x+=100;
        for(auto& loop:patch.loops)for(auto& vertex:loop.vertices)vertex.x+=100;
        twoBoxes.patches.push_back(std::move(patch));
    }
    reconstruction::SolidInferenceResult inferredTwoBoxes;
    if(!reconstruction::inferSolids(twoBoxes,inferredTwoBoxes,error))return 38;
    const auto independentShellCount=std::count_if(inferredTwoBoxes.candidates.begin(),inferredTwoBoxes.candidates.end(),
        [](const auto& item) {
            return item.supportingPatches.size()==6&&!item.inferredSideWalls&&
                item.status==reconstruction::SolidCandidateStatus::Proposed;
        });
    if(independentShellCount!=2)return 39;

    // Exercise the production detector and support validation with no injected faces or footprints.
    reconstruction::SurfaceAnalysisOptions detectedOptions;
    detectedOptions.detection.detectCylinders=false;
    detectedOptions.detection.detectSpheres=false;
    detectedOptions.detection.detectCones=false;
    detectedOptions.detection.detectTori=false;
    detectedOptions.detection.plane.distanceThreshold=.02;
    detectedOptions.detection.plane.minimumSupportPoints=20;
    detectedOptions.detection.plane.clusterEpsilon=8;
    detectedOptions.relations.distanceTolerance=.05;
    reconstruction::SurfaceAnalysisResult detectedBox;
    if(!reconstruction::analyzeSurfaces(boxPoints,detectedOptions,detectedBox,error)) {
        std::cerr<<"box detection: "<<error<<'\n';
        return 60;
    }
    reconstruction::SurfaceScene detectedScene;
    if(!reconstruction::exportSurfaceScene(boxPoints,detectedBox,detectedScene,error)) {
        std::cerr<<"detected box export: "<<error<<'\n';
        return 61;
    }
    if(detectedBox.detectionPasses!=1||detectedScene.patches.size()!=6||
       std::any_of(detectedScene.patches.begin(),detectedScene.patches.end(),[](const auto& face) {
           return face.boundaryStatus!=reconstruction::SceneBoundaryStatus::ObservedClosed||
               !face.orientationKnown||face.supportPointIds.empty();
       })) {
        std::cerr<<"detected box scene: "<<detectedBox.detectionPasses<<" passes, "
                 <<detectedScene.patches.size()<<" patches\n";
        for(const auto& face:detectedScene.patches)
            std::cerr<<"patch "<<face.id<<", "<<face.supportPointIds.size()<<" points, "
                     <<face.loops.size()<<" loops\n";
        return 62;
    }
    reconstruction::SolidInferenceResult detectedInference;
    if(!reconstruction::inferSolids(detectedScene,detectedInference,error,{},&boxPoints))return 63;
    const auto detectedShell=std::find_if(detectedInference.candidates.begin(),detectedInference.candidates.end(),
        [](const auto& item) {return item.supportingPatches.size()==6&&!item.inferredSideWalls;});
    if(detectedShell==detectedInference.candidates.end()) {
        std::cerr<<"detected box did not produce a six-face shell\n";
        return 64;
    }
    reconstruction::SolidCandidate detectedScore;
    if(!reconstruction::scoreSolidCandidate(*detectedShell,detectedScene,boxPoints,.05,detectedScore,error)||
       detectedScore.status!=reconstruction::SolidCandidateStatus::Accepted||
       detectedScore.observationCoverage<.99)return 65;

    std::vector<reconstruction::PointSample> sphereSamples;
    constexpr double sphereRadius=15;
    const double spherePi=std::acos(-1.0);
    for(int latitude=0;latitude<24;++latitude) {
        const double elevation=-spherePi/2+spherePi*(latitude+.5)/24;
        for(int longitude=0;longitude<48;++longitude) {
            const double azimuth=2*spherePi*(longitude+.5)/48;
            const modeling::Vec3 normal{std::cos(elevation)*std::cos(azimuth),
                std::cos(elevation)*std::sin(azimuth),std::sin(elevation)};
            reconstruction::PointSample sample;
            sample.id=sphereSamples.size()+1;
            sample.position={sphereRadius*normal.x,sphereRadius*normal.y,sphereRadius*normal.z};
            sample.normal=normal;
            sphereSamples.push_back(sample);
        }
    }
    const reconstruction::PointStore spherePoints(std::move(sphereSamples));
    reconstruction::SurfaceScene sphereScene;
    sphereScene.header.unit=reconstruction::LengthUnit::Millimeter;
    sphereScene.header.analysisFingerprint=reconstruction::pointStoreFingerprint(spherePoints);
    sphereScene.header.sourceFingerprint=sphereScene.header.analysisFingerprint;
    sphereScene.header.distanceToleranceMm=.05;
    sphereScene.header.angularToleranceRad=.02;
    reconstruction::SphereEvidence sphereEvidence;
    sphereEvidence.id=1;
    sphereEvidence.sphere={{0,0,0},sphereRadius};
    sphereEvidence.confidence=1;
    sphereEvidence.coverage=1;
    for(const auto& sample:spherePoints.points())sphereEvidence.supportPointIds.push_back(sample.id);
    sphereScene.surfaces.emplace_back(sphereEvidence);
    reconstruction::ScenePatch spherePatch;
    spherePatch.id=1;
    spherePatch.surfaceId=1;
    spherePatch.supportPointIds=sphereEvidence.supportPointIds;
    spherePatch.supportConfidence=1;
    spherePatch.boundaryStatus=reconstruction::SceneBoundaryStatus::Ambiguous;
    sphereScene.patches.push_back(std::move(spherePatch));
    reconstruction::SolidInferenceResult sphereInference;
    if(!reconstruction::inferSolids(sphereScene,sphereInference,error,{},&spherePoints)) {
        std::cerr<<"curved candidate inference: "<<error<<'\n';
        return 66;
    }
    const auto sphereCandidate=std::find_if(sphereInference.candidates.begin(),sphereInference.candidates.end(),
        [](const auto& candidate) {return candidate.curvedSolid.has_value();});
    if(sphereCandidate==sphereInference.candidates.end())return 67;
    reconstruction::SolidCandidate sphereScore;
    if(!reconstruction::scoreSolidCandidate(*sphereCandidate,sphereScene,spherePoints,.05,sphereScore,error)||
       sphereScore.status!=reconstruction::SolidCandidateStatus::Accepted||sphereScore.observationCoverage<.99) {
        std::cerr<<"curved candidate scoring: "<<error<<'\n';
        return 68;
    }
    reconstruction::CompoundCandidate sphereCompound;
    sphereCompound.curvedSolids.push_back(*sphereScore.curvedSolid);
    modeling::ModelingCore sphereModel;
    const auto sphereCommit=reconstruction::commitCompoundCandidate(sphereCompound,sphereModel);
    if(!sphereCommit.success||sphereModel.bodyShape().IsNull()||sphereModel.features().empty()) {
        std::cerr<<"curved candidate commit: "<<sphereCommit.error<<'\n';
        return 69;
    }

    std::vector<reconstruction::PointSample> coneSamples;
    reconstruction::ConeEvidence coneEvidence;
    coneEvidence.id=1;
    coneEvidence.cone={{0,0,22.5},{0,0,-1},std::atan(.4)};
    coneEvidence.angularCoverage=1;
    coneEvidence.confidence=1;
    const auto appendConePoint=[&](modeling::Vec3 position,modeling::Vec3 normal,bool side) {
        reconstruction::PointSample sample;
        sample.id=coneSamples.size()+1;
        sample.position=position;
        sample.normal=normal;
        coneSamples.push_back(sample);
        if(side)coneEvidence.supportPointIds.push_back(sample.id);
        return sample.id;
    };
    for(int axial=0;axial<30;++axial) {
        const double t=axial+.5,radius=.4*t;
        for(int angular=0;angular<48;++angular) {
            const double angle=2*spherePi*angular/48;
            const modeling::Vec3 radial{std::cos(angle),std::sin(angle),0};
            const modeling::Vec3 normal{radial.x/std::sqrt(1.16),radial.y/std::sqrt(1.16),.4/std::sqrt(1.16)};
            appendConePoint({radius*radial.x,radius*radial.y,22.5-t},normal,true);
        }
    }
    reconstruction::PlaneEvidence coneCap;
    coneCap.id=2;
    coneCap.plane={{0,0,-1},-7.5};
    coneCap.confidence=1;
    coneCap.coverage=1;
    for(int radial=0;radial<12;++radial)for(int angular=0;angular<48;++angular) {
        const double angle=2*spherePi*angular/48;
        const double radius=12*(radial+.5)/12;
        coneCap.supportPointIds.push_back(appendConePoint(
            {radius*std::cos(angle),radius*std::sin(angle),-7.5},{0,0,-1},false));
    }
    const reconstruction::PointStore conePoints(std::move(coneSamples));
    reconstruction::SurfaceScene coneScene;
    coneScene.header.unit=reconstruction::LengthUnit::Millimeter;
    coneScene.header.analysisFingerprint=reconstruction::pointStoreFingerprint(conePoints);
    coneScene.header.sourceFingerprint=coneScene.header.analysisFingerprint;
    coneScene.header.distanceToleranceMm=.05;
    coneScene.header.angularToleranceRad=.02;
    coneScene.surfaces={coneEvidence,coneCap};
    reconstruction::ScenePatch coneWallPatch;
    coneWallPatch.id=1;
    coneWallPatch.surfaceId=1;
    coneWallPatch.supportPointIds=coneEvidence.supportPointIds;
    coneWallPatch.supportConfidence=1;
    coneWallPatch.spacingMm=1;
    coneScene.patches.push_back(coneWallPatch);
    reconstruction::ScenePatch coneCapPatch;
    coneCapPatch.id=2;
    coneCapPatch.surfaceId=2;
    coneCapPatch.supportPointIds=coneCap.supportPointIds;
    coneCapPatch.supportConfidence=1;
    coneCapPatch.spacingMm=1;
    coneScene.patches.push_back(coneCapPatch);
    reconstruction::SolidInferenceResult coneInference;
    if(!reconstruction::inferSolids(coneScene,coneInference,error,{},&conePoints)) {
        std::cerr<<"pointed cone inference: "<<error<<'\n';
        return 70;
    }
    const auto coneCandidate=std::find_if(coneInference.candidates.begin(),coneInference.candidates.end(),
        [](const auto& candidate) {return candidate.curvedSolid.has_value();});
    if(coneCandidate==coneInference.candidates.end()) {
        std::cerr<<"pointed cone candidate: "<<error<<'\n';
        return 71;
    }
    reconstruction::SolidCandidate coneScore;
    if(!reconstruction::scoreSolidCandidate(*coneCandidate,coneScene,conePoints,.05,coneScore,error)||
       coneScore.status!=reconstruction::SolidCandidateStatus::Accepted) {
        std::cerr<<"pointed cone scoring: "<<error<<'\n';
        return 72;
    }
    std::cout << "surface scene contract passed\n";
    return 0;
}
