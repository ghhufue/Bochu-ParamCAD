#include <reconstruction/preprocessing/PointCloudPreprocessor.h>
#include <reconstruction/modeling/ModelingAdapter.h>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
using Clock = std::chrono::steady_clock;
template<class F> double median(F work) {
    work();
    std::vector<double> times;
    for (int i=0;i<3;++i) {
        const auto start=Clock::now();work();
        times.push_back(std::chrono::duration<double,std::milli>(Clock::now()-start).count());
    }
    std::sort(times.begin(),times.end());return times[1];
}
}
int main() {
    try {
        std::cout << std::setprecision(12) << "{\"preprocessing\":[";
        bool first=true;
        for (const auto count : {100000U,400000U}) {
            std::vector<reconstruction::PointSample> observations(count);
            for (std::size_t i=0;i<count;++i) {
                auto& p=observations[i];p.id=i+1;
                p.position={double(i%701)*.05,double(i/701)*.05,.02*std::sin(double(i%701)*.02)};
                if (i%997==0)p.position.z+=20;
                if (i%5==0)p.normal=modeling::Vec3{0,0,-1};
            }
            const reconstruction::PointStore source(std::move(observations));
            reconstruction::PointCloudPreprocessingOptions options;
            reconstruction::PointStore output({});
            reconstruction::PointCloudPreprocessingReport report;std::string error;
            std::uint64_t fingerprint=0;
            const double ms=median([&] {
                if(!reconstruction::preprocessPointCloud(source,options,output,report,error))throw std::runtime_error(error);
                const auto current=reconstruction::pointStoreFingerprint(output);
                if(fingerprint && fingerprint!=current)throw std::runtime_error("Non-deterministic preprocessing");
                fingerprint=current;
            });
            if(!first)std::cout<<',';first=false;
            std::cout<<"{\"input_points\":"<<count<<",\"median_ms\":"<<ms
                <<",\"output_points\":"<<output.size()<<",\"fingerprint\":\""<<fingerprint
                <<"\",\"estimated_normals\":"<<report.normalsEstimated
                <<",\"unoriented_normals\":"<<report.normalsWithoutOrientation<<'}';
        }
        reconstruction::BoxWithThroughHolesCandidate candidate;
        candidate.box.primitive.sizeX=80;candidate.box.primitive.sizeY=60;candidate.box.primitive.sizeZ=20;
        candidate.box.confidence=1;candidate.confidence=1;
        for (int i=0;i<6;++i) {
            reconstruction::ThroughHoleCandidate hole;
            hole.primitive.axisPoint={15.0+(i%3)*25,15.0+(i/3)*30,0};
            hole.primitive.axisDirection={0,0,1};hole.primitive.radius=4;
            hole.bodyThickness=20;hole.confidence=1;candidate.throughHoles.push_back(hole);
        }
        double volume=0;
        const double ms=median([&] {
            modeling::ModelingCore model;
            const auto result=reconstruction::commitBoxWithThroughHolesCandidate(candidate,model,0);
            if(!result.success)throw std::runtime_error(result.error);
            if(!BRepCheck_Analyzer(model.bodyShape()).IsValid())throw std::runtime_error("Invalid B-Rep");
            GProp_GProps properties;BRepGProp::VolumeProperties(model.bodyShape(),properties);
            volume=properties.Mass();
            if(std::abs(volume-(80*60*20-6*3.14159265358979323846*16*20))>1e-5)
                throw std::runtime_error("Incorrect six-hole volume");
        });
        std::cout<<"],\"cad_six_holes\":{\"median_ms\":"<<ms<<",\"volume_mm3\":"<<volume<<"}}\n";
        return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
