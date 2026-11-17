#include "AssemblyDiagnostics.h"
#include "io/PointCloudAdapter.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <cmath>
#include <stdexcept>
void probeAssemblyBoundaries(const QStringList& args,QJsonObject& report) {
    if(args.size()!=5)throw std::runtime_error("Usage: probe-assembly cloud.ply report.json options.json");
    QFile configuration(args[4]);
    if(!configuration.open(QIODevice::ReadOnly))throw std::runtime_error("Cannot read probe options");
    QJsonParseError parseError;
    const auto document=QJsonDocument::fromJson(configuration.readAll(),&parseError);
    if(parseError.error!=QJsonParseError::NoError || !document.isObject())throw std::runtime_error("Expected probe options object");
    const auto values=document.object();reconstruction::AssemblyOptions options;
    options.maximumProjectionParts = 0;
    for(auto it=values.begin();it!=values.end();++it) {
        if(!it.value().isDouble() || !std::isfinite(it.value().toDouble()))throw std::runtime_error("Probe options must be finite numbers");
        const auto value=it.value().toDouble();
        if(it.key()=="maximum_working_points") {
            if(value<24 || value>2000000 || value!=std::floor(value))throw std::runtime_error("Probe point budget must be an integer in [24, 2000000]");
            options.maximumWorkingPoints=size_t(value);
        } else if(it.key()=="alpha_factor")options.alphaFactor=value;
        else if(it.key()=="connectivity_factor")options.connectivityFactor=value;
        else if(it.key()=="distance_tolerance_mm")options.distanceTolerance=value;
        else if(it.key()=="normal_angle_degrees")options.normalAngleRadians=value*std::acos(-1.0)/180;
        else throw std::runtime_error("Unknown assembly probe option");
    }
    const auto unit=PointCloudAdapter::lengthUnitFor(args[2]);
    if(unit==reconstruction::LengthUnit::Arbitrary)throw std::runtime_error("Probe input requires declared mm/m units");
    PlyCloud cloud;QString failure;
    if(!PlyReader::read(args[2],cloud,failure))throw std::runtime_error(failure.toStdString());
    const reconstruction::ObservationSnapshot input(PointCloudAdapter::toPointStore(cloud,unit));
    reconstruction::AssemblyReconstructionResult result;std::string error;
    if(!reconstruction::reconstructAssembly(input.observations(),options,result,error))
        throw std::runtime_error(error);
    report["input_points"]=double(cloud.positions.size());
    report["source_model_read"]=false;report["reference_labels_used"]=false;
    report["probe_options"]=QJsonObject{{"maximum_working_points",double(options.maximumWorkingPoints)},
        {"alpha_factor",options.alphaFactor},{"connectivity_factor",options.connectivityFactor},
        {"normal_angle_degrees",options.normalAngleRadians*180/std::acos(-1.0)},
        {"distance_tolerance_mm",options.distanceTolerance}};
    report["assembly"]=writeAssemblyDiagnostics(args[3],cloud,result,unit==reconstruction::LengthUnit::Meter?"m":"mm");
    report["status"]="assembly_probe_completed";
    // No CAD export or source-completeness claim is made by this diagnostic mode.
}
