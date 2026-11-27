#include "OfflineCadCommands.h"
#include "io/AssemblyDiagnostics.h"
#include "app/CadReconstructionTask.h"
#include "io/PointCloudAdapter.h"
#include "io/ProjectStore.h"
#include "io/FeatureCodec.h"
#include "io/StepModelExporter.h"
#include <reconstruction/data/StageProfile.h>
#include <reconstruction/modeling/ModelingAdapter.h>
#include <reconstruction/parts/ProjectionExtrusion.h>
#include <reconstruction/parts/ProjectionSettings.h>
#include <modeling/StepExporter.h>
#include <BRepCheck_Analyzer.hxx>
#include <BRepBndLib.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <QEventLoop>
#include <QFileInfo>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <filesystem>
#include <fstream>
#include <optional>
#include <cmath>
#include <stdexcept>

namespace {
void require(bool value,const std::string& message){if(!value)throw std::runtime_error(message);}
void save(const QString& path,const QJsonObject& report){QSaveFile file(path);require(file.open(QIODevice::WriteOnly),"Cannot save cloud-only report");
    const auto data=QJsonDocument(report).toJson();require(file.write(data)==data.size() && file.commit(),"Cannot commit cloud-only report");}
int solids(const TopoDS_Shape& shape){int count=0;for(TopExp_Explorer i(shape,TopAbs_SOLID);i.More();i.Next())++count;return count;}
void exportCad(const modeling::ModelingCore& core, const QString& path, QJsonObject& report);
void exportClosedBrep(const TopoDS_Solid& solid, const QString& path, QJsonObject& report);

struct ProjectionArguments {
    reconstruction::ProjectionExtrusionOptions options;
    std::filesystem::path file;
    bool projectionFallback = true;
    bool specializedRecovery = true;
    bool legacyAssembly = false;
    bool parametric = false;
    reconstruction::BrepClosureOptions closure;
    double toleranceMm = 0;
    QString path = QStringLiteral("auto");
};

ProjectionArguments projectionArguments(const QStringList& args, bool assembly, bool defaultToClosedBrep = true)
{
    ProjectionArguments result;
    QStringList overrides;
    bool projectionSpecified = false, pathSpecified = false;
    bool areaSpecified = false, toleranceSpecified = false;
    for (qsizetype i = 5; i < args.size(); ++i) {
        const auto argument = args[i];
        const auto readValue = [&]() {
            require(i+1 < args.size() && !args[i+1].startsWith("--"),
                    "Missing value for " + argument.toStdString());
            return args[++i];
        };
        if (argument == "--config") {
            require(result.file.empty(),"Duplicate --config");
            result.file = std::filesystem::path(readValue().toStdU16String());
        } else if (argument == "--set") {
            overrides.append(readValue());
        } else if (argument == "--path") {
            require(!pathSpecified,"Duplicate --path");
            result.path = readValue();
            pathSpecified = true;
        } else if (assembly && argument == "--legacy-assembly" && !result.legacyAssembly) {
            result.legacyAssembly = true;
        } else if (assembly && argument == "--parametric" && !result.parametric) {
            result.parametric = true;
        } else if (assembly && argument == "--parallel-bop" && !result.closure.enableParallelBop) {
            result.closure.enableParallelBop = true;
        } else if (assembly && argument == "--partition-holes" && result.closure.separateRegularHoles) {
            result.closure.separateRegularHoles = false;
        } else if (assembly && argument == "--tolerance-mm" && !toleranceSpecified) {
            bool parsed = false;
            result.toleranceMm = readValue().toDouble(&parsed);
            require(parsed && std::isfinite(result.toleranceMm) && result.toleranceMm > 0,
                    "Expected a positive tolerance in millimetres");
            toleranceSpecified = true;
        } else if (assembly && argument == "--no-specialized-recovery" && result.specializedRecovery) {
            result.specializedRecovery = false;
        } else if (assembly && !projectionSpecified &&
                   (argument == "--projection-fallback" || argument == "--no-projection-fallback")) {
            result.projectionFallback = argument == "--projection-fallback";
            projectionSpecified = true;
        } else if (!assembly && argument == "--area-directions" && !areaSpecified) {
            areaSpecified = true;
            overrides.append(QStringLiteral("include_area_directions=true"));
        } else if (!assembly && !toleranceSpecified && !argument.startsWith("--")) {
            bool parsed = false;
            const double tolerance = argument.toDouble(&parsed);
            require(parsed && std::isfinite(tolerance) && tolerance > 0,"Expected a positive tolerance in millimetres");
            toleranceSpecified = true;
            overrides.append(QStringLiteral("distance_tolerance_mm=") + argument);
        } else {
            throw std::invalid_argument("Unknown or repeated option: " + argument.toStdString());
        }
    }
    if (assembly) {
        if (!defaultToClosedBrep && !result.legacyAssembly) result.parametric = true;
        require(!(result.legacyAssembly && result.parametric), "Choose either --legacy-assembly or --parametric");
        if (!result.legacyAssembly && !result.parametric) {
            require(result.file.empty() && overrides.empty() && !pathSpecified,
                    "--config, --set and --path require --legacy-assembly or --parametric");
            return result;
        }
        require(!result.closure.enableParallelBop && result.closure.separateRegularHoles,
                "--parallel-bop and --partition-holes apply to closed B-Rep reconstruction");
    }
    if (result.file.empty()) result.file = reconstruction::projectionConfigPath();
    result.options = reconstruction::loadProjectionOptions(result.file);
    for (const auto& assignment : overrides)
        reconstruction::setProjectionOption(result.options,assignment.toStdString());
    using Path = reconstruction::ProjectionPath;
    if (result.path == "single-axis") result.options.path = Path::SingleAxis;
    else if (result.path == "two-axis-intersection") result.options.path = Path::TwoAxisIntersection;
    else if (result.path == "segmented") result.options.path = Path::Segmented;
    else require(result.path == "auto","Unknown projection path. Use auto, single-axis, two-axis-intersection or segmented");
    reconstruction::validateProjectionOptions(result.options);
    return result;
}

void reportProjectionSettings(QJsonObject& report, const ProjectionArguments& arguments)
{
    report["projection_config"] = QString::fromStdU16String(arguments.file.u16string());
    report["projection_path"] = arguments.path;
    QJsonObject settings;
    for (const auto& setting : reconstruction::projectionSettings()) {
        const auto value = setting.value(arguments.options);
        settings[QString::fromLatin1(setting.key)] = setting.type == reconstruction::ProjectionSetting::Type::Boolean
            ? QJsonValue(value != 0) : QJsonValue(value);
    }
    report["projection_settings"] = settings;
}
}
void reconstructOfflineCad(const QStringList& args, QJsonObject& report, bool defaultToClosedBrep) {
    using reconstruction::diagnostics::profileStage;
    const reconstruction::diagnostics::StageProfile profile("cad.offline");
    require(args.size()>=5,
            "Usage: reconstruct-cloud cloud.ply report.json output.step [--parallel-bop] [--partition-holes] [--tolerance-mm N] [--parametric | --legacy-assembly]");
    const auto arguments = projectionArguments(args, true, defaultToClosedBrep);
    const auto projectionFallback = arguments.projectionFallback;
    const auto specializedRecovery = arguments.specializedRecovery;
    const bool closedBrep = !arguments.parametric && !arguments.legacyAssembly;
    if (!closedBrep) reportProjectionSettings(report,arguments);
    const auto unit=PointCloudAdapter::lengthUnitFor(args[2]);
    require(unit!=reconstruction::LengthUnit::Arbitrary,"Point cloud needs mm/m unit metadata; declare its unit explicitly");
    report["input_kind"]="point_cloud_only";report["source_model_read"]=false;report["topology_correspondence_used"]=false;
    report["legacy_assembly"]=arguments.legacyAssembly;
    report["history_free"]=closedBrep;
    report["specialized_recovery_enabled"]=arguments.legacyAssembly && specializedRecovery;
    report["projection_fallback_enabled"]=arguments.legacyAssembly && projectionFallback;
    report["reference_labels_used"]=false;report["source_completeness_verified"]=false;report["cad_constructed"]=false;
    report["cloud"]=QFileInfo(args[2]).absoluteFilePath();report["unit"]=unit==reconstruction::LengthUnit::Millimeter?"mm":"m";
    auto stage=[&](const QString& name){report["stage"]=name;report["status"]="running";save(args[3],report);};
    stage("loading_cloud");PlyCloud cloud;QString error;
    const bool loaded = profileStage("cad.load_cloud", [&] { return PlyReader::read(args[2], cloud, error); });
    require(loaded, error.toStdString());
    report["input_points"]=double(cloud.positions.size());
    CadReconstructionTask task;QEventLoop loop;std::optional<CadReconstructionResult> result;
    QObject::connect(&task,&CadReconstructionTask::stageChanged,&loop,[&](auto value){
        stage(value==CadReconstructionTask::Stage::Preprocessing?"preprocessing":
              value==CadReconstructionTask::Stage::ClosingBrep?"partitioning_brep":
              value==CadReconstructionTask::Stage::SeparatingParts?"separating_and_closing_parts":"recognizing_from_points");});
    QObject::connect(&task,&CadReconstructionTask::finished,&loop,[&](const auto& value){result=value;loop.quit();});
    std::optional<reconstruction::BrepClosureOptions> closure;
    if (closedBrep) closure = arguments.closure;
    const auto sourceTolerance = arguments.toleranceMm / reconstruction::millimetersPerUnit(unit).value();
    require(task.start(cloud,unit,{},sourceTolerance,{},{},projectionFallback,specializedRecovery,arguments.options,
                       false,arguments.legacyAssembly,{},closure),
        "Cannot start point-cloud worker");
    if(!result)loop.exec();require(result.has_value(),"Point-cloud worker returned no result");
    report["cache"] = result->cache;
    if(result->preprocessingComplete)report["preprocessing"]=QJsonObject{{"input_points",double(result->preprocessing.inputPointCount)},
        {"output_points",double(result->preprocessing.outputPointCount)},{"removed_voxel_duplicates",double(result->preprocessing.removedByVoxel)},
        {"removed_outliers",double(result->preprocessing.removedAsOutliers)},{"estimated_normals",double(result->preprocessing.normalsEstimated)}};
    using Branch = CadReconstructionResult::Branch;
    switch (result->branch) {
    case Branch::None: report["recognition_branch"] = "none"; break;
    case Branch::Conventional: report["recognition_branch"] = "conventional"; break;
    case Branch::Guided: report["recognition_branch"] = "guided"; break;
    case Branch::Assembly: report["recognition_branch"] = "assembly"; break;
    case Branch::ClosedBrep: report["recognition_branch"] = "closed_brep"; break;
    }
    report["detected_surface_patches"]=double(result->analysis.patches.size());
    report["unresolved_component_points"]=double(result->candidate.unresolvedPointIds.size());
    if (result->closedBrep) {
        report["brep_quality"] = closedBrepReport(*result->closedBrep);
        report["distance_tolerance_mm"] = result->analysis.options.detection.plane.distanceThreshold;
    }
    if (result->assembly) {
        stage("writing_separation_diagnostics");
        report["assembly"] = profileStage("cad.write_diagnostics", [&] {
            return writeAssemblyDiagnostics(args[3], cloud, *result->assembly, report["unit"].toString());
        });
    }
    if(result->outcome!=CadReconstructionResult::Outcome::Success) {
        report["status"]=result->outcome==CadReconstructionResult::Outcome::Unsupported?"unsupported":"failed";
        report["error"]=result->error;return;
    }
    if (closedBrep) {
        require(result->closedBrep && !result->closedBrep->solid.IsNull(), "Closed B-Rep worker returned no solid");
        stage("exporting_cad");
        profileStage("cad.export", [&] { exportClosedBrep(result->closedBrep->solid, args[4], report); });
        return;
    }
    stage("building_parametric_cad");modeling::ModelingCore core;
    profileStage("cad.build_and_validate", [&] {
        const auto committed = reconstruction::commitCompoundCandidate(result->candidate, core);
        require(committed.success, committed.error);
        require(!core.bodyShape().IsNull() && BRepCheck_Analyzer(core.bodyShape()).IsValid(), "Invalid point-cloud CAD");
    });
    report["surface_coverage"]=result->candidate.surfaceCoverage;report["features"]=double(core.features().size());
    if(result->assembly){
        report["coverage_basis"]="fraction_of_input_observations_assigned_to_closed_candidates";
        report["assigned_observation_fraction"]=result->candidate.surfaceCoverage;
    } else report["coverage_basis"]="recognizer_surface_support";
    stage("exporting_cad");
    profileStage("cad.export", [&] { exportCad(core, args[4], report); });
    if(result->partialAssembly) {
        report["status"]="cad_exported_partial";report["partial_reconstruction"]=true;
    }
}

namespace {
void writeAndCheckStep(const TopoDS_Shape& shape, const QString& path, QJsonObject& report) {
    QString error;
    report["solids"]=solids(shape);report["brep_valid"]=true;
    const bool written = StepModelExporter::write(shape, path, error);
    require(written, error.toStdString());
    STEPControl_Reader reader;std::ifstream input(std::filesystem::path(path.toStdWString()),std::ios::binary);
    require(input.good() && reader.ReadStream("exported.step",input)==IFSelect_RetDone && reader.TransferRoots()>0,"Cannot read exported CAD");
    require(solids(reader.OneShape())==solids(shape) && BRepCheck_Analyzer(reader.OneShape()).IsValid(),"Exported CAD round trip lost valid solids");
    report["step_round_trip_verified"]=true;
}

ProjectSnapshot projectSnapshot(const TopoDS_Shape& shape) {
    ProjectSnapshot snapshot;
    snapshot.currentBody = shape;
    snapshot.view.originLock = false;
    Bnd_Box box;BRepBndLib::AddOptimal(shape,box,false,false);double x0,y0,z0,x1,y1,z1;box.Get(x0,y0,z0,x1,y1,z1);
    const double span=std::hypot(x1-x0,y1-y0,z1-z0);
    for(int i=0;i<3;++i)snapshot.view.camera.target[i]=(i==0?x0+x1:i==1?y0+y1:z0+z1)*.5;
    snapshot.view.camera.position[0]=snapshot.view.camera.target[0]+span;snapshot.view.camera.position[1]=snapshot.view.camera.target[1]-span;
    snapshot.view.camera.position[2]=snapshot.view.camera.target[2]+span;
    return snapshot;
}

void exportClosedBrep(const TopoDS_Solid& solid, const QString& path, QJsonObject& report) {
    writeAndCheckStep(solid, path, report);
    const auto info = QFileInfo(path);
    const auto brepPath = info.dir().filePath(info.completeBaseName() + ".brep");
    QString error;
    const bool nativeWritten = StepModelExporter::writeBrep(solid, brepPath, error);
    require(nativeWritten, error.toStdString());
    std::ifstream input(std::filesystem::path(brepPath.toStdWString()), std::ios::binary);
    TopoDS_Shape native;
    BRep_Builder builder;
    BRepTools::Read(native, input, builder);
    require(!input.fail() && !native.IsNull() && native.ShapeType() == TopAbs_SOLID &&
            BRepCheck_Analyzer(native).IsValid(), "Native B-Rep round trip lost the valid solid");
    const auto project = info.dir().filePath(info.completeBaseName() + ".pcad");
    const bool saved = ProjectStore::save(project, projectSnapshot(solid), error);
    require(saved, error.toStdString());
    const auto restored = ProjectStore::load(project, error);
    require(restored && restored->model.features().empty() && !restored->currentBody.IsNull() &&
            restored->currentBody.ShapeType() == TopAbs_SOLID, "History-free project round trip failed: " + error.toStdString());
    report["features"] = 0;
    report["step"] = info.absoluteFilePath();
    report["brep"] = brepPath;
    report["project"] = project;
    report["brep_round_trip_verified"] = true;
    report["project_round_trip_verified"] = true;
    report["cad_constructed"] = true;
    report["status"] = "cad_exported";
}

void exportCad(const modeling::ModelingCore& core, const QString& path, QJsonObject& report) {
    QString error;
    writeAndCheckStep(core.bodyShape(), path, report);
    const auto info=QFileInfo(path);const auto projectPath=info.dir().filePath(info.completeBaseName()+".pcad");
    const auto featuresPath=info.dir().filePath(info.completeBaseName()+".features.json");
    QSaveFile definitions(featuresPath);require(definitions.open(QIODevice::WriteOnly),"Cannot save feature parameters");
    const auto data=QJsonDocument(FeatureCodec::encode(core.features())).toJson();require(definitions.write(data)==data.size() && definitions.commit(),"Cannot commit feature parameters");
    auto snapshot = projectSnapshot(core.bodyShape());
    snapshot.features = core.features();
    snapshot.parametricBody = true;
    const auto saved=ProjectStore::save(projectPath,snapshot,error);
    require(saved,error.toStdString());
    const auto restored=ProjectStore::load(projectPath,error);require(restored && restored->model.features().size()==core.features().size(),error.toStdString());
    report["project_round_trip_verified"]=true;report["project"]=projectPath;report["features_file"]=featuresPath;
    report["step"]=QFileInfo(path).absoluteFilePath();report["cad_constructed"]=true;report["status"]="cad_exported";
}

QJsonArray pointIds(const std::vector<reconstruction::PointId>& ids) {
    QJsonArray result;
    for (auto id : ids) result.append(qint64(id));
    return result;
}
}

void reconstructOfflineProjection(const QStringList& args, QJsonObject& report) {
    require(args.size()>=5,
        "Usage: fit-projection cloud.ply report.json output.step [--config file] [--set key=value] [--path mode] [tolerance-mm] [--area-directions]");
    const auto arguments = projectionArguments(args,false);
    const auto& options = arguments.options;
    reportProjectionSettings(report,arguments);
    const auto unit=PointCloudAdapter::lengthUnitFor(args[2]);
    const auto scale=reconstruction::millimetersPerUnit(unit);
    require(scale.has_value(),"Point cloud needs mm/m unit metadata; declare its unit explicitly");
    PlyCloud cloud;
    QString readError;
    require(PlyReader::read(args[2],cloud,readError),readError.toStdString());
    const reconstruction::ObservationSnapshot input(PointCloudAdapter::toPointStore(cloud,unit));
    const auto& points = input.observations();
    report["input_kind"]="point_cloud_only";
    report["source_model_read"]=false;
    report["reference_labels_used"]=false;
    report["point_id_basis"]="one_based_row_in_input_ply";
    report["cloud"]=QFileInfo(args[2]).absoluteFilePath();
    report["input_points"]=qint64(points.size());
    report["area_directions_enabled"]=options.includeAreaDirections;
    report["cad_constructed"]=false;
    reconstruction::ProjectionExtrusionResult fit;
    const bool accepted=reconstruction::fitProjectionExtrusion(
        points,options,fit);
    const auto axisDiagnostics = projectionAxisDiagnostics(fit);
    for (auto field = axisDiagnostics.begin(); field != axisDiagnostics.end(); ++field)
        report[field.key()] = field.value();
    QJsonArray operations;
    for (const auto& operation : fit.operations) {
        operations.append(QJsonObject{{"operation",int(operation.operation)},
            {"depth_mm",operation.depth},{"reverse",operation.reverse},
            {"curve_count",qint64(operation.sketch.entities.size())},{"support_point_ids",pointIds(operation.supportPointIds)}});
    }
    report["operations"]=operations;
    report["distance_tolerance_mm"]=fit.distanceTolerance;
    report["quality"]=QJsonObject{{"scored_points",qint64(fit.quality.scoredPoints)},
        {"median_distance_mm",fit.quality.medianDistance},
        {"p95_distance_mm",fit.quality.p95Distance},{"surface_inlier_fraction",fit.quality.surfaceInlierFraction},
        {"unsupported_face_area_fraction",fit.quality.unsupportedFaceAreaFraction},
        {"mean_normal_error_degrees",fit.quality.meanNormalErrorDegrees},
        {"normal_comparisons",qint64(fit.quality.normalComparisons)},{"score",fit.quality.score}};
    report["sampled_residual_point_ids"]=pointIds(fit.residualPointIds);
    if (!accepted) {
        report["status"]="unsupported";
        report["error"]=QString::fromStdString(fit.reason);
        return;
    }
    modeling::ModelingCore core;
    std::string error;
    require(reconstruction::buildProjectionProgram(fit.operations,core,error),error);
    report["features"]=qint64(core.features().size());
    exportCad(core,args[4],report);
}
