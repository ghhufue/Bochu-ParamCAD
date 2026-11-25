#include "StepCloudSampler.h"
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Curve2d.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Tool.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <Poly_Triangulation.hxx>
#include <Precision.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Torus.hxx>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <set>

namespace {
void check(bool ok, const char* message) { if(!ok) throw std::runtime_error(message); }
void save(const QString& path,const QJsonObject& object) {
    QFile file(path);check(file.open(QIODevice::WriteOnly),"Cannot write sampler metadata");
    const auto data=QJsonDocument(object).toJson();check(file.write(data)==data.size(),"Cannot save sampler metadata");
}

const char* kind(GeomAbs_SurfaceType t);

struct SampledPoint { gp_Pnt point; gp_Vec normal; };

std::size_t appendFaceEdgeSamples(const TopoDS_Face& face, double spacing,
                                  std::vector<SampledPoint>& cloud)
{
    // This topology-informed edge treatment is a showcase aid for the
    // reconstruction algorithm. It deliberately over-represents sharp edges
    // and must stay disabled in correctness-validation sampling.
    const BRepAdaptor_Surface surface(face);
    const double keyTolerance = std::max(1e-9, spacing * 1e-6);
    std::set<std::array<std::int64_t, 3>> sampledPositions;
    std::size_t added = 0;
    for (TopExp_Explorer it(face, TopAbs_EDGE); it.More(); it.Next()) {
        const auto edge = TopoDS::Edge(it.Current());
        if (BRep_Tool::Degenerated(edge)) continue;

        const BRepAdaptor_Curve curve(edge, face);
        const BRepAdaptor_Curve2d curve2d(edge, face);
        const double first = curve.FirstParameter();
        const double last = curve.LastParameter();
        if (!std::isfinite(first) || !std::isfinite(last) || last <= first) continue;
        const double length = GCPnts_AbscissaPoint::Length(curve, first, last);
        if (!std::isfinite(length) || length <= Precision::Confusion()) continue;
        const double intervalCount = std::max(1.0, std::ceil(length / spacing));
        check(intervalCount < 50000000.0,
              "Requested spacing exceeds the per-edge sampling limit");
        const GCPnts_UniformAbscissa distribution(
            curve, static_cast<int>(intervalCount + 1), first, last);
        check(distribution.IsDone(), "Cannot distribute samples along STEP edge");

        for (int index = 1; index <= distribution.NbPoints(); ++index) {
            const double parameter = distribution.Parameter(index);
            const auto uv = curve2d.Value(parameter);
            gp_Pnt point;
            gp_Vec tangentU, tangentV;
            surface.D1(uv.X(), uv.Y(), point, tangentU, tangentV);
            auto normal = tangentU.Crossed(tangentV);
            if (normal.SquareMagnitude() <= 1e-24) {
                // At a sphere pole, the boundary point is valid while the UV
                // derivatives are singular. Use a limiting normal from this
                // same edge without moving the sampled XYZ position.
                const double span = last - first;
                bool foundNormal = false;
                for (const double fraction : {1e-9, 1e-8, 1e-7, 1e-6, 1e-5}) {
                    for (const double direction : {-1.0, 1.0}) {
                        const double probe = parameter + direction * span * fraction;
                        if (probe <= first || probe >= last) continue;
                        const auto probeUv = curve2d.Value(probe);
                        gp_Pnt ignoredPoint;
                        gp_Vec probeU, probeV;
                        surface.D1(probeUv.X(), probeUv.Y(), ignoredPoint, probeU, probeV);
                        normal = probeU.Crossed(probeV);
                        if (normal.SquareMagnitude() > 1e-24) {
                            foundNormal = true;
                            break;
                        }
                    }
                    if (foundNormal) break;
                }
                check(foundNormal, "Singular surface normal at STEP edge");
            }
            normal.Normalize();
            if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();

            const auto coordinateKey = [keyTolerance](double value) {
                const double scaled = std::round(value / keyTolerance);
                check(std::isfinite(scaled) && std::abs(scaled) <= 9.0e15,
                      "STEP edge coordinate exceeds sampling key range");
                return static_cast<std::int64_t>(scaled);
            };
            const std::array<std::int64_t, 3> key{
                coordinateKey(point.X()), coordinateKey(point.Y()), coordinateKey(point.Z())};
            if (!sampledPositions.insert(key).second) continue;
            cloud.push_back({point, normal});
            ++added;
        }
    }
    return added;
}

void inspectStepTopologyImpl(const QStringList& args,QJsonObject& report) {
    check(args.size()==4,"Usage: inspect-step source.step report.json");
    STEPControl_Reader reader;std::ifstream input(std::filesystem::path(args[2].toStdWString()),std::ios::binary);
    check(input.good() && reader.ReadStream("source.step",input)==IFSelect_RetDone,"Cannot read STEP");
    check(reader.TransferRoots()>0,"No STEP roots");QJsonArray solids;int id=0;
    auto vector=[](auto v){return QJsonArray{v.X(),v.Y(),v.Z()};};
    for(TopExp_Explorer si(reader.OneShape(),TopAbs_SOLID);si.More();si.Next()) {
        QJsonArray faces;int faceId=0;
        for(TopExp_Explorer fi(si.Current(),TopAbs_FACE);fi.More();fi.Next()) {
            const auto face=TopoDS::Face(fi.Current());BRepAdaptor_Surface surface(face);
            GProp_GProps props;BRepGProp::SurfaceProperties(face,props);
            QJsonObject f{{"id",++faceId},{"kind",kind(surface.GetType())},{"area_mm2",props.Mass()},
                {"center_mm",vector(props.CentreOfMass())},{"reversed",face.Orientation()==TopAbs_REVERSED}};
            f["occt_surface_type"] = int(surface.GetType());
            if(surface.GetType()==GeomAbs_Plane){const auto p=surface.Plane();f["normal"]=vector(p.Axis().Direction());}
            if(surface.GetType()==GeomAbs_Cylinder){const auto c=surface.Cylinder();f["axis"]=vector(c.Axis().Direction());f["axis_point"]=vector(c.Location());f["radius_mm"]=c.Radius();}
            if(surface.GetType()==GeomAbs_Cone){const auto c=surface.Cone();f["axis"]=vector(c.Axis().Direction());f["angle_rad"]=c.SemiAngle();f["apex_mm"]=vector(c.Apex());}
            if (surface.GetType()==GeomAbs_Torus) {
                const auto torus=surface.Torus();
                f["axis"]=vector(torus.Axis().Direction());
                f["torus_center_mm"]=vector(torus.Location());
                f["major_radius_mm"]=torus.MajorRadius();
                f["minor_radius_mm"]=torus.MinorRadius();
            }
            faces.append(f);
        }
        Bnd_Box bounds;BRepBndLib::AddOptimal(si.Current(),bounds,false,false);double x0,y0,z0,x1,y1,z1;bounds.Get(x0,y0,z0,x1,y1,z1);
        solids.append(QJsonObject{{"id",++id},{"bounds_mm",QJsonArray{x0,y0,z0,x1,y1,z1}},{"faces",faces}});
    }
    report["status"]="inspected";report["solids"]=solids;
}
const char* kind(GeomAbs_SurfaceType t) {
    switch(t) {case GeomAbs_Plane:return "plane";case GeomAbs_Cylinder:return "cylinder";
    case GeomAbs_Cone:return "cone";case GeomAbs_Sphere:return "sphere";
    case GeomAbs_Torus:return "torus";default:return "other";}
}
}

void inspectStepTopology(const QStringList& args,QJsonObject& report) {inspectStepTopologyImpl(args,report);}

namespace {
void compareObservedFaces(const QStringList& args, QJsonObject& report, bool splineOnly) {
    check(args.size()==6,"Expected fitted STEP, report path, reference STEP and tolerance in millimetres");
    bool parsed=false;
    const double tolerance=args[5].toDouble(&parsed);
    check(parsed && std::isfinite(tolerance) && tolerance>0,"Expected a positive tolerance in millimetres");
    const auto load=[](const QString& path) {
        STEPControl_Reader reader;
        std::ifstream input(std::filesystem::path(path.toStdWString()),std::ios::binary);
        check(input.good() && reader.ReadStream("observed.step",input)==IFSelect_RetDone && reader.TransferRoots()>0,
              "Cannot read spline comparison STEP");
        return reader.OneShape();
    };
    const auto isSpline=[](const TopoDS_Face& face) {
        const auto kind=BRepAdaptor_Surface(face).GetType();
        return kind==GeomAbs_BSplineSurface || kind==GeomAbs_BezierSurface;
    };
    const auto fitted=load(args[2]), reference=load(args[4]);
    struct ReferenceFace { TopoDS_Face face; Bnd_Box bounds; bool spline; };
    std::vector<ReferenceFace> references;
    for (TopExp_Explorer it(reference,TopAbs_FACE); it.More(); it.Next()) {
        const auto face=TopoDS::Face(it.Current());
        Bnd_Box bounds;
        BRepBndLib::AddOptimal(face,bounds,false,false);
        bounds.Enlarge(4*tolerance);
        references.push_back({face,bounds,isSpline(face)});
    }
    QJsonArray faces;
    bool matched=true;
    for (TopExp_Explorer it(fitted,TopAbs_FACE); it.More(); it.Next()) {
        const auto face=TopoDS::Face(it.Current());
        if (splineOnly && !isSpline(face)) continue;
        const BRepAdaptor_Surface surface(face);
        int samples=0, within=0, outsideBounds=0, otherSurfaceMatches=0;
        int consistentNormals=0;
        double maximum=0;
        gp_Pnt worstPoint;
        const auto sampleGrid=[&](int grid) {
            for (int i=0; i<grid; ++i) for (int j=0; j<grid; ++j) {
                const double u=surface.FirstUParameter()+(i+.5)*(surface.LastUParameter()-surface.FirstUParameter())/grid;
                const double v=surface.FirstVParameter()+(j+.5)*(surface.LastVParameter()-surface.FirstVParameter())/grid;
                const BRepClass_FaceClassifier classifier(face,gp_Pnt2d(u,v),1e-7);
                if (classifier.State()!=TopAbs_IN) continue;
                ++samples;
                gp_Pnt point;
                gp_Vec tangentU,tangentV;
                surface.D1(u,v,point,tangentU,tangentV);
                auto fittedNormal=tangentU.Crossed(tangentV);
                if (face.Orientation()==TopAbs_REVERSED) fittedNormal.Reverse();
                const auto vertex=BRepBuilderAPI_MakeVertex(point).Vertex();
                double closest=std::numeric_limits<double>::infinity();
                bool normalMatched=false;
                for (const auto& candidate : references) {
                    if ((splineOnly && !candidate.spline) || candidate.bounds.IsOut(point)) continue;
                    const BRepExtrema_DistShapeShape distance(vertex,candidate.face);
                    if (!distance.IsDone()) continue;
                    closest=std::min(closest,distance.Value());
                    if (splineOnly || normalMatched || distance.Value()>tolerance) continue;
                    for (int k=1; k<=distance.NbSolution(); ++k) {
                        if (distance.SupportTypeShape2(k)!=BRepExtrema_IsInFace) continue;
                        double refU=0, refV=0;
                        distance.ParOnFaceS2(k,refU,refV);
                        gp_Pnt referencePoint;
                        gp_Vec referenceU,referenceV;
                        BRepAdaptor_Surface(candidate.face).D1(refU,refV,referencePoint,referenceU,referenceV);
                        auto referenceNormal=referenceU.Crossed(referenceV);
                        if (candidate.face.Orientation()==TopAbs_REVERSED) referenceNormal.Reverse();
                        const double magnitudes=fittedNormal.Magnitude()*referenceNormal.Magnitude();
                        normalMatched=magnitudes>0 && fittedNormal.Dot(referenceNormal)>=.999*magnitudes;
                        if (normalMatched) break;
                    }
                }
                if (std::isfinite(closest) && closest>maximum) {
                    maximum=closest;
                    worstPoint=point;
                }
                if (!std::isfinite(closest)) ++outsideBounds;
                within += closest<=tolerance;
                consistentNormals += normalMatched;
                if (splineOnly && closest>tolerance) {
                    for (const auto& candidate : references) {
                        if (candidate.spline || candidate.bounds.IsOut(point)) continue;
                        const BRepExtrema_DistShapeShape distance(vertex,candidate.face);
                        if (!distance.IsDone() || distance.Value()>tolerance) continue;
                        ++otherSurfaceMatches;
                        break;
                    }
                }
            }
        };
        sampleGrid(15);
        if (!splineOnly && samples<32) sampleGrid(31);
        const bool faceMatched=samples>=16 && within==samples && (splineOnly || consistentNormals>=samples*.98);
        matched &= faceMatched;
        faces.append(QJsonObject{{"samples",samples},{"within_tolerance",within},{"without_reference_candidate",outsideBounds},
            {"maximum_measured_distance_mm",maximum},{"other_surface_matches",otherSurfaceMatches},
            {"maximum_distance_point_mm",QJsonArray{worstPoint.X(),worstPoint.Y(),worstPoint.Z()}},
            {"consistent_normals",consistentNormals},{"kind",kind(surface.GetType())},
            {"reference_matched",faceMatched}});
    }
    report["reference_matched"]=matched && !faces.isEmpty();
    const bool success=report["reference_matched"].toBool();
    report["status"]=splineOnly ? (success ? "spline_reference_matched" : "spline_reference_mismatch")
                                : (success ? "surface_reference_matched" : "surface_reference_mismatch");
    report["faces"]=faces;
}
}

void compareObservedSplines(const QStringList& args, QJsonObject& report) { compareObservedFaces(args,report,true); }
void compareObservedSurfaces(const QStringList& args, QJsonObject& report) { compareObservedFaces(args,report,false); }

void sampleStepCloud(const QStringList& args,QJsonObject& report) {
    check(args.size()==6 || args.size()==7,
          "Usage: sample source.step report.json cloud.ply spacing_mm [--preserve-edges]");
    bool valid=false;const double spacing=args[5].toDouble(&valid);
    check(valid && std::isfinite(spacing) && spacing>0,"Sampling spacing must be positive");
    const bool preserveEdges = args.size() == 7 && args[6] == "--preserve-edges";
    check(args.size() == 6 || preserveEdges,
          "The only supported optional sampling flag is --preserve-edges");
    STEPControl_Reader reader;
    std::ifstream source(std::filesystem::path(args[2].toStdWString()),std::ios::binary);
    check(source.good() && reader.ReadStream("source.step",source)==IFSelect_RetDone,"Cannot read STEP");
    check(reader.TransferRoots()>0,"STEP has no transferable roots");
    const auto shape=reader.OneShape();
    // Preserve source placement. Evaluate the analytic surface at interpolated
    // UV coordinates rather than emitting faceted triangle positions/normals.
    BRepMesh_IncrementalMesh mesh(shape,std::min(spacing*0.15,0.05),false,0.15,true);
    check(mesh.IsDone(),"STEP triangulation failed");
    std::vector<SampledPoint> cloud;
    QJsonArray faces,solids;int solidId=0,faceId=0;
    std::size_t edgePointCount = 0;
    std::mt19937 random(4500);std::uniform_real_distribution<double> unit(0,1);
    std::size_t minimumSamplesPerFace=std::numeric_limits<std::size_t>::max();
    std::size_t maximumSamplesPerFace=0;
    for(TopExp_Explorer si(shape,TopAbs_SOLID);si.More();si.Next()) {
        ++solidId;const auto start=cloud.size();const int firstFace=faceId+1;
        GProp_GProps props;BRepGProp::VolumeProperties(si.Current(),props);
        for(TopExp_Explorer fi(si.Current(),TopAbs_FACE);fi.More();fi.Next()) {
            ++faceId;const auto face=TopoDS::Face(fi.Current());BRepAdaptor_Surface surface(face);
            TopLoc_Location location;const auto triangles=BRep_Tool::Triangulation(face,location);
            check(!triangles.IsNull() && triangles->HasUVNodes(),"Face has no UV triangulation");
            const auto begin=cloud.size();std::vector<double> areas;
            double total=0;
            for(int i=1;i<=triangles->NbTriangles();++i) {
                int a,b,c;triangles->Triangle(i).Get(a,b,c);
                const auto p=triangles->Node(a),q=triangles->Node(b),r=triangles->Node(c);
                total+=gp_Vec(p,q).Crossed(gp_Vec(p,r)).Magnitude()*0.5;areas.push_back(total);
            }
            check(total>0,"Degenerate source face");
            // Keep the requested area density on every face. Reject impractical
            // requests explicitly rather than silently reducing large-face density.
            const double requestedCount=std::ceil(total/(spacing*spacing));
            check(std::isfinite(requestedCount) && requestedCount<=50000000.0,
                  "Requested spacing exceeds the per-face sampling limit");
            const auto count=static_cast<std::size_t>(std::max(1.0,requestedCount));
            minimumSamplesPerFace=std::min(minimumSamplesPerFace,count);
            maximumSamplesPerFace=std::max(maximumSamplesPerFace,count);
            std::size_t rejected=0;
            for(std::size_t i=0;i<count;) {
                const double target=(i+unit(random))*total/count;
                const int triangle=int(std::lower_bound(areas.begin(),areas.end(),target)-areas.begin())+1;
                int a,b,c;triangles->Triangle(triangle).Get(a,b,c);
                const double r=std::sqrt(unit(random)),t=unit(random);
                const auto uv=triangles->UVNode(a).XY()*(1-r)+triangles->UVNode(b).XY()*(r*(1-t))+triangles->UVNode(c).XY()*(r*t);
                BRepClass_FaceClassifier classifier(face,gp_Pnt2d(uv),1e-7);
                if(classifier.State()!=TopAbs_IN && classifier.State()!=TopAbs_ON) {
                    check(++rejected<count*100,"Cannot sample the trimmed face interior");continue;
                }
                gp_Pnt p;gp_Vec du,dv;surface.D1(uv.X(),uv.Y(),p,du,dv);
                auto n=du.Crossed(dv);check(n.SquareMagnitude()>1e-24,"Singular surface sample");
                n.Normalize();if(face.Orientation()==TopAbs_REVERSED)n.Reverse();
                cloud.push_back({p,n});
                ++i;
            }
            const auto interiorPointCount = cloud.size() - begin;
            const auto faceEdgePointCount = preserveEdges
                ? appendFaceEdgeSamples(face, spacing, cloud) : 0;
            edgePointCount += faceEdgePointCount;
            faces.append(QJsonObject{{"id",faceId},{"solid_id",solidId},{"kind",kind(surface.GetType())},
                {"first_point_id",double(begin+1)},{"point_count",double(cloud.size()-begin)},
                {"interior_point_count",double(interiorPointCount)},
                {"edge_point_count",double(faceEdgePointCount)},{"area_mm2",total},
                {"effective_spacing_mm",std::sqrt(total/count)},{"outside_trim_rejected",double(rejected)}});
        }
        solids.append(QJsonObject{{"id",solidId},{"first_face_id",firstFace},{"face_count",faceId-firstFace+1},
            {"first_point_id",double(start+1)},{"point_count",double(cloud.size()-start)},{"volume_mm3",props.Mass()}});
    }
    check(!cloud.empty(),"STEP contains no sampled solids");
    std::ofstream ply(std::filesystem::path(args[4].toStdWString()));check(ply.good(),"Cannot create PLY");
    ply<<"ply\nformat ascii 1.0\ncomment analytic STEP samples; units mm\nelement vertex "<<cloud.size()
       <<"\nproperty double x\nproperty double y\nproperty double z\nproperty double nx\nproperty double ny\nproperty double nz\nend_header\n"<<std::setprecision(17);
    for(const auto& p:cloud)ply<<p.point.X()<<' '<<p.point.Y()<<' '<<p.point.Z()<<' '
        <<p.normal.X()<<' '<<p.normal.Y()<<' '<<p.normal.Z()<<'\n';
    ply.close();check(ply.good(),"Cannot finish PLY");
    const QFileInfo path(args[4]);
    save(path.dir().filePath(path.completeBaseName()+".json"),QJsonObject{
        {"unit","mm"},{"source","analytic_step_sampling"},{"spacing_mm",spacing},
        {"edge_fidelity_enabled",preserveEdges},{"edge_point_count",double(edgePointCount)}});
    report["status"]="sampled";report["source_step"]=QFileInfo(args[2]).absoluteFilePath();
    report["unit"]="mm";report["spacing_mm"]=spacing;report["points"]=double(cloud.size());
    report["edge_fidelity_enabled"] = preserveEdges;
    report["edge_point_count"] = double(edgePointCount);
    report["solid_count"]=solidId;report["face_count"]=faceId;report["faces"]=faces;report["solids"]=solids;
    report["minimum_samples_per_face"]=double(minimumSamplesPerFace);
    report["maximum_samples_per_face"]=double(maximumSamplesPerFace);
    report["reference_labels_used_by_recognition"]=false;
}
