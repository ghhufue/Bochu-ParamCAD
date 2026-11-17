#include "io/AnalysisBundleExporter.h"
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <iostream>
#include <stdexcept>

static void require(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        QTemporaryDir temp;require(temp.isValid(),"temporary directory");
        const QString path=argc==2?QString::fromLocal8Bit(argv[1]):temp.filePath("fixture.pcad-analysis");
        AnalysisBundleSnapshot s;
        s.cloud.positions={{{0,0,0}},{{1,0,0}},{{0,2,0}},{{100,50,0}},{{9,9,9}},{{3,3,3}}};
        s.deleted={0,0,0,0,1,0};s.memberships={182,182,182,418,182,0};s.selected={0,2};
        s.sourceName="fixture.ply";s.sourceUnit="mm";s.documentRevision=17;
        PointCloudRegion face;face.id=182;face.name=QStringLiteral("平面 406");face.geometry="plane";
        face.parameters={{"normal",QJsonArray{0,0,1}},{"offset",0}};face.source="detected";
        PointCloudRegion remote=face;remote.id=418;remote.name=QStringLiteral("平面 407");
        s.regions={face,remote};s.relations={{1,182,418,"coplanar",true,.9}};
        s.regions[0].supportValidation={{"verdict","accepted"},{"density_ratio",1.0}};
        s.supportValidation={{"version",1},{"scope","test_fixture"},{"reference",QJsonObject{{"available",true},{"density",1.0}}}};
        s.surfaces.colorRegions={{182,182},{418,182}};s.displayedSurfaces={182};
        auto& mesh=s.surfaces.meshes[182];mesh.positions={{{0,0,0}},{{1,0,0}},{{100,50,0}}};
        mesh.sourcePointIds={0,1,3};mesh.triangles={{{0,1,2}}};mesh.coveredPointIds={0,1};
        PlanarDomainSupportCheck support;support.observedPoints=4;support.outerBoundaryPoints=4;
        support.rejected=true;support.area=25;s.surfaces.planeSupportChecks[182]={support};
        QString error;require(exportAnalysisBundle(s,path,error),qPrintable(error));
        require(QDir(path).entryList({"*.py"},QDir::Files).empty(),"export contains no Python programs");
        QFile guide(QDir(path).filePath("README.md"));require(guide.open(QIODevice::ReadOnly),"analysis guide retained");
        require(guide.readAll().contains("pcad inspect"),"guide uses native CLI");
        QFile manifest(QDir(path).filePath("manifest.json"));require(manifest.open(QIODevice::ReadOnly),"bundle manifest");
        const auto assets=QJsonDocument::fromJson(manifest.readAll()).object()["assets"].toArray();
        for(const auto& asset:assets)
            require(!asset.toObject()["path"].toString().endsWith(".py",Qt::CaseInsensitive),"manifest contains no Python programs");
        QFile metadata(QDir(path).filePath("surfaces/00000182.json"));require(metadata.open(QIODevice::ReadOnly),"surface metadata");
        const auto bytes=metadata.readAll();const auto object=QJsonDocument::fromJson(bytes).object();
        require(object["point_count"].toInt()==3,"deleted observations excluded");
        require(!object.contains("points") && !object.contains("point_indices"),"raw data separated");
        require(object["render_mesh"].toObject()["foreign_source_vertices"].toInt()==1,"foreign mesh provenance retained");
        require(object["support_validation"].toObject()["verdict"]=="accepted","surface support summary exported");
        QFile supportReport(QDir(path).filePath("support-validation.json"));require(supportReport.open(QIODevice::ReadOnly),"support report exported");
        const auto validation=QJsonDocument::fromJson(supportReport.readAll()).object();
        require(validation["version"].toInt()==1 && validation["document_revision"]=="17" && !validation["bundle_id"].toString().isEmpty(),"validation report retains snapshot identity");
        require(QFileInfo(QDir(path).filePath("points.bin")).size()==48+6*32,"binary point stride");
        require(QFileInfo(QDir(path).filePath("point-map.bin")).size()==48+7*8,"active mapping and selection stride");
        require(!exportAnalysisBundle(s,path,error),"existing export refused");
        const auto cancelledPath=temp.filePath("cancelled.pcad-analysis");int attempts=0;
        require(!exportAnalysisBundle(s,cancelledPath,error,[&]{return ++attempts>3;}),"mid-export cancellation");
        require(!QFileInfo::exists(cancelledPath),"no partial publication on cancellation");
        s.memberships.pop_back();require(!exportAnalysisBundle(s,temp.filePath("invalid"),error),"invalid dimensions rejected");
        require(QDir(temp.path()).entryList({".pcad-analysis-*"},QDir::Dirs|QDir::Hidden|QDir::NoDotAndDotDot).empty(),"staging cleaned");
        std::cout<<"Analysis bundle exporter passed.\n";return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
