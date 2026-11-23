#include "app/MainWindow.h"
#include "app/ModelingController.h"
#include "app/ModelViewController.h"
#include "app/PointCloudWorkspaceController.h"
#include "app/ProjectController.h"
#include "app/CaptureWorkspaceController.h"
#include "cli/CommandArguments.h"
#include "core/PointCloudDocument.h"
#include "core/OrbitCamera.h"
#include "ui/viewport/VTKViewer.h"
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QJsonArray>
#include <QPointer>
#include <QPainter>
#include <QUuid>
#include <utility>

void MainWindow::executeCliCommand(const QJsonObject& request,pcad::Reply reply) {
    bool ownsReply=false;
    const auto command=request["command"].toString();QStringList args;
    for(const auto& value:request["arguments"].toArray()){
        if(!value.isString()){reply(pcad::response("failed","Arguments must be strings",2));return;}args.append(value.toString());}
    try {
        if(command=="status"){
            pcad::Options options(args,{});auto result=pcad::response("complete");
            result["busy"]=m_pointCloudWorkspace->commandBusy()||bool(m_cliReply)||m_captureBusy;
            result["source"]=m_pointCloudDocument->sourcePath();result["unit"]=m_pointCloudDocument->sourceUnit();
            result["point_count"]=m_pointCloudDocument->pointCount();result["document_revision"]=QString::number(m_pointCloudDocument->revision());
            result["feature_count"]=qint64(m_modelingController->features().size());result["last_command"]=m_lastCliResult;
            reply(result);return;
        }
        if(command=="cloud info"){
            pcad::Options options(args,{"--limit","--offset"});bool ok1=false,ok2=false;
            const int limit=options.get("--limit","20").toInt(&ok1),offset=options.get("--offset","0").toInt(&ok2);
            if(!ok1||!ok2||limit<1||limit>100||offset<0)pcad::Options::fail("Use --limit 1..100 and nonnegative --offset");
            const auto& selection=m_pointCloudDocument->selection();QJsonArray rows;const auto& regions=selection.regions();
            for(size_t i=size_t(offset);i<regions.size()&&rows.size()<limit;++i){const auto& r=regions[i];
                rows.append(QJsonObject{{"id",r.id},{"name",r.name},{"geometry",r.geometry},{"parameters",r.parameters},
                    {"point_count",qint64(selection.regionPointCount(r.id))},{"fit_stale",r.fitStale},{"confidence",r.confidence}});}
            auto result=pcad::response("complete");result["source"]=m_pointCloudDocument->sourcePath();
            result["unit"]=m_pointCloudDocument->sourceUnit();result["point_count"]=m_pointCloudDocument->pointCount();
            result["collection_count"]=qint64(regions.size());result["collections"]=rows;reply(result);return;
        }
        if(command=="capture frame"){
            pcad::Options options(args,{"--output","--target","--azimuth","--elevation","--distance","--width","--height"});
            const auto output=QFileInfo(options.require("--output")).absoluteFilePath();
            const auto target=options.get("--target","cloud");
            if(target!="cloud"&&target!="surfaces"&&target!="model"&&target!="cad")
                pcad::Options::fail("--target must be cloud, surfaces, model or cad");
            const auto number=[&](const QString& key,double fallback,double minimum,double maximum,bool whole=false){
                bool ok=false;const double value=options.get(key,QString::number(fallback)).toDouble(&ok);
                if(!ok||!std::isfinite(value)||value<minimum||value>maximum||(whole&&std::floor(value)!=value))
                    pcad::Options::fail("Invalid "+key);
                return value;
            };
            const int width=int(number("--width",960,32,4096,true));
            const int height=int(number("--height",540,32,4096,true));
            const CameraParameters original=m_viewer->cameraParameters();
            const OrbitParameters current=OrbitCamera::fromCamera(original);
            const OrbitParameters orbit{
                number("--distance",current.distance,1e-6,1e9),
                number("--azimuth",current.azimuthDeg,-1e9,1e9),
                number("--elevation",current.elevationDeg,-90,90)};
            const bool originalModelVisible=m_viewer->objectVisible();
            const bool originalCloudVisible=m_viewer->pointCloudVisible();
            const bool showModel=target=="model"||target=="cad";
            const bool ok=QDir().mkpath(QFileInfo(output).absolutePath());
            if(!ok)pcad::Options::fail("Cannot create screenshot directory");
            m_viewer->setObjectVisible(showModel&&m_viewer->hasBody());
            m_viewer->setPointCloudVisible(!showModel&&m_viewer->hasPointCloud());
            m_viewer->applyCamera(OrbitCamera::toCamera(orbit,original,false));
            m_viewer->renderNow();
            const QSize sourceSize=m_viewer->captureSourceSize();
            const QString nativePath=output+QStringLiteral(".")+QUuid::createUuid().toString(QUuid::WithoutBraces)
                +QStringLiteral(".png");
            const bool captured=target=="surfaces"?m_viewer->captureViewport(nativePath)
                :m_viewer->captureImage(nativePath,sourceSize.width(),sourceSize.height());
            bool written=false;
            if(captured){
                const QImage source(nativePath);
                if(!source.isNull()){
                    QImage frame(width,height,QImage::Format_RGB32);
                    frame.fill(QColor(35,39,46));
                    const QImage fitted=source.scaled(width,height,Qt::KeepAspectRatio,Qt::SmoothTransformation);
                    QPainter painter(&frame);
                    painter.drawImage((width-fitted.width())/2,(height-fitted.height())/2,fitted);
                    painter.end();
                    written=frame.save(output,"PNG");
                }
                QFile::remove(nativePath);
            }
            m_viewer->setObjectVisible(originalModelVisible);
            m_viewer->setPointCloudVisible(originalCloudVisible);
            m_viewer->applyCamera(original);
            if(!written)pcad::Options::fail("Could not write screenshot: "+output);
            auto result=pcad::response("complete");result["path"]=output;result["target"]=target;
            result["azimuth_deg"]=orbit.azimuthDeg;result["elevation_deg"]=orbit.elevationDeg;
            result["width"]=width;result["height"]=height;reply(result);return;
        }
        if(command=="cancel"){
            pcad::Options options(args,{});QString error;m_pointCloudWorkspace->dispatchCliOperation(command,{},error);
            m_captureWorkspace->cancelStructuredScan();
            reply(pcad::response("complete"));return;
        }
        if(m_cliReply||m_pointCloudWorkspace->commandBusy()||m_captureBusy)pcad::Options::fail("Studio is busy; use pcad status/cancel");
        if(QApplication::activeModalWidget())pcad::Options::fail("Studio has an open dialog; finish it before sending another command");
        if(command=="capture views"||command=="capture structured-light"){
            const bool structured=command=="capture structured-light";
            pcad::Options o(args,structured?QStringList{"--in","--output","--scan-stations","--scan-size","--projector-size",
                "--scan-renderer","--scan-samples","--scan-max-depth","--scan-metallic","--scan-roughness","--scan-light-intensity","--scan-baseline"}
                :QStringList{"--in","--output","--texture"},structured?QStringList{"--scan-reconstruct","--scan-no-phase-shift","--scan-phase-shift","--scan-multi-exposure"}:QStringList{});
            const auto model=o.require("--in"),directory=o.require("--output");capture::ScanOptions scan;
            const auto number=[&](const QString& key,double fallback,double low,double high,bool whole=false){bool ok=false;
                const double v=o.get(key,QString::number(fallback)).toDouble(&ok);
                if(!ok||!std::isfinite(v)||v<low||v>high||(whole&&std::floor(v)!=v))pcad::Options::fail("Invalid "+key);return v;};
            if(structured){
                scan.stationCount=int(number("--scan-stations",16,1,64,true));scan.samplesPerPixel=int(number("--scan-samples",32,1,4096,true));
                scan.maxPathDepth=int(number("--scan-max-depth",4,2,16,true));scan.material.metallic=number("--scan-metallic",.9,0,1);
                scan.material.roughness=number("--scan-roughness",.4,.04,1);scan.lightIntensity=number("--scan-light-intensity",2,.05,100);
                scan.baseline=number("--scan-baseline",0,0,10000);scan.renderBackend=o.get("--scan-renderer","vtk");
                if(scan.renderBackend!="vtk"&&scan.renderBackend!="mitsuba")pcad::Options::fail("Invalid renderer");
                const auto size=[&](const QString& key,const QString& fallback,int& width,int& height){auto parts=o.get(key,fallback).toLower().split('x');bool a=false,b=false;
                    if(parts.size()!=2)pcad::Options::fail("Invalid "+key);width=parts[0].toInt(&a);height=parts[1].toInt(&b);
                    if(!a||!b||width<32||height<32||width>4096||height>4096)pcad::Options::fail("Invalid "+key);};
                size("--scan-size","1280x960",scan.cameraWidth,scan.cameraHeight);size("--projector-size","512x512",scan.projectorWidth,scan.projectorHeight);
                if(o.flags.contains("--scan-phase-shift")&&o.flags.contains("--scan-no-phase-shift"))pcad::Options::fail("Conflicting phase options");
                scan.phaseShift=!o.flags.contains("--scan-no-phase-shift");scan.multiExposure=o.flags.contains("--scan-multi-exposure");
            }
            m_cliReply=std::move(reply);ownsReply=true;m_lastCliResult={{"id",request["id"]},{"command",command},{"status","accepted"}};
            m_cliReply(pcad::response("accepted"));QString error,cloud;
            const bool ok=structured?runStructuredCapture(model,directory,scan,o.flags.contains("--scan-reconstruct"),cloud,error)
                :runPerformanceCapture(model,directory,o.get("--texture"),error);
            auto result=pcad::response(ok?"complete":"failed",error,ok?0:1);result["command"]=command;result["output"]=directory;
            result["point_cloud"]=cloud;result["id"]=request["id"];m_lastCliResult=result;
            auto callback=std::exchange(m_cliReply,{});if(callback)callback(result);return;
        }
        if(command=="cloud open"||command=="project open"){
            pcad::Options options(args,{"--in"});auto path=QFileInfo(options.require("--in")).absoluteFilePath();
            if(!QFileInfo::exists(path))pcad::Options::fail("Input file does not exist");QString error;
            bool ok=false;if(command=="project open")ok=m_projectController->loadFrom(path,error);
            else {m_pointCloudWorkspace->loadPointCloudFromFile(path);ok=m_pointCloudDocument->sourcePath()==path;}
            auto result=pcad::response(ok?"complete":"failed",ok?QString():error.isEmpty()?QStringLiteral("Could not load input"):error,ok?0:2);
            result["path"]=path;reply(result);return;
        }
        if(command=="project save"||command=="export step"){
            pcad::Options options(args,{"--out"});auto path=QFileInfo(options.require("--out")).absoluteFilePath();QString error;
            const bool ok=command=="project save"?m_projectController->saveTo(path,error):m_modelViewController->exportStepTo(path,error);
            auto result=pcad::response(ok?"complete":"failed",error,ok?0:2);result["path"]=path;reply(result);return;
        }
        if(command!="reconstruct cad"&&command!="reconstruct cloud"&&command!="detect"&&command!="export analysis")
            pcad::Options::fail("This command has no online Studio action");
        // Capture the reply before invoking the same methods connected to GUI intents.
        // Modal confirmations may run a nested event loop, so the busy flag must
        // already be visible to a second CLI client.
        m_cliReply=std::move(reply);ownsReply=true;m_lastCliResult={{"id",request["id"]},{"command",command},{"status","accepted"}};
        QString error;
        if(!m_pointCloudWorkspace->dispatchCliOperation(command,args,error)){
            auto result=pcad::response("failed",error,2);result["command"]=command;result["id"]=request["id"];
            m_lastCliResult=result;if(auto callback=std::exchange(m_cliReply,{}))callback(result);return;
        }
        if(m_cliReply){auto accepted=pcad::response("accepted");accepted["command"]=command;m_cliReply(accepted);}
    }catch(const std::exception& error){
        auto result=pcad::response("failed",QString::fromUtf8(error.what()),2);
        if(ownsReply&&m_cliReply){m_lastCliResult=result;auto callback=std::exchange(m_cliReply,{});callback(result);}else if(reply)reply(result);
    }
}
