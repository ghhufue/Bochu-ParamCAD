#include "app/PointCloudWorkspaceController.h"
#include "app/CadReconstructionTask.h"
#include "app/ReconstructionController.h"
#include "core/PointCloudDocument.h"
#include "cli/CommandArguments.h"
#include "cli/CommandProtocol.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/ReconstructPanel.h"
#include <QThread>
#include <QAction>
#include <QDir>
#include <QFileInfo>
#include <utility>

bool PointCloudWorkspaceController::commandBusy()const {
    return reconstructionCacheInUse()||m_downsampleThread
        ||m_exportThread||m_importThread||m_reconstructionController->isRunning()||!m_cliOperation.isEmpty();
}

bool PointCloudWorkspaceController::reconstructionCacheInUse() const
{
    return m_startingCad || m_cadTask->isRunning() || m_analysisThread;
}

void PointCloudWorkspaceController::finishCliOperation(bool success,const QString& error,QJsonObject details) {
    if(m_cliOperation.isEmpty())return;
    auto result=pcad::response(success?"complete":"failed",error,success?0:1);
    result["command"]=std::exchange(m_cliOperation,{});result["result"]=details;
    emit cliOperationFinished(result);
}
bool PointCloudWorkspaceController::dispatchCliOperation(const QString& command,const QStringList& args,QString& error) {
    if(command=="cancel"){cancelCadReconstruction();m_reconstructionController->cancel();if(m_exportThread)m_exportThread->requestInterruption();return true;}
    if(commandBusy()){error="Studio is busy; use pcad status or pcad cancel";return false;}
    try {
        if(command=="reconstruct cloud"){
            auto r=pcad::reconstructionRequest(args);
            m_ui.reconstructToolAction->setChecked(true);
            m_ui.propertyPanel->reconstructPanel()->setRequest(r);
            m_cliOperation=command;startReconstruction(r);
            if(!m_reconstructionController->isRunning()&&!m_cliOperation.isEmpty()){
                error=m_reconstructionController->lastError();m_cliOperation.clear();return false;
            }return true;
        }
        const bool cad=command=="reconstruct cad";
        pcad::Options options(args,cad?QStringList{"--in"}:command=="detect"?QStringList{"--in"}:QStringList{"--out"},
            cad?QStringList{"--no-projection-fallback"}:QStringList{});
        if(!options.get("--in").isEmpty()){
            const auto file=QFileInfo(options.get("--in"));
            if(!file.isFile()){error="Input PLY does not exist";return false;}
            loadPointCloudFromFile(file.absoluteFilePath());
            if(m_document->sourcePath()!=file.absoluteFilePath()){error="Could not load input PLY";return false;}
        }
        if(!m_document->hasCloud()){error="Load a point cloud in Studio first (pcad cloud open --in FILE)";return false;}
        if(command=="export analysis"){
            auto path=options.require("--out");if(!path.endsWith(".pcad-analysis",Qt::CaseInsensitive))path+=".pcad-analysis";
            if(QFileInfo::exists(path)){error="Analysis output already exists";return false;}
            m_cliOperation=command;startAnalysisExportTo(path);return true;
        }
        m_cliOperation=command;
        if(cad){
            if(options.flags.contains("--no-projection-fallback"))reconstructCadWithoutProjection();else reconstructCad();
            if(!m_cadTask->isRunning()){error="CAD reconstruction was not started (cancelled confirmation or invalid input)";m_cliOperation.clear();return false;}
        }else if(command=="detect"){
            startPointCloudAnalysis();if(!m_analysisThread){error="Surface detection needs at least 20 unassigned active points";m_cliOperation.clear();return false;}
        }else{error="Unsupported workspace command";m_cliOperation.clear();return false;}
        return true;
    }catch(const std::exception& failure){m_cliOperation.clear();error=QString::fromUtf8(failure.what());return false;}
}
