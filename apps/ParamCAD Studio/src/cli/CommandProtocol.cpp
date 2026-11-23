#include "CommandProtocol.h"
#include <QDir>
#include <QJsonArray>
#include <limits>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace pcad {
QList<Capability> capabilities() {
    return {{"studio",false,true,"Start Studio (never started implicitly by another command)"},
        {"sessions",false,true,"List responding Studio sessions"},
        {"status",true,false,"Read the current Studio document and task state"},
        {"cloud open",true,false,"Load a PLY through the Studio workspace"},
        {"cloud info",true,true,"Read cloud metadata and collection summaries"},
        {"project open",true,false,"Open a PCAD project in Studio"},
        {"project save",true,false,"Save the current Studio project"},
        {"export step",true,false,"Export the current Studio CAD"},
        {"export analysis",true,false,"Export the current recognition snapshot"},
        {"cancel",true,false,"Request cancellation through Studio"},
        {"capture views",true,false,"Capture a STEP model through the Studio viewport"},
        {"capture frame",true,false,"Capture the current Studio viewport to a specified PNG path"},
        {"capture structured-light",true,false,"Acquire a structured-light scan through Studio"},
        {"preprocess",false,true,"Preprocess a PLY using the C++ backend"},
        {"detect",true,true,"Detect surfaces; online uses the Studio Recognize Surfaces action"},
        {"analyze",false,true,"Run independent C++ surface/patch/relation analysis"},
        {"recognize",false,true,"Recognize parametric solid candidates"},
        {"assemble",false,true,"Separate and inspect assembly candidates"},
        {"project",false,true,"Fit an extrusion program with optional STEP export"},
        {"reconstruct cad",true,true,"Reconstruct CAD through the shared C++ task"},
        {"reconstruct cloud",true,true,"Reconstruct images through the shared reconstruction controller"},
        {"inspect",false,true,"Query an exported analysis bundle without Studio"}};
}
QJsonObject capabilityDocument() {
    QJsonArray commands;
    for(const auto& c:capabilities()) commands.append(QJsonObject{{"command",c.command},
        {"online",c.online},{"offline",c.offline},{"description",c.description}});
    return {{"schema","paramcad.cli.capabilities"},{"version",protocolVersion},{"commands",commands}};
}
QString sessionDirectory() {
    const auto override = qEnvironmentVariable("PCAD_SESSION_DIR");
    return override.isEmpty()?QDir::homePath()+"/.paramcad/cli-sessions":QDir(override).absolutePath();
}
QJsonObject response(const QString& status,const QString& error,int exitCode) {
    QJsonObject result{{"schema","paramcad.cli.response"},{"version",protocolVersion},
        {"status",status},{"exit_code",exitCode}};
    if(!error.isEmpty())result["error"]=error;
    return result;
}
QString processToken(qint64 processId) {
#ifdef _WIN32
    if(processId<=0||processId>std::numeric_limits<DWORD>::max())return {};
    const auto process=OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION,FALSE,DWORD(processId));if(!process)return {};
    FILETIME created{},exited{},kernel{},user{};DWORD status=0;wchar_t executable[32768];DWORD length=32768;
    const bool ok=GetExitCodeProcess(process,&status)&&status==STILL_ACTIVE&&GetProcessTimes(process,&created,&exited,&kernel,&user)
        &&QueryFullProcessImageNameW(process,0,executable,&length);CloseHandle(process);if(!ok)return {};
    return QString::number((quint64(created.dwHighDateTime)<<32)|created.dwLowDateTime)+":"+QString::fromWCharArray(executable,int(length)).toCaseFolded();
#else
    Q_UNUSED(processId);return {};
#endif
}
}
