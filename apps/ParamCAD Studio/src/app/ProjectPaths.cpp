#include "app/ProjectPaths.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSettings>
#include <QStandardPaths>

namespace {

// The reconstruction pipeline is the one dependency that is always present in a
// checkout and never in an installed copy, which makes it a reliable marker.
constexpr char kRootMarker[] = "core/reconstruction/reconstruct.py";

} // namespace

QString ProjectPaths::projectRoot()
{
    // The executable lives several levels below the root (build/<preset>/Release),
    // so walk up until the marker shows up.
    QDir directory(QCoreApplication::applicationDirPath());
    QString discovered;
    for (int depth = 0; depth <= 8; ++depth) {
        if (QFileInfo::exists(directory.filePath(QLatin1String(kRootMarker)))) {
            discovered=directory.absolutePath();break;
        }
        if (!directory.cdUp()) {
            break;
        }
    }
    // An installed offline payload owns its code and interpreter. Developer
    // preferences from this machine must not redirect it to another checkout.
    if(!discovered.isEmpty() && QFileInfo::exists(QDir(discovered).filePath("deps/python/python.exe")))return discovered;
    const QSettings settings;
    const QString configured=settings.value(QStringLiteral("reconstruction/repoRoot")).toString();
    if(!configured.isEmpty() && QFileInfo::exists(QDir(configured).filePath(QLatin1String(kRootMarker))))
        return QDir(configured).absolutePath();
    return discovered;
}

QString ProjectPaths::pythonInterpreter(QStringList& prefixArguments)
{
    const QString root=projectRoot();
    const QString bundled=QDir(root).filePath("deps/python/python.exe");
    if(!root.isEmpty() && QFileInfo(bundled).isFile())return bundled;
    const QString configured=QSettings().value("reconstruction/pythonInterpreter").toString();
    if(!configured.isEmpty() && QFileInfo(configured).isFile())return configured;
    for(const auto& name:{QString("python"),QString("python3")}) {
        const auto executable=QStandardPaths::findExecutable(name);if(!executable.isEmpty())return executable;
    }
    const auto launcher=QStandardPaths::findExecutable("py");
    if(!launcher.isEmpty())prefixArguments<<"-3";
    return launcher;
}

QString ProjectPaths::outputsRoot()
{
    const QString root = projectRoot();
    // Even without a repository root there is no reason to write outside the
    // project, so the folder beside the executable is the fallback.
    const QDir base(root.isEmpty() ? QCoreApplication::applicationDirPath() : root);
    return QDir::cleanPath(base.filePath(QStringLiteral("outputs")));
}

QString ProjectPaths::capturesRoot()
{
    return QDir::cleanPath(outputsRoot() + QStringLiteral("/captures"));
}

QString ProjectPaths::reconstructionsRoot()
{
    return QDir::cleanPath(outputsRoot() + QStringLiteral("/reconstructions"));
}

QString ProjectPaths::screenshotsRoot()
{
    return QDir::cleanPath(outputsRoot() + QStringLiteral("/screenshots"));
}
