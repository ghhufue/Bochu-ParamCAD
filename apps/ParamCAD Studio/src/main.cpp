#include "app/MainWindow.h"
#include "app/UiLanguage.h"
#include "cli/StudioCommandServer.h"
#include "ParamCADBuildInfo.h"

#include <reconstruction/settings/ModuleSettings.h>

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QFileInfo>
#include <QIcon>
#include <QSurfaceFormat>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextStream>
#include <QVTKOpenGLNativeWidget.h>

#include <filesystem>
#include <cmath>

int main(int argc, char* argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    QApplication application(argc, argv);
    QApplication::setApplicationName(QStringLiteral("ParamCAD Studio"));
    QApplication::setOrganizationName(QStringLiteral("ECE4500J"));
    QApplication::setApplicationVersion(QString::fromLatin1(pcad::build::version));
    QApplication::setWindowIcon(QIcon(QStringLiteral(":/icons/paramcad-studio.png")));

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("ParamCAD Studio"));
    parser.addHelpOption();
    parser.addVersionOption();
    const QCommandLineOption performanceCapture(
        QStringLiteral("performance-capture"),
        QStringLiteral("Capture a STEP model as a self-contained validation case."),
        QStringLiteral("step-file"));
    const QCommandLineOption structuredCapture("structured-light-capture","Capture a STEP model through the client structured-light workflow.","step-file");
    const QCommandLineOption scanStations("scan-stations","Number of structured-light scan stations.","count","16");
    const QCommandLineOption scanSize("scan-size","Camera image dimensions WIDTHxHEIGHT.","size","1280x960");
    const QCommandLineOption projectorSize("projector-size","Projector dimensions WIDTHxHEIGHT.","size","512x512");
    const QCommandLineOption scanPhase("scan-phase-shift","Capture four-step phase refinement patterns.");
    const QCommandLineOption scanNoPhase("scan-no-phase-shift","Disable four-step phase refinement (enabled by default).");
    const QCommandLineOption scanHdr("scan-multi-exposure","Capture three fixed exposure sequences.");
    const QCommandLineOption scanReconstruct("scan-reconstruct","Also reconstruct and load the resulting point cloud in the client.");
    const QCommandLineOption scanRenderer("scan-renderer","Capture renderer: vtk or mitsuba.","backend","vtk");
    const QCommandLineOption scanSamples("scan-samples","Mitsuba samples per pixel.","count","32");
    const QCommandLineOption scanDepth("scan-max-depth","Mitsuba maximum path depth; 2 means direct only.","count","4");
    const QCommandLineOption scanMetallic("scan-metallic","Material metallic factor [0,1].","value","0.9");
    const QCommandLineOption scanRoughness("scan-roughness","Material roughness [0.04,1].","value","0.4");
    const QCommandLineOption scanLight("scan-light-intensity","Projector radiance scale at the reference distance.","value","2.0");
    const QCommandLineOption scanBaseline("scan-baseline","Camera-to-projector baseline in mm; 0 selects 20% of the scene diagonal.","value","0");
    const QCommandLineOption outputDirectory(
        QStringLiteral("output"), QStringLiteral("Validation case directory."),
        QStringLiteral("directory"));
    const QCommandLineOption texture(
        QStringLiteral("texture"), QStringLiteral("Optional feature-rich surface texture."),
        QStringLiteral("image-file"));
    const QCommandLineOption configDirectory(
        QStringLiteral("config-dir"), QStringLiteral("Directory that holds the module .conf files."),
        QStringLiteral("directory"));
    const QCommandLineOption overrides(
        QStringLiteral("set"), QStringLiteral("Override one hyperparameter (module.key=value). Repeatable."),
        QStringLiteral("module.key=value"));
    const QCommandLineOption language(
        QStringLiteral("lang"), QStringLiteral("Interface language: zh (default) or en."),
        QStringLiteral("language"));
    parser.addOption(performanceCapture);
    for(const auto& option:{structuredCapture,scanStations,scanSize,projectorSize,scanPhase,scanNoPhase,scanHdr,
                           scanReconstruct,scanMetallic,scanRoughness,scanLight,scanBaseline,scanRenderer,scanSamples,scanDepth})parser.addOption(option);
    parser.addOption(outputDirectory);
    parser.addOption(texture);
    parser.addOption(configDirectory);
    parser.addOption(overrides);
    parser.addOption(language);
    parser.process(application);

    // Catalogs must be installed before the first widget is constructed so
    // every menu entry and panel uses the requested language.
    i18n::installTranslators(application, parser.value(language));

    if (parser.isSet(configDirectory)) {
        reconstruction::settings::setConfigDirectory(
            std::filesystem::path(parser.value(configDirectory).toStdU16String()));
    }
    for (const auto& assignment : parser.values(overrides))
        reconstruction::settings::addOverride(assignment.toStdString());

    MainWindow window;
    StudioCommandServer commandServer([&window](const QJsonObject& request,pcad::Reply reply) {
        window.executeCliCommand(request,std::move(reply));
    });
    QString commandServerError;
    if(!commandServer.start(commandServerError))window.logWarning(commandServerError);
    if(parser.isSet(structuredCapture))window.setAttribute(Qt::WA_DontShowOnScreen);
    window.show();

    if(parser.isSet(structuredCapture)) {
        const auto parseSize=[&](const QString& value,int& width,int& height) {
            const auto parts=value.toLower().split('x');bool ok1=false,ok2=false;
            if(parts.size()!=2)return false;width=parts[0].toInt(&ok1);height=parts[1].toInt(&ok2);
            return ok1 && ok2 && width>=32 && height>=32 && width<=4096 && height<=4096;
        };
        capture::ScanOptions options;bool okStations=false,okMetal=false,okRough=false;
        bool okSamples=false,okDepth=false,okLight=false,okBaseline=false;
        options.renderBackend=parser.value(scanRenderer);
        options.samplesPerPixel=parser.value(scanSamples).toInt(&okSamples);
        options.maxPathDepth=parser.value(scanDepth).toInt(&okDepth);
        options.stationCount=parser.value(scanStations).toInt(&okStations);
        options.material.metallic=parser.value(scanMetallic).toDouble(&okMetal);
        options.material.roughness=parser.value(scanRoughness).toDouble(&okRough);
        options.lightIntensity=parser.value(scanLight).toDouble(&okLight);
        options.baseline=parser.value(scanBaseline).toDouble(&okBaseline);
        if(!okStations || !okMetal || !okRough || !okSamples || !okDepth || !okLight || !okBaseline || options.stationCount<1 || options.stationCount>64
            || (options.renderBackend!="vtk" && options.renderBackend!="mitsuba")
            || options.samplesPerPixel<1 || options.samplesPerPixel>4096 || options.maxPathDepth<2 || options.maxPathDepth>16
            || !std::isfinite(options.lightIntensity) || options.lightIntensity<.05 || options.lightIntensity>100
            || !std::isfinite(options.baseline) || options.baseline<0 || options.baseline>10000
            || !std::isfinite(options.material.metallic) || options.material.metallic<0 || options.material.metallic>1
            || !std::isfinite(options.material.roughness) || options.material.roughness<.04 || options.material.roughness>1
            || !parseSize(parser.value(scanSize),options.cameraWidth,options.cameraHeight)
            || !parseSize(parser.value(projectorSize),options.projectorWidth,options.projectorHeight)
            || parser.value(outputDirectory).isEmpty())parser.showHelp(2);
        options.phaseShift=!parser.isSet(scanNoPhase);options.multiExposure=parser.isSet(scanHdr);
        QString error,cloud;
        const QString output=QFileInfo(parser.value(outputDirectory)).absoluteFilePath();
        const bool success=window.runStructuredCapture(QFileInfo(parser.value(structuredCapture)).absoluteFilePath(),
            output,options,parser.isSet(scanReconstruct),cloud,error);
        QJsonObject result{{"status",success?"complete":"failed"},{"capture",output+"/capture/capture.json"},
                           {"point_cloud",cloud},{"error",error}};
        QTextStream(stdout)<<QJsonDocument(result).toJson(QJsonDocument::Compact)<<Qt::endl;
        return success?0:2;
    }

    if (parser.isSet(performanceCapture)) {
        const QString modelPath = parser.value(performanceCapture);
        const QString outputPath = parser.value(outputDirectory);
        if (modelPath.isEmpty() || outputPath.isEmpty()) {
            parser.showHelp(2);
        }
        application.processEvents();
        QString error;
        const bool succeeded = window.runPerformanceCapture(
            QFileInfo(modelPath).absoluteFilePath(), QFileInfo(outputPath).absoluteFilePath(),
            parser.value(texture).isEmpty()
                ? QString()
                : QFileInfo(parser.value(texture)).absoluteFilePath(),
            error);
        if (!succeeded) {
            window.logError(error);
            return 2;
        }
        return 0;
    }
    return application.exec();
}
