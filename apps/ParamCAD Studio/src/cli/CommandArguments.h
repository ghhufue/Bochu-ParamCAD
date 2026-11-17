#pragma once
#include "app/ReconstructionController.h"
#include <QMap>
#include <QSet>
#include <QStringList>
#include <stdexcept>
#include <cmath>

namespace pcad {
class Options {
public:
    QMap<QString,QStringList> values;
    QSet<QString> flags;
    Options(const QStringList& arguments,const QStringList& valueNames,const QStringList& flagNames={}) {
        for(qsizetype i=0;i<arguments.size();++i){const auto key=arguments[i];
            if(flagNames.contains(key)){if(flags.contains(key))fail("Duplicate option: "+key);flags.insert(key);}
            else if(valueNames.contains(key)){
                if(i+1>=arguments.size()||arguments[i+1].startsWith("--"))fail("Missing value: "+key);
                if(values.contains(key)&&key!="--set")fail("Duplicate option: "+key);
                values[key].append(arguments[++i]);
            }else fail("Unsupported option: "+key);
        }
    }
    QString get(const QString& key,const QString& fallback={})const{return values.contains(key)?values[key].last():fallback;}
    QString require(const QString& key)const{const auto v=get(key);if(v.isEmpty())fail("Required option: "+key);return v;}
    static void fail(const QString& message){throw std::invalid_argument(message.toUtf8().constData());}
};
inline ReconstructRequest reconstructionRequest(const QStringList& arguments){
    Options o(arguments,{"--images","--output","--engine","--matcher","--voxel-size-mm","--min-contrast"},{"--dense","--no-camera-info"});
    ReconstructRequest r;r.imageDirectory=o.require("--images");r.outputRoot=o.require("--output");
    r.engine=o.get("--engine","auto");r.matcher=o.get("--matcher","exhaustive");
    if(!QStringList{"auto","colmap","structured_light"}.contains(r.engine))Options::fail("Invalid --engine");
    if(!QStringList{"exhaustive","sequential"}.contains(r.matcher))Options::fail("Invalid --matcher");
    bool ok=false;r.voxelSizeMm=o.get("--voxel-size-mm","0.2").toDouble(&ok);
    if(!ok||!(r.voxelSizeMm>0)||!std::isfinite(r.voxelSizeMm))Options::fail("Invalid --voxel-size-mm");
    r.minContrast=o.get("--min-contrast","8").toDouble(&ok);
    if(!ok||r.minContrast<0||!std::isfinite(r.minContrast))Options::fail("Invalid --min-contrast");
    r.dense=o.flags.contains("--dense");r.useCameraInfo=!o.flags.contains("--no-camera-info");return r;
}
}
