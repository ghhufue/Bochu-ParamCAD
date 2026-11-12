#include "capture/CaptureJob.h"

#include <QFileInfo>
#include <QJsonArray>
#include <QRegularExpression>
#include <QSet>

#include <algorithm>
#include <cmath>

namespace capture {
namespace {
constexpr double pi = 3.14159265358979323846;
int bits(int size) { int n = 0; for (int value = size - 1; value > 0; value >>= 1) ++n; return n; }
bool finiteTransform(const Matrix4& m) {
    if (!std::all_of(m.begin(), m.end(), [](double v) { return std::isfinite(v); })) return false;
    if (std::abs(m[12]) + std::abs(m[13]) + std::abs(m[14]) + std::abs(m[15] - 1) > 1e-8) return false;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        double dot = 0;
        for (int k = 0; k < 3; ++k) dot += m[k * 4 + i] * m[k * 4 + j];
        if (std::abs(dot - (i == j ? 1.0 : 0.0)) > 1e-6) return false;
    }
    const double det = m[0]*(m[5]*m[10]-m[6]*m[9])-m[1]*(m[4]*m[10]-m[6]*m[8])+m[2]*(m[4]*m[9]-m[5]*m[8]);
    return std::abs(det - 1) < 1e-6;
}
}

Matrix4 identity() { return {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}; }
Matrix4 multiply(const Matrix4& a, const Matrix4& b) {
    Matrix4 result{};
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j)
        for (int k = 0; k < 4; ++k) result[i*4+j] += a[i*4+k]*b[k*4+j];
    return result;
}

QJsonArray matrixJson(const Matrix4& m) {
    QJsonArray result;
    for (int i = 0; i < 4; ++i) result.append(QJsonArray{m[i*4],m[i*4+1],m[i*4+2],m[i*4+3]});
    return result;
}

QJsonObject deviceJson(const Device& d) {
    QJsonArray k;
    for (int i = 0; i < 3; ++i) k.append(QJsonArray{d.K[i*3],d.K[i*3+1],d.K[i*3+2]});
    return {{"model", "PINHOLE"}, {"width",d.width}, {"height",d.height}, {"K",k},
            {"device_to_rig",matrixJson(d.deviceToRig)}};
}

QJsonObject Pattern::json() const {
    return {{"id",id},{"kind",kind},{"axis",axis},{"bit",bit},{"inverted",inverted},{"phase",phase}};
}

std::vector<Pattern> grayPatterns(int width, int height, int phaseSteps) {
    std::vector<Pattern> result{{"black","black"},{"white","white"}};
    for (int axis = 0; axis < 2; ++axis) {
        const QString name = axis == 0 ? "x" : "y";
        for (int bit = bits(axis == 0 ? width : height)-1; bit >= 0; --bit)
            for (int inverse = 0; inverse < 2; ++inverse)
                result.push_back({QString("%1_%2_%3").arg(name).arg(bit).arg(inverse),
                                  "gray",name,bit,inverse != 0});
    }
    for (const QString& axis : {QString("x"),QString("y")})
        for (int step = 0; step < phaseSteps; ++step)
            result.push_back({QString("%1_phase_%2").arg(axis).arg(step),"phase",axis,-1,false,
                              step*2*pi/phaseSteps});
    return result;
}

QImage patternImage(const Pattern& p, int width, int height, double period) {
    QImage image(width,height,QImage::Format_Grayscale8);
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
        const int coordinate = p.axis == "x" ? x : y;
        int value = 0;
        if (p.kind == "white") value = 255;
        else if (p.kind == "gray") {
            const bool bit = (((coordinate ^ (coordinate >> 1)) >> p.bit) & 1) != 0;
            value = bit != p.inverted ? 255 : 0;
        } else if (p.kind == "phase")
            value = static_cast<int>(std::nearbyint(127.5*(1+std::cos(2*pi*(coordinate+.5)/period+p.phase))));
        image.scanLine(y)[x] = static_cast<uchar>(std::clamp(value,0,255));
    }
    return image;
}

std::vector<Frame> RenderJob::frames() const {
    std::vector<Frame> result;
    for (int station = 0; station < static_cast<int>(stations.size()); ++station)
        for (int exposure = 0; exposure < static_cast<int>(exposures.size()); ++exposure)
            for (int pattern = 0; pattern < static_cast<int>(patterns.size()); ++pattern)
                result.push_back({station,exposure,pattern,QString("stations/%1/e%2/%3.png")
                    .arg(stations[station].id).arg(exposure).arg(patterns[pattern].id)});
    return result;
}

QString RenderJob::validate() const {
    if (id.isEmpty() || directory.isEmpty() || stations.empty() || geometry.positions.empty()
        || geometry.triangles.empty()) return "Missing job identity, scene or scan stations";
    if (geometry.normals.size() != geometry.positions.size() || geometry.uv.size() != geometry.positions.size())
        return "Geometry needs one normal and UV coordinate per vertex";
    for (const auto& triangle : geometry.triangles) for (const auto index : triangle)
        if (index >= geometry.positions.size()) return "Invalid mesh triangle index";
    for (const auto& p : geometry.positions) for (double v : p)
        if (!std::isfinite(v)) return "Non-finite mesh position";
    for (const auto& p : geometry.normals) {
        for(double v:p)if(!std::isfinite(v))return "Non-finite surface normal";
        if(std::hypot(p[0],p[1],p[2])<1e-8)return "Zero surface normal";
    }
    for (const auto& uv : geometry.uv) for(double v:uv)if(!std::isfinite(v))return "Non-finite material UV";
    for (const Device* d : {&camera,&projector}) {
        if (d->width < 2 || d->height < 2 || d->width > 16384 || d->height > 16384
            || !finiteTransform(d->deviceToRig)) return "Invalid camera/projector dimensions or transform";
        if (!std::all_of(d->K.begin(),d->K.end(),[](double v){return std::isfinite(v);})
            || d->K[0] <= 0 || d->K[4] <= 0 || std::abs(d->K[1])+std::abs(d->K[3])
            +std::abs(d->K[6])+std::abs(d->K[7])+std::abs(d->K[8]-1) > 1e-8)
            return "Only zero-skew PINHOLE intrinsics are supported";
    }
    double baseline2=0;
    for (int i : {3,7,11}) baseline2 += std::pow(camera.deviceToRig[i]-projector.deviceToRig[i],2);
    if (baseline2 < 1e-12) return "Camera/projector baseline is zero";
    QSet<QString> ids;const QRegularExpression safeId("^[A-Za-z0-9_-]+$");
    for (const auto& station : stations) {
        if (!finiteTransform(station.rigToWorld)) return "Invalid station transform";
        if(!safeId.match(station.id).hasMatch() || ids.contains(station.id))return "Invalid or duplicate station identity";
        ids.insert(station.id);
    }
    if (phaseSteps != 0 && phaseSteps != 4) return "Phase steps must be 0 or 4";
    const auto expected = grayPatterns(projector.width,projector.height,phaseSteps);
    if (patterns.size() != expected.size()) return "Incomplete pattern sequence";
    for (std::size_t i=0;i<patterns.size();++i) if (patterns[i].json()!=expected[i].json()) return "Invalid pattern sequence";
    if (exposures.empty()) return "No exposure settings";
    for (double exposure : exposures) if (!std::isfinite(exposure) || exposure<=0) return "Invalid exposure";
    if (!std::isfinite(nearPlane) || !std::isfinite(farPlane) || nearPlane<=0 || farPlane<=nearPlane
        || !std::isfinite(referenceDistance) || referenceDistance<=0 || !std::isfinite(lightIntensity)
        || lightIntensity<=0 || !std::isfinite(ambient) || ambient<0 || !std::isfinite(phasePeriod) || phasePeriod<=2)
        return "Invalid clipping, projection or illumination parameters";
    for (double color : material.baseColor) if (!std::isfinite(color) || color<0 || color>1) return "Invalid linear base color";
    if (!std::isfinite(material.metallic) || material.metallic<0 || material.metallic>1
        || !std::isfinite(material.roughness) || material.roughness<.04 || material.roughness>1)
        return "Metallic must be [0,1], roughness must be [0.04,1]";
    for (const auto& path : {material.baseColorMap,material.metallicMap,material.roughnessMap})
        if (!path.isEmpty() && QImage(path).isNull()) return "Material map cannot be read: "+path;
    return {};
}

QJsonObject RenderJob::calibrationJson() const {
    return {{"schema","paramcad.structured-light.calibration"},{"version",1},{"unit","mm"},
            {"devices",QJsonObject{{"camera",deviceJson(camera)},{"projector",deviceJson(projector)}}}};
}

QJsonObject RenderJob::sceneJson() const {
    QJsonArray points,normals,uv,triangles;
    for (const auto& p:geometry.positions) points.append(QJsonArray{p[0],p[1],p[2]});
    for (const auto& p:geometry.normals) normals.append(QJsonArray{p[0],p[1],p[2]});
    for (const auto& p:geometry.uv) uv.append(QJsonArray{p[0],p[1]});
    for (const auto& p:geometry.triangles) triangles.append(QJsonArray{static_cast<int>(p[0]),static_cast<int>(p[1]),static_cast<int>(p[2])});
    return {{"schema","paramcad.capture.scene"},{"version",1},{"unit","mm"},
        {"geometry",QJsonObject{{"positions",points},{"normals",normals},{"uv",uv},{"triangles",triangles}}},
        {"material",QJsonObject{{"base_color",QJsonArray{material.baseColor[0],material.baseColor[1],material.baseColor[2]}},
            {"metallic",material.metallic},{"roughness",material.roughness},
            {"workflow","metallic-roughness"},{"map_channels","separate images; scalar data in red"},
            {"mapping","frozen mesh UV; STEP viewport export uses box mapping"},
            {"base_color_space","linear constant; sRGB image"},{"data_map_space","linear"},
            {"base_color_map",material.baseColorMap},{"metallic_map",material.metallicMap},{"roughness_map",material.roughnessMap}}},
        {"lighting",QJsonObject{{"intensity",lightIntensity},{"reference_distance_mm",referenceDistance},{"ambient",ambient}}}};
}
} // namespace capture
