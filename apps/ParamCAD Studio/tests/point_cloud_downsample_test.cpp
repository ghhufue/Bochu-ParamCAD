#include "io/PointCloudDownsampleStore.h"
#include "io/PointCloudDownsampler.h"
#include "io/PointCloudMetadata.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

void touch(const QString& path)
{
    require(QDir().mkpath(QFileInfo(path).absolutePath()), "cannot create directory");
    QFile file(path);
    require(file.open(QIODevice::WriteOnly), "cannot create fixture file");
    file.close();
}

void testVoxelAverage()
{
    PlyCloud source;
    source.positions = {{0.1F, 0.0F, 0.0F}, {0.2F, 0.0F, 0.0F},
                        {1.1F, 0.0F, 0.0F}, {1.2F, 0.0F, 0.0F}};
    source.normals = {{1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F},
                      {0.0F, 1.0F, 0.0F}, {0.0F, 1.0F, 0.0F}};
    source.colors = {{0, 10, 20}, {100, 30, 40}, {200, 0, 0}, {0, 0, 0}};
    PlyCloud result;
    QString error;
    require(pointcloud::voxelDownsample(source, 1.0, result, error), error.toStdString());
    require(result.positions.size() == 2U, "two voxels must produce two points");
    require(std::abs(result.positions[0][0] - 0.15F) < 1.0e-6F,
            "position must be the voxel average");
    require(std::abs(result.positions[1][0] - 1.15F) < 1.0e-6F,
            "second voxel average");
    require(result.normals.size() == result.positions.size(), "normals stay aligned");
    require(std::abs(result.normals[0][0] - 1.0F) < 1.0e-6F, "normals are re-normalised");
    require(result.colors.size() == result.positions.size(), "colors stay aligned");
    require(result.colors[0][0] == 50 && result.colors[0][1] == 20,
            "colors are averaged per channel");
    require(result.sourceRows.empty(), "downsampled rows do not reference source rows");
}

void testInvalidSpacing()
{
    PlyCloud source;
    source.positions = {{0.0F, 0.0F, 0.0F}};
    PlyCloud result;
    QString error;
    require(!pointcloud::voxelDownsample(source, 0.0, result, error),
            "zero spacing must be rejected");
    require(!error.isEmpty(), "rejection must carry a reason");
}

void testThinWallsRemainDistinct()
{
    PlyCloud source;
    source.positions={{.1F,.1F,.1F},{.2F,.1F,.1F},{.1F,.1F,.8F},{.2F,.1F,.8F},{.3F,.1F,.1F}};
    source.normals={{0,0,-1},{0,0,-2},{0,0,1},{0,0,1},{0,0,0}};
    PlyCloud result; QString error;
    require(pointcloud::voxelDownsample(source,1,result,error),error.toStdString());
    require(result.positions.size()==3 && result.normals.size()==3,"opposite walls and unknown normals have separate representatives");
    require(result.normals[0][2]<-.99F && std::abs(result.positions[0][2]-.1F)<1e-6F,"front wall retains its observed position and sign");
    require(result.normals[1][2]>.99F && std::abs(result.positions[1][2]-.8F)<1e-6F,"back wall retains its observed position and sign");
    for(auto& n:source.normals) if(n[2]!=0) n={0,0,1};
    require(pointcloud::voxelDownsample(source,1,result,error) && result.positions.size()==3,
        "separated parallel layers must not be averaged even with matching signs");
}

void testCacheLayout()
{
    QTemporaryDir temporary;
    require(temporary.isValid(), "cannot create temporary directory");
    const QString root = temporary.filePath(QStringLiteral("cloud.ply"));
    touch(root);
    const QString ten = pointcloud::downsamplePath(root, 10.0);
    require(ten == temporary.filePath(QStringLiteral("downsampled/cloud_10mm.ply")),
            "cached file must live in the downsampled folder beside its source");
    touch(ten);
    touch(pointcloud::downsamplePath(root, 5.0));
    touch(temporary.filePath(QStringLiteral("downsampled/other_20mm.ply")));

    double scale = 0.0;
    require(pointcloud::isDownsampledCloud(ten, &scale), "cached file must be recognised");
    require(std::abs(scale - 10.0) < 1.0e-9, "spacing must come back from the file name");
    require(pointcloud::rootCloudPath(ten) == root, "cached file must resolve to its source");
    require(pointcloud::rootCloudPath(root) == root, "a source file resolves to itself");

    const auto scales = pointcloud::existingScales(root);
    require(scales.size() == 2, "only this source's cached scales are listed");
    require(std::abs(scales[0] - 5.0) < 1.0e-9 && std::abs(scales[1] - 10.0) < 1.0e-9,
            "cached scales are sorted");
}

void testUnitSidecar()
{
    QTemporaryDir temporary;
    require(temporary.isValid(), "cannot create temporary directory");
    const QString root = temporary.filePath(QStringLiteral("cloud.ply"));
    touch(root);
    const QString cached = pointcloud::downsamplePath(root, 10.0);
    touch(cached);
    QString error;
    require(pointcloud::writeUnitSidecar(cached, QStringLiteral("mm"), 10.0, error),
            error.toStdString());
    require(pointCloudUnitName(cached) == QStringLiteral("mm"),
            "the sidecar must preserve the physical unit for the document");
}

} // namespace

int main()
{
    try {
        testVoxelAverage();
        testInvalidSpacing();
        testThinWallsRemainDistinct();
        testCacheLayout();
        testUnitSidecar();
        std::cout << "point cloud downsampling contracts passed" << std::endl;
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << std::endl;
        return 1;
    }
}
