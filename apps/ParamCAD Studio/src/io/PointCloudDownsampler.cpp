#include "io/PointCloudDownsampler.h"

#include <QCoreApplication>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace pointcloud {
namespace {

struct VoxelEntry
{
    long long x;
    long long y;
    long long z;
    std::uint32_t index;
};

bool voxelKey(const std::array<float, 3>& position, double voxelSize, long long key[3])
{
    // Keep the key exactly representable: a double only counts integers up to
    // 2^53, and a larger index could collapse distinct voxels together.
    constexpr double limit = 9.0e15;
    for (int axis = 0; axis < 3; ++axis) {
        const double value = std::floor(static_cast<double>(position[axis]) / voxelSize);
        if (!std::isfinite(value) || std::abs(value) >= limit) return false;
        key[axis] = static_cast<long long>(value);
    }
    return true;
}

} // namespace

bool voxelDownsample(const PlyCloud& source, double voxelSize, PlyCloud& result,
                     QString& error)
{
    if (!std::isfinite(voxelSize) || voxelSize <= 0) {
        error = QCoreApplication::translate("PointCloudDownsampler",
                                            "Voxel size must be finite and positive.");
        return false;
    }
    if (source.positions.empty()) {
        error = QCoreApplication::translate("PointCloudDownsampler",
                                            "The point cloud is empty.");
        return false;
    }

    std::vector<VoxelEntry> entries;
    entries.reserve(source.positions.size());
    for (std::size_t i = 0; i < source.positions.size(); ++i) {
        long long key[3]{};
        if (!voxelKey(source.positions[i], voxelSize, key)) {
            error = QCoreApplication::translate(
                "PointCloudDownsampler",
                "Voxel indices overflow; check the point cloud scale and the chosen spacing.");
            return false;
        }
        entries.push_back({key[0], key[1], key[2], static_cast<std::uint32_t>(i)});
    }
    std::sort(entries.begin(), entries.end(), [](const VoxelEntry& left, const VoxelEntry& right) {
        if (left.x != right.x) return left.x < right.x;
        if (left.y != right.y) return left.y < right.y;
        return left.z < right.z;
    });

    const bool hasNormals = source.normals.size() == source.positions.size();
    const bool hasColors = source.colors.size() == source.positions.size();
    result = PlyCloud{};
    result.positions.reserve(entries.size());
    if (hasNormals) result.normals.reserve(entries.size());
    if (hasColors) result.colors.reserve(entries.size());

    std::size_t begin = 0;
    while (begin < entries.size()) {
        std::size_t end = begin + 1;
        while (end < entries.size() && entries[end].x == entries[begin].x
               && entries[end].y == entries[begin].y && entries[end].z == entries[begin].z) {
            ++end;
        }
        double position[3]{0.0, 0.0, 0.0};
        double normal[3]{0.0, 0.0, 0.0};
        double color[3]{0.0, 0.0, 0.0};
        for (std::size_t i = begin; i < end; ++i) {
            const auto index = entries[i].index;
            for (int axis = 0; axis < 3; ++axis) {
                position[axis] += source.positions[index][axis];
                if (hasNormals) normal[axis] += source.normals[index][axis];
                if (hasColors) color[axis] += source.colors[index][axis];
            }
        }
        const double members = static_cast<double>(end - begin);
        result.positions.push_back({static_cast<float>(position[0] / members),
                                    static_cast<float>(position[1] / members),
                                    static_cast<float>(position[2] / members)});
        if (hasNormals) {
            double length = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1]
                                      + normal[2] * normal[2]);
            std::array<float, 3> value{};
            if (length > 1e-12) {
                value = {static_cast<float>(normal[0] / length),
                         static_cast<float>(normal[1] / length),
                         static_cast<float>(normal[2] / length)};
            } else {
                // Opposing normals cancel; keep a representative orientation.
                value = source.normals[entries[begin].index];
            }
            result.normals.push_back(value);
        }
        if (hasColors) {
            std::array<unsigned char, 3> value{};
            for (int axis = 0; axis < 3; ++axis) {
                const double average = color[axis] / members;
                value[axis] = static_cast<unsigned char>(std::clamp(average, 0.0, 255.0));
            }
            result.colors.push_back(value);
        }
        begin = end;
    }
    return true;
}

} // namespace pointcloud
