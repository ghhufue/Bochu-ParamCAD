#include "detail/PreprocessingInternal.h"

namespace reconstruction::detail::preprocessing {
struct VoxelKey {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const VoxelKey& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const noexcept {
        std::size_t seed = std::hash<std::int64_t>{}(key.x);
        seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U +
            (seed << 6U) + (seed >> 2U);
        seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U +
            (seed << 6U) + (seed >> 2U);
        return seed;
    }
};

struct VoxelRepresentative {
    std::size_t pointIndex = 0;
    double squaredDistanceToCenter = std::numeric_limits<double>::infinity();
    std::size_t seedIndex = 0;
};

std::vector<PointSample> voxelDownsample(
    const std::vector<PointSample>& points,
    double voxelSize,
    const std::unordered_set<PointId>& protectedIds) {
    if (voxelSize <= 0.0 || points.empty()) {
        return points;
    }

    std::unordered_map<VoxelKey, std::vector<VoxelRepresentative>, VoxelKeyHash> voxels;
    std::vector<std::size_t> selected;
    voxels.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
        if (protectedIds.count(points[index].id)) {
            selected.push_back(index);
            continue;
        }
        const Vec3& point = points[index].position;
        const std::array<double, 3> cells{{std::floor(point.x / voxelSize),
            std::floor(point.y / voxelSize), std::floor(point.z / voxelSize)}};
        const double minimumCell = static_cast<double>(std::numeric_limits<std::int64_t>::min());
        const double maximumCell = static_cast<double>(std::numeric_limits<std::int64_t>::max());
        if (std::any_of(cells.begin(), cells.end(), [&](double cell) {
                return !std::isfinite(cell) || cell < minimumCell || cell >= maximumCell;
            })) {
            // A robust part-scale voxel can be tiny relative to a far outlier.
            // Keep unindexable observations for filtering instead of invoking
            // an overflowing float-to-integer conversion or losing their IDs.
            selected.push_back(index);
            continue;
        }
        const VoxelKey key{
            static_cast<std::int64_t>(cells[0]),
            static_cast<std::int64_t>(cells[1]),
            static_cast<std::int64_t>(cells[2]),
        };
        const Vec3 center{
            (static_cast<double>(key.x) + 0.5) * voxelSize,
            (static_cast<double>(key.y) + 0.5) * voxelSize,
            (static_cast<double>(key.z) + 0.5) * voxelSize,
        };
        const double distance = squaredDistance(point, center);
        auto& representatives = voxels[key];
        const auto direction = [](const PointSample& sample) -> std::optional<Vec3> {
            if (!sample.normal || !std::isfinite(math3::length(*sample.normal))
                || math3::length(*sample.normal) <= 1e-12) return std::nullopt;
            return math3::normalized(*sample.normal);
        };
        const auto normal = direction(points[index]);
        auto compatible = [&](const VoxelRepresentative& representative) {
            const auto& seed = points[representative.seedIndex];
            const auto other = direction(seed);
            if (bool(normal) != bool(other)) return false;
            if (!normal) return true;
            if (points[index].normalOrientationKnown != seed.normalOrientationKnown) return false;
            double alignment = math3::dot(*normal, *other);
            if (!seed.normalOrientationKnown) alignment = std::abs(alignment);
            const auto delta = math3::subtract(point, seed.position);
            return alignment >= 0.8660254037844386 &&
                std::max(std::abs(math3::dot(delta, *normal)), std::abs(math3::dot(delta, *other))) <= voxelSize * .25;
        };
        auto representative = std::find_if(representatives.begin(), representatives.end(), compatible);
        if (representative == representatives.end()) representatives.push_back({index, distance, index});
        else if (distance < representative->squaredDistanceToCenter ||
            (distance == representative->squaredDistanceToCenter && points[index].id < points[representative->pointIndex].id))
            *representative = {index, distance, representative->seedIndex};
    }

    selected.reserve(selected.size() + voxels.size());
    for (const auto& entry : voxels) {
        for (const auto& representative : entry.second) selected.push_back(representative.pointIndex);
    }
    // Preserve source order and deterministic PointId ordering regardless of
    // unordered_map bucket layout.
    std::sort(selected.begin(), selected.end());
    std::vector<PointSample> result;
    result.reserve(selected.size());
    for (const std::size_t index : selected) {
        result.push_back(points[index]);
    }
    return result;
}
} // namespace reconstruction::detail::preprocessing
