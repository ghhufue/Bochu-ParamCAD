#include "detail/RansacEvidence.h"

namespace reconstruction::detail::cgal {
namespace {

double median(std::vector<double> values) {
    const auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

} // namespace

PlaneEquation fitPlane(
    const InputRange& input,
    const std::vector<std::size_t>& indices,
    bool robust) {
    std::vector<double> weights(indices.size(), 1.0);
    PlaneEquation plane;
    const int iterations = robust ? 5 : 1;
    for (int iteration = 0; iteration < iterations; ++iteration) {
        modeling::Vec3 centroid{};
        double totalWeight = 0;
        for (std::size_t i = 0; i < indices.size(); ++i) {
            centroid = math3::add(centroid,
                math3::scale(toVec3(std::get<0>(input[indices[i]])), weights[i]));
            totalWeight += weights[i];
        }
        centroid = math3::scale(centroid, 1.0 / totalWeight);

        std::array<std::array<double, 3>, 3> covariance{};
        for (std::size_t i = 0; i < indices.size(); ++i) {
            const modeling::Vec3 delta = math3::subtract(
                toVec3(std::get<0>(input[indices[i]])), centroid);
            const std::array<double, 3> value{delta.x, delta.y, delta.z};
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    covariance[row][column] += weights[i] * value[row] * value[column];
                }
            }
        }

        const modeling::Vec3 normal = math3::smallestEigenvector(covariance);
        plane = {normal, -math3::dot(normal, centroid)};
        std::vector<double> residuals;
        residuals.reserve(indices.size());
        for (const std::size_t index : indices) {
            residuals.push_back(math3::dot(normal, toVec3(std::get<0>(input[index]))) + plane.offset);
        }
        const double center = median(residuals);
        for (double& residual : residuals) residual = std::abs(residual - center);
        const double scale = 1.4826 * median(std::move(residuals));
        if (!(scale > 1e-12) || !std::isfinite(scale)) break;

        const double cutoff = 1.345 * scale;
        for (std::size_t i = 0; i < indices.size(); ++i) {
            const double residual = std::abs(
                math3::dot(normal, toVec3(std::get<0>(input[indices[i]]))) + plane.offset - center);
            weights[i] = residual > cutoff ? cutoff / residual : 1.0;
        }
    }
    return plane;
}

modeling::Vec3 centroidOf(
    const InputRange& input,
    const std::vector<std::size_t>& indices) {
    modeling::Vec3 centroid{};
    for (const std::size_t index : indices) {
        centroid = math3::add(centroid, toVec3(std::get<0>(input[index])));
    }
    return math3::scale(
        centroid, 1.0 / static_cast<double>(indices.size()));
}

bool duplicatePlane(
    const PlaneEquation& candidate,
    const modeling::Vec3& candidateCentroid,
    const PlaneEquation& accepted,
    const modeling::Vec3& acceptedCentroid,
    const PlaneDetectionOptions& options) {
    const double minimumAlignment =
        std::cos(options.duplicatePlaneAngularToleranceRadians);
    if (std::abs(math3::dot(candidate.normal, accepted.normal)) <
        minimumAlignment) {
        return false;
    }
    const double tolerance = options.distanceThreshold *
        options.duplicatePlaneDistanceMultiplier;
    return planeResidual(accepted, candidateCentroid) <= tolerance &&
        planeResidual(candidate, acceptedCentroid) <= tolerance;
}

std::vector<std::size_t> filteredPlaneSupport(
    const InputRange& input,
    const std::vector<std::size_t>& candidateIndices,
    const PlaneEquation& plane,
    const PlaneDetectionOptions& options) {
    std::vector<std::size_t> result;
    result.reserve(candidateIndices.size());
    for (const std::size_t index : candidateIndices) {
        const modeling::Vec3 normal = math3::normalized(
            toVec3(std::get<1>(input[index])));
        if (planeResidual(plane, toVec3(std::get<0>(input[index]))) <=
                options.distanceThreshold &&
            std::abs(math3::dot(normal, plane.normal)) >=
                options.minimumNormalAlignment) {
            result.push_back(index);
        }
    }
    return result;
}

bool makePlaneEvidence(
    const InputRange& input,
    const CgalPlane& shape,
    const PointStore& points,
    const PlaneDetectionOptions& options,
    PlaneEvidence& evidence,
    modeling::Vec3& centroid) {
    std::vector<std::size_t> support = shape.indices_of_assigned_points();
    if (support.size() < options.minimumSupportPoints) {
        return false;
    }

    PlaneEquation plane = fitPlane(input, support);
    support = filteredPlaneSupport(input, support, plane, options);
    if (support.size() < options.minimumSupportPoints) {
        return false;
    }
    plane = fitPlane(input, support);
    support = filteredPlaneSupport(input, support, plane, options);
    if (support.size() < options.minimumSupportPoints) {
        return false;
    }
    plane = fitPlane(input, support, true);

    evidence = {};
    evidence.plane = plane;
    evidence.supportPointIds.reserve(support.size());
    double residualSum = 0.0;
    for (const std::size_t index : support) {
        const double residual = planeResidual(
            plane, toVec3(std::get<0>(input[index])));
        evidence.supportPointIds.push_back(std::get<2>(input[index]));
        residualSum += residual;
        evidence.maxAbsoluteResidual =
            std::max(evidence.maxAbsoluteResidual, residual);
    }
    evidence.meanAbsoluteResidual = residualSum /
        static_cast<double>(support.size());
    evidence.coverage = static_cast<double>(support.size()) /
        static_cast<double>(points.size());
    evidence.confidence = std::clamp(
        1.0 - evidence.meanAbsoluteResidual / options.distanceThreshold,
        0.0,
        1.0);
    centroid = centroidOf(input, support);
    return true;
}
} // namespace reconstruction::detail::cgal
