#pragma once

#include "reconstruction/solid_inference/SolidInference.h"

namespace reconstruction {

std::optional<SolidCandidate> proposeSideThroughHoles(const SurfaceScene& scene, const SolidCandidate& base,
    std::size_t maximumOperations);

} // namespace reconstruction
