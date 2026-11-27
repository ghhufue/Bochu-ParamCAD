#pragma once

#include "reconstruction/solid_inference/SolidInference.h"

namespace reconstruction {

void proposeTriangularGussets(const SurfaceScene& scene, const SolidCandidate& base,
    std::vector<SolidCandidate>& output, std::size_t maximumProposals);

} // namespace reconstruction
