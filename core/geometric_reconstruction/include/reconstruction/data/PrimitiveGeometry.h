#pragma once
#include "reconstruction/data/PrimitiveEvidence.h"

namespace reconstruction {
modeling::Vec3 primitiveAxis(const PrimitiveEvidence& evidence);
modeling::Vec3 primitiveCenter(const PrimitiveEvidence& evidence);
double surfaceResidual(const PrimitiveEvidence& evidence, modeling::Vec3 point);
double surfaceNormalAlignment(const PrimitiveEvidence& evidence, modeling::Vec3 point,
                             modeling::Vec3 normal);
} // namespace reconstruction
