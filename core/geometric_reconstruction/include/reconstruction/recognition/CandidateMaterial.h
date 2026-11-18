#pragma once
#include "modeling/Geometry.h"

namespace reconstruction {
struct CompoundCandidate;
// Material predicate shared by mixed-feature recognition and repair validation.
// Covers the box/prism/attached-cylinder-or-cone/through-hole representation;
// arbitrary profile B-Reps require the CAD-backed boundary checks instead.
bool recognizedMaterialContains(const CompoundCandidate& candidate, modeling::Vec3 point);
} // namespace reconstruction
