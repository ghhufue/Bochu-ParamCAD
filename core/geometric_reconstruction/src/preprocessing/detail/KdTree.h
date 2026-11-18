#pragma once
// Compatibility for private helper users. New implementation code uses common/spatial.
#include "common/spatial/KdTree.h"
namespace reconstruction::detail::preprocessing { using KdTree = spatial::KdTree; }
