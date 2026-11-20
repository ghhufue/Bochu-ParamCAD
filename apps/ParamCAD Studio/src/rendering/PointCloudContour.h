#pragma once

#include <contour_fitting/ContourFitting.h>
#include <modeling/FaceReference.h>

struct PointCloudContourInput {
    modeling::Plane3d plane;
    std::vector<contour_fitting::Contour> contours;
    double spacing = 0;
};
