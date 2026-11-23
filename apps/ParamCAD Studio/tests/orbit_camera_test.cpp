#include "core/OrbitCamera.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool near(double left, double right, double tolerance = 1.0e-12)
{
    return std::abs(left - right) <= tolerance;
}

void checkPole(double elevation, double expectedZ, double expectedUpY)
{
    CameraParameters base;
    const CameraParameters camera =
        OrbitCamera::toCamera({10.0, 270.0, elevation}, base, true);
    require(near(camera.position[0], 0.0) && near(camera.position[1], 0.0),
            "pole X/Y must be pinned to the target");
    require(near(camera.position[2], expectedZ),
            "pole Z must equal the signed orbit radius");
    require(near(camera.target[0], 0.0) && near(camera.target[1], 0.0)
                && near(camera.target[2], 0.0),
            "pole camera must look at the origin");
    require(near(camera.up[0], 0.0) && near(camera.up[1], expectedUpY)
                && near(camera.up[2], 0.0),
            "pole camera must have a stable horizontal up vector");

    const double view[3]{camera.target[0] - camera.position[0],
                         camera.target[1] - camera.position[1],
                         camera.target[2] - camera.position[2]};
    const double dot = view[0] * camera.up[0] + view[1] * camera.up[1]
                       + view[2] * camera.up[2];
    require(near(dot, 0.0), "pole up vector must be perpendicular to view direction");
}

} // namespace

int main()
{
    try {
        checkPole(90.0, 10.0, 1.0);
        checkPole(-90.0, -10.0, -1.0);

        CameraParameters base;
        base.target[0] = 12.0;
        base.target[1] = -7.0;
        base.target[2] = 4.0;
        base.up[0] = 0.0;
        base.up[1] = 1.0;
        base.up[2] = 0.0;
        const CameraParameters orbit =
            OrbitCamera::toCamera({20.0, 90.0, 45.0}, base, false);
        require(near(orbit.target[0], base.target[0])
                    && near(orbit.target[1], base.target[1])
                    && near(orbit.target[2], base.target[2]),
                "orbit around a retained target must preserve that target");
        require(near(orbit.position[0], base.target[0])
                    && near(orbit.position[1], base.target[1] + 20.0 / std::sqrt(2.0))
                    && near(orbit.position[2], base.target[2] + 20.0 / std::sqrt(2.0)),
                "orbit position must use the requested azimuth and elevation");
        require(near(orbit.up[0], 0.0) && near(orbit.up[1], -1.0 / std::sqrt(2.0))
                    && near(orbit.up[2], 1.0 / std::sqrt(2.0)),
                "orbit around a retained target must update the horizon up vector");

        std::cout << "Orbit camera tests passed.\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Orbit camera pole test failed: " << exception.what() << '\n';
        return 1;
    }
}
