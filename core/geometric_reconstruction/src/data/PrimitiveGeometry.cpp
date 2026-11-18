#include "reconstruction/data/PrimitiveGeometry.h"
#include "common/Math3.h"

namespace reconstruction {
using namespace math3;
using modeling::Vec3;
Vec3 primitiveAxis(const PrimitiveEvidence& e) {
    return std::visit([](const auto& s) -> Vec3 {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, PlaneEvidence>) return normalized(s.plane.normal);
        else if constexpr (std::is_same_v<T, CylinderEvidence>) return normalized(s.cylinder.axisDirection);
        else if constexpr (std::is_same_v<T, ConeEvidence>) return normalized(s.cone.axisDirection);
        else if constexpr (std::is_same_v<T, TorusEvidence>) return normalized(s.torus.axisDirection);
        else return {};
    }, e);
}

Vec3 primitiveCenter(const PrimitiveEvidence& e) {
    return std::visit([](const auto& s) -> Vec3 {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, CylinderEvidence>) return s.cylinder.axisPoint;
        else if constexpr (std::is_same_v<T, ConeEvidence>) return s.cone.apex;
        else if constexpr (std::is_same_v<T, SphereEvidence>) return s.sphere.center;
        else if constexpr (std::is_same_v<T, TorusEvidence>) return s.torus.center;
        else return {};
    }, e);
}

}

namespace reconstruction {
using namespace math3;
using modeling::Vec3;
double surfaceResidual(const PrimitiveEvidence& e, Vec3 p) {
    return std::visit([&](const auto& s) -> double {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, PlaneEvidence>) return std::abs(dot(s.plane.normal, p)+s.plane.offset);
        else if constexpr (std::is_same_v<T, SphereEvidence>) return std::abs(length(subtract(p, s.sphere.center))-s.sphere.radius);
        else {
            const Vec3 a = primitiveAxis(e), d = subtract(p, primitiveCenter(e));
            const double t = dot(d, a), r = length(subtract(d, scale(a, t)));
            if constexpr (std::is_same_v<T, CylinderEvidence>) return std::abs(r-s.cylinder.radius);
            else if constexpr (std::is_same_v<T, ConeEvidence>) {
                if (t < 0) return INFINITY;
                return std::abs(r*std::cos(s.cone.openingAngleRadians)-t*std::sin(s.cone.openingAngleRadians));
            } else return std::abs(std::hypot(r-s.torus.majorRadius, t)-s.torus.minorRadius);
        }
    }, e);
}
double surfaceNormalAlignment(const PrimitiveEvidence& e,Vec3 p,Vec3 normal) {
    if(!std::isfinite(length(normal)) || length(normal)<1e-12)return 0;
    normal=normalized(normal);
    return std::visit([&](const auto& s)->double {
        using T=std::decay_t<decltype(s)>;Vec3 n;
        if constexpr(std::is_same_v<T,PlaneEvidence>)n=s.plane.normal;
        else if constexpr(std::is_same_v<T,SphereEvidence>)n=subtract(p,s.sphere.center);
        else {
            const auto a=primitiveAxis(e),d=subtract(p,primitiveCenter(e));const double t=dot(d,a);
            const auto radial=subtract(d,scale(a,t));const double r=length(radial);
            if(r<1e-12)return 0;
            if constexpr(std::is_same_v<T,CylinderEvidence>)n=radial;
            else if constexpr(std::is_same_v<T,ConeEvidence>)n=subtract(scale(radial,std::cos(s.cone.openingAngleRadians)/r),scale(a,std::sin(s.cone.openingAngleRadians)));
            else n=subtract(d,scale(radial,s.torus.majorRadius/r));
        }
        return length(n)>1e-12?std::abs(dot(normal,normalized(n))):0;
    },e);
}
} // namespace reconstruction
