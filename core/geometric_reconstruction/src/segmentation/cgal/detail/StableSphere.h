#pragma once

#include <CGAL/Shape_detection/Efficient_RANSAC/Shape_base.h>
#include "common/Math3.h"

namespace reconstruction::detail::cgal {

// The geometric sphere is the same for inward, outward and unoriented normals.
template<class Traits>
class StableSphere final : public CGAL::Shape_detection::Shape_base<Traits> {
    using Point = typename Traits::Point_3;
    using Vector = typename Traits::Vector_3;
    using FT = typename Traits::FT;
    using Vec = modeling::Vec3;

    Vec center_{};
    double radius_ = 0;

    template<class Coordinate>
    static Vec vec(const Coordinate& p) {
        return {CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())};
    }

public:
    Point center() const { return {center_.x, center_.y, center_.z}; }
    double radius() const { return radius_; }

    FT squared_distance(const Point& p) const override {
        const double residual = math3::length(math3::subtract(vec(p), center_)) - radius_;
        return residual * residual;
    }

protected:
    void create_shape(const std::vector<std::size_t>& indices) override {
        using namespace math3;
        this->m_is_valid = false;
        const auto first = vec(this->point(indices[0]));
        const auto second = vec(this->point(indices[1]));
        const auto firstNormal = vec(this->normal(indices[0]));
        const auto secondNormal = vec(this->normal(indices[1]));
        const auto separation = subtract(second, first);
        const double alignment = dot(firstNormal, secondNormal);
        const double determinant = 1 - alignment * alignment;
        if (determinant < 1e-5) return;

        // A sphere center lies on each radial line. Average the nearest points
        // on the first two lines, which can be skew under measurement noise.
        const double d = dot(firstNormal, separation);
        const double e = dot(secondNormal, separation);
        const auto a = scale(firstNormal, (d - alignment * e) / determinant);
        const auto b = add(separation, scale(secondNormal, (alignment * d - e) / determinant));
        const auto midpoint = scale(add(a, b), .5);
        center_ = add(first, midpoint);
        radius_ = .5 * (length(midpoint) + length(subtract(separation, midpoint)));
        if (!std::isfinite(radius_) || radius_ <= 1e-12) return;
        for (auto index : indices) {
            const auto position = this->point(index);
            if (squared_distance(position) > this->m_epsilon * this->m_epsilon ||
                normalAlignment(position, this->normal(index)) < this->m_normal_threshold)
                return;
        }
        this->m_is_valid = true;
    }

    void squared_distance(const std::vector<std::size_t>& indices,
                          std::vector<FT>& distances) const override {
        for (std::size_t i = 0; i < indices.size(); ++i)
            distances[i] = squared_distance(this->point(indices[i]));
    }

    FT normalAlignment(const Point& position, const Vector& normal) const {
        const auto radial = math3::subtract(vec(position), center_);
        const double distance = math3::length(radial);
        return distance > 1e-12 ? std::abs(math3::dot(vec(normal), radial)) / distance : 0;
    }

    void cos_to_normal(const std::vector<std::size_t>& indices,
                       std::vector<FT>& alignments) const override {
        for (std::size_t i = 0; i < indices.size(); ++i)
            alignments[i] = normalAlignment(this->point(indices[i]), this->normal(indices[i]));
    }

    std::size_t minimum_sample_size() const override { return 3; }
};

} // namespace reconstruction::detail::cgal
