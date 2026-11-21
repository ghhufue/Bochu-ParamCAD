#pragma once

#include "modeling/Id.h"

#include <cmath>
#include <optional>
#include <variant>

namespace modeling {

struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

struct Line2D {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
};

struct Rectangle2D {
    double x = 0.0;
    double y = 0.0;
    double width = 0.0;
    double height = 0.0;
};

struct Circle2D {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
};

struct Arc2D {
    Point2D start, middle, end;
};

struct ArcParameters {
    Point2D center;
    double radius, startAngle, sweep;

    Point2D pointAt(double fraction) const {
        const double angle = startAngle + fraction * sweep;
        return {center.x + radius * std::cos(angle), center.y + radius * std::sin(angle)};
    }
};

inline std::optional<ArcParameters> arcParameters(const Arc2D& arc) {
    const double ux = arc.middle.x - arc.start.x, uy = arc.middle.y - arc.start.y;
    const double vx = arc.end.x - arc.start.x, vy = arc.end.y - arc.start.y;
    const double uu = ux*ux + uy*uy, vv = vx*vx + vy*vy;
    const double determinant = 2 * (ux*vy - uy*vx);
    if (!std::isfinite(determinant) || std::abs(determinant) <= 1e-10 * std::sqrt(uu*vv))
        return {};
    const double x = (uu*vy - vv*uy) / determinant;
    const double y = (ux*vv - vx*uu) / determinant;
    const double radius = std::hypot(x, y);
    Point2D center{arc.start.x + x, arc.start.y + y};
    if (!(radius > 0) || !std::isfinite(radius) || !std::isfinite(center.x) || !std::isfinite(center.y))
        return {};
    constexpr double turn = 6.28318530717958647692;
    const double start = std::atan2(-y, -x);
    const double end = std::atan2(vy-y, vx-x);
    double sweep = std::fmod(end - start + turn, turn);
    if (determinant < 0) sweep -= turn;
    return ArcParameters{center, radius, start, sweep};
}

enum class SketchEntityType {
    Point,
    Line,
    Rectangle,
    Circle,
    Arc,
};

using SketchGeometry = std::variant<Point2D, Line2D, Rectangle2D, Circle2D, Arc2D>;

struct SketchEntity {
    SketchEntityId id = kInvalidSketchEntityId;
    SketchGeometry geometry;
};

inline SketchEntityType sketchEntityType(const SketchEntity& entity) {
    if (std::holds_alternative<Point2D>(entity.geometry)) {
        return SketchEntityType::Point;
    }
    if (std::holds_alternative<Line2D>(entity.geometry)) {
        return SketchEntityType::Line;
    }
    if (std::holds_alternative<Rectangle2D>(entity.geometry)) {
        return SketchEntityType::Rectangle;
    }
    return std::holds_alternative<Circle2D>(entity.geometry) ? SketchEntityType::Circle : SketchEntityType::Arc;
}

}  // namespace modeling
