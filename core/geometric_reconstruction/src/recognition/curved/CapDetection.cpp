#include "detail/CurvedInternal.h"

namespace reconstruction::detail::curved {
std::vector<Cap> caps(const PointStore &points, Vec3 origin, Vec3 axis, double tolerance,
                      const std::function<double(double)> &radius,
                      const CurvedSolidRecognitionOptions &options) {
    struct Item {
        double t;
        const PointSample *p;
        int sign;
    };
    std::vector<Item> items;
    for (const auto &p : points.points()) {
        const double projection = dot(normalized(*p.normal), axis);
        if (std::abs(projection) < options.capNormalCosine)
            continue;
        auto d = subtract(p.position, origin);
        const double t = dot(d, axis);
        if (radialLength(d, axis) > radius(t) + options.capRadialToleranceMultiplier * tolerance)
            continue;
        items.push_back({t, &p, projection > 0 ? 1 : -1});
    }
    std::sort(items.begin(), items.end(), [](auto a, auto b) { return a.t < b.t; });
    std::vector<Cap> result;
    for (std::size_t i = 0; i < items.size();) {
        auto end = i + 1;
        while (end < items.size() &&
               items[end].t - items[i].t <= options.capGroupToleranceMultiplier * tolerance)
            ++end;
        for (int sign : {-1, 1}) {
            Cap cap;
            cap.sign = sign;
            for (auto j = i; j < end; ++j)
                if (items[j].sign == sign)
                    cap.data.push_back(items[j].p);
            if (cap.data.size() >= std::max<std::size_t>(options.minimumCapPoints,
                    points.size() / std::max<std::size_t>(1, options.capPointDivisor))) {
                std::sort(cap.data.begin(), cap.data.end(),
                          [](auto a, auto b) { return a->id < b->id; });
                for (auto p : cap.data)
                    cap.t += dot(subtract(p->position, origin), axis);
                cap.t /= cap.data.size();
                // A ring around a through-hole mouth can look like an axial
                // cap if the radial tolerance reaches past the cylinder wall.
                // Require support in both radial bands of a filled disk.
                const double capRadius = radius(cap.t);
                const double outerMargin = std::min(tolerance, capRadius * 0.05);
                if (!std::isfinite(capRadius) || capRadius <= tolerance ||
                    capRadius - outerMargin <= capRadius * 0.5)
                    continue;
                std::array<std::size_t, 2> radialBandCounts{};
                for (const auto* point : cap.data) {
                    const double pointRadius = radialLength(
                        subtract(point->position, origin), axis);
                    if (pointRadius <= capRadius * 0.5)
                        ++radialBandCounts[0];
                    else if (pointRadius < capRadius - outerMargin)
                        ++radialBandCounts[1];
                }
                const auto minimumBandPoints = std::max<std::size_t>(
                    3, static_cast<std::size_t>(std::ceil(
                        cap.data.size() * options.minimumCapRadialBandPointFraction)));
                if (radialBandCounts[0] < minimumBandPoints ||
                    radialBandCounts[1] < minimumBandPoints)
                    continue;
                result.push_back(std::move(cap));
            }
        }
        i = end;
    }
    return result;
}

bool ends(const std::vector<Cap> &planes, double &start, double &end, bool pointed) {
    const Cap *low = nullptr;
    const Cap *high = nullptr;
    for (const auto &cap : planes) {
        auto *&chosen = cap.sign < 0 ? low : high;
        if (!chosen || cap.data.size() > chosen->data.size())
            chosen = &cap;
    }
    if (pointed && !(low && high)) {
        const Cap* base = high ? high : low;
        if (!base) return false;
        start = 0;
        end = base->t;
        return end > start;
    }
    if (!high || !low) return false;
    start = low->t;
    end = high->t;
    return end > start;
}
} // namespace reconstruction::detail::curved
