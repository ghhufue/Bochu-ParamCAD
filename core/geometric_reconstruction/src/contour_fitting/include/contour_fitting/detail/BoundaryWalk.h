#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace contour_fitting::detail {

// Each edge borders a retained face on its left. Follow that face's fan at
// the ending vertex so components that meet at a point keep separate rings.
template<class Edge, class Next, class Visit>
void appendBoundaryRings(Edge start, Next next, Visit visit,
                         std::vector<std::vector<std::size_t>>& rings)
{
    std::vector<std::size_t> walk;
    auto edge = start;
    do {
        walk.push_back(visit(edge));
        edge = next(edge);
    } while (edge != start);

    std::vector<std::size_t> path;
    std::unordered_map<std::size_t, std::size_t> positions;
    for (std::size_t i = 0; i <= walk.size(); ++i) {
        const auto vertex = walk[i % walk.size()];
        const auto [entry, inserted] = positions.emplace(vertex, path.size());
        if (inserted) {
            path.push_back(vertex);
            continue;
        }
        const auto begin = entry->second;
        if (path.size() - begin >= 3) rings.emplace_back(path.begin() + begin, path.end());
        for (auto j = begin + 1; j < path.size(); ++j) positions.erase(path[j]);
        path.resize(begin + 1);
    }
}

template<class Triangulation>
std::vector<std::vector<std::size_t>> traceBoundaryRings(Triangulation& triangulation)
{
    const auto kept = [&triangulation](auto face) {
        return !triangulation.is_infinite(face) && face->info().kept;
    };
    const auto next = [&](typename Triangulation::Edge edge) {
        auto [face, side] = edge;
        side = triangulation.ccw(side);
        while (kept(face->neighbor(side))) {
            const auto neighbor = face->neighbor(side);
            side = triangulation.ccw(neighbor->index(face));
            face = neighbor;
        }
        return typename Triangulation::Edge{face, side};
    };
    const auto visit = [&](typename Triangulation::Edge edge) {
        auto [face, side] = edge;
        face->info().boundaryVisited[side] = true;
        return face->vertex(triangulation.ccw(side))->info();
    };

    std::vector<std::vector<std::size_t>> rings;
    for (auto face = triangulation.finite_faces_begin(); face != triangulation.finite_faces_end(); ++face) {
        if (!face->info().kept) continue;
        for (int side = 0; side < 3; ++side) {
            if (!face->info().boundaryVisited[side] && !kept(face->neighbor(side)))
                appendBoundaryRings(typename Triangulation::Edge{face, side}, next, visit, rings);
        }
    }
    return rings;
}

} // namespace contour_fitting::detail
