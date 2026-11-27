#include "ClosureInternal.h"
#include "reconstruction/data/StageProfile.h"

#include <BOPAlgo_CellsBuilder.hxx>
#include <BRepBndLib.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

#include <array>
#include <sstream>

namespace reconstruction::detail::closure {
namespace {

class Partition : public BOPAlgo_CellsBuilder {
public:
    void select(const TopoDS_Solid& cell) {
        // OCCT selects parts by argument membership. A split of the envelope
        // has the same membership as its peers, so bind the individual part
        // to its own key for AddToResult.
        NCollection_List<TopoDS_Shape> take;
        take.Append(cell);
        if (myIndex.Contains(cell)) myIndex.ChangeFromKey(cell) = take;
        else myIndex.Add(cell, take);
        AddToResult(take, {}, 1, false);
    }
};

constexpr std::size_t infiniteCrossings = std::numeric_limits<std::size_t>::max();
struct Cell {
    TopoDS_Solid solid;
    std::pair<Vec3, Vec3> box;
    double volume = 0;
    bool exterior = false;
    std::size_t crossings = infiniteCrossings;
};

struct SharedFace {
    std::size_t first, second;
    double area, support;
};

double radicalInverse(std::size_t index, std::size_t base) {
    double value = 0, place = 1;
    while (index > 0) {
        place /= static_cast<double>(base);
        value += place * static_cast<double>(index % base);
        index /= base;
    }
    return value;
}

double faceSupport(const TopoDS_Face& face, const PointStore& points, const KdTree& tree,
                   double spacing, double tolerance) {
    BRepMesh_IncrementalMesh mesh(face, std::max(.5 * tolerance, .12 * spacing), false, .25, false);
    TopLoc_Location location;
    const auto triangles = BRep_Tool::Triangulation(face, location);
    if (triangles.IsNull()) return 0;
    struct Triangle { Vec3 a, b, c, normal; double endArea; };
    std::vector<Triangle> samples;
    double totalArea = 0;
    for (int i = 1; i <= triangles->NbTriangles(); ++i) {
        const auto& t = triangles->Triangle(i);
        const auto a = vector(triangles->Node(t.Value(1)).Transformed(location.Transformation()));
        const auto b = vector(triangles->Node(t.Value(2)).Transformed(location.Transformation()));
        const auto c = vector(triangles->Node(t.Value(3)).Transformed(location.Transformation()));
        const auto n = cross(subtract(b, a), subtract(c, a));
        const auto area = .5 * length(n);
        if (area <= 0) continue;
        totalArea += area;
        samples.push_back({a, b, c, normalized(n), totalArea});
    }
    const auto range = std::max(1.6 * spacing, 2.5 * tolerance);
    if (samples.empty()) return 0;
    const auto count = static_cast<int>(std::clamp(totalArea / (spacing * spacing), 24.0, 160.0));
    std::size_t triangle = 0;
    int supported = 0;
    for (int i = 0; i < count; ++i) {
        const double area = totalArea * (i + .5) / count;
        while (samples[triangle].endArea < area) ++triangle;
        const auto& t = samples[triangle];
        const double u = std::sqrt(radicalInverse(i + 1, 2));
        const double v = radicalInverse(i + 1, 3);
        const auto p = add(scale(t.a, 1 - u), add(scale(t.b, u * (1 - v)), scale(t.c, u * v)));
        for (const auto& [distance, j] : tree.nearest(p, 8)) {
            if (distance > range * range) break;
            const auto& sample = points.points()[j];
            const auto normal = sampleNormal(sample);
            if (std::abs(dot(subtract(sample.position, p), t.normal)) <=
                    std::max(2 * tolerance, .35 * spacing) &&
                (!normal || std::abs(dot(*normal, t.normal)) > .72)) {
                ++supported;
                break;
            }
        }
    }
    return static_cast<double>(supported) / count;
}

// Unsigned observations still constrain occupancy. Traverse from the exterior
// with zero cost across unsupported cuts and count observed walls along the
// least-crossing route. Even parity also represents enclosed voids.
void countObservedWalls(std::vector<Cell>& cells, const std::vector<SharedFace>& faces) {
    std::vector<std::vector<std::pair<std::size_t, bool>>> links(cells.size());
    for (const auto& face : faces) {
        const bool observed = face.support >= .55;
        links[face.first].emplace_back(face.second, observed);
        links[face.second].emplace_back(face.first, observed);
    }
    using Entry = std::pair<std::size_t, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    for (std::size_t i = 0; i < cells.size(); ++i)
        if (cells[i].exterior) {
            cells[i].crossings = 0;
            queue.emplace(0, i);
        }
    while (!queue.empty()) {
        const auto [distance, i] = queue.top();
        queue.pop();
        if (distance != cells[i].crossings) continue;
        for (const auto& [neighbor, wall] : links[i])
            if (distance + wall < cells[neighbor].crossings) {
                cells[neighbor].crossings = distance + wall;
                queue.emplace(distance + wall, neighbor);
            }
    }
}

double normalVote(Vec3 p, const PointStore& points, const KdTree& tree, double tolerance) {
    double sum = 0, weights = 0;
    const auto near = tree.nearest(p, 32);
    if (near.empty()) return 0;
    const double nearest = std::max(near.front().first, tolerance * tolerance);
    for (const auto& [distance, i] : near) {
        if (distance > 4 * nearest) break;
        const auto& sample = points.points()[i];
        if (!sample.normalOrientationKnown || distance <= 0) continue;
        const auto n = sampleNormal(sample);
        if (!n) continue;
        const double side = dot(*n, subtract(sample.position, p)) / std::sqrt(distance);
        const double weight = nearest / std::max(nearest, distance);
        sum += side * weight;
        weights += weight;
    }
    return weights >= 1 ? sum / weights : 0;
}

double insideEvidence(const Cell& cell, const PointStore& points, const KdTree& tree,
                      double spacing, double tolerance) {
    const auto& [lo, hi] = cell.box;
    const auto size = subtract(hi, lo);
    const auto target = static_cast<std::size_t>(std::clamp(std::cbrt(cell.volume) / spacing * 4, 16., 200.));
    BRepClass3d_SolidClassifier classifier(cell.solid);
    double votes = 0;
    std::size_t n = 0;
    for (std::size_t i = 1; i <= target * 5 + 32 && n < target; ++i) {
        const Vec3 p{lo.x + size.x * radicalInverse(i, 2), lo.y + size.y * radicalInverse(i, 3),
                     lo.z + size.z * radicalInverse(i, 5)};
        classifier.Perform(point(p), std::min(.1 * tolerance, .02 * spacing));
        if (classifier.State() != TopAbs_IN) continue;
        votes += normalVote(p, points, tree, tolerance);
        ++n;
    }
    const double observedWalls = cell.crossings == infiniteCrossings ? 0 : (cell.crossings % 2 ? 1.0 : -1.0);
    return .6 * observedWalls + (n ? votes / n : 0);
}

using Edge = boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::directedS>::edge_descriptor;
using FlowGraph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, boost::no_property,
    boost::property<boost::edge_capacity_t, double,
        boost::property<boost::edge_residual_capacity_t, double,
            boost::property<boost::edge_reverse_t, Edge>>>>;

std::vector<boost::default_color_type> classify(std::vector<Cell>& cells, const std::vector<SharedFace>& faces,
    const PointStore& points, const KdTree& tree, double spacing, double tolerance,
    const std::function<bool()>& cancelled) {
    countObservedWalls(cells, faces);
    const auto source = cells.size(), sink = source + 1;
    FlowGraph graph(cells.size() + 2);
    const auto capacity = get(boost::edge_capacity, graph);
    const auto reverse = get(boost::edge_reverse, graph);
    const auto connect = [&](std::size_t a, std::size_t b, double ab, double ba) {
        const auto forward = add_edge(a, b, graph).first, backward = add_edge(b, a, graph).first;
        put(capacity, forward, ab);
        put(capacity, backward, ba);
        put(reverse, forward, backward);
        put(reverse, backward, forward);
    };
    double finiteCost = 1;
    for (const auto& face : faces) {
        const double scale = face.area / (spacing * spacing);
        const double weight = scale * (.0001 + .18 * std::pow(1 - face.support, 4));
        connect(face.first, face.second, weight, weight);
        finiteCost += 2 * weight;
    }
    for (std::size_t i = 0; i < cells.size(); ++i) {
        if (cancelled && cancelled()) return {};
        if (cells[i].exterior) continue;
        const double evidence = insideEvidence(cells[i], points, tree, spacing, tolerance);
        const double weight = std::max(1.0, std::pow(cells[i].volume, 2. / 3.) / (spacing * spacing));
        const double inside = weight * std::max(0., -evidence);
        const double outside = weight * std::max(0., evidence);
        connect(source, i, outside, 0);
        connect(i, sink, inside, 0);
        finiteCost += inside + outside;
    }
    for (std::size_t i = 0; i < cells.size(); ++i)
        if (cells[i].exterior) connect(i, sink, finiteCost, 0);
    std::vector<boost::default_color_type> colors(cells.size() + 2);
    const auto index = get(boost::vertex_index, graph);
    boost::boykov_kolmogorov_max_flow(graph, capacity, get(boost::edge_residual_capacity, graph), reverse,
        boost::make_iterator_property_map(colors.begin(), index), index, source, sink);
    return colors;
}
} // namespace

TopoDS_Solid partitionAndSelect(const PointStore& points, const KdTree& tree, const Bnd_Box& envelope,
    const CuttingGeometry& geometry, double spacing, double tolerance, double fuzzy,
    const BrepClosureOptions& options, ClosedBrepResult& result, std::string& error,
    const std::function<bool()>& cancelled, BrepInspectionAttempt* inspection) {
    const auto [lo, hi] = bounds(envelope);
    Partition builder;
    builder.AddArgument(BRepPrimAPI_MakeBox(point(lo), point(hi)).Solid());
    for (const auto& face : geometry.faces) builder.AddArgument(face);
    builder.SetFuzzyValue(fuzzy);
    builder.SetNonDestructive(true);
    builder.SetRunParallel(options.enableParallelBop);
    diagnostics::profileStage("brep.cells_builder", [&] { builder.Perform(); });
    if (cancelled && cancelled()) return {};
    if (builder.HasErrors()) {
        std::ostringstream messages;
        builder.DumpErrors(messages);
        error = "CellsBuilder partition failed: " + messages.str();
        return {};
    }
    const auto& parts = builder.GetAllParts();
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> solids;
    TopExp::MapShapes(parts, TopAbs_SOLID, solids);
    result.quality.subSolidCount = solids.Extent();
    result.quality.insideSolidCount = 0;
    if (solids.Extent() < 2 && !inspection) {
        error = "Cutting faces did not partition the exterior envelope";
        return {};
    }
    std::vector<Cell> cells;
    const double boundaryTolerance = 2 * fuzzy + Precision::Confusion();
    for (int i = 1; i <= solids.Extent(); ++i) {
        const auto solid = TopoDS::Solid(solids(i));
        Bnd_Box box;
        BRepBndLib::AddOptimal(solid, box, false, false);
        const auto [a, b] = bounds(box);
        const bool exterior = a.x - lo.x <= boundaryTolerance || a.y - lo.y <= boundaryTolerance ||
            a.z - lo.z <= boundaryTolerance || hi.x - b.x <= boundaryTolerance ||
            hi.y - b.y <= boundaryTolerance || hi.z - b.z <= boundaryTolerance;
        GProp_GProps properties;
        BRepGProp::VolumeProperties(solid, properties, true);
        cells.push_back({solid, {a, b}, std::max(0., properties.Mass()), exterior});
    }
    if (inspection && !cells.empty()) {
        inspection->stage = BrepInspectionAttempt::Stage::Partitioned;
        inspection->cells.reserve(cells.size());
        for (const auto& cell : cells) {
            const auto label = cell.exterior ? BrepInspectionCell::Label::ExteriorAnchor :
                                               BrepInspectionCell::Label::Unclassified;
            inspection->cells.push_back({cell.solid, label, cell.volume});
        }
    }
    if (cells.size() < 2) {
        error = "Cutting faces did not partition the exterior envelope";
        return {};
    }
    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher> ancestors;
    TopExp::MapShapesAndUniqueAncestors(parts, TopAbs_FACE, TopAbs_SOLID, ancestors);
    std::vector<SharedFace> faces;
    for (int i = 1; i <= ancestors.Extent(); ++i) {
        if (cancelled && cancelled()) return {};
        const auto& owners = ancestors(i);
        if (owners.Extent() != 2) continue;
        const auto face = TopoDS::Face(ancestors.FindKey(i));
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(face, properties);
        faces.push_back({std::size_t(solids.FindIndex(owners.First()) - 1),
            std::size_t(solids.FindIndex(owners.Last()) - 1), properties.Mass(),
            faceSupport(face, points, tree, spacing, tolerance)});
    }
    const auto labels = diagnostics::profileStage("brep.graph_cut", [&] {
        return classify(cells, faces, points, tree, spacing, tolerance, cancelled);
    });
    if (labels.empty()) return {};
    if (inspection) {
        inspection->stage = BrepInspectionAttempt::Stage::Classified;
        for (std::size_t i = 0; i < cells.size(); ++i) {
            if (cells[i].exterior) continue;
            inspection->cells[i].label = labels[i] == labels[cells.size()] ? BrepInspectionCell::Label::Inside :
                                                                          BrepInspectionCell::Label::Outside;
        }
    }
    for (std::size_t i = 0; i < cells.size(); ++i) {
        if (cells[i].exterior || labels[i] != labels[cells.size()]) continue;
        builder.select(cells[i].solid);
        ++result.quality.insideSolidCount;
    }
    if (!result.quality.insideSolidCount) {
        error = "Point-cloud graph cut found no interior cells";
        return {};
    }
    builder.RemoveInternalBoundaries();
    TopExp_Explorer explorer(builder.Shape(), TopAbs_SOLID);
    if (!explorer.More()) {
        error = "CellsBuilder could not retain the selected cells";
        return {};
    }
    const auto solid = TopoDS::Solid(explorer.Current());
    explorer.Next();
    if (explorer.More()) {
        error = "Point-cloud graph cut produced several disconnected solids";
        return {};
    }
    return solid;
}

} // namespace reconstruction::detail::closure
