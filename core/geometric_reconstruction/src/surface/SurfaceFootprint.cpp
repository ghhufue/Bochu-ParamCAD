#include "reconstruction/surface/SurfaceFootprint.h"
#include "common/Math3.h"
#include "common/spatial/KdTree.h"
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_segment_primitive_3.h>
#include <CGAL/AABB_triangle_primitive_3.h>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <unordered_map>

namespace reconstruction {
namespace {
using XY = std::array<double, 2>;
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
constexpr double pi = 3.14159265358979323846;
using Options = reconstruction::SurfaceMeshingSettings;
using Point = Kernel::Point_2;
using Vertex = CGAL::Triangulation_vertex_base_with_info_2<size_t, Kernel>;
struct FaceState {
    bool kept = false, visited = false;
    std::array<bool, 3> boundaryVisited{};
};
using Face = CGAL::Triangulation_face_base_with_info_2<FaceState, Kernel>;
using DT = CGAL::Delaunay_triangulation_2<Kernel, CGAL::Triangulation_data_structure_2<Vertex, Face>>;
struct TrimState { int nesting = -1; bool kept = false; };
using TrimFace = CGAL::Triangulation_face_base_with_info_2<TrimState, Kernel,
    CGAL::Constrained_triangulation_face_base_2<Kernel>>;
using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel,
    CGAL::Triangulation_data_structure_2<Vertex, TrimFace>, CGAL::Exact_predicates_tag>;
struct Sample {
    XY position;
    std::vector<std::uint64_t> sources;
    double spacing = 0;
    double observedSpacing = 0;
    double density = 0;
    bool regularSpacing = false;
    bool supported = false;
    std::vector<size_t> neighbors;
};
double distance2(XY a, XY b) {
    const double x = a[0]-b[0], y = a[1]-b[1];
    return x*x+y*y;
}
double median(std::vector<double> values) {
    const auto middle = values.begin()+values.size()/2;
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

double areaWeightedDensity(const std::vector<double>& densities) {
    std::vector<std::pair<double,double>> values;
    for(double density:densities)values.emplace_back(std::log(density),1/density);
    std::sort(values.begin(),values.end());
    double total=0;for(const auto& value:values)total+=value.second;
    double cumulative=0,sum=0,weight=0;
    for(const auto& value:values) {
        const double next=cumulative+value.second;
        // Each observation votes for its represented area. Trim the outer
        // quarters before averaging logs, including at density transitions.
        const double kept=std::max(0.,std::min(next,total*.75)-std::max(cumulative,total*.25));
        sum+=kept*value.first;weight+=kept;cumulative=next;
    }
    return std::exp(sum/weight);
}

// Spatial queries are Euclidean kNN/radius queries, independent of any
// triangulation edges. In particular, a remote point cannot set a line's scale.
class Locator {
    std::vector<PointSample> points;
    std::unique_ptr<detail::spatial::KdTree> tree;
public:
    explicit Locator(const std::vector<Sample>& samples) {
        points.reserve(samples.size());
        for(std::size_t i=0;i<samples.size();++i) {
            PointSample p;p.id=i+1;p.position={samples[i].position[0],samples[i].position[1],0};points.push_back(p);
        }
        tree=std::make_unique<detail::spatial::KdTree>(points);
    }
    std::vector<size_t> nearest(XY p,size_t count) const {
        std::vector<size_t> result;
        for(const auto& item:tree->nearest(modeling::Vec3{p[0],p[1],0},count))result.push_back(item.second);
        return result;
    }
    size_t closest(XY p) const {return nearest(p,1).front();}
};

std::vector<Sample> distinctSamples(const std::vector<PlanarSurfaceSample>& input) {
    std::vector<Sample> raw;raw.reserve(input.size());
    for (const auto& sample:input) { Sample p;p.position=sample.position;raw.push_back(std::move(p)); }
    Locator locator(raw);double finestSpacing=INFINITY;
    for (const auto& sample:raw) {
        std::vector<double> distances;
        for (size_t id:locator.nearest(sample.position,std::min<size_t>(13,raw.size()))) {
            const double d=std::sqrt(distance2(sample.position,raw[id].position));
            if (d>0)distances.push_back(d);
        }
        if (distances.empty())continue;
        std::sort(distances.begin(),distances.end());double spacing=distances.front();
        // A precision-sized cluster followed by several comparable neighbours
        // is repeated sampling. A lone distant neighbour is never such evidence.
        for (size_t i=1;i+2<distances.size();++i)
            if (distances[i]>100*distances[i-1] && distances[i+2]<=3*distances[i]) { spacing=distances[i];break; }
        finestSpacing=std::min(finestSpacing,spacing);
    }
    // The separation of coplanar islands cannot inflate a precision weld.
    // Use the finest observed local scale so a coarser island cannot erase a
    // genuinely fine, independent patch on the same infinite support plane.
    const double epsilon=std::isfinite(finestSpacing)?std::max(finestSpacing*.001,1e-12):1e-12;
    using Cell = std::pair<long long, long long>;
    std::map<Cell, std::vector<size_t>> cells;
    std::vector<Sample> result;
    for (const auto& inputSample : input) {
        const auto p = inputSample.position;
        const double u = std::floor(p[0]/epsilon), v = std::floor(p[1]/epsilon);
        if (std::abs(u)>9e18 || std::abs(v)>9e18) continue;
        const Cell cell{static_cast<long long>(u), static_cast<long long>(v)};
        size_t match = result.size();
        for (int x = -1; x <= 1 && match == result.size(); ++x) for (int y = -1; y <= 1 && match == result.size(); ++y) {
            const auto found = cells.find({cell.first+x, cell.second+y});
            if (found == cells.end()) continue;
            for (size_t id : found->second) if (distance2(p, result[id].position)<=epsilon*epsilon) { match = id; break; }
        }
        if (match == result.size()) {
            cells[cell].push_back(match);
            Sample sample; sample.position = p; result.push_back(std::move(sample));
        }
        result[match].sources.push_back(inputSample.sourcePointId);
    }
    return result;
}

std::vector<std::vector<size_t>> supportedComponents(std::vector<Sample>& samples,const Options& settings,double maximumSupportSpacing) {
    Locator locator(samples);
    for (size_t i = 0; i < samples.size(); ++i) {
        auto neighbors = locator.nearest(samples[i].position, std::min(settings.densityNeighborCount+1, samples.size()));
        std::vector<double> distances;
        for (size_t j : neighbors) if (j != i) distances.push_back(std::sqrt(distance2(samples[i].position, samples[j].position)));
        if (distances.size()<3) continue;
        std::sort(distances.begin(), distances.end());
        // Median of the first three genuine neighbours tolerates narrow strips
        // while never promoting the farthest triangulation neighbour to scale.
        const double spacing = distances[1];
        if (!(spacing>0)) continue;
        samples[i].spacing = spacing;
        samples[i].observedSpacing = spacing;
        samples[i].regularSpacing = distances.size()>=3 && distances[2]-distances[0]<=settings.regularSpacingTolerance*spacing;
        // Estimate area density at several neighbour ranks. A very close pair
        // must not shrink a whole patch's support radius. Stop before a remote
        // island or isolated observation can become part of the estimate.
        for (size_t j = 5; j < distances.size(); ++j) if (distances[j]>settings.densityGapMultiplier*distances[j-1]) {
            distances.resize(j); break;
        }
        if (distances.size()<5) continue;
        std::vector<double> densities;
        for (size_t rank : {size_t(8), size_t(12), size_t(16)}) {
            rank = std::min(rank, distances.size());
            densities.push_back(double(rank)/(pi*distances[rank-1]*distances[rank-1]));
        }
        samples[i].density = median(std::move(densities));
        const double supportRadius = std::min(settings.localSupportRadiusMultiplier/std::sqrt(samples[i].density), distances.back());
        XY mean{};size_t count = 0;
        for (size_t j : neighbors) if (distance2(samples[i].position, samples[j].position)<=supportRadius*supportRadius) {
            samples[i].neighbors.push_back(j);
            mean[0]+=samples[j].position[0];mean[1]+=samples[j].position[1];++count;
        }
        if (count<6) continue;
        mean[0]/=count;mean[1]/=count;
        double xx = 0, xy = 0, yy = 0;
        for (size_t j : samples[i].neighbors) {
            const double x = samples[j].position[0]-mean[0], y = samples[j].position[1]-mean[1];
            xx+=x*x;xy+=x*y;yy+=y*y;
        }
        xx/=count;xy/=count;yy/=count;
        const double transverse = (xx+yy-std::hypot(xx-yy, 2*xy))*.5;
        samples[i].supported = transverse>=spacing*spacing*settings.minimumTransverseVarianceRatio;
    }
    // Smooth only on observed, two-dimensional support. Empty locations never
    // participate in this background fit. Trimmed log means resist clusters
    // without imposing one scene-wide density on sparse parts of the face.
    std::vector<double> densities(samples.size());
    for (size_t i = 0; i < samples.size(); ++i) if (samples[i].supported) {
        std::vector<double> values;
        for (size_t j : samples[i].neighbors) if (samples[j].supported)
            values.push_back(samples[j].density);
        densities[i] = areaWeightedDensity(values);
    }
    for (size_t i = 0; i < samples.size(); ++i) if (samples[i].supported) {
        samples[i].density = densities[i];
        // Repeated short spacings give direct evidence for a sharp scan edge.
        // With irregular sampling those spacings are unreliable: use the area
        // density instead of expanding accidental close pairs into voids.
        const double limit=samples[i].regularSpacing?settings.regularSpacingLimitMultiplier*samples[i].spacing:INFINITY;
        samples[i].spacing = std::max(samples[i].spacing,std::min(1/std::sqrt(densities[i]),limit));
        if(maximumSupportSpacing>0)samples[i].spacing=std::min(samples[i].spacing,maximumSupportSpacing);
    }
    std::vector<size_t> parents(samples.size());std::iota(parents.begin(), parents.end(), 0);
    const auto root = [&](size_t id) { while (parents[id]!=id) { parents[id]=parents[parents[id]];id=parents[id]; } return id; };
    for (size_t i = 0; i < samples.size(); ++i) if (samples[i].supported)
        for (size_t j : samples[i].neighbors) if (samples[j].supported) {
            // A density transition needs the scale from both sides. Using the
            // smaller spacing alone disconnects a coarse patch at a dense rim.
            const double localReach = settings.connectivityRadiusMultiplier/std::pow(samples[i].density*samples[j].density, .25);
            const double reach=maximumSupportSpacing>0?std::min(localReach,settings.connectivityRadiusMultiplier*maximumSupportSpacing):localReach;
            if (distance2(samples[i].position, samples[j].position)<=reach*reach) parents[root(j)]=root(i);
        }
    std::map<size_t, std::vector<size_t>> groups;
    for (size_t i = 0; i < samples.size(); ++i) if (samples[i].supported) groups[root(i)].push_back(i);
    std::vector<std::vector<size_t>> result;
    for (auto& group : groups) if (group.second.size()>=settings.minimumSurfacePoints) result.push_back(std::move(group.second));
    return result;
}

struct Support {
    const std::vector<Sample>& samples;
    const Locator& locator;
    const Options& settings;
    bool at(XY p, double spacing) const {
        const auto& nearest = samples[locator.closest(p)];
        const double reach = settings.coverageRadiusMultiplier*std::min(spacing, nearest.spacing);
        if (distance2(p, nearest.position)<=reach*reach) return true;
        // At a density transition, the closest fine sample must not hide the
        // equally close, genuinely coarser observation on the other side.
        for (size_t id : locator.nearest(p, std::min(settings.densityNeighborCount, samples.size()))) {
            const double localReach=settings.coverageRadiusMultiplier*std::min(spacing,samples[id].spacing);
            if (distance2(p,samples[id].position)<=localReach*localReach) return true;
        }
        return false;
    }
    bool triangle(const std::array<XY, 3>& points, double spacing) const {
        for (int edge = 0; edge < 3; ++edge) {
            const auto a = points[size_t(edge)], b = points[size_t((edge+1)%3)];
            if (!at({(a[0]+b[0])*.5,(a[1]+b[1])*.5},spacing)) return false;
            const int steps = int(std::clamp(std::ceil(std::sqrt(distance2(a,b))/(spacing*.75)), 2., 128.));
            for (int i = 1; i < steps; ++i) {
                const double t = double(i)/steps;
                if (!at({a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t}, spacing)) return false;
            }
        }
        for (const auto& weights : {std::array<double,3>{1./3,1./3,1./3}, { .6,.2,.2 }, { .2,.6,.2 }, { .2,.2,.6 }}) {
            XY p{};
            for (size_t i = 0; i < 3; ++i) { p[0]+=points[i][0]*weights[i];p[1]+=points[i][1]*weights[i]; }
            if (!at(p,spacing)) return false;
        }
        return true;
    }
};

// Density is measured at observations, then interpolated into a candidate
// void. Estimating it from the void's own kNN radius would make a larger hole
// look like a more sparsely sampled surface and erase the very signal we need.
class RelativeDensity {
public:
    RelativeDensity(const std::vector<Sample>& samples, const Locator& locator,const Options& settings)
        : m_samples(samples), m_locator(locator),m_settings(settings) {}

    bool missingAt(XY position) const {
        const auto needed=std::max(m_settings.densityNeighborCount,size_t(std::ceil(m_settings.expectedHolePoints)));
        const auto neighbors = m_locator.nearest(position, std::min(needed, m_samples.size()));
        std::vector<double> background;
        for (size_t id : neighbors) background.push_back(m_samples[id].density);
        const double density = areaWeightedDensity(background);
        // At both scales an ordinary sampling gap needs enough expected
        // observations before it can become evidence of a hole.
        const double radius2 = m_settings.expectedHolePoints/(pi*density);
        for (double scale : {m_settings.smallDensityScale, 1.}) {
            size_t observed = 0;
            for (size_t id : neighbors)
                observed += distance2(position, m_samples[id].position)<=radius2*scale*scale;
            // Query at least the expected window population so truncation
            // cannot manufacture a deficit when the configured window grows.
            if (double(observed)>=m_settings.holeDensityRatio*m_settings.expectedHolePoints*scale*scale) return false;
        }
        return true;
    }
private:
    const std::vector<Sample>& m_samples;
    const Locator& m_locator;
    const Options& m_settings;
};

bool kept(const DT& dt, DT::Face_handle face) { return !dt.is_infinite(face) && face->info().kept; }
std::array<XY,3> coordinates(DT::Face_handle face) {
    std::array<XY,3> points;
    for (size_t i = 0; i < 3; ++i) points[i]={face->vertex(int(i))->point().x(),face->vertex(int(i))->point().y()};
    return points;
}

bool stableSlot(const DT& dt,const std::vector<DT::Face_handle>& cavity,const std::vector<Sample>& samples,const Options& settings) {
    std::vector<std::pair<XY,XY>> edges;
    std::set<size_t> boundary;
    for(auto face:cavity)for(int side=0;side<3;++side)if(kept(dt,face->neighbor(side))) {
        const size_t a=face->vertex(dt.ccw(side))->info(),b=face->vertex(dt.cw(side))->info();
        edges.emplace_back(samples[a].position,samples[b].position);
        boundary.insert(a);boundary.insert(b);
    }
    if(boundary.size()<12)return false;
    XY center{};std::vector<double> densities;
    for(size_t id:boundary) {
        center[0]+=samples[id].position[0];center[1]+=samples[id].position[1];
        densities.push_back(samples[id].density);
    }
    center[0]/=boundary.size();center[1]/=boundary.size();
    double xx=0,xy=0,yy=0;
    for(size_t id:boundary) {
        const double x=samples[id].position[0]-center[0],y=samples[id].position[1]-center[1];
        xx+=x*x;xy+=x*y;yy+=y*y;
    }
    const double angle=.5*std::atan2(2*xy,xx-yy);
    const XY along{std::cos(angle),std::sin(angle)},across{-along[1],along[0]};
    XY low{INFINITY,INFINITY},high{-INFINITY,-INFINITY};
    for(size_t id:boundary)for(size_t axis=0;axis<2;++axis) {
        const auto direction=axis==0?along:across;
        const auto p=samples[id].position;
        const double value=(p[0]-center[0])*direction[0]+(p[1]-center[1])*direction[1];
        low[axis]=std::min(low[axis],value);high[axis]=std::max(high[axis],value);
    }
    const double spacing=1/std::sqrt(median(std::move(densities)));
    if(high[0]-low[0]<settings.slotMinimumLengthSpacings*spacing || high[1]-low[1]>settings.slotMaximumWidthSpacings*spacing)return false;
    double perimeter=0,positive=0,negative=0;
    for(const auto& edge:edges) {
        const double dx=edge.second[0]-edge.first[0],dy=edge.second[1]-edge.first[1];
        const double length=std::hypot(dx,dy);perimeter+=length;
        if(std::abs(dx*along[0]+dy*along[1])<settings.slotEdgeAlignment*length)continue;
        const XY middle{(edge.first[0]+edge.second[0])*.5-center[0],(edge.first[1]+edge.second[1])*.5-center[1]};
        if(middle[0]*across[0]+middle[1]*across[1]>0)positive+=length;else negative+=length;
    }
    // Long, consistently opposed rims are direct boundary evidence even when
    // a circular density window necessarily includes material across the slot.
    return positive>=settings.slotMinimumSideSpacings*spacing && negative>=settings.slotMinimumSideSpacings*spacing
        && positive+negative>=settings.slotPerimeterSupportRatio*perimeter;
}

void recoverDomain(DT& dt, const Support& support) {
    const auto& settings=support.settings;
    for (auto face = dt.finite_faces_begin(); face != dt.finite_faces_end(); ++face) {
        std::array<double,3> spacing;
        for (int i = 0; i < 3; ++i) spacing[size_t(i)]=support.samples[face->vertex(i)->info()].spacing;
        std::sort(spacing.begin(), spacing.end());
        const double radius = settings.planarAlphaRadiusMultiplier*spacing[2];
        face->info().kept = CGAL::squared_radius(face->vertex(0)->point(),face->vertex(1)->point(),face->vertex(2)->point())<=radius*radius
            && support.triangle(coordinates(face),spacing[2]);
    }
    const RelativeDensity density(support.samples, support.locator,settings);
    const double minimumHoleEvidence=settings.expectedHolePoints*settings.smallDensityScale*settings.smallDensityScale;
    // The strict footprint determines the exterior boundary, open notches and
    // separate islands. A bounded cavity is kept only when its relative loss of
    // observations is significant at multiple local sampling scales.
    for (auto start = dt.finite_faces_begin(); start != dt.finite_faces_end(); ++start) {
        if (start->info().kept || start->info().visited) continue;
        std::vector<DT::Face_handle> cavity{start};start->info().visited=true;
        bool bounded=true;double expected=0;
        for (size_t i = 0; i < cavity.size(); ++i) {
            const auto face=cavity[i];
            const double area=std::abs(CGAL::to_double(CGAL::area(face->vertex(0)->point(),face->vertex(1)->point(),face->vertex(2)->point())));
            std::vector<double> localDensity;
            for (int j=0;j<3;++j) {
                localDensity.push_back(support.samples[face->vertex(j)->info()].density);
                const auto neighbor=face->neighbor(j);
                if (dt.is_infinite(neighbor)) { bounded=false;continue; }
                if (!neighbor->info().kept && !neighbor->info().visited) { neighbor->info().visited=true;cavity.push_back(neighbor); }
            }
            expected+=area*median(std::move(localDensity));
        }
        if (!bounded) continue;
        bool missing=false;
        if (expected>=minimumHoleEvidence) for (auto face:cavity) {
            const auto p=coordinates(face);
            const XY center{(p[0][0]+p[1][0]+p[2][0])/3, (p[0][1]+p[1][1]+p[2][1])/3};
            if (density.missingAt(center)) { missing=true;break; }
        }
        if(!missing && expected>=minimumHoleEvidence)missing=stableSlot(dt,cavity,support.samples,settings);
        if (!missing) for (auto face:cavity) face->info().kept=true;
    }
    for (auto face=dt.finite_faces_begin();face!=dt.finite_faces_end();++face) face->info().visited=false;
    // A few triangles at a line/point junction are not an observed surface.
    for (auto start=dt.finite_faces_begin();start!=dt.finite_faces_end();++start) {
        if (!start->info().kept || start->info().visited) continue;
        std::vector<DT::Face_handle> faces{start};start->info().visited=true;std::set<size_t> vertices;
        for (size_t i=0;i<faces.size();++i) for (int j=0;j<3;++j) {
            vertices.insert(faces[i]->vertex(j)->info());const auto neighbor=faces[i]->neighbor(j);
            if (kept(dt,neighbor) && !neighbor->info().visited) { neighbor->info().visited=true;faces.push_back(neighbor); }
        }
        if (vertices.size()<settings.minimumSurfacePoints) for (auto face:faces) face->info().kept=false;
    }
}

std::vector<std::vector<size_t>> boundaryLoops(DT& dt) {
    std::vector<std::vector<size_t>> loops;
    for (auto face=dt.finite_faces_begin();face!=dt.finite_faces_end();++face) if (face->info().kept)
        for (int side=0;side<3;++side) {
            if (face->info().boundaryVisited[size_t(side)] || kept(dt,face->neighbor(side))) continue;
            const DT::Edge start{face,side};DT::Edge edge=start;std::vector<size_t> walk;
            do {
                auto current=edge.first;int index=edge.second;
                current->info().boundaryVisited[size_t(index)]=true;
                walk.push_back(current->vertex(dt.ccw(index))->info());
                index=dt.ccw(index);
                while (kept(dt,current->neighbor(index))) { auto next=current->neighbor(index);index=dt.ccw(next->index(current));current=next; }
                edge={current,index};
            } while (edge!=start);
            // Split vertex-touching cycles before introducing trim constraints.
            std::vector<size_t> path;std::map<size_t,size_t> positions;
            for (size_t i=0;i<=walk.size();++i) {
                const size_t vertex=walk[i%walk.size()];const auto inserted=positions.emplace(vertex,path.size());
                if (inserted.second) { path.push_back(vertex);continue; }
                const auto begin=inserted.first->second;
                if (path.size()-begin>=3) loops.emplace_back(path.begin()+begin,path.end());
                for (size_t j=begin+1;j<path.size();++j) positions.erase(path[j]);
                path.resize(begin+1);
            }
        }
    return loops;
}

// Segment distances measure actual geometric boundary bands. They must not
// reuse the density-expanded reach that admitted an isolated point upstream.
class BoundaryDistances {
    using Segment=Kernel::Segment_3;
    using Iterator=std::vector<Segment>::const_iterator;
    using Primitive=CGAL::AABB_segment_primitive_3<Kernel,Iterator>;
    using Tree=CGAL::AABB_tree<CGAL::AABB_traits_3<Kernel,Primitive>>;
    std::vector<Segment> segments;
    Tree tree;
public:
    explicit BoundaryDistances(const std::vector<std::pair<XY,XY>>& input) {
        for(const auto& s:input) if(s.first!=s.second)
            segments.emplace_back(Kernel::Point_3(s.first[0],s.first[1],0),Kernel::Point_3(s.second[0],s.second[1],0));
        tree.insert(segments.cbegin(),segments.cend());
        if(!segments.empty())tree.accelerate_distance_queries();
    }
    double squared(XY p) const {
        return segments.empty()?INFINITY:CGAL::to_double(tree.squared_distance(Kernel::Point_3(p[0],p[1],0)));
    }
};

std::vector<PlanarDomainSupportCheck> validateInteriorSupport(
    DT& dt,const std::vector<std::vector<size_t>>& loops,const std::vector<Sample>& samples,const Options& settings)
{
    using Edge=std::pair<size_t,size_t>;
    const auto key=[](size_t a,size_t b){return Edge{std::min(a,b),std::max(a,b)};};
    std::set<Edge> outerEdges;
    for(const auto& loop:loops) {
        CGAL::Polygon_2<Kernel> polygon;
        for(size_t id:loop)polygon.push_back(Point(samples[id].position[0],samples[id].position[1]));
        if(polygon.orientation()!=CGAL::COUNTERCLOCKWISE)continue;
        for(size_t i=0;i<loop.size();++i)outerEdges.insert(key(loop[i],loop[(i+1)%loop.size()]));
    }
    for(auto face=dt.finite_faces_begin();face!=dt.finite_faces_end();++face)face->info().visited=false;
    std::vector<PlanarDomainSupportCheck> checks;
    for(auto start=dt.finite_faces_begin();start!=dt.finite_faces_end();++start) {
        if(!start->info().kept || start->info().visited)continue;
        std::vector<DT::Face_handle> faces{start};start->info().visited=true;
        std::set<size_t> observations;
        std::vector<std::pair<XY,XY>> outside,holes;
        for(size_t i=0;i<faces.size();++i)for(int side=0;side<3;++side) {
            const auto face=faces[i];observations.insert(face->vertex(side)->info());
            const auto neighbor=face->neighbor(side);
            if(kept(dt,neighbor)) {
                if(!neighbor->info().visited){neighbor->info().visited=true;faces.push_back(neighbor);}
                continue;
            }
            const size_t a=face->vertex(dt.ccw(side))->info(),b=face->vertex(dt.cw(side))->info();
            (outerEdges.count(key(a,b))?outside:holes).emplace_back(samples[a].position,samples[b].position);
        }
        if(outside.empty())continue;
        PlanarDomainSupportCheck check;check.observedPoints=observations.size();
        std::vector<double> spacings;
        for(size_t id:observations)spacings.push_back(samples[id].observedSpacing);
        const double spacing=median(std::move(spacings));
        check.boundaryBandWidth=spacing*settings.boundaryBandSpacingMultiplier;
        const double band2=check.boundaryBandWidth*check.boundaryBandWidth;
        BoundaryDistances outer(outside),inner(holes);
        std::set<std::pair<long long,long long>> occupied;
        const XY origin=samples[*observations.begin()].position;
        for(size_t id:observations) {
            const auto p=samples[id].position;
            if(outer.squared(p)<=band2)++check.outerBoundaryPoints;
            else if(inner.squared(p)<=band2)++check.holeBoundaryPoints;
            else {
                ++check.interiorPoints;
                occupied.emplace(static_cast<long long>(std::floor((p[0]-origin[0])/spacing)),
                    static_cast<long long>(std::floor((p[1]-origin[1])/spacing)));
            }
        }
        check.occupiedInteriorCells=occupied.size();
        const bool boundaryDominates=check.outerBoundaryPoints>=settings.boundaryDominanceRatio*check.observedPoints;
        double deepArea=0;
        for(auto face:faces) {
            const double area=std::abs(CGAL::to_double(CGAL::area(face->vertex(0)->point(),face->vertex(1)->point(),face->vertex(2)->point())));
            check.area+=area;
            if(!boundaryDominates)continue;
            const auto p=coordinates(face);
            for(const auto& weights:{std::array<double,3>{1./3,1./3,1./3},{.6,.2,.2},{.2,.6,.2},{.2,.2,.6}}) {
                XY position{};
                for(size_t j=0;j<3;++j){position[0]+=weights[j]*p[j][0];position[1]+=weights[j]*p[j][1];}
                const double distance=std::min(outer.squared(position),inner.squared(position));
                if(distance>band2)check.interiorArea+=area*.25;
                if(distance>std::pow(settings.deepInteriorDistanceMultiplier*spacing,2))deepArea+=area*.25;
            }
        }
        check.interiorAreaMeasured=boundaryDominates;
        check.expectedInteriorPoints=check.interiorArea/(spacing*spacing);
        // All-boundary observations are legitimate for a narrow strip or a
        // small, low-resolution patch. Reject only a resolved interior whose
        // independent observed cells cannot substantiate the proposed area.
        check.rejected=boundaryDominates && deepArea>=settings.minimumDeepInteriorAreaSpacings*spacing*spacing
            && check.expectedInteriorPoints>=settings.minimumExpectedInteriorPoints
            && check.occupiedInteriorCells<settings.minimumInteriorSupportRatio*check.expectedInteriorPoints;
        if(check.rejected)for(auto face:faces)face->info().kept=false;
        checks.push_back(check);
    }
    return checks;
}

void markDomains(CDT& cdt) {
    std::vector<CDT::Edge> borders;
    const auto flood=[&](CDT::Face_handle start,int level) {
        std::deque<CDT::Face_handle> queue{start};
        while (!queue.empty()) {
            auto face=queue.front();queue.pop_front();if (face->info().nesting!=-1) continue;
            face->info().nesting=level;
            for (int i=0;i<3;++i) if (face->neighbor(i)->info().nesting==-1) {
                if (cdt.is_constrained({face,i})) borders.emplace_back(face,i);
                else queue.push_back(face->neighbor(i));
            }
        }
    };
    flood(cdt.infinite_face(),0);
    for (size_t i=0;i<borders.size();++i) {
        const auto edge=borders[i];const auto next=edge.first->neighbor(edge.second);
        if (next->info().nesting==-1) flood(next,edge.first->info().nesting+1);
    }
}

SurfaceFootprint triangulateDomain(const std::vector<Sample>& samples,const Options& settings) {
    SurfaceFootprint output;Locator locator(samples);Support support{samples,locator,settings};
    std::vector<std::pair<Point,size_t>> input;
    for (size_t i=0;i<samples.size();++i) input.emplace_back(Point(samples[i].position[0],samples[i].position[1]),i);
    DT dt;dt.insert(input.begin(),input.end());if (dt.dimension()!=2) return output;
    recoverDomain(dt,support);
    auto loops=boundaryLoops(dt);if (loops.empty()) return output;
    output.domainSupportChecks=validateInteriorSupport(dt,loops,samples,settings);
    if(std::any_of(output.domainSupportChecks.begin(),output.domainSupportChecks.end(),[](const auto& check){return check.rejected;})) {
        for(auto face=dt.finite_faces_begin();face!=dt.finite_faces_end();++face)face->info().boundaryVisited={};
        loops=boundaryLoops(dt);
        if(loops.empty())return output;
    }
    CDT cdt;
    for (const auto& loop:loops) {
        CGAL::Polygon_2<Kernel> polygon;
        for (size_t id:loop) polygon.push_back(input[id].first);
        if (!polygon.is_simple() || polygon.orientation()==CGAL::COLLINEAR) return {};
        if (polygon.orientation()==CGAL::CLOCKWISE) {
            PlanarFootprintHole hole;
            hole.area=std::abs(CGAL::to_double(polygon.area()));
            std::vector<double> spacings;
            for (size_t id:loop) {
                hole.boundary.push_back(samples[id].position);
                spacings.push_back(samples[id].observedSpacing);
            }
            hole.spacing=median(std::move(spacings));
            output.holes.push_back(std::move(hole));
        }
        std::vector<CDT::Vertex_handle> vertices;
        for (size_t id:loop) { auto vertex=cdt.insert(input[id].first);vertex->info()=id;vertices.push_back(vertex); }
        for (size_t i=0;i<vertices.size();++i) cdt.insert_constraint(vertices[i],vertices[(i+1)%vertices.size()]);
    }
    // Trim boundaries always retain their observed vertices. Bound only the
    // interior refinement budget; thinning cannot close an observed opening.
    const size_t stride=std::max<size_t>(1,(samples.size()+settings.meshPointBudget-1)/settings.meshPointBudget);
    for (size_t i=0;i<samples.size();i+=stride) { auto vertex=cdt.insert(input[i].first);vertex->info()=i; }
    markDomains(cdt);
    std::map<size_t,std::uint64_t> remap;
    for (auto face=cdt.finite_faces_begin();face!=cdt.finite_faces_end();++face) {
        if (face->info().nesting%2!=1) continue;
        // Boundaries already encode the density decision. Rechecking coverage
        // per output triangle would punch the same sampling gaps open again.
        face->info().kept=true;std::array<std::uint64_t,3> triangle;
        for (int i=0;i<3;++i) {
            const size_t id=face->vertex(i)->info();const auto inserted=remap.emplace(id,std::uint64_t(output.positions.size()));
            if (inserted.second) { output.positions.push_back({samples[id].position[0],samples[id].position[1],0});output.sourcePointIds.push_back(samples[id].sources.front()); }
            triangle[size_t(i)]=inserted.first->second;
        }
        output.triangles.push_back(triangle);
    }
    // Keep unsupported line tails and isolated samples visible even when a
    // different region of this same collection has a successfully rendered face.
    for (size_t i=0;i<samples.size();++i) {
        CDT::Locate_type type;int index=0;const auto face=cdt.locate(input[i].first,type,index);bool covered=face->info().kept;
        if (!covered && type==CDT::EDGE) covered=face->neighbor(index)->info().kept;
        if (!covered && type==CDT::VERTEX) {
            auto adjacent=cdt.incident_faces(face->vertex(index)),end=adjacent;
            if (adjacent!=0) do { covered=covered || adjacent->info().kept; } while (++adjacent!=end);
        }
        if (covered) output.coveredPointIds.insert(output.coveredPointIds.end(),samples[i].sources.begin(),samples[i].sources.end());
    }
    return output;
}
}

SurfaceFootprint buildPlanarSurfaceFootprint(const std::vector<PlanarSurfaceSample>& input,const Options& settings,double maximumSupportSpacing)
{
    SurfaceFootprint output;if (input.size()<settings.minimumSurfacePoints) return output;
    auto samples=distinctSamples(input);if (samples.size()<settings.minimumSurfacePoints) return output;
    const auto components=supportedComponents(samples,settings,maximumSupportSpacing);
    for (const auto& component:components) {
        std::vector<Sample> local;local.reserve(component.size());
        for (size_t id:component) local.push_back(samples[id]);
        auto mesh=triangulateDomain(local,settings);const auto offset=std::uint64_t(output.positions.size());
        output.domainSupportChecks.insert(output.domainSupportChecks.end(),mesh.domainSupportChecks.begin(),mesh.domainSupportChecks.end());
        for(auto& hole:mesh.holes)output.holes.push_back(std::move(hole));
        output.positions.insert(output.positions.end(),mesh.positions.begin(),mesh.positions.end());
        output.sourcePointIds.insert(output.sourcePointIds.end(),mesh.sourcePointIds.begin(),mesh.sourcePointIds.end());
        output.coveredPointIds.insert(output.coveredPointIds.end(),mesh.coveredPointIds.begin(),mesh.coveredPointIds.end());
        for (auto triangle:mesh.triangles) { for (auto& id:triangle) id+=offset;output.triangles.push_back(triangle); }
    }
    if(!output.triangles.empty()) {
        // Local support selects the domain; membership is geometric. A noisy
        // observation inside that domain must not disappear merely because its
        // own neighbourhood was excluded from triangulation.
        const SurfaceFootprintQuery query(output);
        output.coveredPointIds.clear();
        for(const auto& sample:samples)if(query.containsProjection({sample.position[0],sample.position[1],0})) {
            output.coveredPointIds.insert(output.coveredPointIds.end(),sample.sources.begin(),sample.sources.end());
            ++output.effectiveCoveredPoints;
        }
        std::sort(output.coveredPointIds.begin(),output.coveredPointIds.end());
        output.coveredPointIds.erase(std::unique(output.coveredPointIds.begin(),output.coveredPointIds.end()),output.coveredPointIds.end());
    }
    return output;
}

} // namespace reconstruction

namespace reconstruction {
struct SurfaceFootprintQuery::Impl {
    using Triangle=Kernel::Triangle_3;
    using Iterator=std::vector<Triangle>::const_iterator;
    using Primitive=CGAL::AABB_triangle_primitive_3<Kernel,Iterator>;
    using Tree=CGAL::AABB_tree<CGAL::AABB_traits_3<Kernel,Primitive>>;
    std::vector<Triangle> triangles;
    std::vector<PointId> anchors;
    Tree tree;
    modeling::Vec3 origin,u,v;
    double toleranceSquared=0;
    Impl(const SurfaceFootprint& domain,double tolerance):origin(domain.origin),u(domain.axisU),v(domain.axisV),toleranceSquared(tolerance*tolerance) {
        for(auto t:domain.triangles) {
            const auto a=domain.positions.at(t[0]),b=domain.positions.at(t[1]),c=domain.positions.at(t[2]);
            Triangle triangle(Kernel::Point_3(a[0],a[1],0),Kernel::Point_3(b[0],b[1],0),Kernel::Point_3(c[0],c[1],0));
            if(!triangle.is_degenerate()){triangles.push_back(triangle);anchors.push_back(domain.sourcePointIds.at(t[0]));}
        }
        tree.insert(triangles.cbegin(),triangles.cend());if(!triangles.empty())tree.accelerate_distance_queries();
    }
};
SurfaceFootprintQuery::SurfaceFootprintQuery(const SurfaceFootprint& domain,double tolerance):m_impl(std::make_unique<Impl>(domain,tolerance)){}
SurfaceFootprintQuery::~SurfaceFootprintQuery()=default;
bool SurfaceFootprintQuery::containsProjection(modeling::Vec3 point) const {
    if(m_impl->triangles.empty())return false;
    const auto q=math3::subtract(point,m_impl->origin);
    const Kernel::Point_3 uv(math3::dot(q,m_impl->u),math3::dot(q,m_impl->v),0);
    return CGAL::to_double(m_impl->tree.squared_distance(uv))<=m_impl->toleranceSquared;
}
std::optional<PointId> SurfaceFootprintQuery::anchorAtProjection(modeling::Vec3 point) const {
    if(m_impl->triangles.empty())return {};
    const auto q=math3::subtract(point,m_impl->origin);
    const Kernel::Point_3 uv(math3::dot(q,m_impl->u),math3::dot(q,m_impl->v),0);
    const auto nearest=m_impl->tree.closest_point_and_primitive(uv);
    if(CGAL::to_double(CGAL::squared_distance(uv,nearest.first))>m_impl->toleranceSquared)return {};
    return m_impl->anchors.at(std::size_t(std::distance(m_impl->triangles.cbegin(),nearest.second)));
}
} // namespace reconstruction
