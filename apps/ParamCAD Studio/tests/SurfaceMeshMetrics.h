#pragma once

#include "rendering/PointCloudSurfaceMesh.h"
#include <algorithm>
#include <map>
#include <numeric>
#include <set>

// Count connected triangle patches by shared edges, not vertex touches.
inline size_t surfaceMeshComponentCount(const PointCloudSurfaceMesh& mesh)
{
    std::vector<size_t> parents(mesh.triangles.size());
    std::iota(parents.begin(),parents.end(),0);
    const auto root=[&](size_t id) {
        while(parents[id]!=id) {parents[id]=parents[parents[id]];id=parents[id];}
        return id;
    };
    std::map<std::pair<vtkIdType,vtkIdType>,size_t> edges;
    for(size_t i=0;i<mesh.triangles.size();++i)for(size_t side=0;side<3;++side) {
        const auto a=mesh.triangles[i][side],b=mesh.triangles[i][(side+1)%3];
        const auto inserted=edges.emplace(std::make_pair(std::min(a,b),std::max(a,b)),i);
        if(!inserted.second)parents[root(i)]=root(inserted.first->second);
    }
    std::set<size_t> components;
    for(size_t i=0;i<parents.size();++i)components.insert(root(i));
    return components.size();
}
