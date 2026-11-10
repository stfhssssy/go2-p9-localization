#ifndef FAR_PLANNER_TERRAIN_KDTREE_H
#define FAR_PLANNER_TERRAIN_KDTREE_H

#include <vector>

namespace far_planner_terrain {
// PCL's FLANN query dereferences an unbuilt index. During startup/reset the
// obstacle cloud may exist before any traversable terrain has been observed.
// No index is an ordinary "no terrain match", not a synthetic ground sample.
template <typename TreePtr, typename Point>
int radiusSearch(const TreePtr& tree, const Point& point, double radius,
                 std::vector<int>& indices, std::vector<float>& squared_distances) {
    indices.clear(); squared_distances.clear();
    if (!tree || !tree->getInputCloud() || tree->getInputCloud()->empty()) return 0;
    return tree->radiusSearch(point, radius, indices, squared_distances);
}

template <typename TreePtr, typename Point>
int nearestKSearch(const TreePtr& tree, const Point& point, int count,
                   std::vector<int>& indices, std::vector<float>& squared_distances) {
    indices.clear(); squared_distances.clear();
    if (!tree || !tree->getInputCloud() || tree->getInputCloud()->empty()) return 0;
    return tree->nearestKSearch(point, count, indices, squared_distances);
}
}  // namespace far_planner_terrain
#endif
