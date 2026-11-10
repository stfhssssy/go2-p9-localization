// Regression for obstacle-before-free-terrain startup. Works without ROS.
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <far_planner/terrain_kdtree.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
int main() {
    using Tree = pcl::KdTreeFLANN<pcl::PointXYZI>;
    std::shared_ptr<Tree> tree;
    pcl::PointXYZI query{};
    std::vector<int> indices{99};
    std::vector<float> distances{99};
    auto empty_result = [&]() {
        return far_planner_terrain::radiusSearch(tree, query, .3, indices, distances)==0 && indices.empty() && distances.empty()
            && far_planner_terrain::nearestKSearch(tree, query, 1, indices, distances)==0 && indices.empty() && distances.empty();
    };
    if (!empty_result()) return 1;
    tree=std::make_shared<Tree>();tree->setSortedResults(false);
    if (!empty_result()) return 2; // Previously a deterministic SIGSEGV in PCL 1.14.
    auto cloud=std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    tree->setInputCloud(cloud);
    if (!empty_result()) return 3;
    pcl::PointXYZI terrain{};terrain.x=.1f;terrain.intensity=.25f;cloud->push_back(terrain);
    tree->setInputCloud(cloud);
    if (far_planner_terrain::radiusSearch(tree,query,.3,indices,distances)!=1 || indices[0]!=0 || std::abs(distances[0]-.01f)>1e-6) return 4;
    if (far_planner_terrain::nearestKSearch(tree,query,1,indices,distances)!=1 || indices[0]!=0) return 5;
    // Clearing the input after a good scan must again be a no-match.
    cloud=std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();tree->setInputCloud(cloud);
    if (!empty_result()) return 6;
    std::cout << "terrain KD-tree startup/reset regression: PASS\n";
}
