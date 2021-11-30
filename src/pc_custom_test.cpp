#include "pc_custom_test/pc_custom_test.h"

PCCustomTest::PCCustomTest() {
}

// Remove parts of the point cloud by removing points that are not in the assigned limits.
void PCCustomTest::passthroughFilter(PointCloud::Ptr &input_point_cloud, PointCloud::Ptr &output_point_cloud, const Filter &filter) {
  passthrough_.setInputCloud(input_point_cloud);
  passthrough_.setFilterX(filter.min_x, filter.max_x);
  passthrough_.setFilterY(filter.min_y, filter.max_y);
  passthrough_.setFilterZ(filter.min_z, filter.max_z);
  passthrough_.filter(output_point_cloud);
}

// Remove the largest flat surface.
void PCCustomTest::segment(const PointCloud::Ptr &input_point_cloud, PointCloud::Ptr &output_point_cloud, const Filter &filter) { 
  segmentation_.setMaxIterations(100);
  segmentation_.setSurfaceThreshold(filter.surface_threshold);
  segmentation_.segment(input_point_cloud, output_point_cloud);
}

// Get clusters from the input point cloud.
void PCCustomTest::getClusters(const PointCloud::Ptr &input_point_cloud, std::vector<PointCloud::Ptr> &clusters, const Filter &filter) {
  pcl::search::KdTree<Point>::Ptr kdtree(new pcl::search::KdTree<Point>());
  kdtree->setInputCloud(input_point_cloud);
  clustering_.setSearchMethod(kdtree);
  clustering_.setMinClusterSize(filter.min_cluster_size);
  clustering_.setMaxClusterSize(filter.max_cluster_size);
  clustering_.setClusteringThreshold(filter.cluster_threshold);
  clustering_.extract(input_point_cloud, clusters);
}