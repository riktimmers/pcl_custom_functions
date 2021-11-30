#include "pcl_test/pcl_test.h"

PCLTest::PCLTest() {
};

void PCLTest::passthroughFilter(const PointCloud::Ptr &input_point_cloud, PointCloud::Ptr &output_point_cloud, const Filter &filter) {
  passthrough_.setInputCloud(input_point_cloud);
  passthrough_.setFilterFieldName("x");
  passthrough_.setFilterLimits(filter.min_x, filter.max_x);
  passthrough_.filter(*output_point_cloud);

  passthrough_.setInputCloud(output_point_cloud);
  passthrough_.setFilterFieldName("y");
  passthrough_.setFilterLimits(filter.min_y, filter.max_y);
  passthrough_.filter(*output_point_cloud);

  passthrough_.setInputCloud(output_point_cloud);
  passthrough_.setFilterFieldName("z");
  passthrough_.setFilterLimits(filter.min_z, filter.max_z);
  passthrough_.filter(*output_point_cloud);
}

void PCLTest::segment(const PointCloud::Ptr &input_point_cloud, PointCloud::Ptr &output_point_cloud, const Filter &filter) {
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  segmentation_.setInputCloud(input_point_cloud);
  segmentation_.setModelType(pcl::SacModel::SACMODEL_PLANE);
  segmentation_.setOptimizeCoefficients(true);
  segmentation_.setMaxIterations(100);
  segmentation_.setMethodType(pcl::SAC_RANSAC);
  segmentation_.setDistanceThreshold(filter.surface_threshold);
  segmentation_.segment(*inliers, *coefficients);
 
  extract_.setInputCloud(input_point_cloud);
  extract_.setIndices(inliers);
  extract_.setNegative(true);
  extract_.filter(*output_point_cloud);
}

void PCLTest::getClusters(const PointCloud::Ptr &input_point_cloud, std::vector<PointCloud::Ptr> &clusters, const Filter &filter) {
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::search::KdTree<Point>::Ptr kdtree(new pcl::search::KdTree<Point>());
  kdtree->setInputCloud(input_point_cloud);
  pcl::EuclideanClusterExtraction<Point> ece; 
  ece.setInputCloud(input_point_cloud);
  ece.setMinClusterSize(filter.min_cluster_size);
  ece.setMaxClusterSize(filter.max_cluster_size);
  ece.setSearchMethod(kdtree);
  ece.setClusterTolerance(filter.cluster_threshold);
  ece.extract(cluster_indices);

  clusters.reserve(cluster_indices.size());

  for(auto &cluster: cluster_indices) {
    PointCloud::Ptr cluster_cloud(new PointCloud());
    cluster_cloud->points.reserve(cluster.indices.size());
    
    for (const auto &index: cluster.indices) {
      cluster_cloud->points.push_back(input_point_cloud->points.at(index));
    }

    clusters.push_back(cluster_cloud);
  }
}