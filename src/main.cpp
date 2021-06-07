#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pcl_test/pcl_test.h"
#include "pcl_custom_test/pcl_custom_test.h"
#include "filter/filter.h"

int main(int argc, char **argv) {
  PointCloud::Ptr point_cloud(new PointCloud());
  pcl::io::loadPCDFile("../data/region_growing_rgb_tutorial.pcd", *point_cloud);
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");

  Filter filter;
  filter.min_cluster_size = 500; // Min cluster size.
  filter.max_cluster_size = 10000000; // Max cluster size.
  filter.min_x = -0.5f; // Min filter x value in meters.
  filter.max_x = 0.5f; // Max filter x value in meters.
  filter.min_y = -0.5f; // Min filter y value in meters.
  filter.max_y = 0.5f; // Max filter y value in meters.
  filter.min_z = 0.0f; // Min filter z value in meters.
  filter.max_z = 1.5f; // Max filter z value in meters.
  filter.surface_threshold = 0.015f; // Threshold for when a point belongs to the plane, in meters.
  filter.cluster_threshold = 0.05f; // Threshold for when a points belongs to a cluster, in meters.

  std::vector<PointCloud::Ptr> pcl_clusters;
  PointCloud::Ptr pcl_point_cloud(new PointCloud());
  *pcl_point_cloud = *point_cloud;

  PCLTest pcl_test;
  PointCloud::Ptr passthrough_point_cloud(new PointCloud());
  pcl_test.passthrough_filter(pcl_point_cloud, passthrough_point_cloud, filter);

  PointCloud::Ptr segmented_point_cloud(new PointCloud());
  pcl_test.segment(passthrough_point_cloud, segmented_point_cloud, filter);
  pcl_test.get_clusters(segmented_point_cloud, pcl_clusters, filter);

  std::cout << "PCL Clusters: " << pcl_clusters.size() << "\n";

  std::vector<PointCloud::Ptr> pcl_custom_clusters;
  PointCloud::Ptr pcl_custom_point_cloud(new PointCloud());
  *pcl_custom_point_cloud = *point_cloud;

  PCLCustomTest pcl_custom_test;
  PointCloud::Ptr passthrough_custom_point_cloud(new PointCloud());
  pcl_custom_test.passthrough_filter(pcl_custom_point_cloud, passthrough_custom_point_cloud, filter);

  PointCloud::Ptr segmented_custom_point_cloud(new PointCloud());
  pcl_custom_test.segment(passthrough_custom_point_cloud, segmented_custom_point_cloud, filter);
  
  pcl_custom_test.get_clusters(segmented_custom_point_cloud, pcl_custom_clusters, filter);

  std::cout << "PCL Custom Clusters: " << pcl_custom_clusters.size() << "\n";

  viewer.addCoordinateSystem();
  viewer.addPointCloud(pcl_custom_clusters.at(0), "pcl_cloud");

  while (!viewer.wasStopped()) {
    viewer.spinOnce(1000);
  }
}