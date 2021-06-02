#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pcl_test/pcl_test.h"
#include "pcl_custom_test/pcl_custom_test.h"
#include "filter/filter.h"
#include "custom_timer/custom_timer.h"

int main(int argc, char **argv) {
  PointCloud::Ptr point_cloud(new PointCloud());
  pcl::io::loadPCDFile("../data/region_growing_rgb_tutorial.pcd", *point_cloud);
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");

  Filter filter;
  filter.min_cluster_size = 500;
  filter.max_cluster_size = 10000000;
  filter.min_x = -0.5f;
  filter.max_x = 0.5f;
  filter.min_y = -0.5f;
  filter.max_y = 0.5f;
  filter.min_z = 0.0f;
  filter.max_z = 1.5f;
  filter.surface_threshold = 0.015f;
  filter.cluster_threshold = 0.05f;

  std::vector<PointCloud::Ptr> pcl_clusters;
  {Timer timer("Pcl total", "milliseconds"); 
  PointCloud::Ptr pcl_point_cloud(new PointCloud());
  *pcl_point_cloud = *point_cloud;

  PCLTest pcl_test;
  PointCloud::Ptr passthrough_point_cloud(new PointCloud());
  {Timer timer("Passthrough", "milliseconds"); 
  pcl_test.passthrough_filter(pcl_point_cloud, passthrough_point_cloud, filter);
  } // timer

  PointCloud::Ptr segmented_point_cloud(new PointCloud());
  {Timer timer("Segmentation", "milliseconds"); 
  pcl_test.segment(passthrough_point_cloud, segmented_point_cloud, filter);
  } // timer 
  {Timer timer("Clustering", "milliseconds"); 
  pcl_test.get_clusters(segmented_point_cloud, pcl_clusters, filter);
  } // timer
  } // timer total

  std::cout << "Clusters: " << pcl_clusters.size() << "\n";

  std::vector<PointCloud::Ptr> pcl_custom_clusters;
  {Timer timer("Pcl custom total", "milliseconds"); 
  PointCloud::Ptr pcl_custom_point_cloud(new PointCloud());
  *pcl_custom_point_cloud = *point_cloud;

  PCLCustomTest pcl_custom_test;
  PointCloud::Ptr passthrough_custom_point_cloud(new PointCloud());
  {Timer timer("Custom passthrough", "milliseconds"); 
  pcl_custom_test.passthrough_filter(pcl_custom_point_cloud, passthrough_custom_point_cloud, filter);
  } // timer 

  PointCloud::Ptr segmented_custom_point_cloud(new PointCloud());
  {Timer timer("Custom segmentation", "milliseconds"); 
  pcl_custom_test.segment(passthrough_custom_point_cloud, segmented_custom_point_cloud, filter);
  } // timer
  
  {Timer timer("Custom clustering", "milliseconds"); 
  pcl_custom_test.get_clusters(segmented_custom_point_cloud, pcl_custom_clusters, filter);
  } // timer
  } // timer total

  std::cout << "Clusters: " << pcl_custom_clusters.size() << "\n";

  viewer.addCoordinateSystem();
  viewer.addPointCloud(pcl_custom_clusters.at(0), "pcl_cloud");

  while (!viewer.wasStopped()) {
    viewer.spinOnce(1000);
  }
  //*/
}