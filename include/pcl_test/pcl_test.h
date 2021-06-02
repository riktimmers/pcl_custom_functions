#ifndef _H_PCL_TEST__
#define _H_PCL_TEST__

#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include "filter/filter.h"

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class PCLTest {

  pcl::PassThrough<Point> passthrough_;
  pcl::SACSegmentation<Point> segmentation_;
  pcl::ExtractIndices<Point> extract_;

public:
  PCLTest();

  void passthrough_filter(const PointCloud::Ptr &input_point_cloud, PointCloud::Ptr &output_point_cloud, const Filter &filter);
  void segment(const PointCloud::Ptr &input_point_cloud, PointCloud::Ptr &output_point_cloud, const Filter &filter);
  void get_clusters(const PointCloud::Ptr &input_point_cloud, std::vector<PointCloud::Ptr> &clusters, const Filter &filter); 

};
#endif