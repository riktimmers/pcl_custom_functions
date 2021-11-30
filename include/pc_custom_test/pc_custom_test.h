#ifndef _H_PC_CUSTOM_TEST__
#define _H_PC_CUSTOM_TEST__

#include <pcl/point_cloud.h>
#include <math.h>
#include "filter/filter.h"
#include "pc_custom/pc_custom_passthrough.h"
#include "pc_custom/pc_custom_clustering.h"
#include "pc_custom/pc_custom_segmentation.h"

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class PCCustomTest {

  PCCustomPassthrough<Point> passthrough_;
  PCCustomClustering<Point> clustering_; 
  PCCustomSegmentation<Point> segmentation_;

public:
  PCCustomTest();

  void passthroughFilter(PointCloud::Ptr &input_point_cloud, PointCloud::Ptr &output_point_cloud, const Filter &filter);
  void segment(const PointCloud::Ptr &input_point_cloud, PointCloud::Ptr &output_point_cloud, const Filter &filter);
  void getClusters(const PointCloud::Ptr &input_point_cloud, std::vector<PointCloud::Ptr> &clusters, const Filter &filter); 
};

#endif