#ifndef _H_PCL_CUSTOM_TEST__
#define _H_PCL_CUSTOM_TEST__

#include <pcl/point_cloud.h>
#include <math.h>
#include "filter/filter.h"
#include "pcl_custom/pcl_custom_passthrough.h"
#include "pcl_custom/pcl_custom_clustering.h"
#include "pcl_custom/pcl_custom_segmentation.h"

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class PCLCustomTest {

  PCLCustomPassthrough<Point> passthrough_;
  PCLCustomClustering<Point> clustering_; 
  PCLCustomSegmentation<Point> segmentation_;

public:
  PCLCustomTest();

  void passthrough_filter(PointCloud::Ptr &input_point_cloud, PointCloud::Ptr &output_point_cloud, const Filter &filter);
  void segment(const PointCloud::Ptr &input_point_cloud, PointCloud::Ptr &output_point_cloud, const Filter &filter);
  void get_clusters(const PointCloud::Ptr &input_point_cloud, std::vector<PointCloud::Ptr> &clusters, const Filter &filter); 


};

#endif