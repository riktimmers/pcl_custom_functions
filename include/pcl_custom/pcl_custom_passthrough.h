#ifndef _H_PCL_CUSTOM_PASSTHROUGH__
#define _H_PCL_CUSTOM_PASSTHROUGH__

#include <pcl/point_cloud.h>

template <typename PointT>
class PCLCustomPassthrough {
  bool filter_x_;
  bool filter_y_;
  bool filter_z_;
  float min_x_, max_x_;
  float min_y_, max_y_;
  float min_z_, max_z_;
  
  pcl::PointCloud<PointT>::Ptr input_point_cloud_;

public:
  PCLCustomPassthrough() {
  }

  void setInputCloud(pcl::PointCloud<PointT>::Ptr &input_point_cloud) {
    input_point_cloud_ = input_point_cloud;
  }

  void setFilterX(float min_x, float max_x) {
    filter_x_ = true;
    min_x_ = min_x;
    max_x_ = max_x;
  }

  void setFilterY(float min_y, float max_y) {
    filter_y_ = true;
    min_y_ = min_y;
    max_y_ = max_y;
  }

  void setFilterZ(float min_z, float max_z) {
    filter_z_ = true;
    min_z_ = min_z;
    max_z_ = max_z;
  }

  void filter(pcl::PointCloud<PointT>::Ptr &output_point_cloud) {
    output_point_cloud->points.clear();
    output_point_cloud->points.reserve(input_point_cloud_->points.size());
    output_point_cloud->sensor_orientation_ = input_point_cloud_->sensor_orientation_;

    for (PointT &point: input_point_cloud_->points) {
      
      if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
        continue;
      }

      if (filter_x_ && (point.x < min_x_ || point.x > max_x_)) {
        continue;
      }

      if (filter_y_ && (point.y < min_y_ || point.y > max_y_)) {
        continue;
      }

      if (filter_z_ && (point.z < min_z_ || point.z > max_z_)) {
        continue;
      }

      output_point_cloud->points.push_back(point);
    }
  }
};

#endif 