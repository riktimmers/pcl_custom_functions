#ifndef _H_PCL_CUSTOM_SEGMENTATION__
#define _H_PCL_CUSTOM_SEGMENTATION__

#include <pcl/point_cloud.h>
#include <random>
#include <chrono>
#include <Eigen/Dense>
#include <vector>

struct Coeff {
  float A;
  float B;
  float C;
  float D; 
};

template <typename PointT>
class PCLCustomSegmentation {

  const pcl::PointCloud<PointT>::Ptr input_point_cloud_;
  size_t max_iterations_;
  float surface_threshold_{0.0};
  std::default_random_engine generator_;

public:
  PCLCustomSegmentation() : input_point_cloud_(new pcl::PointCloud<PointT>()), 
                            max_iterations_(100),
                            generator_(std::chrono::system_clock::now().time_since_epoch().count()) {
  }

  void setInputCloud(const pcl::PointCloud<PointT>::Ptr &point_cloud) {
    *input_point_cloud_ = *point_cloud;
  }

  void setMaxIterations(size_t max_iterations) {
    max_iterations_  = max_iterations;
  }

  void setSurfaceThreshold(float surface_treshold) {
    surface_threshold_ = surface_treshold;
  }

  void segment(pcl::PointCloud<PointT>::Ptr &output_point_cloud) {
    std::uniform_int_distribution<size_t> distribution(0, input_point_cloud_->points.size() - 1);
    std::uniform_int_distribution<size_t> index_distribution(0, 9);
    size_t max_size = 0;
    std::vector<size_t> cloud_indices; 

    float Ad = 0.0f;
    float Bd = 0.0f;
    float Cd = 0.0f;
    float Dd = 0.0f;
    
    std::vector<size_t> indices;
    for (size_t iterations = 0; iterations < max_iterations_; ++iterations) {
      indices.clear();
      indices.reserve(input_point_cloud_->points.size());

      size_t index1 = distribution(generator_);
      size_t index2 = distribution(generator_);
      size_t index3 = distribution(generator_);      


      PointT &point1 = input_point_cloud_->points.at(index1);
      PointT &point2 = input_point_cloud_->points.at(index2);
      PointT &point3 = input_point_cloud_->points.at(index3);
      
      const Eigen::Vector3f PQ(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z);
      const Eigen::Vector3f PR(point3.x - point1.x, point3.y - point1.y, point3.z - point1.z);

      const Eigen::Vector3f cross = PQ.cross(PR);
      const float A = cross[0];
      const float B = cross[1];
      const float C = cross[2];

      const float D = -(A * point1.x + B * point1.y + C * point1.z);
      size_t count_in = 0; 
      
      for (size_t index = index_distribution(generator_); index < input_point_cloud_->points.size(); index += 10) {
        PointT &point = input_point_cloud_->points.at(index);

        const float distance = std::abs(A*point.x + B*point.y + C*point.z + D) / std::sqrt(std::pow(A, 2) + std::pow(B, 2) + std::pow(C, 2));

        if (distance <= surface_threshold_) {
          ++count_in;
        } 
      }

      if (count_in >= max_size) {
        max_size = count_in;
        Ad = A;
        Bd = B;
        Cd = C;
        Dd = D;
      }
    }

    output_point_cloud->points.reserve(input_point_cloud_->points.size());

    for (PointT &point: input_point_cloud_->points) {
      const float distance = std::abs(Ad*point.x + Bd*point.y + Cd*point.z + Dd) / std::sqrt(std::pow(Ad, 2) + std::pow(Bd, 2) + std::pow(Cd, 2));

      if (distance >= surface_threshold_) {
        output_point_cloud->points.push_back(point);
      }
    }

    output_point_cloud->sensor_orientation_ = Eigen::Quaternionf(1.0, 0, 0, 0);
  }
};
#endif 