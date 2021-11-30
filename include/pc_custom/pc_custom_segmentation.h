#ifndef _H_PC_CUSTOM_SEGMENTATION__
#define _H_PC_CUSTOM_SEGMENTATION__

#include <pcl/point_cloud.h>
#include <random>
#include <chrono>
#include <Eigen/Dense>
#include <vector>

struct Coeff { // Struct for containing the coefficients for a plane
  float A = 0.0f;
  float B = 0.0f;
  float C = 0.0f;
  float D = 0.0f; 
};

template <typename PointT>
class PCCustomSegmentation {

  size_t max_iterations_; // Max iterations to find the plane.
  float surface_threshold_{0.0}; // Surface threshold to determine if a point belongs to a surface, in cm.
  std::default_random_engine generator_; // Generator for random index values.

public:
  PCCustomSegmentation() : max_iterations_(100), // Set default 100 iterations.
                            generator_(std::chrono::system_clock::now().time_since_epoch().count()) // Set random seed.
                            {
  }

  void setMaxIterations(size_t max_iterations) { // Set max iterations.
    max_iterations_  = max_iterations;
  }

  void setSurfaceThreshold(float surface_treshold) { // Set surface threshold.
    surface_threshold_ = surface_treshold;
  }

  void segment(const pcl::PointCloud<PointT>::Ptr &input_point_cloud,  //Segment out the planar surface.
               pcl::PointCloud<PointT>::Ptr &output_point_cloud) { // Output cloud that does not contain a planar surface.
    std::uniform_int_distribution<size_t> distribution(0, input_point_cloud->points.size() - 1); // Set range for random index values.
    std::uniform_int_distribution<size_t> index_distribution(0, 9); // Set range for offset when iteration of points.
    size_t max_size = 0; // Max size of found plane.
    std::vector<size_t> cloud_indices;

    Coeff coeff_desired;
    std::vector<size_t> indices;

    for (size_t iterations = 0; iterations < max_iterations_; ++iterations) {
      indices.clear();
      indices.reserve(input_point_cloud->points.size());


      PointT point1 = getRandomPoint(input_point_cloud, distribution);
      PointT point2 = getRandomPoint(input_point_cloud, distribution);
      PointT point3 = getRandomPoint(input_point_cloud, distribution);
      
      const Eigen::Vector3f PQ(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z);
      const Eigen::Vector3f PR(point3.x - point1.x, point3.y - point1.y, point3.z - point1.z);
      const Eigen::Vector3f cross = PQ.cross(PR);

      Coeff coeff;
      coeff.A = cross[0];
      coeff.B = cross[1];
      coeff.C = cross[2];
      coeff.D = -(coeff.A * point1.x + coeff.B * point1.y + coeff.C * point1.z);

      size_t count_part_of_plane = 0; 

      // iterate over all points with an offset of 10, starting index is random value 0 - 10.  
      for (size_t index = index_distribution(generator_); index < input_point_cloud->points.size(); index += 10) {
        PointT &point = input_point_cloud->points.at(index);

        if (calculateDistance(coeff, point) <= surface_threshold_) {
          ++count_part_of_plane;
        } 
      }

      if (count_part_of_plane >= max_size) {
        max_size = count_part_of_plane;
        coeff_desired = coeff;
      }
    }

    output_point_cloud->points.reserve(input_point_cloud->points.size());

    for (PointT &point: input_point_cloud->points) {

      if (calculateDistance(coeff_desired, point) >= surface_threshold_) {
        output_point_cloud->points.push_back(point);
      }
    }
  }

private:
  inline float calculateDistance(const Coeff coeff, const PointT point) {
    return std::abs(coeff.A*point.x + coeff.B*point.y + coeff.C*point.z + coeff.D) / std::sqrt(std::pow(coeff.A, 2) + std::pow(coeff.B, 2) + std::pow(coeff.C, 2));
  }

  inline PointT getRandomPoint(const pcl::PointCloud<PointT>::Ptr &cloud, std::uniform_int_distribution<size_t> &distribution) { 
    return cloud->points.at(distribution(generator_));
  }
};
#endif 