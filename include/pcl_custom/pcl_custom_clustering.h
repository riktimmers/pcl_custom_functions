#ifndef _H_PCL_CUSTOM_CLUSTERING__
#define _H_PCL_CUSTOM_CLUSTERING__

#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <unordered_set>

template <typename PointT>
class PCLCustomClustering {


  pcl::search::KdTree<PointT>::Ptr kdtree_;
  float clustering_threshold_;
  size_t min_points_;
  size_t max_points_;

public:
  PCLCustomClustering() : 
    clustering_threshold_(0.05f),
    min_points_(0), 
    max_points_(10000) {}

  void setSearchMethod(pcl::search::KdTree<PointT>::Ptr kdtree) {
    kdtree_ = kdtree;
  }

  void setMinClusterSize(size_t min_points) {
    min_points_ = min_points;
  }

  void setMaxClusterSize(size_t max_points) {
    max_points_ = max_points;
  }

  void setClusteringThreshold(float clustering_threshold) {
    clustering_threshold_ = clustering_threshold;
  }

  bool index_found(std::vector<std::unordered_set<int>> &indices, size_t index_to_find) {
    
    for (size_t index = 0; index < indices.size(); ++index) { 
      if (indices.at(index).count(index_to_find)) {
        return true;
      }
    }

    return false;
  }

  bool inserted(std::vector<std::unordered_set<int>> &indices, std::vector<int> &cluster_indices) {
    for (size_t set_index = 0; set_index < indices.size(); ++set_index) {
      bool should_fuse = false; 

      for (auto &index: cluster_indices) {
      
        if (indices.at(set_index).count(index) > 0) {
          should_fuse = true;
          break;
        }
        
        if (should_fuse) {
          break;
        }
      }

      if (should_fuse) {
        indices.at(set_index).insert(cluster_indices.begin(), cluster_indices.end());
        return true;
      }
    }

    return false;
  }

  void extract(const pcl::PointCloud<PointT>::Ptr &point_cloud, std::vector<typename pcl::PointCloud<PointT>::Ptr> &clusters) {
    std::vector<std::unordered_set<int>> indices;
    indices.reserve(100);

    const int K = 10000;
    std::vector<int> included_indices(K);
    std::vector<float> distances(K);

    for (size_t index = 0; index < point_cloud->points.size(); ++index) {
      bool index_has_been_found = index_found(indices, index);

      if (index_has_been_found) {
        continue;
      }

      if (kdtree_->radiusSearch(point_cloud->points.at(index), clustering_threshold_, included_indices, distances) > 0) {

        if (indices.size() == 0) {
          std::unordered_set<int> cluster_indices;
          cluster_indices.insert(included_indices.begin(), included_indices.end());   
          indices.push_back(cluster_indices);
        } else {
          bool has_been_inserted = inserted(indices, included_indices);

          if (!has_been_inserted) {
            std::unordered_set<int> cluster_indices;
            cluster_indices.insert(included_indices.begin(), included_indices.end());   
            indices.push_back(cluster_indices);
          }
        } 
      }
    }

    clusters.reserve(indices.size());

    for (size_t index = 0; index< indices.size(); ++index) {

      if (indices.at(index).size() >= min_points_ && indices.at(index).size() <= max_points_) {
        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        cloud->points.reserve(indices.at(index).size());

        for (auto &idx: indices.at(index)) {
          cloud->points.push_back(point_cloud->points.at(idx));
        }

        clusters.push_back(cloud);
      }
    }
  }
};
#endif