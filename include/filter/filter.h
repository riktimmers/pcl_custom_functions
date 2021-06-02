#ifndef _H_FILTER__
#define _H_FILTER__

struct Filter {
  float min_x = 0.0f;
  float max_x = 0.0f;
  float min_y = 0.0f;
  float max_y = 0.0f;
  float min_z = 0.0f;
  float max_z = 0.0f;
  float surface_threshold = 0.0f;
  float cluster_threshold = 0.0f;
  size_t min_cluster_size = 0;
  size_t max_cluster_size = 0;
};

#endif 