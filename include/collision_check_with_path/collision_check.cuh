#ifndef COLLISION_CHECK_CUH_
#define COLLISION_CHECK_CUH_

#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>

// #include <thrust/device_ptr.h>
// #include <thrust/device_vector.h>
// #include <thrust/copy.h>
// #include <thrust/scan.h>
// #include <thrust/fill.h>

typedef struct _PathCandidates {
  
  double* points_x_;
  double* points_y_;
  double* yaw_;
  double* curvature_;

  int points_size_;
  int yaw_size_;
  int curvature_size_;
  double cost_total_;

} PathCandidates;

typedef struct _GridMap {
  
  double* grid_x_;
  double* grid_y_;
  int* occupied_intensity_;

  int grid_x_size_;
  int grid_y_size_;
  int occupied_intensity_size_;
} GridMap;




void path_candidates_initialize(
    PathCandidates* path_candidates, int paths_size,
    GridMap* grid_map, int grid_map_width_size, int grid_map_height_size);

// void calculate_global_paths_finalize(
//     CUDAFrenetPath* cuda_frenet_paths, int cuda_frenet_paths_size,
//     CUDAFrenetPath* device_cuda_frenet_paths,
//     CUDACubicSpliner2D* cuda_cubic_spliner_2D,
//     CUDACubicSpliner2D* device_cuda_cubic_spliner_2D);

#endif  // COLLISION_CHECK_CUH_
