#include "collision_check_with_path/collision_check.cuh"

#define PI 3.1415926535897
#define arraySize 100
__global__ void print_cuda_kernel_test() 
{
    printf("Hello from block %d, thread %d\n", blockIdx.x, threadIdx.x);
}
// Run on GPU
__global__ void add(int *a, int *b, int *c) {
    *c = *a + *b;
}

__global__ void calculate_collision_cuda_kernel(GridMap* device_grid_map)//PathCandidates* device_candidates)//, GridMap* device_grid_map, int grid_map_width_size, int grid_map_height_size)
{
    // int index = blockIdx.x * blockDim.x + threadIdx.x;
    // PathCandidates single_path = device_candidates[index];
    // device_candidates -> points_x_
    
    
    // GridMap * grid_map_ = (GridMap*)malloc(m_OccupancyGrid_ptr->info.width * sizeof(GridMap));
    // for (unsigned int width = 0; width < m_OccupancyGrid_ptr->info.width; width++){
    //     grid_map_[width].occupied_intensity_ = (int*)malloc(m_OccupancyGrid_ptr->info.height * sizeof(int));
    //     for (unsigned int height = 0; height < m_OccupancyGrid_ptr->info.height; height++)
    //     {
    //     if(m_OccupancyGrid_ptr->data[height * m_OccupancyGrid_ptr->info.width + width] > 0)
    //     {
    //         *(grid_map_[width].occupied_intensity_ + height) = 255;
    //         geometry_msgs::Pose obstacle;
    //         obstacle.position.x = width * m_OccupancyGrid_ptr->info.resolution + m_OccupancyGrid_ptr->info.resolution /2 + m_OccupancyGrid_ptr->info.origin.position.x;
    //         obstacle.position.y = height * m_OccupancyGrid_ptr->info.resolution + m_OccupancyGrid_ptr->info.resolution /2 + m_OccupancyGrid_ptr->info.origin.position.y;
    //         m_Obstacles.push_back(obstacle);
    //     }
    //     else
    //     {
    //         *(grid_map_[width].occupied_intensity_ + height) = 0;
    //     }
    //     }
    // }
    

    // for (int k =0; k < m_Obstacles.size(); k++)
    // {
    //     for (int i =0; i < RollOut.size(); i++)
    //     {
    //     for (int j = 0; j < RollOut.at(i).size(); j++)
    //     {
    //         double x_wpt_ = RollOut.at(i).at(j).pos.x;
    //         double y_wpt_ = RollOut.at(i).at(j).pos.y;      
    //         double dist_to_obstacle = sqrt(pow(m_Obstacles.at(k).position.x - x_wpt_ ,2) + pow(m_Obstacles.at(k).position.y - y_wpt_,2));
    //         if (dist_to_obstacle < m_obstacle_radius)
    //         {
    //         int once;
    //         if (once == k)
    //         continue;
    //         std::cout << "x: "<<m_Obstacles.at(k).position.x << " y: " << m_Obstacles.at(k).position.y << "dist: "<<  dist_to_obstacle<< std::endl;
    //         visualization_msgs::Marker test_marker;
    //         once = k;
    //         }  
    //     }
    //     }
    // }
}
__global__ void addKernel( int *c, const int *a, const int *b )
{
    // int i = threadIdx.x;
    int i = blockIdx.x ;
	if( i < arraySize )
		c[i] = a[i] + b[i];
}
void path_candidates_initialize(
    PathCandidates* candidates, int candidates_size,
    GridMap* grid_map, int grid_map_width_size, int grid_map_height_size)
{
    
    int a[arraySize];
    int b[arraySize];
    int c[arraySize];

    int *dev_a = 0;
    int *dev_b = 0;
    int *dev_c = 0;

	// fill the arrays 'a' and 'b' on the CPU
    for( int i = 0 ; i < arraySize ; i++ ) {
		a[i] = i;
		b[i] = i;
	}

	// Add vectors in parallel.
	// Allocate GPU buffers for three vectors (two input, one output)
    cudaMalloc((void**)&dev_c, arraySize * sizeof(int));
    cudaMalloc((void**)&dev_a, arraySize * sizeof(int));
    cudaMalloc((void**)&dev_b, arraySize * sizeof(int));

// copy the arrays 'a' and 'b' to the GPU
    cudaMemcpy(dev_a, a, arraySize * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(dev_b, b, arraySize * sizeof(int), cudaMemcpyHostToDevice);

    addKernel<<<arraySize, 1>>>(dev_c, dev_a, dev_b);
    cudaDeviceSynchronize();

// copy the array 'c' back from the GPU to the CPU
    cudaMemcpy(&c, dev_c, arraySize * sizeof(int), cudaMemcpyDeviceToHost);

// display the results
    for( int i = 0 ; i < arraySize ; i++ ) {
	printf( "%d + %d = %d\n", a[i], b[i], c[i] );
	}

	// free the memory allocated on the GPU
    cudaFree(dev_c);
    cudaFree(dev_a);
    cudaFree(dev_b);



    // printf("initialize is working, candidates size: %d\n", candidates_size);
    // PathCandidates* device_candidates;
    // cudaMalloc((void**)&device_candidates, candidates_size * sizeof(PathCandidates));
    // cudaMemcpy(device_candidates, candidates, candidates_size * sizeof(PathCandidates), cudaMemcpyHostToDevice);
    // printf("1\n");
    // for (int i =0; i < candidates_size; i++)
    // {
    //     printf("points size: %d \n" ,candidates[i].points_size_); 
    //     PathCandidates single_path = candidates[i];
    //     // points_x_
    //     cudaMalloc((void**)&(device_candidates[i].points_x_), single_path.points_size_ * sizeof(double));
    //     cudaMemcpy(device_candidates[i].points_x_, single_path.points_x_, single_path.points_size_ * sizeof(double), cudaMemcpyHostToDevice);        
    //     //points_y_
    //     cudaMalloc((void**)&(device_candidates[i].points_y_), single_path.points_size_ * sizeof(double));
    //     cudaMemcpy(device_candidates[i].points_y_, single_path.points_y_, single_path.points_size_ * sizeof(double), cudaMemcpyHostToDevice);
    // }

    // printf("grid size: (%d * %d) \n", grid_map_width_size, grid_map_height_size);
    // GridMap* device_grid_map;
    // cudaMalloc((void**)&device_grid_map, grid_map_width_size * sizeof(GridMap));
    // cudaMemcpy(device_grid_map, grid_map, grid_map_width_size * sizeof(GridMap), cudaMemcpyHostToDevice);

    // for (int i =0; i < grid_map_width_size; i++){
    //     GridMap host_grid_map = grid_map[i];
    //     //host_grid_map_x
    //     cudaMalloc((void**)&(device_grid_map[i].grid_x_), grid_map_height_size * sizeof(double));
    //     cudaMemcpy(device_grid_map[i].grid_x_, host_grid_map.grid_x_, grid_map_height_size * sizeof(double), cudaMemcpyHostToDevice);
    //     //host_grid_map_y
    //     cudaMalloc((void**)&(device_grid_map[i].grid_y_), grid_map_height_size * sizeof(double));
    //     cudaMemcpy(device_grid_map[i].grid_y_, host_grid_map.grid_y_, grid_map_height_size * sizeof(double), cudaMemcpyHostToDevice);
    //     //intensity
    //     cudaMalloc((void**)&(device_grid_map[i].occupied_intensity_), grid_map_height_size * sizeof(int));
    //     cudaMemcpy(device_grid_map[i].occupied_intensity_, host_grid_map.occupied_intensity_, grid_map_height_size * sizeof(int), cudaMemcpyHostToDevice);
    // }

    // dim3 grid(grid_map_width_size, grid_map_height_size);
    // // dim3 block(candidates_size, 100, 100);
  
    // // calculate_collision_cuda_kernel<<<grid, 1>>>(device_grid_map);

    // // calculate_collision_cuda_kernel<<<1, 1>>>();//, candidates_size); //, device_grid_map, grid_map_width_size, grid_map_height_size);

    // for (int i =0; i < candidates_size; i++){

    //     //points_x_
    //     cudaMemcpy(candidates[i].points_x_, device_candidates[i].points_x_, candidates[i].points_size_ * sizeof(double), cudaMemcpyDeviceToHost);
    //     //points_y_
    //     cudaMemcpy(candidates[i].points_y_, device_candidates[i].points_y_, candidates[i].points_size_ * sizeof(double), cudaMemcpyDeviceToHost);
    // }

    // for (int i =0; i < grid_map_width_size; i++)
    // {
    //     //host_grid_map_x
    //     cudaMemcpy(grid_map[i].grid_x_, device_grid_map[i].grid_x_, grid_map[i].grid_x_size_ * sizeof(unsigned int), cudaMemcpyDeviceToHost);
    //     //host_grid_map_y
    //     cudaMemcpy(grid_map[i].grid_y_, device_grid_map[i].grid_y_, grid_map[i].grid_y_size_ * sizeof(unsigned int), cudaMemcpyDeviceToHost);
    //     //intensity
    //     cudaMemcpy(grid_map[i].occupied_intensity_, device_grid_map[i].occupied_intensity_, grid_map[i].occupied_intensity_size_ * sizeof(unsigned int), cudaMemcpyDeviceToHost);
    // }

    // // Cleanup
    // printf("1\n");
    // for (int i =0; i < candidates_size; i++)
    // {
    //   free(candidates[i].points_x_); //    candidates[i].points_x_ = (double*)malloc(device_single_path.points_size_ * sizeof(double));
    //   free(candidates[i].points_y_); // device_single_path.points_y_ = (double*)malloc(device_single_path.points_size_ * sizeof(double));    
    // }
    // free(candidates);

    // printf("2\n");
    // for (int i =0; i < grid_map_width_size; i++)
    // {
    //   free(grid_map[i].grid_x_); 
    //   free(grid_map[i].grid_y_); 
    //   free(grid_map[i].occupied_intensity_);

    // }
    // free(grid_map);

    // printf("3\n");
    // for (int i =0; i < candidates_size; i++)
    // {
    //     cudaFree(device_candidates[i].points_x_);
    //     cudaFree(device_candidates[i].points_y_);
    // }
    // cudaFree(device_candidates);
    // for (int i =0; i < candidates_size; i++)
    // {
    //     cudaFree(device_grid_map[i].grid_x_);
    //     cudaFree(device_grid_map[i].grid_y_);
    //     cudaFree(device_grid_map[i].occupied_intensity_);
    // }
    // cudaFree(device_grid_map);
}