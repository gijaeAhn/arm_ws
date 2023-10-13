#include "Voxel_Grid.h"
#include "Occupancy_Grid.h"


int main()
{
    double origin[3] = {0,0,0};
    double world_dimensions[3] = {848,480,256};
    double test_xyz[3] = {88.5, 62.4, 13.7};
    // double test_xyz[3] = {30.5, 0, 0};
    double test_resolution = 0.1;

    int test_ixyz[3] = {8,6,1};
    float test_value = 0.7;
    int test_index = 84*48*1 + 84*6 + 8;
    // int test_index = 29;
    

    voxelgrid::VoxelGrid<float> test_voxel(origin,world_dimensions,test_resolution);

    test_voxel.GetState();
    // test_voxel.WriteValue(test_xyz, test_value);
    test_voxel.WriteValue(test_ixyz, test_value);
    std::cout <<"Test Index : "<< test_index << 
    "Testing Index value : " << test_voxel.GetIndexData(test_index)<<std::endl;
    // test_voxel.PrintData();

    



}
