#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <asser.h>
#include <vector>

#inlcude <vector>
#include <cstring>
#include <iostream>



namespace Grid {

template<typename T>
class VoxelGrid {
    private:

    int[3] First_Voxel;
    int[3] Grid_Dimensions_;
    float[3] Origin_;
    float Resolution_;
    float VoxelSize_;
    float Treshold_;
    int Cell_num_;
    std::vector<T> Data_;



    public:

    VoxelGrid();
    VoxelGrid(const double origin[3], const double world_dimensions[3], const double resolution);
    VoxelGrid(const double origin[3], const double world_dimensions[3], const double resolution, const std::vector<T>& data);
    VoxelGrid(const double origin[3], const int grid_dimensions[3], const double resolution, const std::vector<int>& data);
    
    bool GetIndexData(int index);
    void GetGridDimension(int dimensions);

    void WorldToVoxel(const double xyz[3], int voxels[3]);
    void WorldToGrid(const float xyz[3], int ixyz[3]);
    int  WorldToIndex(const float xyz[3]);
    
    void VoxelToWorld(const double voxel[3], double xyz[3]);
    void VoxelToGrid(const double voxel[3], int ixyz[3]);
    int  VoxelToIndex(const double voxel[3]);
    
    void GridToWorld(const int ixyz[3], float xyz[3]);
    void GridToVoxel(const int ixyz[3], int voxel[3]);
    int  GridToIndex(const int ixyz[3],);
    
    void IndexToWorld(const int ixyz[3], double xyz[3]);
    void IndexToVoxel(const int index, int voxel[3]); 
    void IndexToGrid(cosnt int index, int ixyz[3]);
    
    void WriteValue(const double xyz[3], T value);
    void WriteValue(const int ixzy[3], T value);
    void WriteValue(const int index, T value);

    void InitializeData();
    void InitializeData(std::vector<int> data);

   
}

















}