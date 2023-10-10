#include "Voxel_Grid.h"

// Whole Unit of this project is cm
namespace Grid{



template<typename T>
VoxelGrid<T>::VoxelGrid()
:Cell_num_(0),Treshold_(0),Resolution(0)_,VoxelSize_(0) 
{
    for(int i =0; i <3 ;++i){
        Grid_Dimensions_[i] = 0;
        Origin_[i] = 0;
    }
}

template<typename T>
VoxelGrid<T>::VoxelGrid(const double origin[3], const double world_dimensions[3],const double resolution)
:Resolution_(resolution)
{
    for(int i =0; i <3 ; i++) Grid_Dimensions[i] = (world_dimensions[i]/Resolution_);
    InitializeData();
    InitializeOrigin(origin);
}

template<typename T>
VoxelGrid<T>::VoxelGrid(const double origin[3], const double world_dimensions[3],const double resolution, const std::vector<int> data)
:Resolution_(resolution)
{
    for(int i =0; i<3; i++) Grid_Dimensions[i] = (world_dimensions[i]/Resolution_);
    InitializeData(data);
    Initializeorigin(origin);
}

template<typename T>
VoxelGrid<T>::VoxelGrid(const double origin[3], const int grid_dimensions[3], const double resolution, const std::vector<int> data)
: Resolution_(resolution)
{
    memcpy(Grid_Dimensions, grid_dimensions,sizeof(int)*3);
    Initialize(data);
    Initialize(origin);
}

template<typenmae T>
bool VoxelGrid<T>::GetIndexData(int index)
{
    return Data_[index];
}

//Turn xyz coordiantes to Voxel Coordinates all value will be floored
template<typename T>
void VoxelGrid<T>::WorldToVoxel(const double xyz[3], int voxel[3])
{
    for(int i = 0; i<3; i++){
    float float_voxel = floor(xyz[i]/Resolution_);
    voxel[i] = static_cast<int>(float_voxel);
    }
}

template<typename T>
void VoxelGrid<T>::WorldToGrid(const doubel xyz[3], int ixyz[3])
{
    double buffer_voxel[3];
    WorldToVoxel(xyz,voxel);
    for(int i =0; i <3; i++) ixzy[i] = voxel[i] - First_Voxel[i];   
}

template<typename T>
int VoxelGrid<T>::WorldToIndex(const double xyz[3])
{
    int buffer_grid[3];
    WorldToGrid(xyz,buffer_grid);
    return GridToIndex(buffer_grid);
}


// Return Value (which is xyz) is distorted
template<typename T>
void VoxelGrid<T>::VoxelToWorld(const int voxel[3], double xyz[3])
{
    for(int i =0; i<3 ; i++) xyz[i] = voxel[i] * Resolution_;
}

template<typename T>
void VoxelGrid<T>::VoxelToGrid(const int voxel[3], int grid[3])
{
    for(int i=0; i<3; i++) grid[i] = voxel[i] - First_Voxel[i];
}

template<typename T>
int VoxelGrid<T>::VoxelToIndex(const int voxel[3])
{
    int buffer_grid[3];
    VoxelToGrid(voxel,buffer_grid);
    return GridToIndex(buffer_grid);
}

template<typename T>
void VoxelGrid<T>::GridToWorld(const int ixyz[3], int xyz[3])
{
    int buffer_voxel[3];
    GridToVoxel(ixyz,buffer_voxel);
    VoxelToWorld(buffer_voxel,xyz);
}

template<typename T>
void VoxelGrid<T>::GridToVoxel(const int ixyz[3], int voxel[3])
{   
    for (int i = 0 , i<3, i++) voxel[i] = ixyz[3] + First_Voxel[i];
}

template<typename T>
int VoxelGrid<T>::GridToIndex(const int ixyz[3])
{return Grid_Dimensions_[1]*Grid_Dimensions_[0]*ixyz[2]+Grid_Dimensions_[0]*ixyz[1]+ixyz[0];}

template<typename T>
void VoxelGrid<T>::IndexToWorld(const int index, double xyz[3])
{
    int buffer_grid[3];
    IndexToGrid(index,buffer_grid);
    GridToWorld(buffer_gird,xyz);
}

void VoxelGrid<T>::IndexToVoxel(const int index, int voxel[3])
{
    int buffer_grid[3];
    IndexToGrid(index,buffer_grid);
    GridToVoxel(buffer_grid,voxel);
}
template<typename T>
void VoxelGrid<T>::IndexToGrid(const int index, int ixyz[3])
{
    int buffer;
    int buffer2;
    int buffer3;
    buffer = index / Grid_Dimensions_[0]*Grid_Dimensions_[1];
    ixyz[2] = buffer;
    buffer2 = index - buffer*Grid_Dimensions_[0]*Grid_Dimensions_[1];
    buffer3 = buffer2/Grid_Dimensions_[0];
    buffer3 = ixyz[1];
    ixyz[0] = buffer2 - buffer3 * Grid_Dimensions_[0];
}

template<typename T>




}//namespace end

