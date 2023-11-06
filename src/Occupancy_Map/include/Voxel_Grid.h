#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <vector>
#include <cstring>
#include <iostream>

#define M2CM  100
#define CM2M  0.01



namespace voxelgrid {

template<typename T>
class VoxelGrid {
    private:

    int First_Voxel_[3];
    
    
    
    int VoxelSize_;
    int Cell_Num_;


    

    protected:
    float Origin_[3];
    int Grid_Dimensions_[3];
    std::vector<T> Data_;
    double Resolution_;
    double Threshold_;



    public:

    
    VoxelGrid();
    VoxelGrid(const double origin[3], const double world_dimensions[3], const double resolution);
    VoxelGrid(const double origin[3], const double world_dimensions[3], const double resolution, const std::vector<T>& data);
    VoxelGrid(const double origin[3], const int grid_dimensions[3], const double resolution, const std::vector<T>& data);
    
  
    T GetIndexData(int index);
    void GetState();

    void WorldToVoxel(const double xyz[3], int voxels[3]);
    void WorldToGrid(const double xyz[3], int ixyz[3]);
    int  WorldToIndex(const double xyz[3]);
    void VoxelToWorld(const int voxel[3], double xyz[3]);
    void VoxelToGrid(const int voxel[3], int ixyz[3]);
    int  VoxelToIndex(const int voxel[3]);
    void GridToWorld(const int ixyz[3], double xyz[3]);
    void GridToVoxel(const int ixyz[3], int voxel[3]);
    int  GridToIndex(const int ixyz[3]);
    void IndexToWorld(const int index, double xyz[3]);
    void IndexToVoxel(const int index, int voxel[3]); 
    void IndexToGrid(const int index, int ixyz[3]);
    
    void WriteValue(const double xyz[3], T value);
    void WriteValue(const int ixzy[3], T value);
    void WriteValue(const int index, T value);

    void InitializeData();
    void InitializeData(const std::vector<T>& data);
    void InitializeOrigin(const double origin[3]);
    int GetCellNum();
    void PrintData();
    void CalcFirstVoxel(const double origin[3]);

    void getResoltuion(const double resolution);
    void getThreshold(const double threshold);
    void printFirstVoxel() const ;
    // void getGridDimesion(const double world_D[3], const double resolutuion);
};


template<typename T>
VoxelGrid<T>::VoxelGrid()
:Cell_Num_(0),Threshold_(0),Resolution_(0),VoxelSize_(0.0) 
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
    for(int i =0; i <3 ; i++) Grid_Dimensions_[i] = static_cast<int>(world_dimensions[i]*M2CM*Resolution_);
    InitializeData();
    InitializeOrigin(origin);
}

template<typename T>
VoxelGrid<T>::VoxelGrid(const double origin[3], const double world_dimensions[3],const double resolution, const std::vector<T>& data)
:Resolution_(resolution)
{
    for(int i =0; i<3; i++) Grid_Dimensions_[i] = static_cast<int>(world_dimensions[i]*M2CM*Resolution_);
    InitializeData(data);
    InitializeOrigin(origin);
}

template<typename T>
VoxelGrid<T>::VoxelGrid(const double origin[3], const int grid_dimensions[3], const double resolution, const std::vector<T>& data)
: Resolution_(resolution)
{
    memcpy(Grid_Dimensions_, grid_dimensions,sizeof(int)*3);
    InitializeData(data);
    InitializeOrigin(origin);
}

template<typename T>
T VoxelGrid<T>::GetIndexData(int index)
{   
    if(index > Data_.size()) std::cout<< "Index is not in the range" << std::endl;
    return Data_[index];
}

template<typename T>
void VoxelGrid<T>::GetState()
{
    std::cout << " Cell Number : " << Cell_Num_ << " Resolution : " << Resolution_ << " Grid Dimensions : " << Grid_Dimensions_[0] << " " 
    << Grid_Dimensions_[1] << " " << Grid_Dimensions_[2] << std::endl;
}

//Turn xyz coordiantes to Voxel Coordinates all value will be floored
template<typename T>
void VoxelGrid<T>::WorldToVoxel(const double xyz[3], int voxel[3])
{
    for(int i = 0; i<3; i++){
    float float_voxel = floor(xyz[i]*M2CM*Resolution_);
    voxel[i] = static_cast<int>(float_voxel);
    }
}

template<typename T>
void VoxelGrid<T>::WorldToGrid(const double xyz[3], int ixyz[3])
{
    int buffer_voxel[3];
    WorldToVoxel(xyz,buffer_voxel);
    for(int i =0; i <3; i++) ixyz[i] = buffer_voxel[i] - First_Voxel_[i];   
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
    for(int i =0; i<3 ; i++) xyz[i] = ((static_cast<float>(voxel[i])) / Resolution_)*CM2M;
    printf("%f %f %f\n",xyz[0],xyz[1],xyz[2]);
}

template<typename T>
void VoxelGrid<T>::VoxelToGrid(const int voxel[3], int grid[3])
{
    for(int i=0; i<3; i++) grid[i] = voxel[i] - First_Voxel_[i];
}

template<typename T>
int VoxelGrid<T>::VoxelToIndex(const int voxel[3])
{
    int buffer_grid[3];
    VoxelToGrid(voxel,buffer_grid);
    return GridToIndex(buffer_grid);
}

template<typename T>
void VoxelGrid<T>::GridToWorld(const int ixyz[3], double xyz[3])
{
    int buffer_voxel[3];
    printf("%d %d %d\n", ixyz[0],ixyz[1],ixyz[2]);
    GridToVoxel(ixyz,buffer_voxel);

    VoxelToWorld(buffer_voxel,xyz);
}

template<typename T>
void VoxelGrid<T>::GridToVoxel(const int ixyz[3], int voxel[3])
{   
    printf("Print First Voxel %d %d %d\n", First_Voxel_[0],First_Voxel_[1],First_Voxel_[2]);
    for (int i = 0; i<3; i++) voxel[i] = ixyz[i] + First_Voxel_[i];
   
    printf("%d %d %d\n", voxel[0],voxel[1],voxel[2]);

}

template<typename T>
int VoxelGrid<T>::GridToIndex(const int ixyz[3])
{return Grid_Dimensions_[1]*Grid_Dimensions_[0]*ixyz[2]+Grid_Dimensions_[0]*ixyz[1]+ixyz[0];}

template<typename T>
void VoxelGrid<T>::IndexToWorld(const int index, double xyz[3])
{
    int buffer_grid[3];
    IndexToGrid(index,buffer_grid);
    GridToWorld(buffer_grid,xyz);
}

template<typename T>
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
    buffer = index / (Grid_Dimensions_[0]*Grid_Dimensions_[1]);
    ixyz[2] = buffer;
    buffer2 = index - (buffer*Grid_Dimensions_[0]*Grid_Dimensions_[1]);
    buffer3 = buffer2/Grid_Dimensions_[0];
    ixyz[1] = buffer3;
    ixyz[0] = buffer2 - buffer3 * Grid_Dimensions_[0];
    // printf("%d %d %d\n",ixyz[0],ixyz[1],ixyz[2]);
}

template<typename T>
void VoxelGrid<T>::WriteValue(const double xyz[3], T value)
{
    int index = WorldToIndex(xyz);
    std::cout << index << " " << value << std::endl;
    Data_[index] = value;
    std::cout  << Data_[index] << std::endl;

}

template<typename T>
void VoxelGrid<T>::WriteValue(const int ixyz[3], T value)
{
    int index = GridToIndex(ixyz);
    Data_[index] = value;
}

template<typename T>
void VoxelGrid<T>::WriteValue(const int index, T value)
{
    Data_[index] = value;
}

template<typename T>
void VoxelGrid<T>::InitializeData()
{
  Cell_Num_ = Grid_Dimensions_[0] * Grid_Dimensions_[1] * Grid_Dimensions_[2];
  for (int i = 0; i < Cell_Num_; i++)
  {
    Data_.push_back(T());
  }
}
template<typename T>
void VoxelGrid<T>::InitializeData(const std::vector<T>& data)
{
  Cell_Num_ = Grid_Dimensions_[0] * Grid_Dimensions_[1] * Grid_Dimensions_[2];
  Data_ = data;
}

template<typename T>
void VoxelGrid<T>::InitializeOrigin(const double origin[3])
{
    for( int i =0; i<3;i++) Origin_[i] = origin[i];
}
template<typename T>
int VoxelGrid<T>::GetCellNum()
{return Cell_Num_;}

template<typename T>
void VoxelGrid<T>::PrintData()
{for(int i =0; i<Cell_Num_; i++)
std::cout << Data_[i] ;
}

template<typename T>
void VoxelGrid<T>::CalcFirstVoxel(const double origin[3])
{   
    int buf_voxel[3] = {0};
    WorldToVoxel(origin,buf_voxel);
    First_Voxel_[0] = buf_voxel[0] - (Grid_Dimensions_[0] >> 1);
}

template<typename T>
void VoxelGrid<T>::getResoltuion(const double resolution)
{Resolution_ = resolution;}

template<typename T>
void VoxelGrid<T>::getThreshold(const double threshold)
{Threshold_ = threshold;}


template<typename T>
void VoxelGrid<T>::printFirstVoxel() const
{
    printf("%d %d %d\n",First_Voxel_[0],First_Voxel_[1],First_Voxel_[2]);
}
// template<typename T>
// void VoxelGrid<T>::getGridDimesion(const double world_D[3], const double resolution)
// {
//     for(int i = 0; i<3; i++) ocgrid_.Grid_Dimensions_[i] = static_cast<int>(world_D[i]*resolution);
// }
}// namespace end