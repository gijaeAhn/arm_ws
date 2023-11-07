#include "Occupancy_Grid.h"


namespace occupancygrid
{
OccupancyGrid::OccupancyGrid(const double origin[3], const double world_dimensions[3], const double resolution)
: voxelgrid::VoxelGrid<bool>(origin,world_dimensions,resolution)
{}

void OccupancyGrid::UpdateValue(const pcl::PointXYZ& point, bool value)
{
    double d_point[3] = {point.x,point.y,point.z};
    UpdateValue(d_point,value);
}

void OccupancyGrid::UpdateValue(const double xyz[3], bool value)
{UpdateValue(WorldToIndex(xyz),value);}

void OccupancyGrid::UpdateValue(const int ixyz[3], bool value)
{UpdateValue(GridToIndex(ixyz),value);}

void OccupancyGrid::UpdateValue(const int index, bool value)
{WriteValue(index,value);}

bool OccupancyGrid::OccupancyCheck(const pcl::PointXYZ& point)
{
    double buf_xyz[3] = {point.x,point.y,point.z};
    return OccupancyCheck(buf_xyz);
}

bool OccupancyGrid::OccupancyCheck(const double xyz[3])
{
    int index = WorldToIndex(xyz);
    return GetIndexData(index);
}

bool OccupancyGrid::OccupancyCheck(const int ixyz[3])
{
    int index = GridToIndex(ixyz);
    return GetIndexData(index);
}

bool OccupancyGrid::OccupancyCheck(const int index)
{return GetIndexData(index);}

void OccupancyGrid::toMarkList(const int index)
{   
    double buf_xyz[3];
    IndexToWorld(index,buf_xyz);
    // printf("%d\n",index);
    // printf("%f %f %f\n",buf_xyz[0],buf_xyz[1],buf_xyz[2]);
    
    Point point;
    for(int i =0; i <3 ; i++){
        point.data[i] = buf_xyz[i];
    }

    Marking_List_.push_back(point);
}

void OccupancyGrid::printMarkList()
{
    for(int i =0; i < Marking_List_.size() ; i++)
    {std::cout <<"{" << Marking_List_[i].data[0] << " "<< Marking_List_[i].data[1] << " "<< Marking_List_[i].data[2] << "}";}
    std::cout << std::endl;
}

// float OccupancyGrid::Clamping(const int num, const float max, const float min)
// {
//     if(num > max) return max;
//     else if(num < min) return min;
//     return num;
// }



} // namespace end

