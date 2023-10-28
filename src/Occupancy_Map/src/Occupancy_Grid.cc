#include "Occupancy_Grid.h"


namespace occupancygrid
{
OccupancyGrid::OccupancyGrid(const double origin[3], const double world_dimensions[3], const double resolution, const double threshold)
: voxelgrid::VoxelGrid<bool>(origin,world_dimensions,resolution)
{
    Threshold_ = threshold;
}

void OccupancyGrid::UpdateValue(const pcl::PointXYZ& point, bool value)
{double d_point[3] = {point.x,point.y,point.z};
UpdateValue(d_point,value);}
void OccupancyGrid::UpdateValue(const double xyz[3], bool value)
{UpdateValue(WorldToIndex(xyz),value);}

void OccupancyGrid::UpdateValue(const int ixyz[3], bool value)
{UpdateValue(GridToIndex(ixyz),value);}

void OccupancyGrid::UpdateValue(const int index, bool value)
{WriteValue(index,value);}


bool OccupancyGrid::OccupancyCheck(const double xyz[3])
{
    int index = WorldToIndex(xyz);
    return Threshold_ < GetIndexData(index);
}

bool OccupancyGrid::OccupancyCheck(const int ixyz[3])
{
    int index = GridToIndex(ixyz);
    return Threshold_ < GetIndexData(index);
}

bool OccupancyGrid::OccupancyCheck(const int index)
{return Threshold_ < GetIndexData(index);}

// float OccupancyGrid::Clamping(const int num, const float max, const float min)
// {
//     if(num > max) return max;
//     else if(num < min) return min;
//     return num;
// }



} // namespace end

