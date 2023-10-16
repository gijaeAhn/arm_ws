#include "Occupancy_Grid.h"


namespace occupancygrid
{
OccupancyGrid::OccupancyGrid(const double origin[3], const double world_dimensions[3], const double resolution, const double threshold)
: voxelgrid::VoxelGrid<float>(origin,world_dimensions,resolution)
{
    Threshold_ = threshold;
}


void OccupancyGrid::UpdateValue(const double xyz[3], float value)
{UpdateValue(WorldToIndex(xyz),value);}

void OccupancyGrid::UpdateValue(const int ixyz[3], float value)
{UpdateValue(GridToIndex(ixyz),value);}

void OccupancyGrid::UpdateValue(const int index, float delta)
{
    float buf = GetIndexData(index);
    if(buf <0)
    {WriteValue(index,0.0f);
    buf = 0.0f;}
    bool  before_occupied = OccupancyCheck(index);
    float new_value = Clamping(buf+delta,1,0);
    WriteValue(index, new_value);
    bool  after_occupied = OccupancyCheck(index);

    double xyz[3];
    if (!before_occupied && after_occupied)
    {
      IndexToWorld(index, xyz);
      Marking_List_.push_back({ xyz[0], xyz[1], xyz[2] });
    }   
    if (before_occupied && !after_occupied)
    {
      IndexToWorld(index, xyz);
      Clear_list_.push_back({ xyz[0], xyz[1], xyz[2] });
    }
}


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

float OccupancyGrid::Clamping(const int num, const float max, const float min)
{
    if(num > max) return max;
    else if(num < min) return min;
    return num;
}


} // namespace end

