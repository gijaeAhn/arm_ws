#pragma once

#include <array>
#include <vector>
#include <cstring>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "voxel_grid.h"

namespace occupancy_grid {

using Point = std::array<double,3>;
using PointList = std::vector<Point>;

class OccupancyGrid : public voxel_grid::VoxelGrid<float>{

private:
virtual void PreShiftOrigin(const std::vector<int>& slice_indexes) override;
virtual void PostShiftOrigin(const std::vector<int>& slice_indexes) override;
bool IsOccupied(const int ind) const;
void UpdateValue(const int ind, float delta);
float clamp_value(float x, float min, float max) const;

PointList cleared_list_;
PointList marked_list_;
float threshold_;

public:
    OccupancyGrid(const double origin[3], const double world_dimensions[3], const double resolution, float threshold);
    void UpdateValue(const int ixyz[3], float value);
    void UpdateValue(const double xyz[3],float value);
    bool IsOccupied(const int ixyz[3]) const;
    bool IsOccupied(const float value) const;
    bool IsOccupied(const double xyz[3]) const;
    bool IsUnknown(const int ixyz[3]) const;
    bool IsUnknown(const double xyz[3]) const;
    bool IsUnknown(const float value) const;
    const PointList& GetCleared() const { return cleared_list_; }
    void ResetDiffs() { marked_list_.clear(); cleared_list_.clear(); }
    float GetThreshold() const  { return threshold_; }
};
}

namespace occupancy_grid{
OccupancyGrid::OccupancyGrid(const double origin[3], const double world_dimensions[3], const double resolution, float threshold)
                : voxel_grid::VoxelGrid<float>(origin,world_dimensions,resolution), threshold_(threshold)
{
    int num_cells = GetNumCells();
    Reset(-1);
    double indexer_origin[3];
    GetOrigin(indexer_origin);
    UpdateOrigin(indexer_origin);
}

void OccupancyGrid::PreShiftOrigin(const std::vector<int>& slice_indexes)
{
  // nothing
}

void OccupancyGrid::PostShiftOrigin(const std::vector<int>& slice_indexes)
{
  for (int index : slice_indexes)
  {
    WriteValue(index, -1);
  }
}

bool OccupancyGrid::IsOccupied(const int ixyz[3]) const
{
  return IsOccupied(ReadValue(ixyz));
}

bool OccupancyGrid::IsOccupied(const double xyz[3]) const
{
  return IsOccupied(ReadValue(xyz));
}

bool OccupancyGrid::IsOccupied(float value) const
{
    return value > threshold_;
}

void OccupancyGrid::UpdateValue(const int ind, float delta)
{
    float value = ReadValue(ind);
    if (value < 0) {WriteValue(ind,0); value =0; }

    bool occupied_before = IsOccupied(ind);
    float new_value = clamp_value(value + delta, 0.0f, 1.0f);
    WriteValue(ind, new_value);
    bool occupied_after = IsOccupied(ind);

    double xyz[3];
    if(!occupied_before && occupied_after)
    {
        IndexToWorld(ind,xyz);
        marked_list_.push_back({xyz[0],xyz[1],xyz[2]});
    }

    if(occupied_before && !occupied_after)
    {
        IndexToWorld(ind,xyz);
        cleared_list_.push_back({xyz[0],xyz[1],xyz[2]});
    }
}

float OccupancyGrid::clamp_value(float x, float min, float max) const
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

}

