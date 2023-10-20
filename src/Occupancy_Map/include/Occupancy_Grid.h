#pragma once 

#include <array>
#include <vector>
#include <cstring>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <cmath>
#include <assert.h>

#include "Voxel_Grid.h"


namespace occupancygrid
{

using Point = std::array<double,3>;
using PointList = std::vector<Point>;

class OccupancyGrid : public voxelgrid::VoxelGrid<float>
{
    private:
    PointList Marking_List_;
    PointList Clear_list_;
    float Threshold_;





    public:
    OccupancyGrid(const double origin[3], const double world_dimensions[3], const double resolution, const double threshold);
    void UpdateValue(const double xyz[3], float value);
    void UpdateValue(const int ixyz[3], float value);
    void UpdateValue(const int index, float delta);
    bool OccupancyCheck(const double xyz[3]);
    bool OccupancyCheck(const int ixyz[3]);
    bool OccupancyCheck(const int index);
    void ResetDiffs() { Marking_List_.clear(); Marking_List_.clear(); }
    const PointList& GetMarked() const { return Marking_List_; }
    const PointList& GetCleared() const { return Clear_list_; }
    float GetThreshold() const  { return Threshold_; }
    float Clamping(const int num, const float max, const float min);

    void OccupancyCalc(const double xyz[3]);
    void OccupancyCalc(const int ixyz[3]);
    void OccupancyCalc(const int index);

};    
    
} // namespace occupancygrid
