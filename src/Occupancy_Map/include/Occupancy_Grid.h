#pragma once 

#include <array>
#include <vector>
#include <cstring>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <cmath>
#include <assert.h>
#include <cstddef>

#include "Voxel_Grid.h"

#include <pcl/conversions.h>
#include <pcl/point_types.h>



namespace occupancygrid
{

typedef struct Point{
    double data[3];
    ~Point() {}
}Point ;
using PointList = std::vector<Point>;

class OccupancyGrid : public voxelgrid::VoxelGrid<bool>
{
    private:
    PointList Marking_List_;
    PointList Clear_list_;
    float Threshold_;





    public:
    OccupancyGrid();
    OccupancyGrid(const double origin[3], const double world_dimensions[3], const double resolution);
    void UpdateValue(const pcl::PointXYZ& point,bool value);
    void UpdateValue(const double xyz[3], bool value);
    void UpdateValue(const int ixyz[3], bool value);
    void UpdateValue(const int index, bool delta);
    bool OccupancyCheck(const pcl::PointXYZ& point);
    bool OccupancyCheck(const double xyz[3]);
    bool OccupancyCheck(const int ixyz[3]);
    bool OccupancyCheck(const int index);
    void ResetDiffs() { Marking_List_.clear(); Marking_List_.clear(); }
    const PointList& GetMarked() const { return Marking_List_; }
    const PointList& GetCleared() const { return Clear_list_; }
    // float GetThreshold() const  { return Threshold_; }
    // float Clamping(const int num, const float max, const float min);

    void OccupancyCalc(const double xyz[3]);
    void OccupancyCalc(const int ixyz[3]);
    void OccupancyCalc(const int index);

    void toMarkList(const int index);
    void printMarkList();
    
    class Iterator
    {
        private:
        OccupancyGrid& grid_;
        int index_;

        public:
            using iterator_category = std::forward_iterator_tag;
            using value_type = bool;
            using difference_type = std::ptrdiff_t;
            using pointer = float*;
            using reference = float&;

            Iterator(OccupancyGrid& grid, int index) : grid_(grid),index_(index) {}

            bool operator*() {
                return grid_.GetIndexData(index_);
            }

            Iterator& operator++(){
                index_++;
                return *this;
            } 

            bool operator==(const Iterator& other) const {
                return index_ == other.index_;
            }

            bool operator!=(const Iterator& other) const {
                return !(*this ==other);
            }

            int getIndex(){return index_;}
    };

    Iterator begin() {
        return Iterator(*this,0);
    }

    Iterator end() {
        return Iterator(*this,Data_.size());
    }
    

};    
    
} // namespace occupancygrid
