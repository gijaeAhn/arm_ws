#include "Occupancy_Grid.h"
#include "Voxel_Grid.h"


int main()
{
    double world_test[3] = {1.5,1.5,1.5};
    double res_test = 0.1;
    double org_test[3] = {0};

occupancygrid::OccupancyGrid oc(org_test,world_test,res_test);

auto begin = oc.begin();

oc.GetState();
oc.CalcFirstVoxel(org_test);
printf("Print First Voxel ");
oc.printFirstVoxel();


for ( occupancygrid::OccupancyGrid::Iterator it  = oc.begin(); it != oc.end(); ++it){

    if((it.getIndex() % 500) ==0) {

        oc.UpdateValue(it.getIndex(),true);
        printf(" Update \n");
        
    }
}

for ( occupancygrid::OccupancyGrid::Iterator it  = oc.begin(); it != oc.end(); ++it){
    if(*it)
    {
    oc.toMarkList(it.getIndex());
    printf("\n%d\n",*it);
    }

}
oc.printMarkList();
}