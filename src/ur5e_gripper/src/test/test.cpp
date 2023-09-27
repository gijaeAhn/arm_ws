#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>

void change_list(const int reference[3], int target[3]){
    for ( int i =0; i <3 ; i++){
        target[i] = (reference[i] >> 1);
    }
}

int main(){
    int new_list[3] = { 0,0,0};
    int refer_list[3] = { 1, 2, 3};

    change_list(refer_list, new_list);
    for(int i =0 ; i < 3 ; i ++){
        std::cout << new_list[i] << std::endl;
    }

    int i = 0 ;
}

