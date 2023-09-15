#include <iostream>
#include <vector>
#include <algorithm>
int main(){
    std::vector<double> q;
    std::vector<double> t;
    t.push_back(1);
    t.push_back(2);
    t.push_back(3);
    t.push_back(4);
    t.push_back(5);
    t.push_back(6);
    q.resize(t.size());
    std::copy(t.begin(),t.end(),q.begin());

    std::cout << sizeof(t) <<std::endl;
    for (int i = 0 ; i < sizeof(t); i++){
        std::cout << q[i] << std::endl;
    }

}

