#include <iostream>
#include <cmath>
#include <list>
#include <algorithm>
#include <random>
#include <chrono>
#include <vector>
#include <stdio.h>

using namespace std;
int main (){
    list<double> a;
    a.push_back(1);
    a.push_back(12);
    a.push_back(11);
    a.push_back(15);
    a.push_back(17);
    for(auto& l :a){
        cout << l<< endl;
    }
    a.sort();
     cout << "------------"<< endl;
    for(auto& l :a){
        cout << l<< endl;
    }
    auto it = lower_bound(a.begin(), a.end(), -5);
    a.insert(it, -5);
     cout << "------------"<< endl;
    for(auto& l :a){
        cout << l<< endl;
    }
}