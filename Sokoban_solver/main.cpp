
#include <iostream>
#include "graph.h"
#include "Image.hpp"
#include "Map.h"

using namespace std;

int main() {
    
    //make Map object
    Map map("inputMap");

    map.plotMap();
    
    system("pause");
}