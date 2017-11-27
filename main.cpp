#include <iostream>
#include "graph.h"
#include "Image.hpp"
#include "Map.h"
#include "sokobanSolver.h"

using namespace std;

int main() {
    
    //make Map object
    Map map("../inputMaps/Sokoban2");

    map.plotMap();

    //make solver
    sokobanSolver solver;
    //try solve
    //solver.solve(*map.getMapGraph());
    
    //system("pause");
}