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
    vector<string> plan = solver.solve(*map.getMapGraph());

    cout << "Robot Plan:" << endl;
    for(const auto& p : plan)
    {
        cout << p << ",";
    }
    
    system("pause");
}