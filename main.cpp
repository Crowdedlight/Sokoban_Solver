#include <iostream>
#include "Map.h"
#include "sokobanSolver.h"

using namespace std;

int main() {
    
    //make Map object
    Map map("../inputMaps/2017-competation-map");

    map.plotMap();

    //make solver
    sokobanSolver solver;
    //try solve
    vector<string> plan = solver.solve(map);

    cout << "Robot Plan:" << endl;
    for(const auto& p : plan)
    {
        cout << p ;//<< ",";
    }
    cout << endl;
    
    system("pause");
}