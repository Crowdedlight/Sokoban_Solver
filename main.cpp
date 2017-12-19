#include <iostream>
#include <fstream>
#include "Map.h"
#include "sokobanSolver.h"

using namespace std;

int main() {
    
    //make Map object
    //Map map("../inputMaps/2017-competation-map");
    Map map("../inputMaps/Sokoban2");

    map.plotMap();

    //make solver
    sokobanSolver solver;
    //try solve
    vector<string> plan = solver.solve(map);

    ofstream out;
    out.open("../output_path/outPlan.txt");
    cout << "Robot Plan:" << endl;
    out << "Robot Plan:" << endl;
    for(const auto& p : plan)
    {
        cout << p ;//<< ",";
        out << p ;//<< ",";
    }
    cout << endl;
    out << endl;
    out.close();
    system("pause");
}