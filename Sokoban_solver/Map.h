#include "graph.h"
#include <iostream>
#include <fstream>
using namespace std;

class Map
{
public:
    Map();
    Map(string filename);
    void LoadMap(string filename);
    Graph* getMapGraph();
    void plotMap();


    int getWidth();
    int getHeight();
    int getTotalDiamonds();
    
    ~Map();

private:
    void setWidth(int width);
    void setHeight(int height);
    void setTotalDiamonds(int diamonds);
    Graph mapGraph;
    Image mapPlot;
    int width, height, diamonds;
};

