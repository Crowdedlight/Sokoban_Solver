#ifndef SOKOBAN_SOLVER_MAP_H
#define SOKOBAN_SOLVER_MAP_H

#include <iostream>
#include <fstream>
#include "graph.h"
#include <vector>
#include <cmath>
using namespace std;

class Map
{
public:
    Map();
    Map(string filename);
    void LoadMap(string filename);
    Graph* getMapGraph();

    void plotMap();
    void plotDeadlocks(vector<Pixel>& deadlocks);

    int getWidth();
    int getHeight();
    int getTotalDiamonds();
    
    ~Map();

private:
    int mapScale = 100;
    int objectScale = mapScale / 5;
    int robotScale = mapScale / 10 + 5;

    void drawCross(Image &img, Vertex& pos);
    void drawCircle(Image &img, Vertex& pos, vector<uint8_t> rgb, int scale);
    void drawSquare(Image &img, Pixel pos, vector<uint8_t> rgb);

    void setWidth(int width);
    void setHeight(int height);
    void setTotalDiamonds(int diamonds);

    Graph mapGraph;
    Image* mapPlot;
    Image* mapPlotDeadlocks;
    int width, height, diamonds;
};

#endif //SOKOBAN_SOLVER_MAP_H