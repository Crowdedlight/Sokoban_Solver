#include "graph.h"
#include <iostream>
#include <fstream>
#include "graph.h"
#include <vector>
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
    int mapScale = 100;
    int objectScale = mapScale / 5;
    int robotScale = mapScale / 10 + 5;

    void drawCross(Image &img, Vertex& pos);
    void drawCircle(Image &img, Vertex& pos, vector<uint8_t> rgb);

    void setWidth(int width);
    void setHeight(int height);
    void setTotalDiamonds(int diamonds);

    Graph mapGraph;
    Image mapPlot;
    int width, height, diamonds;
};

