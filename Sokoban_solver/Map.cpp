#include "Map.h"



Map::Map()
{
}

Map::Map(string filename)
{
    this->LoadMap(filename);
}

//load txt file and save as a graph
void Map::LoadMap(string filename)
{
    //Init graph object
    Graph map;

    //open txt file stream
    string line;
    ifstream myfile(filename + ".txt");
    if (myfile.is_open())
    {
        //get first line and the settings in it
        getline(myfile, line);

        int width = stoi(line.substr(0,2));
        int height = stoi(line.substr(3, 2));;
        int diamonds = stoi(line.substr(6, 2));;

        this->setWidth(width);
        this->setHeight(height);
        this->setTotalDiamonds(diamonds);

        int rowNum = 0;
        //get rest of lines and make vertexes and add to map
        while (getline(myfile, line))
        {
            //go though each char
            for (auto i = 0; i < line.size(); i++)
            {
                //switch case what is char?
                switch (line[i])
                {
                case 'X': //WALL

                    //new vertex
                    map.addNode(Vertex(Pixel(i, rowNum), Wall));

                    break;
                case 'J': //DIAMOND

                    //new vertex
                    map.addNode(Vertex(Pixel(i, rowNum), Diamond));

                    break;
                case 'G': //GOAL

                    //new vertex
                    map.addNode(Vertex(Pixel(i, rowNum), Goal));

                    break;
                case '.': //ROAD

                    //new vertex
                    map.addNode(Vertex(Pixel(i, rowNum), Road));

                    break;
                case 'M': //START POSITION

                    //new vertex
                    map.addNode(Vertex(Pixel(i, rowNum), Start));

                    break;
                case ' ': // OUTSIDE MAP
                    //do nothing
                    break;
                default:
                    break;
                }
            }
            rowNum++;
        }
        myfile.close();
    }
    else
    {
        cout << "Unable to open file";
        return;
    }

    //go though each node in map and add adj
    vector<Vertex>* allNodes = map.getNodesPointer();
    for (auto node : *allNodes)
    {
        //4 cases, up, down, left and right. 
        for (int i = -1; i <= 1; i ++)
        {
            for (int j = -1; j <= 1; j ++)
            {                
                Pixel adj(node.data.x + i, node.data.y + j);

                //check map bounds
                if (adj.x < 0 || adj.x > this->getWidth() || adj.y < 0 || adj.y > this->getHeight())
                    continue;

                //skip my current pos
                if (adj == node.data)
                    continue;

                //find node that matches
                Vertex* adjV = map.findNode(adj);
                //add edge if adj is not wall && Current node is not wall
                if (adjV != nullptr && adjV->pathType != Wall && node.pathType != Wall)
                {
                    //depending on type change weight - TODO
                    map.addEdge(&node, adjV, 1);
                }
            }
        }
    }

    //save map
    this->mapGraph = map;
}

void Map::setWidth(int width)
{
    this->width = width;
}
void Map::setHeight(int height)
{
    this->height = height;
}
void Map::setTotalDiamonds(int diamonds)
{
    this->diamonds = diamonds;
}

int Map::getWidth()
{
    return this->width;
}
int Map::getHeight()
{
    return this->height;
}
int Map::getTotalDiamonds()
{
    return this->diamonds;
}

void Map::plotMap()
{
    //create image
    Image plot(this->getWidth(), this->getHeight(), Image::RGB, Image::Depth8U);

    //draw entire image black
    for (int x = 0; x < plot.getWidth(); x++) {
        for (int y = 0; y < plot.getHeight(); y++) {
            plot.setPixel8U(x,y,0,0,0);
        }
    }

    for (auto node : *this->getMapGraph()->getNodesPointer())
    {
        switch (node.pathType)
        {
        case Wall: //BLACK
            //plot.setPixel8U(node.data.x, node.data.y, 139, 141, 145);
            break;
        case Diamond: //BLUE
            //plot.setPixel8U(node.data.x, node.data.y, 0, 0, 255);
            //drawCircle(plot, node, {0,0,255});
            break;
        case Goal: //GREEN
            //plot.setPixel8U(node.data.x, node.data.y, 0, 255, 0);
            //drawCircle(plot, node, {0,255,0});
            break;
        case Road: //Grey with black cross
            //plot.setPixel8U(node.data.x, node.data.y, 0, 0, 0);
            drawCross(plot, node);
            break;
        case Start: //START - Robot
            //plot.setPixel8U(node.data.x, node.data.y, 255, 0, 0);
            //drawCircle(plot, node, {255,0,0});
            break;
        default: //NOTHING
            break;
        }        
    }

    //save plot as file
    plot.saveAsPPM("map.ppm");
}

void Map::drawCross(Image &img, Vertex &pos)
{
    int x_curr = (pos.data.x*mapScale);
    int y_curr = (pos.data.y*mapScale);
    int x_mid = x_curr + (mapScale/2);
    int y_mid = y_curr + (mapScale/2);

    //draw entire "pixel area" grey
    for (int x = x_curr; x < x_curr+mapScale; x++) {
        for (int y = y_curr; y < y_curr+mapScale; y++) {
            img.setPixel8U(x, y, 139, 141, 145);
        }
    }

    //draw black cross
    for (int x = x_mid-(mapScale/2); x < x_mid+(mapScale/2); x++) {
        img.setPixel8U(x, y_mid, 0, 0, 0);
    }
    for (int y = y_mid-(mapScale/2); y < y_mid+(mapScale/2); y++) {
        img.setPixel8U(x_mid, y, 0, 0, 0);
    }
}

void Map::drawCircle(Image &img, Vertex &pos, vector<uint8_t> rgb)
{
    int x_curr = pos.data.x;
    int y_curr = pos.data.y;
    int x_mid = x_curr + (mapScale/2);
    int y_mid = y_curr + (mapScale/2);

    for (int x = x_mid-(mapScale/2); x < x_mid+(mapScale/2); x++) {
        for (int y = y_mid-(mapScale/2); y < y_mid+(mapScale/2); y++) {

            double radius = sqrt(pow(x-x_mid,2)+pow(y-y_mid,2));
            if (radius <= (mapScale/2)) {
                img.setPixel8U(x, y, rgb[0], rgb[1], rgb[2]);
            }
        }
    }
}

Graph* Map::getMapGraph()
{
    return &mapGraph;
}

Map::~Map()
{
}
