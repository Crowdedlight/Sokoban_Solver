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

    for (auto node : *this->getMapGraph()->getNodesPointer())
    {
        switch (node.pathType)
        {
        case Wall: //Grey
            plot.setPixel8U(node.data.x, node.data.y, 139, 141, 145);
            break;
        case Diamond: //BLUE
            plot.setPixel8U(node.data.x, node.data.y, 0, 0, 255);
            break;
        case Goal: //GREEN
            plot.setPixel8U(node.data.x, node.data.y, 0, 255, 0);
            break;
        case Road: //BLACK
            plot.setPixel8U(node.data.x, node.data.y, 0, 0, 0);
            break;
        case Start: //START
            plot.setPixel8U(node.data.x, node.data.y, 255, 0, 0);
            break;
        default: //NOTHING
            break;
        }        
    }

    //save plot as file
    plot.saveAsPPM("map.ppm");
}

Graph* Map::getMapGraph()
{
    return &mapGraph;
}

Map::~Map()
{
}
