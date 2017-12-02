#pragma once
#include<iostream>
#include <list>
#include <stack>
#include <string>
#include <vector>
#include <queue>
#include <algorithm>
#include <climits>
#define INFINITY INT_MAX

#include "Image.hpp"
// Class to represent a graph
using namespace std;
using namespace rw::sensor;

enum PathType {WALL, DIAMOND, GOAL, ROAD, START};

struct Pixel
{
    int x;
    int y;

    Pixel(int x, int y) : x(x), y(y) {};
    Pixel() {};

    int getValue(Image* img)
    {
        return img->getPixelValuei(x, y, 0);
    }


    bool operator==(const Pixel& rhs)
    {
        return this->x == rhs.x && this->y == rhs.y;
    }

	bool operator!=(const Pixel& rhs)
	{
		return this->x != rhs.x || this->y != rhs.y;
	}

    Pixel operator-(const Pixel& rhs) const {
        return Pixel(x-rhs.x, y-rhs.y);
    }

    Pixel operator+(const Pixel& rhs) const {
        return Pixel(x+rhs.x, y+rhs.y);
    }

    friend ostream &operator<<(ostream &os, const Pixel &pixel) {
        os << "x: " << pixel.x << " y: " << pixel.y;
        return os;
    }
};

struct Vertex
{
    PathType pathType;
    Pixel data;
    Vertex* path; // parent
    bool known; //used to find first unvisited vertex when standinging at a point with no unvisted neighbours. Is reset multiple times. Don't rely on this for anything else
    int dist;
	int gScore = INFINITY;
	int fScore = INFINITY;
    vector<int> weight;    
    bool visited;
    int index;
  
    vector<Vertex*> adj; // Pointer to an array containing adjacency lists

    //find adj by position
    Vertex* findNeighbour(Pixel data) {
        for (auto v : adj) { //TODO should it be auto* ?
            if (v->data == data)
                return v;
        }
        return nullptr;
    }

    bool operator==(const Vertex &rhs) const {
        return data.x == rhs.data.x && data.y == rhs.data.y;
    }

    Vertex(Pixel pixel, PathType type, int index) : data(pixel), visited(false), pathType(type), index(index) {};
};


class Graph
{
    friend class Dijkstras;

public:
    Graph();   // Constructor

    void addNode(Vertex v);
    void addEdge(Vertex& parent, Vertex& v, int weight);
    Vertex& findNode(Pixel data);

    vector<Vertex>* getNodesPointer();
    vector<Vertex>& getNodesRef();
    int getSize();

protected:
    vector<Vertex> nodes;
    int totalNodes; // Number of nodes'
};
