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

enum PathType {Wall, Diamond, Goal, Road, Start};

struct Pixel
{
    int x;
    int y;

    Pixel(int x, int y) : x(x), y(y) {}
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
  
    list<Vertex*> adj; // Pointer to an array containing adjacency lists

    Vertex(Pixel pixel, PathType type) : data(pixel), visited(false), pathType(type) {};
};


class Graph
{
    friend class Dijkstras;

public:
    Graph();   // Constructor

    void addNode(Vertex v);
    void addEdge(Vertex* parent, Vertex* v, int weight);
    Vertex* findNode(Pixel data);

    vector<Vertex>* getNodesPointer();


protected:
    vector<Vertex> nodes;
    int V; // Number of vertices'
};
