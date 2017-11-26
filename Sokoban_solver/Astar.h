#pragma once
#include <climits>
#include <algorithm>
#include <iterator> 
#include <vector>
#include <algorithm>

#define INFINITY INT_MAX

class Graph;
struct Vertex;
using namespace std;

class AStar
{
public:
	AStar();
	AStar(Graph &g);
	vector<Vertex*> searchAStar(Vertex* start, Vertex* goal);
	vector<Vertex*> getPath(Vertex* start,Vertex* current);

	~AStar();
protected:
	int calculateHscore(Vertex* st, Vertex* go);
	int findSmallestFScoreInOpenSet();
	bool isInClosedSet(Vertex* v);
	bool isInOpenSet(Vertex* v);
	int index; // used to store index value - weight and adjecent is stored in pairs
	Graph* graph; // graph 
	vector<Vertex*> closedSet; // closed set
	vector<Vertex*> openSet;
	Vertex* cameFrom; 
	vector<Vertex*> total_path;
};

