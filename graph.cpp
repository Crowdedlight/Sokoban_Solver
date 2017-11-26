#include "graph.h"

Graph::Graph()
{

}

void Graph::addNode(Vertex v)
{
	//v.gScore = ... ; what cost is the different types?
    nodes.push_back(v);
}

void Graph::addEdge(Vertex* parent, Vertex* v, int w)
{
    parent->weight.push_back(w); // parent contains the information of edge weights
    parent->adj.push_back(v);
}

Vertex* Graph::findNode(Pixel pos)
{
    vector<Vertex>* allNodes = getNodesPointer();
    
    for (auto v : *allNodes)
    {
        if (v.data == pos)
            return &v;
    }
}



vector<Vertex>* Graph::getNodesPointer()
{
    return &nodes;
}
