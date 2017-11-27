#include "graph.h"

Graph::Graph()
{

}

void Graph::addNode(Vertex v)
{
	//v.gScore = ... ; what cost is the different types?
    nodes.push_back(v);
    totalNodes++;
}

void Graph::addEdge(Vertex & parent, Vertex & v, int w)
{
    parent.weight.push_back(w); // parent contains the information of edge weights
    parent.adj.push_back(&v);
    cout << "ref_add:  " << &v << endl;
}

Vertex& Graph::findNode(Pixel pos)
{
    vector<Vertex>& allNodes = getNodesRef();
    
    for (auto &node : allNodes) {
        if (node.data == pos) {
            cout << "ref_find: " << &node << endl;
            return node;
        }
    }
}

int Graph::getSize()
{
    return this->totalNodes;
}


vector<Vertex>* Graph::getNodesPointer()
{
    return &nodes;
}

vector<Vertex>& Graph::getNodesRef()
{
    return nodes;
}
