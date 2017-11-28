#include "AStar.h"
#include "graph.h"


AStar::AStar() {}

AStar::AStar(Graph &g)
{
	graph = &g; // graph is not copied over in this program
}

int AStar::calculateHscore(Vertex* st, Vertex* go)
{
	int H = (abs(st->data.x - go->data.x) + abs(st->data.y - go->data.y));
	return H;
}

int AStar::findSmallestFScoreInOpenSet() // return the smallest distance vector
{
	int num = INFINITY;
	int index = 0;
	for (int i = 0; i < openSet.size(); i++)
	{
		if (openSet[i]->fScore < num)
		{
			num = openSet[i]->fScore;
			index = i;
		}
	}
	return index;
}

bool AStar::isInClosedSet(Vertex* v)
{
	for (auto cl : closedSet)
	{
		if ((cl->data.x == v->data.x) && (cl->data.y == v->data.y))
		{
			return true;
		}
	}
	return false;
}

bool AStar::isInOpenSet(Vertex* v)
{
	for (auto cl : openSet)
	{
		if ((cl->data.x == v->data.x) && (cl->data.y == v->data.y))
		{
			return true;
		}
	}
	return false;
}

vector<Vertex*> AStar::getPath(Vertex* start, Vertex* curr)
{	
	if (start->data == curr->data)
    {
		return total_path;
    }
    else
    {
		getPath(start, curr->path);
		total_path.push_back(curr);
    }
	return total_path;
}

vector<Vertex*> AStar::searchAStar(Vertex* start, Vertex* goal)
{
	total_path.clear(); // TODO shouldn't this be cleared?

	closedSet.clear(); // // closedSet is the set of nodes already evaluated

	openSet.push_back(start); // push start to open set

	openSet.back()->gScore = 0; // The cost of going from start to start is zero.
    
    cameFrom = NULL;

	// For each node, the total cost of getting from the start node to the goal
	// by passing by that node. That value is partly known, partly heuristic.

	// For the first node, that value is completely heuristic.
	openSet.back()->fScore = calculateHscore(start, goal);

	int c = 0; // used to keep tract og current
	int tmp; // used to keep tract of index number

	while (!openSet.empty()) //while openSet is not empty
	{
		c = findSmallestFScoreInOpenSet(); // return the integer index value for openSet vector position
 		Vertex* current = openSet[c]; // set the current node with smallest FScore

		// if current is goal, then append the goal with updated parent information
		// and then find the total path and return it to robot
		if (current->data == goal->data)
		{
			total_path.push_back(start); //TODO should this be goal?
			//goal->path = cameFrom;
			//goal->path = cameFrom;
			total_path = getPath(start, goal);
			return total_path; 
		}

		openSet.erase(openSet.begin()+c); // delete the current nodes from openSet
		closedSet.push_back(current); // and push it to the closedSet

		for (auto w : current->adj) // check all neighbors
		{
            //if type is diamond we cant go though it
            //APPLICATION SPECIFIC
            if (w->pathType == DIAMOND)
                continue;

			if (!(isInClosedSet(w))) // if not on closelist proceed..
			{
				/// THIS IS USED TO FIND THE POSITION IN VECTOR vvvvvvvvv
				auto it = find(current->adj.begin(), current->adj.end(), w);
				if (it == current->adj.end())
                    cout << "not in vector " << endl;
				else
                {
					int i = distance(current->adj.begin(), it);
					tmp = i;
                }/// THIS IS USED TO FIND THE POSITION IN VECTOR ^^^^^^^^

				if (!(isInOpenSet(w)))
				{
					openSet.push_back(w);
				}
                
                int tentative_gScore = current->gScore + current->weight[tmp];
                bool better_tentive;
                
                if ((tentative_gScore < w->gScore))
				{
					//cout << "tentative Gscore lower than neigbor.. do nothing.." << endl;
                    cameFrom = current;
                    w->path = current;
                    w->gScore = tentative_gScore;
                    w->fScore = w->gScore + calculateHscore(w, goal);
				}
				
			} // else skip that one

		}
	}
    return total_path;
}

AStar::~AStar()
{
}


