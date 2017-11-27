//
// Created by frederik on 11/24/17.
//

#include "sokobanSolver.h"
#include <ctime>

sokobanSolver::sokobanSolver() = default;;

vector<string> sokobanSolver::solve(Graph map) {

    //make openlist
    queue<Step> openList;
    vector<Step> closedList;
    vector<Step> solutionList;

    //make hashtable and hashfunction
    unordered_map<int, vector<int>> hashTable;
    initHashFunction(map.getSize());

    //init current robot pos
    Vertex* currentRoboPos;

    //make Astar ready for map
    aStar = AStar(map);

    //best solution move number
    int bestSolutionRobotMoves = 0;

    //find diamond position and add to openlist && add robot current pos to currentRobotPos
    vector<Vertex>* nodes = map.getNodesPointer();
    vector<Vertex *> diamondPosIndex;
    vector<Vertex *> goals;
    for(int i = 0; i < nodes->size(); i++) {
        if (nodes->operator[](i).pathType == DIAMOND) {
            diamondPosIndex.push_back(&nodes->operator[](i));
        }
        else if (nodes->operator[](i).pathType == GOAL) {
            goals.push_back(&nodes->operator[](i));
        }
        else if (nodes->operator[](i).pathType == START) {
            currentRoboPos = &nodes->operator[](i);
        }
    }

    //save first step
    Step firstStep;
    firstStep.currRoboPos = currentRoboPos;
    firstStep.parent = nullptr; //First step
    firstStep.diamonds = diamondPosIndex;
    openList.push(firstStep);

    //debug info
    int counter = 0;

    //MAIN LOOP
    while (!openList.empty()) {
        //todo remove debug
        if (counter < 1000)
            counter++;
        else {
            cout << "Number of steps in openList: " << openList.size() << endl;
            counter = 0;
        }

        //pop next node to go though
        Step currStep = openList.front();
        openList.pop();

        //put currStep in closedlist
        closedList.push_back(currStep);

        //getReference from closedSet as that is persistant
        currStep = closedList.back();

        //set graph to diamond positions if we are not at root
        if (currStep.parent != nullptr)
            setMaptoSnapshot(&currStep, &map, currStep.parent);

        //set robot pos
        currentRoboPos = currStep.currRoboPos;

        //set AStar to new map
        aStar = AStar(map);

        //for each diamond see what can be pushed
        for (Vertex* curr : currStep.diamonds)
        {
            //What sides can robot go to and push
            vector<SidePush> sides = getPushableSides(curr, currentRoboPos);

            //for pushable sides check if newStep is deadlock
            for (auto s : sides) {
                //DEADLOCK
                if (isDeadlock(s.pushTo))
                    continue;

                //Make move
                Step newStep;
                newStep.parent = &currStep; //Parent is the current step
                newStep.currRoboPos = curr; //robot ends up in diamonds prev location
                newStep.diamonds = newDiamondList(currStep.diamonds, curr, s.pushTo);

                //+1 as we move from side and up to next pos with diamond
                newStep.robotTravelledLength = currStep.robotTravelledLength + s.movePath.size() +1;

                //Check if move is end move. Is all diamonds on goals?
                if (isWinStep(&newStep, goals)) {
                    //save solution in solutionList and set current shortest distance
                    if (newStep.robotTravelledLength < bestSolutionRobotMoves)
                        bestSolutionRobotMoves = newStep.robotTravelledLength;
                    //Todo consider just throwing bad solutions away...
                    //Todo for now just save all solutions
                    solutionList.push_back(newStep);
                } else {
                    //add to openList
                    openList.push(newStep);
                }

                //Already tried move? - check has table
                if (!isMoveNew(&newStep, &hashTable))
                    continue;

                //TODO heuistics ?
            }
        }
    }

    //While is done, if solution exists it is in solutionVector
    if (solutionList.empty())
        cout << "Couldn't find a solution..." << endl;
    else {
        //select best solution and backtrack to create solution for robot
        //sort vector so best solution is
        sort(solutionList.begin(), solutionList.end(), []( const Step& lhs, const Step& rhs)
        {
            // < == ascending order(smallest first). > == descending order.
            return lhs.robotTravelledLength < rhs.robotTravelledLength;
        });

        //Best solution is then first element in vector[0].
        //todo backtrack that solution
        //debug
        cout << endl << endl << "DIAMONDS:" << endl;
        for (auto v : solutionList[0].diamonds) {
            cout << v->data << endl;
        }
        cout << "Total Robot length: " << solutionList[0].robotTravelledLength << endl;
    }
}

bool sokobanSolver::isWinStep(Step* step, vector<Vertex*> goals) {
    for(auto v : step->diamonds) {
        if (find(goals.begin(), goals.end(), v) == goals.end()) //TODO i might not be able to do this...
            return false;
    }
    return true;
}

bool sokobanSolver::isMoveNew(Step * step, unordered_map<int, vector<int>>* hashTable) {
    //get hashkey
    int hashkey = getHashKey(step->diamonds);

    //get pos in hashtable
    auto it = hashTable->find(hashkey);
    if (it == hashTable->end()) {
        //key doesn't exists, move is new
        return true;
    }
    else {
        //check if pos for diamonds is the same
        vector<int> newPos = getDiamondsIndex(step->diamonds);
        vector<int> existingPos = it->second;
        return !isDiamondPosEqual(existingPos, newPos);
    }
}

vector<int> sokobanSolver::getDiamondsIndex(vector<Vertex *> diamonds) {
    vector<int> list;
    for (auto v : diamonds)
        list.push_back(v->index);
    return list;
}

bool sokobanSolver::isDiamondPosEqual(vector<int> d1, vector<int> d2) {
    for(auto v : d1) {
        if (find(d2.begin(), d2.end(), v) == d2.end())
            return false;
    }
    return true;
}

vector<Vertex *> sokobanSolver::newDiamondList(vector<Vertex *> oldList, Vertex *oldPos, Vertex *newPos) {
    vector<Vertex*> newList;

    for (auto v : oldList) {
        if (v->data == oldPos->data)
            continue;
        else
            newList.push_back(v);
    }

    //add new pos
    newList.push_back(newPos);
    return newList;
}

void sokobanSolver::setMaptoSnapshot(Step *snapshot, Graph* map, Step* parent) {
    //reset diamonds form parent
    for (auto v : parent->diamonds) {
        //TODO map->findNode(v->data).pathType = ROAD;
    }

    //set new diamonds
    for (auto v : snapshot->diamonds) {
        //TODO map->findNode(v->data).pathType = DIAMOND;
    }
}

bool sokobanSolver::isDeadlock(Vertex *pos) {
    //check if pos is deadlock. If two sides is blocked it is in deadlock
    Vertex * top = pos->findNeighbour(pos->data+Pixel(0,-1));
    Vertex * left = pos->findNeighbour(pos->data+Pixel(-1,0));
    Vertex * right = pos->findNeighbour(pos->data+Pixel(+1,0));
    Vertex * bottom = pos->findNeighbour(pos->data+Pixel(0,+1));

    //CASE TOP_LEFT
    if (isBlocked(top) && isBlocked(left))
        return true;
    //CASE TOP_RIGHT
    if (isBlocked(top) && isBlocked(right))
        return true;
    //CASE BOT_LEFT
    if (isBlocked(bottom) && isBlocked(left))
        return true;
    //CASE BOT_RIGHT
    if (isBlocked(bottom) && isBlocked(right))
        return true;
}

bool sokobanSolver::isBlocked(Vertex * v) {
    return v == nullptr || v->pathType == WALL || v->pathType == DIAMOND;
}

vector<SidePush> sokobanSolver::getPushableSides(Vertex *currPos, Vertex *currRoboPos) {
    //try to walk to left and up sides. If it can't walk to them then it can't push to them either
    //if it can walk to them try and walk to the other also

    vector<SidePush> pushableSides;

    //TODO consider changing this so it utilize sides positions...
    for (Vertex* v : currPos->adj) {

        //if current adj is wall or diamond skip it
        if (v->pathType == DIAMOND)
            continue;

        vector<Vertex*> path = aStar.searchAStar(currRoboPos, v);
        if (!path.empty()) {
            //if path is not empty robot can go there
            //check if side is pushable
            Pixel deltaP = currPos->data - v->data;
            Pixel newPos = currPos->data + deltaP;

            Vertex* newPosV = v->findNeighbour(newPos);
            if(newPosV != nullptr && newPosV->pathType != DIAMOND) {

                pushableSides.emplace_back(v, newPosV, path);
            }
        }
    }
    return pushableSides;
}

int sokobanSolver::getHashKey(vector<Vertex *> diamonds) {
    int hashKey = 0;
    for(auto v : diamonds) {
        hashKey += hashMap[v->index];
    }
    return hashKey;
}

void sokobanSolver::initHashFunction(int size) {
    srand (time(NULL));
    hashMap.resize(size);
    for(int i = 0; i < size; i++) {
        hashMap[i] = rand();
    }
}