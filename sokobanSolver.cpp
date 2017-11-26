//
// Created by frederik on 11/24/17.
//

#include "sokobanSolver.h"
#include <unordered_map>
#include <ctime>

sokobanSolver::sokobanSolver() {};

vector<string> sokobanSolver::solve(Graph map) {

    //make openlist
    queue<Step> openList;
    vector<Step> closedList;

    //make hashtable and hashfunction
    unordered_map<int, vector<int>> hashTable;
    initHashFunction(map.getSize());

    //init current robot pos
    Vertex* currentRoboPos;

    //make Astar ready for map
    aStar = AStar(map);

    //find diamond position and add to openlist && add robot current pos to currentRobotPos
    vector<Vertex>* nodes = map.getNodesPointer();
    vector<Vertex *> diamondPosIndex;
    for(auto v : *nodes) {
        if (v.pathType == DIAMOND) {
            diamondPosIndex.push_back(&v);
        }
        else if (v.pathType == START) {
            currentRoboPos = &v;
        }
    }

    //save first step
    Step firstStep;
    firstStep.currRoboPos = currentRoboPos;
    firstStep.parent = nullptr; //First step
    firstStep.diamonds = diamondPosIndex;
    openList.push(firstStep);


    //MAIN LOOP
    while (!openList.empty()) {
        //pop next node to go though
        Step currStep = openList.front();
        openList.pop();

        //set graph to diamond positions if we are not at root
        if (currStep.parent != nullptr)
            setMaptoSnapshot(&currStep, &map, currStep.parent);

        //set robot pos
        currentRoboPos = currStep.currRoboPos;

        //for each diamond see what can be pushed
        for (auto curr : currStep.diamonds)
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

                //Check if move is end move. Is all diamonds on goals?
                //todo
                //if win, keep searching for better solutions or just end search and backtrack?

                //Already tried move? - check has table
                if (!isMoveNew(&newStep, &hashTable))
                    continue;

                //TODO heuistics

                //Add step to closed list
                closedList.push_back(newStep);
            }
        }
    }
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
        v->pathType = ROAD;
    }

    //set new diamonds
    for (auto v : snapshot->diamonds) {
        v->pathType = DIAMOND;
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
    for (auto v : currPos->adj) {

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

                pushableSides.push_back(SidePush(v, newPosV));
            }
        }
    }

}

int sokobanSolver::getHashKey(vector<Vertex *> diamonds) {
    int hashKey = 0;
    for(auto v : diamonds) {
        hashKey += hashMap[v->index];
    }
    return hashKey;
}

unordered_map sokobanSolver::initHashFunction(int size) {
    srand (time(NULL));
    hashMap[size];
    for(int i = 0; i < size; i++) {
        hashMap[i] = rand();
    }
}

int sokobanSolver::nextPrime(int start) {
    while (!isPrime(start)){
        start++;
    }
    return start;
}

bool sokobanSolver::isPrime(int num) {
    for(int i = 2; i <= num / 2; ++i)
    {
        if(num % i == 0)
        {
            return false;
        }
    }
    return true;
}