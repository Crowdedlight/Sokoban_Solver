//
// Created by frederik on 11/24/17.
//
#ifndef SOKOBAN_SOLVER_SOKOBANSOLVER_H
#define SOKOBAN_SOLVER_SOKOBANSOLVER_H

#include "Astar.h"
#include "graph.h"
#include <string>
#include <iostream>
#include <utility>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <list>

struct Step {
    Step* parent;
    vector<Vertex *> diamonds;
    Vertex* currRoboPos;

    int robotTravelledLength = 0;

    Step() = default;
    Step(Step *parent, vector<Vertex *> diamonds, Vertex *currRoboPos) : parent(parent), diamonds(std::move(diamonds)),
                                                                           currRoboPos(currRoboPos) {};
};

struct SidePush {
    Vertex * pushFrom;
    Vertex * pushTo;
    vector<Vertex*> movePath;

    SidePush(Vertex* from, Vertex* to, vector<Vertex*> path) : pushFrom(from), pushTo(to), movePath(std::move(path)) {};
    SidePush() = default;;
};

class sokobanSolver {


public:
    sokobanSolver();
    vector<string> solve(Graph map);

private:
    void initHashFunction(int size);
    int getHashKey(vector<Vertex*> diamonds);
    AStar aStar;
    vector<int> hashMap;

    vector<SidePush> getPushableSides(Vertex *currPos, Vertex *currRoboPos);
    vector<Vertex *> newDiamondList(vector<Vertex*> oldList, Vertex* oldPos, Vertex* newPos);
    vector<int> getDiamondsIndex(vector<Vertex * > diamonds);

    void setMaptoSnapshot(Step& snapshot, Graph* map);

    bool isDeadlock(Vertex * newPos);
    bool isBlocked(Vertex * v);
    bool isMoveNew(Step* step, unordered_map<int, vector<vector<int>>>* hashTable);
    bool isDiamondPosEqual(vector<int> d1, vector<int> d2);
    bool isWinStep(Step*, vector<Vertex*> goals);
};


#endif //SOKOBAN_SOLVER_SOKOBANSOLVER_H
