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
#include <bits/unordered_map.h>
#include <algorithm>

struct Step {
    Step* parent;
    vector<Vertex *> diamonds;
    Vertex* currRoboPos;

    Step() {};
    Step(Step *parent, vector<Vertex *> diamonds, Vertex *currRoboPos) : parent(parent), diamonds(diamonds),
                                                                           currRoboPos(currRoboPos) {}
};

struct SidePush {
    Vertex * pushFrom;
    Vertex * pushTo;
    SidePush(Vertex* from, Vertex* to) : pushFrom(from), pushTo(to) {};
    SidePush() {};
};

class sokobanSolver {


public:
    sokobanSolver();
    vector<string> solve(Graph map);

private:
    unordered_map initHashFunction(int size);
    bool isPrime(int num);
    int nextPrime(int start);
    int getHashKey(vector<Vertex*> diamonds);
    int hashMap[];
    AStar aStar;
    vector<SidePush> getPushableSides(Vertex *currPos, Vertex *currRoboPos);
    bool isDeadlock(Vertex * pos);
    bool isBlocked(Vertex * v);
    void setMaptoSnapshot(Step* snapshot, Graph* map, Step * parent);
    vector<Vertex *> newDiamondList(vector<Vertex*> oldList, Vertex* oldPos, Vertex* newPos);
    bool isMoveNew(Step* step, unordered_map<int, vector<int>>* hashTable);
    bool isDiamondPosEqual(vector<int> d1, vector<int> d2);
    vector<int> getDiamondsIndex(vector<Vertex * > diamonds);
};


#endif //SOKOBAN_SOLVER_SOKOBANSOLVER_H
