//
// Created by frederik on 11/24/17.
//
#ifndef SOKOBAN_SOLVER_SOKOBANSOLVER_H
#define SOKOBAN_SOLVER_SOKOBANSOLVER_H

#include "Astar.h"
#include "graph.h"
#include "Map.h"
#include <string>
#include <iostream>
#include <utility>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <list>

struct Step {
    Step* parent = nullptr;
    vector<Vertex *> diamonds;
    Vertex* finishedRoboPos;
    vector<Vertex*> movePath;
    Vertex* robotStartPosition;
    int distanceToClosestGoal = INFINITY;

    int robotTravelledLength = 0;

    Step() = default;
    Step(Step *parent, vector<Vertex *> diamonds, Vertex *currRoboPos) : parent(parent), diamonds(diamonds),
                                                                           finishedRoboPos(currRoboPos) {};
};

struct SidePush {
    Vertex * pushFrom;
    Vertex * pushTo;
    vector<Vertex*> movePath;

    SidePush(Vertex* from, Vertex* to, vector<Vertex*> path) : pushFrom(from), pushTo(to), movePath(std::move(path)) {};
    SidePush() = default;;
};

struct Move {
    vector<int> diamonds;
    int robopos;
    int length;

    Move() = default;

    Move(const vector<int> &diamonds, int robopos, int length) : diamonds(diamonds), robopos(robopos),
                                                                       length(length) {};

    bool operator==(const Move &rhs) const {
        return diamonds == rhs.diamonds &&
               robopos == rhs.robopos &&
               length == rhs.length;
    }
};

class sokobanSolver {


public:
    sokobanSolver();
    vector<string> solve(Map& graph);

private:
    AStar aStar;
    vector<int> hashMap;
    vector<Pixel> deadlocks;

    void initHashFunction(int size);
    int getHashKey(Move move);
    int getHashKey(Step &step);
    int getHeuristics(Pixel from, Pixel to);
    Pixel getClosestGoal(Pixel currPos, vector<Vertex *> goals);
    Pixel getClosestGoalWithNoDiamond(Pixel currPos, vector<Vertex *> goals, Graph& map);

    vector<SidePush> getPushableSides(Vertex & currPos, Vertex & currRoboPos, Graph& map);
    vector<Vertex *> newDiamondList(vector<Vertex*> oldList, Vertex* oldPos, Vertex* newPos);
    vector<int> getDiamondsIndex(vector<Vertex * > diamonds);
    vector<string> getRobotPlan(Step & solution);
    vector<string> getRobotMovesFromPath(vector<Vertex*> path, string& startDir, vector<string>& debugPath);
    vector<string> getRotateToDir(string targetDir, string startDir);
    string rotateDir(string curr, string rot);
    string dirToNextPoint(Pixel delta);

    void setMaptoSnapshot(Step& snapshot, Graph* map);
    void initMapDeadlocks(Map& map, int width, int height);

    bool isDeadlock(Vertex * newPos);
    bool isBlocked(Vertex& v);
    bool isMoveNew(Step* step, unordered_map<int, vector<Move>>& hashTable);
    bool isWinStep(Step*, vector<Vertex*> goals);
};


#endif //SOKOBAN_SOLVER_SOKOBANSOLVER_H
