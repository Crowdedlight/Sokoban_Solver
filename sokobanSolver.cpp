//
// Created by frederik on 11/24/17.
//

#include "sokobanSolver.h"
#include <ctime>

sokobanSolver::sokobanSolver() = default;;

vector<string> sokobanSolver::solve(Graph map) {

    //make openlist
    queue<Step> openList;
    list<Step> closedList;
    list<Step> solutionList;

    //make hashtable and hashfunction
    unordered_map<int, vector<vector<int>>> hashTable;
    initHashFunction(map.getSize());

    //init current robot pos
    Vertex* currentRoboPos;

    //make Astar ready for map
    aStar = AStar(map);

    //best solution move number
    int bestSolutionRobotMoves = INFINITY;

    //find diamond position and add to openlist && add robot current pos to currentRobotPos
    vector<Vertex>& nodes = map.getNodesRef();
    vector<Vertex *> diamondPosIndex;
    vector<Vertex *> goals;
    for(Vertex & node : nodes ) {
        if (node.pathType == DIAMOND) {
            diamondPosIndex.push_back(&node);
        }
        else if (node.pathType == GOAL) {
            goals.push_back(&node);
        }
        else if (node.pathType == START) {
            currentRoboPos = &node;
        }
    }

    //save first step
    Step firstStep;
    firstStep.finishedRoboPos = currentRoboPos;
    firstStep.robotStartPosition = currentRoboPos;
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
        Step nextStep = openList.front();
        openList.pop();

        //put currStep in closedlist
        closedList.push_back(nextStep);

        //getReference from closedSet as that is persistant
        Step& currStep = closedList.back();

        //set graph to diamond positions if we are not at root
        if (currStep.parent != nullptr)
            setMaptoSnapshot(currStep, &map);

        //set robot pos
        currentRoboPos = currStep.finishedRoboPos;

        //for each diamond see what can be pushed
        for (Vertex* curr : currStep.diamonds)
        {
            //todo again.. the reference issues
            Vertex & curr_real = map.getNodesRef()[curr->index];

            //What sides can robot go to and push
            vector<SidePush> sides = getPushableSides(curr_real, *currentRoboPos, map);

            //for pushable sides check if newStep is deadlock
            for (auto s : sides) {
                //DEADLOCK
                if (isDeadlock(s.pushTo))
                    continue;

                //Make move
                Step newStep;
                newStep.parent = &currStep; //Parent is the current step
                newStep.finishedRoboPos = &curr_real; //robot ends up in diamonds prev location todo as curr & currPos is not correct refs this is maybe where the problem originates, as then roboPos later on will be wrong ref
                newStep.diamonds = newDiamondList(currStep.diamonds, &curr_real, s.pushTo);
                newStep.movePath = s.movePath;
                newStep.robotStartPosition = currentRoboPos;

                //+1 as we move from side and up to next pos with diamond
                newStep.robotTravelledLength = currStep.robotTravelledLength + s.movePath.size() +1; //todo not counting correctly exactly

                //Already tried move? - check hashtable - If new it also saves in hashtable
                if (!isMoveNew(&newStep, hashTable))
                    continue;

                //Check if move is end move. Is all diamonds on goals?
                if (isWinStep(&newStep)) {
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
            }
        }
    }

    cout << endl << "solution list" << endl;
    for (auto s : solutionList)
    {
        cout << "diamond1:" << s.diamonds[0]->data << ", diamond2:" << s.diamonds[1]->data << endl;
    }

    //While is done, if solution exists it is in solutionVector
    if (solutionList.empty())
    {
        cout << "Couldn't find a solution..." << endl;
    }
    else {
        //select best solution and backtrack to create solution for robot
        //sort vector so best solution is
        solutionList.sort([] (const Step & lhs, const Step & rhs) {
            // < == ascending order(smallest first). > == descending order.
            return lhs.robotTravelledLength < rhs.robotTravelledLength;
        });

        //Best solution is then first element in vector[0]. (Check if that is true...)
        //todo backtrack that solution


        //debug
        cout << endl << endl << "DIAMONDS:" << endl;
        for (auto v : solutionList.front().diamonds) {
            cout << v->data << endl;
        }
        cout << "Total Robot length: " << solutionList.front().robotTravelledLength << endl;
    }

    return getRobotPlan(solutionList.front());
}

vector<string> sokobanSolver::getRobotPlan(Step &solution) {
    vector<string> plan;
    string robDirection = "N";

    vector<Step> solutionActions;
    while (true)
    {
        //break on last pos without including it as no changes happen in first step.
        if (solution.parent == nullptr)
            break;

        //add current to vector and set new solution
        solutionActions.insert(solutionActions.begin(), solution);

        //set new solution
        solution = *solution.parent;
    }

    //go though vector of steps and write out path for robot
    for(auto& st : solutionActions)
    {
        //get moves from current position of robot to position before pushing diamond
        vector<string> movePath = getRobotMovesFromPath(st.movePath, robDirection);
        //push back on plan
        for(const auto& p : movePath)
            plan.push_back(p);

        //rotate to face diamond getting pushed. currRoboPos is where the pushed diamond was, and where the robot will end
        Pixel delta;
        if (st.movePath.empty())
            delta = st.finishedRoboPos->data - st.robotStartPosition->data;
        else
            delta = st.finishedRoboPos->data - st.movePath.back()->data;

        string targetDir = dirToNextPoint(delta);
        vector<string> rotToTarget = getRotateToDir(targetDir, robDirection);

        for(const auto& rot : rotToTarget)
            plan.push_back(rot);

        //robot now is in target dir
        robDirection = targetDir;

        //push diamond one forward. (Pushing diamond one forward is indicated with "D" instead of "F")
        plan.emplace_back("D");
    }
    return plan;
}

vector<string> sokobanSolver::getRobotMovesFromPath(vector<Vertex *> path, string& startDir) {
    vector<string> pathList;

    //null check - As in now needing to move before pushing diamond
    if (path.empty())
        return pathList;

    for(int i = 0; i < path.size() -1; i++)
    {
        //what way is next point
        Pixel delta = path[i+1]->data - path[i]->data;
        string targetDir = dirToNextPoint(delta);

        //get rotations to turn around in right direction
        vector<string> rotations = getRotateToDir(targetDir, startDir);
            for (const auto& rot : rotations)
                pathList.push_back(rot);

        //set startDir to the targetDir, as the robot now is facing a different way
        startDir = targetDir;

        //add a forward as we now have right orientation and need to move one forward
        pathList.emplace_back("F");
    }

    return pathList;
}

string sokobanSolver::dirToNextPoint(Pixel delta) {
    if (delta == Pixel(0,1)) //DOWN
    {
        return "S";
    }
    else if (delta == Pixel(1,0)) //RIGHT
    {
        return "E";
    }
    else if (delta == Pixel(-1,0)) //LEFT
    {
        return "W";
    }
    else if (delta == Pixel(0,-1)) //UP
    {
        return "N";
    }
}

//check what way is fastest to rotate and return vector with commands for that
vector<string> sokobanSolver::getRotateToDir(string targetDir, string startDir) {
    vector<string> rotationsCCW;
    vector<string> rotationsCW;
    string currCCW = startDir;
    string currCW = startDir;

    while (targetDir != currCCW) {
        currCCW = rotateDir(currCCW, "CCW");
        rotationsCCW.emplace_back("CCW");
    }

    while (targetDir != currCW)
    {
        currCW = rotateDir(currCW, "CW");
        rotationsCW.emplace_back("CW");
    }

    if(rotationsCCW.size() <= rotationsCW.size())
        return rotationsCCW;
    else
        return rotationsCW;
}

string sokobanSolver::rotateDir(string curr, string rot) {
    if (rot == "CCW") {
        if (curr == "N")
            return "W";
        else if (curr == "W")
            return "S";
        else if (curr == "S")
            return "E";
        else if (curr == "E")
            return "N";
    }
    else //CW
    {
        if (curr == "N")
            return "E";
        else if (curr == "E")
            return "S";
        else if (curr == "S")
            return "W";
        else if (curr == "W")
            return "N";
    }
}

bool sokobanSolver::isWinStep(Step* step) {
    //if no diamonds are on anything else than "goal" type we have a solution
    for(auto& v : step->diamonds) {
        if (v->pathType != GOAL)
            return false;
    }
    return true;
}

bool sokobanSolver::isMoveNew(Step * step, unordered_map<int, vector<vector<int>>>& hashTable) {
    //get hashkey
    int hashkey = getHashKey(step->diamonds);
    vector<int> newPos = getDiamondsIndex(step->diamonds);

    //add robotPos to vector
    newPos.push_back(step->finishedRoboPos->index);

    //exists in table?
    auto& debug = hashTable[hashkey];
    if (hashTable[hashkey].empty()) {
        //key doesn't exists, move is new and is first move on this pos
        vector<vector<int>> newEntry;
        newEntry.push_back(newPos);
        hashTable[hashkey] =  newEntry;
        return true;
    }
    else {
        //check if pos for diamonds is the same
        const vector<vector<int>>& existingPos = hashTable[hashkey];

        for(const auto &pos : existingPos)
        {
            //if equal move is not new. Also checks for robo pos
            if (isDiamondPosEqual(pos, newPos))
                return false;
        }

        //if passed all values and is still unmatched, then insert new pos & return true;
        hashTable[hashkey].push_back(newPos);
        return true;
    }
}

vector<int> sokobanSolver::getDiamondsIndex(vector<Vertex *> diamonds) {
    vector<int> list;
    for (auto& v : diamonds)
        list.push_back(v->index);
    return list;
}

bool sokobanSolver::isDiamondPosEqual(vector<int> d1, vector<int> d2) {
    for(auto& v : d1) {
        if (find(d2.begin(), d2.end(), v) == d2.end())
            return false;
    }
    return true;
}

vector<Vertex *> sokobanSolver::newDiamondList(vector<Vertex *> oldList, Vertex *oldPos, Vertex *newPos) {
    vector<Vertex*> newList;

    for (auto* v : oldList) {
        if (v->data == oldPos->data)
            continue;
        else
            newList.push_back(v);
    }

    //add new pos
    newList.push_back(newPos);
    return newList;
}

void sokobanSolver::setMaptoSnapshot(Step& snapshot, Graph* map) {
    //reset diamonds form parent
    for (Vertex& v : map->getNodesRef()) {
        if (v.pathType == DIAMOND)
            v.pathType = ROAD;
    }

    //set new diamonds
    for (Vertex* v : snapshot.diamonds) {
        //TODO if I do this directly on v it doesn't save in map. Even though v should be a pointer to the vertex in map
        map->getNodesRef()[v->index].pathType = DIAMOND;
    }
}

bool sokobanSolver::isDeadlock(Vertex* newPos) {
    //check if pos is deadlock. If two sides is blocked it is in deadlock
    Vertex * top = newPos->findNeighbour(newPos->data+Pixel(0,-1));
    Vertex * left = newPos->findNeighbour(newPos->data+Pixel(-1,0));
    Vertex * right = newPos->findNeighbour(newPos->data+Pixel(+1,0));
    Vertex * bottom = newPos->findNeighbour(newPos->data+Pixel(0,+1));

    //never deadlock when target is a goal
    if (newPos->pathType == GOAL)
        return false;

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

    //if all has been zero return false
    return false;
}

bool sokobanSolver::isBlocked(Vertex * v) {
    return v == nullptr || v->pathType == WALL; //TODO can we make deadlock with neighbour diamonds? We could have cases where we got temp deadlock but then move the one diamond and result the deadlock || v->pathType == DIAMOND;
}

vector<SidePush> sokobanSolver::getPushableSides(Vertex& currPos, Vertex& currRoboPos, Graph& map) {
    //try to walk to left and up sides. If it can't walk to them then it can't push to them either
    //if it can walk to them try and walk to the other also

    vector<SidePush> pushableSides;

    //TODO consider changing this so it utilize sides positions...
    for (auto v : currPos.adj) {

        //get real reference due to issues with auto v : adj
        Vertex & v_real = map.getNodesRef()[v->index];

        //if current adj is wall or diamond skip it
        if (v_real.pathType == DIAMOND)
            continue;

        vector<Vertex*> path = aStar.searchAStar(currRoboPos, v_real);
        if (!path.empty()) {
            //if path is not empty robot can go there
            //check if side is pushable
            Pixel deltaP = currPos.data - v->data;
            Pixel newPos = currPos.data + deltaP;

            Vertex* newPosV = currPos.findNeighbour(newPos);
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
    //srand (time(nullptr));
    hashMap.resize(size);
    for(int i = 0; i < size; i++) {
        hashMap[i] = rand();
    }
}