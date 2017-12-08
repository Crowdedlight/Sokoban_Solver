//
// Created by frederik on 11/24/17.
//

#include "sokobanSolver.h"
#include <ctime>

sokobanSolver::sokobanSolver() = default;;

vector<string> sokobanSolver::solve(Map& map) {

    //get graph
    Graph& graph = *map.getMapGraph();

    //make openlist
    list<Step> openList;
    list<Step> closedList;
    list<Step> solutionList;

    //make hashtable and hashfunction
    unordered_map<int, vector<Move>> hashTable;
    initHashFunction(graph.getSize());

    //init current robot pos
    Vertex* currentRoboPos;

    //make Astar ready for map
    aStar = AStar(graph);

    //best solution move number
    auto bestSolutionRobotMoves = INFINITY;

    //find diamond position and add to openlist && add robot current pos to currentRobotPos
    vector<Vertex>& nodes = graph.getNodesRef();
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

    //init deadlocks
    initMapDeadlocks(map, map.getWidth(), map.getHeight());

    //save first step
    Step firstStep;
    firstStep.finishedRoboPos = currentRoboPos;
    firstStep.robotStartPosition = currentRoboPos;
    firstStep.parent = nullptr; //First step
    firstStep.diamonds = diamondPosIndex;
    openList.push_back(firstStep);

    //debug info
    int counter = 0;
    int bestDistanceToDimond = INFINITY;

    //MAIN LOOP
    while (!openList.empty()) {
        //todo remove debug
        if (counter < 4000)
            counter++;
        else {
            cout << "Number of steps in openList: " << openList.size() << endl;
            counter = 0;
        }

        //before popping next node sort list for heuristic
//        openList.sort([] (const Step & lhs, const Step & rhs) {
//            // < == ascending order(smallest first). > == descending order.
//            return lhs.distanceToClosestGoal < rhs.distanceToClosestGoal;
//        });

        //pop next node to go though
        Step nextStep = openList.front();
        openList.pop_front();

        //put currStep in closedlist
        closedList.push_back(nextStep);

        //if current branch is less than the best solution close it off, still saved in closedlist
        if (nextStep.robotTravelledLength > bestSolutionRobotMoves)
            continue;

        //getReference from closedSet as that is persistant
        Step& currStep = closedList.back();

        //set graph to diamond positions if we are not at root
        if (currStep.parent != nullptr)
            setMaptoSnapshot(currStep, &graph);

        //set robot pos
        currentRoboPos = currStep.finishedRoboPos;

        //for each diamond see what can be pushed
        for (Vertex* curr : currStep.diamonds)
        {
            //todo again.. the reference issues
            Vertex & curr_real = graph.getNodesRef()[curr->index];

            //What sides can robot go to and push
            vector<SidePush> sides = getPushableSides(curr_real, *currentRoboPos, graph);

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

                //get manhattan to closest goal & save it
                Pixel goal = getClosestGoal(s.pushTo->data, goals);
                int distanceClosestGoal = getHeuristics(s.pushTo->data, goal);
                newStep.distanceToClosestGoal = distanceClosestGoal;

                //+1 as we move from side and up to next pos with diamond
                newStep.robotTravelledLength = currStep.robotTravelledLength + s.movePath.size(); //+1 todo not counting correctly exactly

                //Already tried move? - check hashtable - If new it also saves in hashtable
                if (!isMoveNew(&newStep, hashTable))
                    continue;

                //Check if move is end move. Is all diamonds on goals?
                if (isWinStep(&newStep, goals)) {
                    //save solution in solutionList and set current shortest distance
                    if (newStep.robotTravelledLength < bestSolutionRobotMoves)
                        bestSolutionRobotMoves = newStep.robotTravelledLength;

                    solutionList.push_back(newStep);
                } else {
                    //add to openList. Add to front if we get closer than what we was before
//                    if (newStep.distanceToClosestGoal < bestDistanceToDimond)
//                    {
//                        bestDistanceToDimond = newStep.distanceToClosestGoal;
//                        openList.push_front(newStep);
//                    }else
                        openList.push_back(newStep);
                }
            }
        }
    }

    cout << endl << "solution list" << endl;
    for (auto s : solutionList)
    {
        for (const auto& d : s.diamonds)
        {
            cout << "diamond:" << d->data << ", ";
        }
        cout << endl;
        cout << "Robot Travel Length: " << s.robotTravelledLength << endl;
    }


    cout << "closed List size: " << closedList.size() << endl;

    //While is done, if solution exists it is in solutionVector
    if (solutionList.empty())
    {
        cout << "Couldn't find a solution..." << endl;
        for (const auto& s : closedList)
        {
            int count = 0;
            for(const auto& d : s.diamonds)
            {
                if(d->data == Pixel(2,4) || d->data == Pixel(4,4) || d->data == Pixel(6,4) || d->data == Pixel(8,4))
                    count++;
            }

            if (count < 4)
                continue;

            for(const auto& d : s.diamonds)
                cout << "(" << d->data << "), ";
            cout << endl;
        }
    }
    else {
        //select best solution and backtrack to create solution for robot
        //sort vector so best solution is
        solutionList.sort([] (const Step & lhs, const Step & rhs) {
            // < == ascending order(smallest first). > == descending order.
            return lhs.robotTravelledLength < rhs.robotTravelledLength;
        });

        //debug
        cout << endl << endl << "DIAMONDS:" << endl;
        for (auto v : solutionList.front().diamonds) {
            cout << v->data << endl;
        }
        cout << "Total Robot length: " << solutionList.front().robotTravelledLength << endl;
    }

    return getRobotPlan(solutionList.front());
}

void sokobanSolver::initMapDeadlocks(Map &map, int width, int height) {

    Graph& graph =  *map.getMapGraph();
    //check map from left to right, up to down
    //first check rows, if a row touches top or bottom and no goals then they are all deadlocks

    //go though rows
    for(int i = 0; i < height; i++ )
    {
        vector<Pixel> tempDeadlocks;
        bool deadStart = false;

        for(int j = 0; j < width; j++)
        {

            Vertex* pos = graph.findNodePointer(Pixel(j,i));
            Vertex* prevPos = graph.findNodePointer(Pixel(j-1,i));

            if (pos == nullptr || prevPos == nullptr) //doesn't exists
                continue;

            //get above and below
            Vertex& above = graph.findNode(Pixel(j,i-1));
            Vertex& below = graph.findNode(Pixel(j,i+1));

            //check if touching walls
            if(above.pathType == WALL || below.pathType == WALL)
            {
                if(pos->pathType == ROAD && prevPos->pathType == WALL)
                {
                    //start deadzone
                    deadStart = true;
                    //save this pos
                    tempDeadlocks.emplace_back(pos->data);
                } else if (pos->pathType == ROAD && deadStart)
                {
                    //another point in temp
                    tempDeadlocks.emplace_back(pos->data);
                } else if (pos->pathType == GOAL || pos->pathType == DIAMOND)
                {
                    //reset
                    tempDeadlocks.clear();
                    deadStart = false;
                } else if (pos->pathType == WALL && deadStart)
                {
                    //save over
                    for(auto d : tempDeadlocks)
                        deadlocks.emplace_back(d);

                    tempDeadlocks.clear();
                    deadStart = false;
                }
            }
        }

        //push deadlock vector
        if (!tempDeadlocks.empty())
        {
            for(const auto& d : tempDeadlocks)
                this->deadlocks.emplace_back(d);
            tempDeadlocks.clear();
        }
    }


    //go though coloums
    for(int col = 0; col < width; col++ )
    {
        vector<Pixel> tempDeadlocks;
        bool deadStart = false;

        for(int row = 0; row < height; row++)
        {

            Vertex* pos = graph.findNodePointer(Pixel(col,row));
            Vertex* prevPos = graph.findNodePointer(Pixel(col,row-1));

            if (pos == nullptr || prevPos == nullptr) //doesn't exists
                continue;

            //get above and below
            Vertex& before = graph.findNode(Pixel(col-1,row));
            Vertex& after = graph.findNode(Pixel(col+1,row));

            //check if touching walls
            if(before.pathType == WALL || after.pathType == WALL)
            {
                if(pos->pathType == ROAD && prevPos->pathType == WALL)
                {
                    //start deadzone
                    deadStart = true;
                    //save this pos
                    tempDeadlocks.emplace_back(pos->data);
                } else if (pos->pathType == ROAD && deadStart)
                {
                    //another point in temp
                    tempDeadlocks.emplace_back(pos->data);
                } else if (pos->pathType == GOAL || pos->pathType == DIAMOND)
                {
                    //reset
                    tempDeadlocks.clear();
                    deadStart = false;
                } else if (pos->pathType == WALL && deadStart)
                {
                    //save over
                    for(auto d : tempDeadlocks)
                        deadlocks.emplace_back(d);

                    tempDeadlocks.clear();
                    deadStart = false;
                }
            }
        }

        //push deadlock vector
        if (!tempDeadlocks.empty())
        {
            for(const auto& d : tempDeadlocks)
                this->deadlocks.emplace_back(d);
            tempDeadlocks.clear();
        }
    }

    //find corners
    for(int i = 0; i < height; i++ )
    {
        for(int j = 0; j < width; j++)
        {
            Vertex* pos = graph.findNodePointer(Pixel(j,i));

            if (pos == nullptr || pos->pathType == GOAL || pos->pathType == WALL || pos->pathType == DIAMOND) //doesn't exists
                continue;

            //get above and below
            Vertex& up = graph.findNode(Pixel(j,i-1));
            Vertex& down = graph.findNode(Pixel(j,i+1));
            Vertex& left = graph.findNode(Pixel(j-1,i));
            Vertex& right = graph.findNode(Pixel(j+1,i));

            //CASE TOP_LEFT
            if (isBlocked(up) && isBlocked(left))
                deadlocks.emplace_back(pos->data);
            //CASE TOP_RIGHT
            if (isBlocked(up) && isBlocked(right))
                deadlocks.emplace_back(pos->data);
            //CASE BOT_LEFT
            if (isBlocked(down) && isBlocked(left))
                deadlocks.emplace_back(pos->data);
            //CASE BOT_RIGHT
            if (isBlocked(down) && isBlocked(right))
                deadlocks.emplace_back(pos->data);
        }
    }

    //plot deadlocks for visual aid
    map.plotDeadlocks(deadlocks);
}

bool sokobanSolver::isBlocked(Vertex & v) {
    return v.pathType == WALL;
}

vector<string> sokobanSolver::getRobotPlan(Step &solution) {
    vector<string> plan;
    vector<string> planDebug;
    string robDirection = "N"; //todo Set depending on what is best

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
        vector<string> movePath = getRobotMovesFromPath(st.movePath, robDirection, planDebug);
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
        planDebug.push_back(targetDir);

        vector<string> rotToTarget = getRotateToDir(targetDir, robDirection);

        for(const auto& rot : rotToTarget)
            plan.push_back(rot);

        //robot now is in target dir
        robDirection = targetDir;

        //push diamond one forward. (Pushing diamond one forward is indicated with "D" instead of "F")
        plan.emplace_back("DDD");
    }

//    //output size of solution
//    int sokobanSteps = 0;
//    for (const auto& s : plan)
//        if (s == "FFF" || s == "DDD" || s == "BBB")
//            sokobanSteps++;

    cout << "Plan Sokoban Steps: " << plan.size() << endl;

    cout << "debug Plan" << endl;
    for(const auto& p : planDebug)
    {
        cout << p << ", ";
    }
    cout << endl;

    return plan;
}

vector<string> sokobanSolver::getRobotMovesFromPath(vector<Vertex *> path, string& startDir, vector<string>& debugPath) {
    vector<string> pathList;

    //null check - As in now needing to move before pushing diamond
    if (path.empty())
        return pathList;

    for(int i = 0; i < path.size() -1; i++)
    {
        //what way is next point
        Pixel delta = path[i+1]->data - path[i]->data;
        string targetDir = dirToNextPoint(delta);

        //todo debug
        debugPath.push_back(targetDir);

        //get rotations to turn around in right direction
        vector<string> rotations = getRotateToDir(targetDir, startDir);
            for (const auto& rot : rotations)
                pathList.push_back(rot);

        //set startDir to the targetDir, as the robot now is facing a different way
        startDir = targetDir;

        //add a forward as we now have right orientation and need to move one forward
        if (rotations.size() == 0)
            pathList.emplace_back("FFF");
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
        rotationsCW.emplace_back("CWW");
    }

    if (rotationsCCW.size() == rotationsCW.size() && rotationsCCW.size() != 0)
    {
        //if turn 2 times around give BBB instead of double turns
        vector<string> turnAround = {"BBB"};
        return turnAround;
    }
    else if(rotationsCCW.size() < rotationsCW.size())
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

bool sokobanSolver::isWinStep(Step* step, vector<Vertex*> goals) {
    //sort both vectors and see if they match
    vector<Vertex*> diamonds = step->diamonds;
    sort(diamonds.begin(), diamonds.end());
    sort(goals.begin(), goals.end());

    return diamonds == goals;
}

bool sokobanSolver::isMoveNew(Step * step, unordered_map<int, vector<Move>>& hashTable) {
    //new Move
    Move mov;
    vector<int> diamonds = getDiamondsIndex(step->diamonds);
    //always sort diamonds to be able to compare
    sort(diamonds.begin(), diamonds.end());

    mov.diamonds = diamonds;
    mov.robopos = step->finishedRoboPos->index;
    mov.length = step->robotTravelledLength;

    //get hashkey
    int hashkey = getHashKey(mov);

    //exists in table?
    if (hashTable[hashkey].empty()) {
        //key doesn't exists, move is new and is first move on this pos
        vector<Move> newEntry;
        newEntry.push_back(mov);
        hashTable[hashkey] = newEntry;
        return true;
    }
    else {
        //check if pos for diamonds is the same
        const vector<Move>& existingPos = hashTable[hashkey];

//        int sizeB = hashTable.bucket(hashkey);
//        int collision = hashTable.bucket_size(sizeB);
//        if (collision > 4)
//            cout << collision << endl;

        for(const auto &move : existingPos)
        {
            //if equal move is not new. Also checks for robo pos
            if (move == mov)
                return false;
        }

        //if passed all values and is still unmatched, then insert new pos & return true;
        hashTable[hashkey].push_back(mov);
        return true;
    }
}

vector<int> sokobanSolver::getDiamondsIndex(vector<Vertex *> diamonds) {
    vector<int> list;
    for (auto& v : diamonds)
        list.push_back(v->index);
    return list;
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
    //check if pos is deadlock.
    return find(deadlocks.begin(), deadlocks.end(), newPos->data) != deadlocks.end();
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

            //Vertex* newPosV = currPos.findNeighbour(newPos);
            Vertex* newPosV = &map.findNode(newPos);
            //todo ref issue, so gotta get real ref to check pathtype
            if (newPosV == nullptr)
                continue;

            //else get real node and check pathType
            Vertex& newPosV_real = map.getNodesRef()[newPosV->index];
            if(newPosV_real.pathType != DIAMOND) {

                pushableSides.emplace_back(v, &newPosV_real, path);
            }
        }
    }
    return pushableSides;
}

int sokobanSolver::getHashKey(Move move) {

    //hashkey is currently diamond indexes + robopos index
    int hashKey = 0;
    for(auto v : move.diamonds) {
        hashKey += hashMap[v];
    }
    hashKey += hashMap[move.robopos];

    return hashKey;
}

void sokobanSolver::initHashFunction(int size) {
    srand (time(NULL));
    hashMap.resize(size);
    for(int i = 0; i < size; i++) {
        hashMap[i] = rand()%(10000-10 + 1) + 10;
    }
}

int sokobanSolver::getHeuristics(Pixel from, Pixel to)
{
    return (from.x + to.x) + (from.y + to.y);
}

Pixel sokobanSolver::getClosestGoal(Pixel currPos, vector<Vertex *> goals)
{
    //calculate the manhattan for each goal and return the pos of the closest goal
    auto closest = INFINITY;
    Pixel closestGoal;

    for(auto d : goals)
    {
        int dist = getHeuristics(currPos, d->data);

        if(dist <= closest)
        {
            closest = dist;
            closestGoal = d->data;
        }
    }
}
