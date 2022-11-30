#include <fstream>
#include <sstream>
#include <string>
using namespace std;

#include "eLib/eMotionControllers/EPursuit.hpp"
#include "eLib/utils/EMath.hpp"
using namespace elib;
using namespace okapi;

namespace elib{
    // Constructor
    EPursuit::EPursuit(shared_ptr<OdomState> shared_pose){
        truePose = shared_pose;
    }

    // ----------------------
    // Pursuit Utilities
    // ----------------------
    int EPursuit::loadPath(const string& pathDir){
        vector<string> row;
        string line, word;

        // File is Formatted as
        // Direction
        // X -- Y -- targetTurnAngle -- targetDistance
        int tmpDirect;
        double tmpX, tmpY, tmpYaw, tmpDist;
        int ind = 0;

        fstream file;
        file.open(pathDir, ios::in);
        if(file.is_open())
        {
            pros::lcd::print(0, "File Opened");
            getline(file, line);
            row.clear();
            std::stringstream str(line);
            path.direction = stoi(row[1]); 

            PathNode tmpNode;

            #if USING_TURN_IN_PURSUIT
            PathTurn tmpTurn;
            #endif

            int index = 0;

            while(getline(file, line))
            {
                // Parse a row
                row.clear();
                std::stringstream str(line);
                while(getline(str, word, ',')) row.push_back(word);

                int nodeType = stoi(row[0]); 

                // Convert string to values
                switch (nodeType){
                    case 0:
                        tmpNode.point.x = 1_in * stod(row[1]); 
                        tmpNode.point.y = 1_in * stod(row[2]); 
                        tmpNode.targetDistance = 1_in * stod(row[3]);
                        // If dist is zero and no end point before
                        if (tmpNode.targetDistance == 0_in && path.endPointIndex == -1){path.endPointIndex = index;}
                        path.nodes.push_back(tmpNode);
                        index++;
                    #if USING_TURN_IN_PURSUIT
                    case 1:
                        tmpTurn.targetTurnAngle = 1_deg * stod(row[3]);
                    #endif
                }

            }
            return index;
        }
        return -1;
    }



    // Find Target Index on the current Path ID Given Current Pose
    int EPursuit::findTargetIndex(){
        // If target Index predates current path
        // Iterate through this path till the end of the current Path
        for (int i = findNearestIndex(); i < path.size();i++){
            // If distance from the current pose to waypoint pose greater than look ahead distance
            if (distanceFromCurrentPose(truePose, path.nodes[i].point) > lookAheadDist) return i; 
        }
        // If none is long enogh, it is close to the end of the path
        // Then set the last point of the path as the target
        return targetIndex = path.nodes.size()-1;
    }

    // Find the closest pose on the current Path ID
    int EPursuit::findNearestIndex(){

        int closestIndex = -1;
        QLength minDist = 999_m;
        // Iterate through the current path

        for(int i = 0;i < path.nodes.size();i++){
            // For every point calculate distance
            QLength ltmp = distanceFromCurrentPose(truePose, path.nodes[i].point);
            // Find smallest distance and set index
            if (ltmp < minDist){
                minDist = ltmp;
                closestIndex = i;
            }

        }

        return closestIndex;
    }
    



    // ----------------------
    // Task API
    // ----------------------
    // This is the regularly called task in Drive during a path following
    // It takes the current position and 
    int EPursuit::calcPursuit(QLength& pursuitDist, QAngle& pursuitAngle, QLength& dist2Target){
        // If the path is completed or meets criteria for reaching path end
        // - Mark current path as completed
        // - Pursuit distance = negative distance between current position and target point
        //   -- This helps slow down
        // - No turn angle

        // Find Nearest Index
        int nIndex = findNearestIndex();
        targetIndex = findTargetIndex();

        #if USING_TURN_IN_PURSUIT
        // If there is a turn needed
        // Logic for [turns.completed turns]
        // --> if no turn is completed, completedTurns == 0
        // --> therefore, we are pointing at path.turns[0], the first desired turn of this path. 
        if (nIndex >= path.turn){return 2;}
        #endif

        // Find the X, Y of target Index
        QLength tx = path.nodes[targetIndex].point.x;
        QLength ty = path.nodes[targetIndex].point.y;

        // Find Desired Parameters
        pursuitDist = path.nodes[nIndex].targetDistance;
        // if(path.direction) pursuitDist *= -1;

        pursuitAngle = (atan2(ty-truePose->y,tx-truePose->x) - truePose->theta) * sgn(pursuitDist.convert(meter));
        if (path.direction) pursuitAngle = getBoundAngle(pursuitAngle - 180_deg);
        
        dist2Target = hypot(path.nodes[targetIndex].point.x-truePose->x, path.nodes[targetIndex].point.y-truePose->y);
        // if(path.direction) pursuitDist *= -1;
        // pros::lcd::print(6, "TX %.2f TY %.2f", path.nodes[1].point.x.convert(inch), path.nodes[1].point.y.convert(inch));

        // Wrap Up task
        return 0;
    }

    void EPursuit::clearPursuit(){targetIndex = -1;}



    // ----------------------
    // User API
    // ----------------------
    void EPursuit::startPath(PathSegment targetPath){path = targetPath;}
    void EPursuit::startPath(const string& pathDir){loadPath(pathDir);}

    // ----------------------
    // Status Monitor
    // ----------------------
    
    // Determine whether robot reaches the end of the path. 
    bool EPursuit::reachedPathEnd(){return (findNearestIndex()>=path.endPointIndex);}
    int EPursuit::getPathSize(){return path.size();}
    int EPursuit::getPathEnd(){return path.endPointIndex;}

} // namespace elib