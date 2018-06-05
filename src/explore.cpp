#include <ros/ros.h>
#include "myGetMap.h"
#include "visualize.h"
#include "findFrontiers.h"
#include <thread>
#include "holyWatcher.h"
#include "move_service.h"
#include <algorithm> 



bool sortFunction(const Frontier& f1, const Frontier& f2) {
    return f1.cost < f2.cost;
}


bool exploration(ros::NodeHandle &nh) {

    // nav_msgs::OccupancyGrid grid = requestMap(nh);
    // std::vector<std::vector<int> > gridMap = readMap(grid);

    std::vector<geometry_msgs::Pose> myVizPos;
    geometry_msgs::Pose dummyPos;

    geometry_msgs::Point myPoint;
    std::vector<gridCell> frontierCells = findFrontierCells(gridMap);
    
    // für die Funktion fillFrontier / alle variablem sind Global
    double robotPos_x = getRobotPosInMapFrame().getOrigin().x(); // getRobotPosInMapFrame erzeugt ein 
    double robotPos_y = getRobotPosInMapFrame().getOrigin().y(); // globales Objekt vom TransformListener
    robotPos_col = kartesisch2grid(grid, robotPos_x, robotPos_y).col;
    robotPos_row = kartesisch2grid(grid, robotPos_x, robotPos_y).row;
    robot_yaw = tf::getYaw(getRobotPosInMapFrame().getRotation());


    // baue Frontiers aus den f-zellen
    std::vector<Frontier> frontier_list = buildFrontiers(frontierCells);
    if(frontier_list.empty()) return false;

    // Kostenfunktion zuordnen!
    // Parameter der Kosten
    const double alpha = 20.0;
    const double beta = 1.0;
    const double gamma = 10.0;
    const double laser_radius = 5;
    const double FOV = 4.71238; // Field of View in radians

    for(auto& frontier : frontier_list) {
        // std::cout << "frontier.centroid.row = " << frontier.centroid.row << std::endl;
        // std::cout << "frontier.centroid.col = " << frontier.centroid.col << std::endl;
        myPoint = grid2Kartesisch(grid,
                                  frontier.connected_f_cells[frontier.pseudoMidPoint].row, 
                                  frontier.connected_f_cells[frontier.pseudoMidPoint].col);

        distanceAndSteering distAndSteer = getDistanceToFrontier(nh, myPoint); //TODO: was ist wenn Zielpunkt nicht erreichbar ist
        frontier.distance2Frontier =  distAndSteer.distance;
        frontier.goalSteeringAngle = distAndSteer.goalSteeringAngle;

        double rotationCost = 100000; // sehr hoch wählen!!! 
        double drivingCost = + alpha * frontier.distance2Frontier 
                             - beta * frontier.numberOfElements 
                             + gamma * fabs(frontier.angleToGoalPoint);
        std::cout << "drivingCost = "<<  drivingCost << std::endl;

        if(frontier.directMinDistance < laser_radius) {
            bool obstacle = isObstacleInViewField(nh,
                                                  grid,
                                                  robotPos_col, 
                                                  robotPos_row,
                                                  frontier.connected_f_cells[frontier.idxOfMinDistance].col, 
                                                  frontier.connected_f_cells[frontier.idxOfMinDistance].row);

            std::cout << "obstacle = " << obstacle << std::endl;

            if(abs(frontier.rotationAngle) >= FOV/2) { // >= damit der Randbereich auch an-rotiert wird
                std::cout << "Frontier liegt außerhalb der Sicht" << std::endl;

                if(!obstacle) {
                    // wenn die obigen 3 Bedingungen (if's) nicht erfüllt sind
                    // bleibt die rotationCost so hoch dass automatisch shouldRotate = false bleibt 
                    rotationCost = - beta * frontier.numberOfElements + gamma * fabs(frontier.rotationAngle);
                    std::cout << "rotationCost = "<<  rotationCost << std::endl;
                }
            }else {
                std::cout << "Frontier liegt bereits in Sicht" << std::endl;
            }
        
        }else {
                std::cout << "Closest Point liegt nicht in Laser Reichweite" << std::endl;
        }

        if(drivingCost > rotationCost) {
            frontier.shouldRotate = true;
            frontier.cost = rotationCost;
        }
        else {
            frontier.shouldRotate = false;
            frontier.cost = drivingCost;
        }
        std::cout << "cost = "<<  frontier.cost << " / " << "shouldRotate = " << frontier.shouldRotate << std::endl;
    }

    std::cout << "--------------------------------------------------------------" << std::endl;

    // Frontiers nach Konsten sortieren. Günstgigestes als erstes
    std::sort(frontier_list.begin(), frontier_list.end(), sortFunction);

    for(auto& frontier : frontier_list) {
        std::cout << "cost = "<<  frontier.cost << " / " << "shouldRotate = " << frontier.shouldRotate << std::endl;
    }

    Visualizer myVisualize;
    int r = 0;
    int g = 0;
    int b = 0;

    for(auto& frontier : frontier_list) {
        if((r == 0) && (g == 0) && (b == 0)) {
            r = 1;
            g = 0;
            b = 0;
        }
        else if((r == 1) && (g == 0) && (b == 0)) {
            r = 0;
            g = 1;
            b = 0;
        }
        else if((r == 0) && (g == 1) && (b == 0)) {
            r = 0;
            g = 0;
            b = 1;
        }
        else if((r == 0) && (g == 0) && (b == 1)) {
            r = 1;
            g = 1;
            b = 0;
        }
        else if((r == 1) && (g == 1) && (b == 0)) {
            r = 1;
            g = 0;
            b = 1;
        }
        else if((r == 1) && (g == 0) && (b == 1)) {
            r = 0;
            g = 1;
            b = 1;
        }
        else if((r == 0) && (g == 1) && (b == 1)) {
            r = 1;
            g = 1;
            b = 1;
        }
        else if((r == 1) && (g == 1) && (b == 1)) {
            r = 0.5;
            g = 1;
            b = 0.1;
        }
        for(int i = 0; i < frontier.connected_f_cells.size(); i++) {
            myPoint = grid2Kartesisch(grid, 
                                        frontier.connected_f_cells[i].row, 
                                        frontier.connected_f_cells[i].col);
            dummyPos.position.x = myPoint.x;
            dummyPos.position.y = myPoint.y;
            myVizPos.push_back(dummyPos);
        }

        myVisualize.setMarkerArray(nh, myVizPos, r,g,b,0);
        // id = myVizPos.size()+1;
        myVizPos.clear();
    }
    // Fahre immer das erste Frontier in der liste an weil es das günstigste ist!
    if(frontier_list[0].shouldRotate == 0) {
        myPoint = grid2Kartesisch(grid,
                                  frontier_list[0].connected_f_cells[frontier_list[0].pseudoMidPoint].row, 
                                  frontier_list[0].connected_f_cells[frontier_list[0].pseudoMidPoint].col);
        sendGoal(myPoint.x, myPoint.y, frontier_list[0].goalSteeringAngle, frontier_list[0].distance2Frontier);
    }
    else {
        rotate(nh, frontier_list[0].rotationAngle);
    }

    myVisualize.setMarkerArray(nh, myVizPos, r,g,b,1); // delete ALL markerArrays
    // myVisualize.deleteMarkerArray(nh);

    // NOTE: Kp ob ich das brauche
    frontierCells.clear();
    frontier_list.clear();
    return true;
}



// MAIN FUNKTION__________________________________
int main(int argc, char **argv) {

    ros::init(argc, argv, "explore");
    ros::NodeHandle nh;

    std::thread watcherThread(startPositionWatcher);

    ros::Rate rate(10);
    rate.sleep(); // warte kurz bis die Callbacks im anderen thread
    rate.sleep(); // aufgerufen wurden und Daten vorliegen
    rate.sleep(); // TODO: schöner lösen !!!

    while(exploration(nh));

    return 0;
}
