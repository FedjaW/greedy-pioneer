#include <ros/ros.h>
#include "myGetMap.h"
#include "visualize.h"
//#include "holyWatcher.h"
#include "findFrontiers.h"
#include <thread>

#include "holyWatcher.h"
#include "move_service.h"

// #include <geometry_msgs/Point.h>
#if 1
int main (int argc, char **argv) {
    ros::init(argc, argv, "myGetMap");
    ros::NodeHandle nh;

    ROS_INFO("Testpunnkt 1");
    std::thread watcherThread(startPositionWatcher);
    ROS_INFO("Testpunnkt 2");

     ros::Rate rate(10);
     rate.sleep(); // warte kurz bis die Callbacks im anderen thread
     rate.sleep(); // aufgerufen wurden und Daten vorliegen
     rate.sleep(); // TODO: schöner lösen !!!


    ROS_INFO("Testpunnkt 3");

    nav_msgs::OccupancyGrid grid = requestMap(nh);
    std::vector<std::vector<int> > gridMap = readMap(grid);

    // ROS_INFO("gridMap[5][5] = %d", gridMap[5][5]);
	// ROS_INFO("gridMap.size() = %d", gridMap.size());
    // std::vector<std::vector<int> > costMap = getCostmap();
	// ROS_INFO("costMap.size() = %d", costMap.size());
    //geometry_msgs::Point myPoint = myMap.grid2Kartesisch(grid,0,0);

    // ROS_INFO("Punkt x = %f", myPoint.x);
    // ROS_INFO("Punkt y = %f", myPoint.y);
    
    //gridCell myCell =  myMap.kartesisch2grid(grid,-10,-10);

    // ROS_INFO("cell row = %d", myCell.row);
    // ROS_INFO("cell col = %d", myCell.col);
    

    std::vector<geometry_msgs::Pose> myVizPos;
    geometry_msgs::Pose dummyPos;


    ROS_INFO("x: %f / y: %f", getRobotPos(/*nh*/).x, getRobotPos(/*nh*/).y);
    //ROS_INFO("myRobot.yaw = %f", myRobot.yaw);

    geometry_msgs::Point myPoint;

    // for(int i = costMap.size() - 1; i >= 0 ; i--){      // durchsuche ganze Karte
    //     for(int j = 0; j < costMap[0].size(); j++){
    //         if(costMap[i][j] == 10){
    //             //ROS_INFO("255");
    //             myPoint = grid2Kartesisch(grid,j,i);
    //             dummyPos.position.x = myPoint.x;
    //             dummyPos.position.y = myPoint.y;
    //             myVizPos.push_back(dummyPos);
    //         }
    //
    //     }
    // }
    
    // double x_ = getRobotPosInMapFrame().getOrigin().x();
    // double y_ = getRobotPosInMapFrame().getOrigin().y();

    //dummyPos.position.x = myRobot.x;
    //dummyPos.position.y = myRobot.y;

    std::vector<gridCell> frontierCells = findFrontierCells(gridMap);
// für die Funktion fillFrontier
    double robotPos_x = getRobotPosInMapFrame().getOrigin().x();
    double robotPos_y = getRobotPosInMapFrame().getOrigin().y();
    robotPos_col = kartesisch2grid(grid, robotPos_x, robotPos_y).col;
    robotPos_row = kartesisch2grid(grid, robotPos_x, robotPos_y).row;
    robot_yaw = atan2(robotPos_y, robotPos_x);
// _______________________________________________________________________-
    std::vector<Frontier> frontier_list = buildFrontiers(frontierCells);
    
    for(auto& frontier : frontier_list) {
        // int pseudoMidPointOfFrontier = ceil(frontier.size() / 2);
        // std::cout << "pseudoMidPointOfFrontier = " << pseudoMidPointOfFrontier << std::endl;
        std::cout << "frontier.centroid.row = " << frontier.centroid.row << std::endl;
        std::cout << "frontier.centroid.col = " << frontier.centroid.col << std::endl;
            myPoint = grid2Kartesisch(grid,
                                            frontier.connected_f_cells[frontier.idxOfMinDistance].row, 
                                            frontier.connected_f_cells[frontier.idxOfMinDistance].col);
            // dummyPos.position.x = myPoint.x;
            // dummyPos.position.y = myPoint.y;
            // getDistanceToFrontier(nh, myPoint, x_, y_);

            //TODO: das ding returned einen double !!!
            double dummyVar = getDistanceToFrontier(nh, myPoint);
    }



    Visualizer myVisualize;
    int id = 0; // KEINE AHNUNG warum das mit der id so klappt

    int r = 1;
    int g = 0;
    int b = 0;

    for(auto& frontier : frontier_list) {
        if((r == 1) && (g == 0) && (b == 0)) {
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

        for(int i = 0; i < frontier.connected_f_cells.size(); i++) {
            myPoint = grid2Kartesisch(grid, frontier.connected_f_cells[i].row, frontier.connected_f_cells[i].col);
            dummyPos.position.x = myPoint.x;
            dummyPos.position.y = myPoint.y;
            myVizPos.push_back(dummyPos);
            
        }
        myVisualize.setMarkerArray(nh, myVizPos, r,g,b);

        id = myVizPos.size()+1;
        myVizPos.clear();
    }




#if 0
    std::vector<gridCell> frontierCells = findFrontierCells(gridMap);
    // Ohne das & (alias) funktinoert es nicht richtig! Warum ist das so???
    int c = 0;
    for(auto& frontierCell : frontierCells) {
        myPoint = grid2Kartesisch(grid, frontierCell.row, frontierCell.col);
        dummyPos.position.x = myPoint.x;
        dummyPos.position.y = myPoint.y;
        myVizPos.push_back(dummyPos);
        c++;
    }

    std::cout << "c = " << c << std::endl;
    std::cout << "myVizPos.Size() = " << myVizPos.size() << std::endl;
#endif 
    




    // ros::spin();

    return 0;
}

#endif

#if 0

int main (int argc, char **argv) {

   gridCell dummyCells; 
    
    std::vector<gridCell> frontierCells;

    dummyCells.row = 2;
    dummyCells.col = 2;
    frontierCells.push_back(dummyCells);

    dummyCells.row = 30;
    dummyCells.col = 30;
    frontierCells.push_back(dummyCells);
    //-------------------------------------
    dummyCells.row = 5;
    dummyCells.col = 5;
    frontierCells.push_back(dummyCells);

    dummyCells.row = 3;
    dummyCells.col = 3;
    frontierCells.push_back(dummyCells);

    dummyCells.row = 4;
    dummyCells.col = 4;
    frontierCells.push_back(dummyCells);
    //-------------------------------------
    dummyCells.row = 9;
    dummyCells.col = 7;
    frontierCells.push_back(dummyCells);
    
    dummyCells.row = 8;
    dummyCells.col = 7;
    frontierCells.push_back(dummyCells);

    dummyCells.row = 10;
    dummyCells.col = 8;
    frontierCells.push_back(dummyCells);
    //-------------------------------------
    dummyCells.row = 100;
    dummyCells.col = 100;
    frontierCells.push_back(dummyCells);

    dummyCells.row = 101;
    dummyCells.col = 101;
    frontierCells.push_back(dummyCells);

    dummyCells.row = 102;
    dummyCells.col = 102;
    frontierCells.push_back(dummyCells);
    //-------------------------------------
    dummyCells.row = 6;
    dummyCells.col = 6;
    frontierCells.push_back(dummyCells);

    std::cout << "sizeofvector = " << frontierCells.size() << std::endl;

    std::vector<std::vector<gridCell> > frontier_list = frontierCellNhood(frontierCells);


    for(auto& frontier : frontier_list) {
        for(int i = 0; i < frontier.size(); i++) {
            std::cout << "frontier[" <<i<< "]" << ".row = " << frontier[i].row << " / frontier[" <<i<< "]" << ".col = " << frontier[i].col << std::endl;
        }
        std::cout << "----------------------------" << std::endl;
    }

    return 0;
}
#endif
