#include "myGetMap.h"
#include "visualize.h"
//#include "holyWatcher.h"
#include "findFrontiers.h"
#include <thread>
// #include <geometry_msgs/Point.h>

int main (int argc, char **argv) {
    ros::init(argc, argv, "myGetMap");
    ros::NodeHandle nh;
     // ros::Subscriber sub = nh.subscribe("odometry/filtered",10,OdomCallback);

    ROS_INFO("Testpunnkt 1");
    std::thread watcherThread(startPositionWatcher);
    ROS_INFO("Testpunnkt 2");

     ros::Rate rate(10);
     rate.sleep(); // warte kurz bis die Callbacks im anderen thread
     rate.sleep(); // aufgerufen wurden und Daten vorliegen
     rate.sleep(); // TODO: schönes lösen !!!

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


    //dummyPos.position.x = myRobot.x;
    //dummyPos.position.y = myRobot.y;


    std::vector<gridCell> frontierCells = findFrontierCells(gridMap);
    std::vector<std::vector<gridCell> > frontier_list = frontierCellNhood(frontierCells);

    
    for(auto& frontier : frontier_list) {
        for(frontier[i] < )

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
    


    Visualizer myVisualize;
    myVisualize.setMarkerArray(nh, myVizPos);


    // ros::spin();

    return 0;
}
