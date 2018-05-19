#include "myGetMap.h"
#include "visualize.h"
// #include <geometry_msgs/Point.h>


int main (int argc, char **argv) {
    ros::init(argc, argv, "myGetMap");
    ros::NodeHandle nh;
     // ros::Subscriber sub = nh.subscribe("odometry/filtered",10,OdomCallback);

     // ros::spinOnce();
     // ros::Rate rate(10);
     // rate.sleep();
     // ros::spinOnce();

    ROS_INFO("Testpunnkt 1");

    MyGetMap myMap;

    nav_msgs::OccupancyGrid grid = myMap.requestMap(nh);
    std::vector<std::vector<int> > gridMap = myMap.readMap(grid);

    ROS_INFO("gridMap[5][5] = %d", gridMap[5][5]);
	ROS_INFO("gridMap.size() = %d", gridMap.size());
    std::vector<std::vector<int> > costMap = myMap.getCostmap(nh);
	ROS_INFO("costMap.size() = %d", costMap.size());
    
    //geometry_msgs::Point myPoint = myMap.grid2Kartesisch(grid,0,0);

    // ROS_INFO("Punkt x = %f", myPoint.x);
    // ROS_INFO("Punkt y = %f", myPoint.y);
    
    //gridCell myCell =  myMap.kartesisch2grid(grid,-10,-10);

    // ROS_INFO("cell row = %d", myCell.row);
    // ROS_INFO("cell col = %d", myCell.col);
    
    robotPose myRobot;
    std::vector<geometry_msgs::Pose> myVizPos;
    geometry_msgs::Pose dummyPos;

    myRobot = myMap.getRobotPos(nh);


    ROS_INFO("myRobot.x = %f", myRobot.x);
    ROS_INFO("myRobot.y = %f", myRobot.y);
    ROS_INFO("myRobot.yaw = %f", myRobot.yaw);
        
	geometry_msgs::Point myPoint;

    for(int i = costMap.size() - 1; i >= 0 ; i--){      // durchsuche ganze Karte
    	for(int j = 0; j < costMap[0].size(); j++){
    		if(costMap[i][j] == 10){
    			//ROS_INFO("255");
    			myPoint = myMap.grid2Kartesisch(grid,j,i);
    			dummyPos.position.x = myPoint.x;
    			dummyPos.position.y = myPoint.y;
    			myVizPos.push_back(dummyPos);
    		}

        }
	}

    //dummyPos.position.x = myRobot.x;
    //dummyPos.position.y = myRobot.y;

    //myVizPos.push_back(dummyPos);
    
    Visualizer myVisualize;

    myVisualize.setMarkerArray(nh, myVizPos);

    // ros::spin();

    return 0;
}
