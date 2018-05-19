#include <vector>
#include <iostream>
#include <thread>

#include <ros/ros.h>

struct position {
    float x;
    float y;
};

position roboterPosition;

void updateRoboterPosition(const nav_msgs::Odometry::ConstPtr& position) {
    roboterPosition.x = position->pose.pose.position.x;
    roboterPosition.y = position->pose.pose.position.y;
}

void startPositionWatcher() {
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("odometry/filtered", 1, updateRoboterPosition);
    ros::spin();
}

#include "myGetMap.h"
#include "myGetMap.cpp"

using namespace std;


class FrontierSearcher {
  public:
    FrontierSearcher(int argc, char **argv) {
        ros::init(argc, argv, "FrontierSearcher");
    } 
    void streamPositionToStdOut() {
        while(true) {
            cout << roboterPosition.x << " / " << roboterPosition.y << endl;
        }
    }
  private:
};

int main (int argc, char **argv) {
    FrontierSearcher frontierSearcher(argc, argv);

    thread watcherThread(startPositionWatcher);

    frontierSearcher.streamPositionToStdOut();
}
