#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
    
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>



using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void sendGoal(double position_x, double position_y, double orientation_w);

    unsigned int grid_cell_x[9050];
    unsigned int grid_cell_y[9050];
    void findFrontiers();
        

    // void setMarker(visualization_msgs::Marker& marker, ros::NodeHandle &nh);
    void setMarker(ros::NodeHandle &nh, double mark_x, double mark_y, int id);
    // visualization_msgs::Marker marker_global;

bool waitForSubscribers(ros::Publisher & pub, ros::Duration timeout);
// grid map
int rows;
int cols;
double mapResolution;
vector<vector<int> > grid;

double map_frame_x;
double map_frame_y;

bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& msg, ros::NodeHandle &nh);
void printGridToFile();
void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);


int main (int argc, char** argv){

    ros::init(argc, argv, "getOccupancyGridMap"); 
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("odometry/filtered",10,OdomCallback);
    ros::Rate rate(10);
    rate.sleep();
    ros::spinOnce();


    if (!requestMap(nh))
        exit(-1);
    
    // findFrontiers();

    // printGridToFile();
    return 0;
}


void setMarker(ros::NodeHandle &nh, double mark_x, double  mark_y, int id){
            ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 20);
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = id;

    // Set the marker type.  
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = mark_x;
    marker.pose.position.y = mark_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    ros::Duration timeout = ros::Duration(3);
    if(waitForSubscribers(marker_pub, timeout)){
        marker_pub.publish(marker);
        ROS_INFO("habe marker %d gesetzt", id);
    }
    else
        ROS_INFO("TIMEOUT");
}




bool waitForSubscribers(ros::Publisher & pub, ros::Duration timeout)
{
    if(pub.getNumSubscribers() > 0)
        return true;
    ros::Time start = ros::Time::now();
    ros::Rate waitTime(0.5);
    while(ros::Time::now() - start < timeout) {
        waitTime.sleep();
        if(pub.getNumSubscribers() > 0)
            break;
    }
    return pub.getNumSubscribers() > 0;
}



bool requestMap(ros::NodeHandle &nh){

  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;

  while (!ros::service::waitForService("dynamic_map", ros::Duration(3.0))){             // dynamic map ist a service provided by gmapping: dynamic_map(mav_msgs/GetMap)
    ROS_INFO("Wating for service dynamic map to become available\n");
  }

  ROS_INFO("Requesting the map..\n");
  ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("dynamic_map");

  if(mapClient.call(req, res)){
    readMap(res.map, nh);

                 
                 // ROS_INFO("res map orin = %.3f\n",res.map.info.origin.position.x);


    return true;
  }
  else{
    ROS_ERROR("Failed to call map service\n");
    return false;
  }
}


void readMap(const nav_msgs::OccupancyGrid& map, ros::NodeHandle &nh){
    ROS_INFO("Received a %d X %d Map @ %.3f m/px\n", 
                                                    map.info.width,
                                                    map.info.height,
                                                    map.info.resolution);
    
  ROS_INFO("Origin.Position x= %.3f \n", map.info.origin.position.x);
  ROS_INFO("Origin.Position y= %.3f \n", map.info.origin.position.y);

    
  ROS_INFO("map_frame_x = %f \n", map_frame_x);
  ROS_INFO("map_frame_y = %f \n", map_frame_y);
    int grid_frame_x = (unsigned int) ((map_frame_x - map.info.origin.position.x)/map.info.resolution);
    int grid_frame_y = (unsigned int) ((map_frame_y - map.info.origin.position.y)/map.info.resolution);
  ROS_INFO("grid_frame_x = %d \n", grid_frame_x);
  ROS_INFO("grid_frame_y = %d \n", grid_frame_y);

 double ziel_map_frame_x = 991 * map.info.resolution + map.info.origin.position.x;
 double ziel_map_frame_y = 991 * map.info.resolution + map.info.origin.position.y;
                 double mark_pos_x = ziel_map_frame_x;
                 double mark_pos_y = ziel_map_frame_y;
                 int id = 0;
                 setMarker(nh, mark_pos_x, mark_pos_y, id);

  ziel_map_frame_x = 0 * map.info.resolution + map.info.origin.position.x;
  ziel_map_frame_y = 991 * map.info.resolution + map.info.origin.position.y;
                  mark_pos_x = ziel_map_frame_x;
                  mark_pos_y = ziel_map_frame_y;
                  id = 1;
                 setMarker(nh, mark_pos_x, mark_pos_y, id);
                 // mark_pos_x = ;
                 // mark_pos_y = ;
                 // id = 1;
                 // setMarker(nh, mark_pos_x, mark_pos_y, id);

  rows = map.info.height;
  cols = map.info.width;
  mapResolution = map.info.resolution;


  // Dynamically resize the Grid
  grid.resize(rows);
  for (int i = 0; i < rows ; i++){
    grid[i].resize(cols);
  }

  int currCell = 0;
  for (int i = 0; i < rows; i++){
           for(int j = 0; j< cols; j++){
                grid[i][j] = map.data[currCell];
            currCell++;
    }
  }


              findFrontiers();
               // for(int i = 50; i < 1400; i = i + 50){
               //      double ziel_map_frame_x = grid_cell_x[i] * map.info.resolution + map.info.origin.position.x;
               //      double ziel_map_frame_y = grid_cell_y[i] * map.info.resolution + map.info.origin.position.y;
               //      double mark_pos_x = ziel_map_frame_x;
               //      double mark_pos_y = ziel_map_frame_y;
               //      int id = i;
               //      setMarker(nh, mark_pos_x, mark_pos_y, id);
               // }

                    ziel_map_frame_x = grid_cell_x[100] * map.info.resolution + map.info.origin.position.x;
                    ziel_map_frame_y = grid_cell_y[100] * map.info.resolution + map.info.origin.position.y;
                    mark_pos_x = ziel_map_frame_x;
                    mark_pos_y = ziel_map_frame_y;
                    id = 2;
                    setMarker(nh, mark_pos_x, mark_pos_y, id);
                    sendGoal(mark_pos_x, mark_pos_y, 1.0);

}

// void printGridToFile(){
//   ofstream gridFile;
//   gridFile.open("grid.txt");
//
//   for(int i = grid.size() - 1; i>= 0 ; i--){
//         for(int j = 0; j < grid[0].size(); j++){
//            // gridFile << (grid[i][j] ? "1" : "0");
//             if(i == 148 && j == 455)
//                 gridFile << "R";
//             else
//                 gridFile << grid[i][j];
//         }
//         gridFile << endl;
//   }
//   
//   gridFile.close();
//   ROS_INFO("Habe das File gespeichert und geschlossen\n");
// }

// Read out the odometry _________________________________________________________
void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        map_frame_x = msg->pose.pose.position.x;
        map_frame_y = msg->pose.pose.position.y;
         // ROS_INFO("x: %f, y: %f",map_frame_x ,map_frame_y );
        ROS_INFO("odom callback");
}



void findFrontiers(){
    unsigned int k = 0;
    for(int i = grid.size() - 1; i>= 0 ; i--){
        for(int j = 0; j < grid[0].size(); j++){
                if(grid[i][j] == 0){
                        // grid_cell_x[k] = j;
                        // grid_cell_y[k] = i;
                        // k++;
                        // ROS_INFO("Free cell [%d]: x = %d",k, i);
                        // ROS_INFO("Free cell [%d]: y = %d",k, j);
                    if(grid[i+1][j] == -1){
                        grid_cell_x[k] = j;
                        grid_cell_y[k] = i;
                        ROS_INFO("frontier rechts");
                        ROS_INFO("frontier [%d]: x = %d",k, j);
                        ROS_INFO("frontier [%d]: y = %d",k, i);
                        ROS_INFO("--------------");
                        k++;
                    }
                    else if(grid[i][j+1] == -1){
                        grid_cell_x[k] = j;
                        grid_cell_y[k] = i;
                        ROS_INFO("frontier oben");
                        ROS_INFO("frontier [%d]: x = %d",k, j);
                        ROS_INFO("frontier [%d]: y = %d",k, i);
                        ROS_INFO("--------------");
                        k++;
                    }
                    else if(grid[i-1][j] == -1){
                        grid_cell_x[k] = j;
                        grid_cell_y[k] = i;
                        ROS_INFO("frontier links");
                        ROS_INFO("frontier [%d]: x = %d",k, j);
                        ROS_INFO("frontier [%d]: y = %d",k, i);
                        ROS_INFO("--------------");
                        k++;
                    }
                    else if(grid[i][j-1] == -1){
                        grid_cell_x[k] = j;
                        grid_cell_y[k] = i;
                        ROS_INFO("frontier unten");
                        ROS_INFO("frontier [%d]: x = %d",k, j);
                        ROS_INFO("frontier [%d]: y = %d",k, i);
                        ROS_INFO("--------------");
                        k++;
                    }
                }
            }
     }

}



void sendGoal(double position_x, double position_y, double orientation_w){

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = position_x;
  goal.target_pose.pose.position.y = position_y;
  goal.target_pose.pose.orientation.w = orientation_w;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, we reached the goal");
  else
    ROS_INFO("We didn't make it :-(");


}




