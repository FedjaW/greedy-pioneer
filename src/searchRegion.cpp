#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <math.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
    
#include "geometry_msgs/Twist.h" // für die Rotation
ros::Publisher velocity_publisher;  // für die Rotation



#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

void rotate();


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void sendGoal(double position_x, double position_y, double orientation_w);

 int grid_cell_x[9050];
 int grid_cell_y[9050];
        

int rows_cost;
int cols_cost;
double mapResolution_cost;
nav_msgs::OccupancyGrid cur_grid;
bool map_set = false;
void saveMap(const nav_msgs::OccupancyGrid& grid);
vector<vector<int> > grid_vec;

void extractFrontierRegion();
unsigned int startFrontier[1000];
unsigned int endFrontier[1000];
unsigned int nr = 0;
unsigned int bigFrontier[1000];
unsigned int n = 0;

// void setMarker(visualization_msgs::Marker& marker, ros::NodeHandle &nh);
void setMarker(ros::NodeHandle &nh, double mark_x, double mark_y, int id, double r, double g, double b);
// visualization_msgs::Marker marker_global;

// grid map
int rows;
int cols;
double mapResolution;
vector<vector<int> > grid;

double map_frame_x;
double map_frame_y;

void findFrontiers();

int x_such_min;
int x_such_max;
int y_such_min;
int y_such_max;
unsigned int global_k;

bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& msg, ros::NodeHandle &nh);

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

//__________________________________________________________________________________________main
int main (int argc, char** argv){

    ros::init(argc, argv, "getOccupancyGridMap"); 
    ros::NodeHandle nh;

    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    ros::Subscriber sub = nh.subscribe("odometry/filtered",10,OdomCallback);
    ros::Subscriber sub2 = nh.subscribe("/move_base/global_costmap/costmap", 10, saveMap); // costmap
    ros::Rate rate(10);
    rate.sleep();
    ros::spinOnce();

    if (!requestMap(nh))
        exit(-1);
    return 0;
}


void setMarker(ros::NodeHandle &nh, double mark_x, double  mark_y, int id, double r, double g, double b){
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 20);
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
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
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;
    
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;


// if (id % 2 == 0) marker.color.r = 0;
// else marker.color.r = 1;

    marker.lifetime = ros::Duration();
    while (marker_pub.getNumSubscribers() < 1){
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(0.1);
    }
    marker_pub.publish(marker);
    ROS_INFO("habe marker %d gesetzt", id);
        
}





void saveMap(const nav_msgs::OccupancyGrid& grid)
{
cur_grid = grid;
map_set = true;


ROS_INFO("Received a %d X %d Costmap @ %.3f m/px\n", 
                                                    cur_grid.info.width,
                                                    cur_grid.info.height,
                                                    cur_grid.info.resolution);

    rows_cost = cur_grid.info.height;
    cols_cost = cur_grid.info.width;
    mapResolution_cost = cur_grid.info.resolution;

   // Dynamically resize the Grid
    grid_vec.resize(rows_cost);
    for (int i = 0; i < rows_cost ; i++){
        grid_vec[i].resize(cols_cost);
    }

    int currCell = 0;
    for (int i = 0; i < rows_cost; i++){
        for(int j = 0; j < cols_cost; j++){
            grid_vec[i][j] = cur_grid.data[currCell];
            currCell++;
        }
    }


ROS_INFO("costmap map_set");
}




bool requestMap(ros::NodeHandle &nh){

  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;

  while (!ros::service::waitForService("dynamic_map", ros::Duration(3.0))){ // dynamic map ist a service provided by gmapping: dynamic_map(mav_msgs/GetMap)
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
                 setMarker(nh, mark_pos_x, mark_pos_y, id, 1, 1, 1);

  ziel_map_frame_x = 0 * map.info.resolution + map.info.origin.position.x;
  ziel_map_frame_y = 991 * map.info.resolution + map.info.origin.position.y;
                  mark_pos_x = ziel_map_frame_x;
                  mark_pos_y = ziel_map_frame_y;
                  id = 1;
                 setMarker(nh, mark_pos_x, mark_pos_y, id, 1, 1, 1);
                
                 // mark_pos_x = map_frame_x;
                 // mark_pos_y = map_frame_y;
                 // id = 2;
                 // setMarker(nh, mark_pos_x, mark_pos_y, id);

// ________________________________SUCH REGION __________________________________________________________________
                 id = 2;
                 setMarker(nh, map_frame_x, map_frame_y, id, 1, 1, 1); // map_frame ist die Position des Roboters

                 id = 3;
                 setMarker(nh, map_frame_x-6, map_frame_y-6, id, 0, 1, 0);
                    grid_frame_x = (unsigned int) ((map_frame_x-6 - map.info.origin.position.x)/map.info.resolution);
                    grid_frame_y = (unsigned int) ((map_frame_y-6 - map.info.origin.position.y)/map.info.resolution);
                    ROS_INFO("grid_frame_x_links_unten = %d \n", grid_frame_x); // x_such_min
                    ROS_INFO("grid_frame_y_links_unten = %d \n", grid_frame_y); // y_such_min
                    x_such_min = grid_frame_x; //x_such_min usw. sind alle Global; grid_frame_x nicht!
                    y_such_min = grid_frame_y;

                 id = 4;
                 setMarker(nh, map_frame_x-6, map_frame_y+6, id, 0, 1, 0);
                    grid_frame_x = (unsigned int) ((map_frame_x-6 - map.info.origin.position.x)/map.info.resolution);
                    grid_frame_y = (unsigned int) ((map_frame_y+6 - map.info.origin.position.y)/map.info.resolution);
                    ROS_INFO("grid_frame_x_links_oben = %d \n", grid_frame_x);
                    ROS_INFO("grid_frame_y_links_oben = %d \n", grid_frame_y); // y_such_max
                    y_such_max= grid_frame_y;

                 id = 5;
                 setMarker(nh, map_frame_x+6, map_frame_y+6, id, 0, 1, 0);
                    grid_frame_x = (unsigned int) ((map_frame_x+6 - map.info.origin.position.x)/map.info.resolution);
                    grid_frame_y = (unsigned int) ((map_frame_y+6 - map.info.origin.position.y)/map.info.resolution);
                    ROS_INFO("grid_frame_x_rechts_oben = %d \n", grid_frame_x);
                    ROS_INFO("grid_frame_y_rechts_oben = %d \n", grid_frame_y);

                 id = 6;
                 setMarker(nh, map_frame_x+6, map_frame_y-6, id, 0, 1, 0);
                    grid_frame_x = (unsigned int) ((map_frame_x+6 - map.info.origin.position.x)/map.info.resolution);
                    grid_frame_y = (unsigned int) ((map_frame_y-6 - map.info.origin.position.y)/map.info.resolution);
                    ROS_INFO("grid_frame_x_rechts_unten = %d \n", grid_frame_x); // x_such_max
                    ROS_INFO("grid_frame_y_rechts_unten = %d \n", grid_frame_y);
                    x_such_max= grid_frame_x;

// ________________________________SUCH REGION ENDE _________________________________________________________________

     // ziel_map_frame_x = grid_frame_x * map.info.resolution + map.info.origin.position.x;
     // ziel_map_frame_y = grid_frame_y * map.info.resolution + map.info.origin.position.y;
     //              mark_pos_x = ziel_map_frame_x;
     //              mark_pos_y = ziel_map_frame_y;
     //              id = 7;
     //             setMarker(nh, mark_pos_x, mark_pos_y, id);


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
    id = 7;
    // for(int i = 0; i < 217; i = i + 10){
    //                 ziel_map_frame_x = grid_cell_x[i] * map.info.resolution + map.info.origin.position.x;
    //                 ziel_map_frame_y = grid_cell_y[i] * map.info.resolution + map.info.origin.position.y;
    //                 mark_pos_x = ziel_map_frame_x;
    //                 mark_pos_y = ziel_map_frame_y;
    //                 id++;
    //                 setMarker(nh, mark_pos_x, mark_pos_y, id, 1, 1, 1);
    // }

    extractFrontierRegion();
    // Zeichne start und end frontier zelle einer großen Frontier Region (schlecht zu erkennn)
    // for(int i = 0; i < n; i++){
    //                 ziel_map_frame_x = grid_cell_x[startFrontier[bigFrontier[i]]] * map.info.resolution + map.info.origin.position.x;
    //                 ziel_map_frame_y = grid_cell_y[startFrontier[bigFrontier[i]]] * map.info.resolution + map.info.origin.position.y;
    //                 mark_pos_x = ziel_map_frame_x;
    //                 mark_pos_y = ziel_map_frame_y;
    //                 id++;
    //                 setMarker(nh, mark_pos_x, mark_pos_y, id, 1, 1, 0);
    //
    //                 ziel_map_frame_x = grid_cell_x[endFrontier[bigFrontier[i]]] * map.info.resolution + map.info.origin.position.x;
    //                 ziel_map_frame_y = grid_cell_y[endFrontier[bigFrontier[i]]] * map.info.resolution + map.info.origin.position.y;
    //                 mark_pos_x = ziel_map_frame_x;
    //                 mark_pos_y = ziel_map_frame_y;
    //                 id++;
    //                 setMarker(nh, mark_pos_x, mark_pos_y, id, 1, 1, 0);
    // }

    for(int l = 0; l < n; l++){
        double r,g,b;
        if(l == 0){
            r = 1;
            g = 0.5;
            b = 1;
        }
        else if(l == 1){
            r = 0;
            g = 1;
            b = 1;
        }
        else if(l == 2){
            r = 0;
            g = 0;
            b = 1;
        }
        else if(l == 3){
            r = 0;
            g = 1;
            b = 0;
        }
        else if(l == 4){
            r = 1;
            g = 0;
            b = 0;
        }
        else{ 
            r = 1;
            g = 1;
            b = 1;
        }
        for(int i = startFrontier[bigFrontier[l]]; i < endFrontier[bigFrontier[l]]; i = i + 1){
            ziel_map_frame_x = grid_cell_x[i] * map.info.resolution + map.info.origin.position.x;
            ziel_map_frame_y = grid_cell_y[i] * map.info.resolution + map.info.origin.position.y;
            mark_pos_x = ziel_map_frame_x;
            mark_pos_y = ziel_map_frame_y;
            id++;
            // ROS_INFO("color r=%.1f g=%.1f b=%.1f",r, g, b);
            setMarker(nh, mark_pos_x, mark_pos_y, id, r, g, b);
        }
    }
    
    int midFrontier = startFrontier[bigFrontier[0]] + ceil ((endFrontier[bigFrontier[0]] - startFrontier[bigFrontier[0]]) / 2);

    ROS_INFO("midFrontier = %d", midFrontier);
    ziel_map_frame_x = grid_cell_x[midFrontier] * map.info.resolution + map.info.origin.position.x;
    ziel_map_frame_y = grid_cell_y[midFrontier] * map.info.resolution + map.info.origin.position.y;
    mark_pos_x = ziel_map_frame_x;
    mark_pos_y = ziel_map_frame_y;

    double diff_x = map_frame_x - ziel_map_frame_x;
    double diff_y = map_frame_y - ziel_map_frame_y;
    ROS_INFO("diff_x = %f", diff_x);
    ROS_INFO("diff_y = %f", diff_y);
    
    double distance_to_Frontier = sqrt( pow(diff_x,2) + pow(diff_y,2) );
    ROS_INFO("distance_to_Frontier = %f", distance_to_Frontier);
    
    if(distance_to_Frontier < 6){
        ROS_INFO("ich rotiere!");
        rotate();
    }
    else sendGoal(mark_pos_x, mark_pos_y, 1.0);
}

// Read out the odometry _________________________________________________________
void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        map_frame_x = msg->pose.pose.position.x;
        map_frame_y = msg->pose.pose.position.y;
         // ROS_INFO("x: %f, y: %f",map_frame_x ,map_frame_y );
        ROS_INFO("odom callback");
}


// Find frontiers________________________________________________________________
// Finde alle Frontiers nach der Neumann-Nachbarschaft (ToDo: auf Moore-Nachbarschaft ändern)
void findFrontiers(){
    unsigned int k = 0;
    for(int i = grid.size() - 1; i>= 0 ; i--){      // durchsuche ganze Karte
        for(int j = 0; j < grid[0].size(); j++){

    // for(int i = y_such_max; i >= y_such_min; i--){       // durchsuche nur in der Such-Region
    //     for(int j = x_such_min; j < x_such_max; j++){
                if((grid[i][j] == 0) && (grid_vec[i][j] == 0)){ //TODO: was wenn es keine Costmap gibt?! -> dann auch kein grid_vec -> Abfrage machen
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
global_k = k;
}


void extractFrontierRegion(){
    int nhood_range = 1; // 1 -> direkte Nachbarschaft 
    int min_size = 30;
    for(int i = 0; i < global_k-1; i++){
        // ROS_INFO("i = %d", i);
        // ROS_INFO("fronteri 166 = %u", grid_cell_x[166]);
        // ROS_INFO("fronteri 167 = %u", grid_cell_x[167]);
        // ROS_INFO("__difference_x_166-167 = %u", grid_cell_x[166] - grid_cell_x[167]);
        //  int diff;
        // diff = grid_cell_x[166] - grid_cell_x[167];
        // ROS_INFO("diff = %d", diff);
        // ROS_INFO("__difference_x = %u", grid_cell_x[i] - grid_cell_x[i+1]);
        // ROS_INFO("__difference_y= %u", grid_cell_y[i] - grid_cell_y[i+1]);

        if ( (abs(grid_cell_x[i] - grid_cell_x[i+1]) <= nhood_range) && (abs(grid_cell_y[i] - grid_cell_y[i+1]) <= nhood_range)){
            startFrontier[nr] = i;
            // ROS_INFO("StartFrontier[%d] = %d",nr, startFrontier[nr]);
            do{
                i++; 
                // ROS_INFO("difference_x = %d", grid_cell_x[i] - grid_cell_x[i+1]);
                // ROS_INFO("difference_y= %d", grid_cell_y[i] - grid_cell_y[i+1]);
            }while((abs(grid_cell_x[i] - grid_cell_x[i+1]) <= nhood_range) && (abs(grid_cell_y[i] - grid_cell_y[i+1]) <= nhood_range));
            endFrontier[nr] = i;
            // ROS_INFO("endFrontier[%d] = %d",nr, endFrontier[nr]);
            // ROS_INFO("--------------");
            if (endFrontier[nr] - startFrontier[nr] >= min_size){
                ROS_INFO("StartFrontier[%d] = %d",nr, startFrontier[nr]);
                ROS_INFO("endFrontier[%d] = %d",nr, endFrontier[nr]);
                bigFrontier[n] = nr;
                ROS_INFO("bigFrontier[%d] = %d",n, bigFrontier[n]);
                ROS_INFO("--------------");
                n++;
            }
            nr++;
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



void rotate(){
    int counter = 0;
    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel_msg;
    do{
        vel_msg.angular.z = 1;
        counter++;
        velocity_publisher.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("in rotate schleife");
    }while(counter < 100);
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}
