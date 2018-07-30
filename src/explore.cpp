#include <ros/ros.h>
#include "myGetMap.h"
#include "visualize.h"
#include "findFrontiers.h"
#include <thread>
#include "holyWatcher.h"
#include "move_service.h"
#include <algorithm> 
#include "explore.h"



bool sortFunction(const Frontier& f1, const Frontier& f2) {
    return f1.cost < f2.cost;
}


bool exploration(ros::NodeHandle &nh) {

    // nav_msgs::OccupancyGrid grid = requestMap(nh);
    // std::vector<std::vector<int> > gridMap = readMap(grid);

    // MODI 0: Zeige nur Frontiers an (endlosschleife)
    // MODI 1: NUR Anfahren; kein Rotieren!
    // MOD1 2: Nach jeder Fahrt 360° rotieren
    // MODI 3: Entscheidung treffen aufgrund der Fahrt/Rot-Kostenfunktion
    int MODI = 0;

    // für den Vizualizer der Frontiers
    std::vector<geometry_msgs::Pose> myVizPos;
    geometry_msgs::Pose dummyPos;
    geometry_msgs::Point myPoint;
    // ---------------------------------


    // für die Funktion fillFrontier / alle variablem sind Global
    double robotPos_x = getRobotPosInMapFrame().getOrigin().x(); // getRobotPosInMapFrame erzeugt ein 
    double robotPos_y = getRobotPosInMapFrame().getOrigin().y(); // globales Objekt vom TransformListener
    robotPos_col = kartesisch2grid(grid, robotPos_x, robotPos_y).col;
    robotPos_row = kartesisch2grid(grid, robotPos_x, robotPos_y).row;
    robot_yaw = tf::getYaw(getRobotPosInMapFrame().getRotation());
    std::cout << "col = " <<  robotPos_col << std::endl;
    std::cout << "row = " <<  robotPos_row << std::endl;


    // lock die aktuelle grid map
    std::vector<std::vector<int> > lock_gridMap = gridMap;

    // filter die Karte
    std::vector<std::vector<int> > filteredMap1 = filterMap(lock_gridMap);

    // befreie die fläche auf der der Roboter steht
    // freeRobotOrigin(filteredMap1, robotPos_row, robotPos_col);

    // floodFill die Fläche in der sich der Roboter befindet 
    // floodFill(filteredMap1, global_x, global_y , 0, 1);
    // floodFillNonRecursive(filteredMap1, robotPos_row, robotPos_col);
    floodFillIterativ(filteredMap1, robotPos_row, robotPos_col);

    // male die freie Fläche aus 
    maleFreieFlacheAus(filteredMap1);

    // finde alle frontierzellen
    std::vector<gridCell> frontierCells = findFrontierCells(filteredMap1);

    // std::vector<gridCell> frontierCells = findFrontierCells(gridMap);

    // baue Frontiers aus den f-zellen
    std::vector<Frontier> frontier_list = buildFrontiers(frontierCells);
    if(frontier_list.empty()) return false;

    // Kostenfunktion zuordnen!
    // Parameter der Kosten
    const double alpha = 0.7; // weight-factor of distance-cost
    const double beta = 0.40; // weight-factor of frontier-size 
    const double gamma = 0; // weight-factor of rotation-cost to reach the goalSteeringAngle 
    const double laser_radius = 4.5;
    const double FOV = 4.7123; // Field of View in radians 270°
    // const double FOV = 3.1415; // Field of View in radians 180°
    // const double FOV = 2.094; // Field of View in radians /120°

    static double maxDistance = 0;
    static double maxNumberOfElements = 0;


    // Fülle die frontierst mit distanz und dem goalSteeringAngle
    for(auto& frontier : frontier_list) {
        // myPoint = grid2Kartesisch(grid,
        //                           frontier.connected_f_cells[frontier.pseudoMidPoint].row, 
        //                           frontier.connected_f_cells[frontier.pseudoMidPoint].col);

        int nextPoint_ = frontier.numberOfElements - 1.7 * frontier.pseudoMidPoint;
        // int row_goalpoint = frontier_list[0].connected_f_cells[frontier_list[0].pseudoMidPoint].row;
        // int col_goalpoint = frontier_list[0].connected_f_cells[frontier_list[0].pseudoMidPoint].col;

        // Nimm Schwerpunḱt als Ziel
        int row_goalpoint_ = frontier.centroid.row;
        int col_goalpoint_ = frontier.centroid.col;

        // Keine unbekannten Ziele zulassen!!!
        if(lock_gridMap[col_goalpoint_][row_goalpoint_] == -1) {
            std::cout << "*Kein freien Schwerpunkt gefunden" << std::endl;
            // Nimm pseudomidpint als Ziel
            row_goalpoint_ = frontier.connected_f_cells[frontier.pseudoMidPoint].row;
            col_goalpoint_ = frontier.connected_f_cells[frontier.pseudoMidPoint].col;

            if(lock_gridMap[col_goalpoint_][row_goalpoint_] == -1) {
                std::cout << "*Kein freien pseudoMidPoint gefunden" << std::endl;
                do{
                    row_goalpoint_ = frontier.connected_f_cells[frontier.pseudoMidPoint+nextPoint_].row;
                    col_goalpoint_ = frontier.connected_f_cells[frontier.pseudoMidPoint+nextPoint_].col;
                    nextPoint_++;
                    if(frontier.pseudoMidPoint+nextPoint_ >= frontier.numberOfElements-1) {
                        std::cout << "*Kein freier Zielpunkt vorhanden" << std::endl;
                        // std::cout << "*row_goal = " << row_goalpoint_  << " col_goal = " << col_goalpoint_ << std::endl;
                        break;
                    }
                }while(lock_gridMap[col_goalpoint_][row_goalpoint_] == -1);
            }
            // std::cout << "Freies Ziel gefunden !" << std::endl;
            // std::cout << "row_goal = " << row_goalpoint  << " col_goal = " << col_goalpoint<< std::endl;
        }
        myPoint = grid2Kartesisch(grid, row_goalpoint_, col_goalpoint_);

        distanceAndSteering distAndSteer = getDistanceToFrontier(nh, myPoint);


        frontier.distance2Frontier =  distAndSteer.distance;
        // goalSteeringAngle für das sendGoal() notwendig
        frontier.goalSteeringAngle = distAndSteer.goalSteeringAngle;
        
        // update die max. werte für die relationsberechnung in der Kostenfunktion live (online)
        if(frontier.distance2Frontier > maxDistance)
            maxDistance = frontier.distance2Frontier;
        if(frontier.numberOfElements > maxNumberOfElements)
            maxNumberOfElements = frontier.numberOfElements;
    }
    

    // berechne jetzt die kostenfunktion
    // nach jeder Fahrt ergeben sich neue max werte durch die obige schleife
    // die max werte werden hier dann zur bewertung der Kosten herangezogen
    for(auto& frontier : frontier_list) {
        double rotationCost = 10000; // sehr hoch wählen!!! 
        double drivingCost = + alpha * (frontier.distance2Frontier / maxDistance)
                             - beta * (frontier.numberOfElements / maxNumberOfElements);
                             // + gamma * ((fabs(frontier.goalSteeringAngle - robot_yaw) - (FOV/2)) / M_PI);
        // std::cout << "drivingCost = "<<  drivingCost << std::endl;

        if(frontier.directMinDistance < laser_radius) {

            if(abs(frontier.rotationAngle) >= 0.8 * FOV/2) { // >= damit der Randbereich auch an-rotiert wird

                bool obstacle = isObstacleInViewField(nh,
                                                  grid,
                                                  robotPos_col, 
                                                  robotPos_row,
                                                  frontier.connected_f_cells[frontier.idxOfMinDistance].col, 
                                                  frontier.connected_f_cells[frontier.idxOfMinDistance].row);

                std::cout << "obstacle = " << obstacle << std::endl;

                if(!obstacle) {
                    // wenn die obigen 3 Bedingungen (if's) nicht erfüllt sind
                    // bleibt die rotationCost so hoch dass automatisch shouldRotate = false bleibt 
                    rotationCost = 20 * (- beta * (frontier.numberOfElements / maxNumberOfElements) + gamma * ((fabs(frontier.rotationAngle) - (FOV/2)) / M_PI));
                    // rotationCost = - 1.5 * beta * (frontier.numberOfElements) + gamma * ((fabs(frontier.rotationAngle) - (FOV/2)) / M_PI);
                    // rotationCost = -1;
                    std::cout << "rotationCost = "<<  rotationCost << std::endl;
                }
            }
            else {
                std::cout << "Frontier liegt bereits in Sicht" << std::endl;
            }
        
        }
        else {
                std::cout << "Closest Point liegt nicht in Laser Reichweite" << std::endl;
        }

        if(drivingCost > rotationCost) {
            frontier.shouldRotate = true;
            frontier.cost = rotationCost;
            // std::cout << std::endl;
            std::cout << "ROTATIONS-kosten = gamma * ((|phi|-FOV/2) / PI) - beta * (sizeOfFront / maxSizeOfFront)" << std::endl;
            std::cout << "   " << rotationCost << "  =   " << gamma <<" * (" << fabs(frontier.rotationAngle) << "  -  " << (FOV/2) << " / " << "PI) " <<  "      - "  << beta << " * (" << frontier.numberOfElements << " / " << maxNumberOfElements << ")"<< std::endl;
            std::cout << std::endl;
        }
        else {
            frontier.shouldRotate = false;
            frontier.cost = drivingCost;
            // std::cout << std::endl;
            std::cout << "FAHRT-kosten = alpha * (distance / maxDistance) - beta * (sizeOfFront / maxSizeOfFront)" << std::endl;
            std::cout << "   " << drivingCost << "   = " << alpha<< " * (" << frontier.distance2Frontier << " / " << maxDistance <<  ")      - "<< beta << " *      ("<< frontier.numberOfElements << "   /   " << maxNumberOfElements << ")" << std::endl;
            std::cout << std::endl;
        }
    }

    std::cout << "--------------------------------------------------------------" << std::endl;

    // Frontiers nach Konsten sortieren. Günstgigestes als erstes
    std::sort(frontier_list.begin(), frontier_list.end(), sortFunction);

    for(auto& frontier : frontier_list) {
        std::cout << "kosten sortiert = "<<  frontier.cost << " / " << "shouldRotate = " << frontier.shouldRotate << std::endl;
    }
    std::cout << std::endl;


    Visualizer myVisualize;
    double r = 0;
    double g = 0;
    double b = 0;
    
    // viusalisiere alle frontiers. Grün -> Zielfrontier
    for(auto& frontier : frontier_list) {
        toggleColor(r,g,b);
        for(int i = 0; i < frontier.connected_f_cells.size(); i++) {
            myPoint = grid2Kartesisch(grid, 
                                        frontier.connected_f_cells[i].row, 
                                        frontier.connected_f_cells[i].col);
            dummyPos.position.x = myPoint.x;
            dummyPos.position.y = myPoint.y;
            myVizPos.push_back(dummyPos);
        }
        myVisualize.setMarkerArray(nh, myVizPos, r,g,b,0);
        myVizPos.clear();
    }

    // caluclate connectivity factor of the all Frontiers
    for(auto& frontier : frontier_list) {
        double numberOfNhoodRelations = 0;
        for(int i = 0; i < frontier.connected_f_cells.size(); i++) {
            for(int d = 0; d < frontier.connected_f_cells.size(); d++){
                if(d != i) {
                    if( (abs(frontier.connected_f_cells[i].row - frontier.connected_f_cells[d].row) <= 1) && 
                        (abs(frontier.connected_f_cells[i].col - frontier.connected_f_cells[d].col) <= 1)) {

                        numberOfNhoodRelations = numberOfNhoodRelations + 1;

                    }
                }
            }
        }
        double minimumNhoodRelations = (2 * frontier.connected_f_cells.size() - 2);
        double conn_factor = numberOfNhoodRelations / minimumNhoodRelations;
        std::cout << "numberOfNhoodRelations = "<<  numberOfNhoodRelations << std::endl;
        std::cout << "minimumNhoodRelations = "<<  minimumNhoodRelations << std::endl;
        std::cout << "conn_factor = "<<  conn_factor << std::endl;
        std::cout << " ---------------  " << std::endl;

    }




    // Fahre immer das erste Frontier in der liste an weil es das günstigste ist!
    // Zielpunkt: das erste (kostengünstigste) Frontier in xy-koordinaten

    //nextPoint geht an den Anfang des Frontiers und nimmt das als Zielpunkt und nicht Pseudomidpoint
    int nextPoint = frontier_list[0].numberOfElements - 1.7 * frontier_list[0].pseudoMidPoint;
    // int row_goalpoint = frontier_list[0].connected_f_cells[frontier_list[0].pseudoMidPoint].row;
    // int col_goalpoint = frontier_list[0].connected_f_cells[frontier_list[0].pseudoMidPoint].col;

    // Nimm Schwerpunḱt als Ziel
    int row_goalpoint = frontier_list[0].centroid.row;
    int col_goalpoint = frontier_list[0].centroid.col;

    // Keine unbekannten Ziele zulassen!!!
    if(lock_gridMap[col_goalpoint][row_goalpoint] == -1) {
        std::cout << "Kein freien Schwerpunkt gefunden" << std::endl;
        // Nimm pseudomidpint als Ziel
        row_goalpoint = frontier_list[0].connected_f_cells[frontier_list[0].pseudoMidPoint].row;
        col_goalpoint = frontier_list[0].connected_f_cells[frontier_list[0].pseudoMidPoint].col;

        if(lock_gridMap[col_goalpoint][row_goalpoint] == -1) {
            std::cout << "Kein freien pseudoMidPoint gefunden" << std::endl;
            do{
                row_goalpoint = frontier_list[0].connected_f_cells[frontier_list[0].pseudoMidPoint+nextPoint].row;
                col_goalpoint = frontier_list[0].connected_f_cells[frontier_list[0].pseudoMidPoint+nextPoint].col;
                nextPoint++;
                if(frontier_list[0].pseudoMidPoint+nextPoint >= frontier_list[0].numberOfElements-1) {
                    std::cout << "Kein freier Zielpunkt vorhanden" << std::endl;
                    std::cout << "row_goal = " << row_goalpoint  << " col_goal = " << col_goalpoint<< std::endl;
                    break;
                }
            }while(lock_gridMap[col_goalpoint][row_goalpoint] == -1);
        }
        // std::cout << "Freies Ziel gefunden !" << std::endl;
        // std::cout << "row_goal = " << row_goalpoint  << " col_goal = " << col_goalpoint<< std::endl;
    }
    myPoint = grid2Kartesisch(grid, row_goalpoint, col_goalpoint);

    // myPoint = grid2Kartesisch(grid,
    //                               frontier_list[0].connected_f_cells[frontier_list[0].pseudoMidPoint].row, 
    //                               frontier_list[0].connected_f_cells[frontier_list[0].pseudoMidPoint].col);
   
    
    switch(MODI) {
        case 0: {
                    std::cout << "MODUS 0 AKTIV - Zeige nur alle Frontiers" << std::endl;
                    while(1);

                    return 0;
                }

        // MODI_1: NUR Anfahren; kein Rotieren!
        case 1: {
                    std::cout << "MODUS 1 AKTIV - Nur fahren" << std::endl;
                    sendGoal(myPoint.x, myPoint.y, frontier_list[0].goalSteeringAngle, getDistanceToFrontier(nh, myPoint).distance);
                    break;
                }

        // MOD1_2: Nach jeder Fahrt 360° rotieren
        case 2: {
                    std::cout << "MODUS 2 AKTIV - 360° Rotation" << std::endl;
                    sendGoal(myPoint.x, myPoint.y, frontier_list[0].goalSteeringAngle, getDistanceToFrontier(nh, myPoint).distance);
                    rotate360(nh);
                    break;
                }

        // MODI 3: Entscheidung treffen aufgrund der Fahrt/Rot-Kostenfunktion
        // Rotation und Anfahren gemixt
        case 3: {
                    std::cout << "MODUS 3 AKTIV - Greedy Pioneer" << std::endl;

                    if(frontier_list[0].shouldRotate == 0) {
                        sendGoal(myPoint.x, myPoint.y, frontier_list[0].goalSteeringAngle, getDistanceToFrontier(nh, myPoint).distance);
                    }
                    else {
                            std::cout << "ROTATION ANGLE ------------> " << frontier_list[0].rotationAngle << std::endl;
                            rotate(nh, frontier_list[0].rotationAngle);
                    }
                    break;
                }
    }


    myVisualize.setMarkerArray(nh, myVizPos, r,g,b,1); // delete ALL markerArrays

    // NOTE: Kp ob ich das brauche
    frontierCells.clear();
    frontier_list.clear();
    ros::Rate rate(10);
    rate.sleep();
    rate.sleep();
    rate.sleep();
    rate.sleep();
    std::cout << "******************* ******************* ******************* ******************* ******************* ******************* START AGAIN " << std::endl;
    return true;
}


void toggleColor(double &r, double &g, double &b) {
        if((r == 0) && (g == 0) && (b == 0)) {
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
            r = 1;
            g = 1;
            b = 1;
        }
        else if((r == 1) && (g == 1) && (b == 1)) {
            r = 0.2;
            g = 0.6;
            b = 0.4;
        }
        else if((r == 0.2) && (g == 0.6) && (b == 0.4)) {
            r = 0.4;
            g = 0.8;
            b = 0.6;
        }
        else if((r == 0.4) && (g == 0.8) && (b == 0.6)) {
            r = 0.7;
            g = 0.1;
            b = 0.9;
        }
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
    rate.sleep(); 

    std::thread timerThread(everySecond);
    while(exploration(nh));

    // watcherThread.detach();

    return 0;
}
