#include <iostream>
#include "findFrontiers.h"
#include "angles/angles.h"
#include "math.h"
#include "holyWatcher.h"
#include "visualize.h"
#include <queue>
#include <stack>
// #include "myGetMap.h"

// Minimum size that a Frontier should have to be accepted as valid Frontier
// Size is the Number of neighbour-gridCells;
// TODO: Calculate the lenght out of Size -> with cell resolution
const unsigned int frontierMinSize = 50;

// Diese Variablen brauche ich buildFrontier()
// Habe sie global gemacht da ich nicht weiß
// wie ich sie sonst da reinbekommen soll
int robotPos_row; // = kartesisch2grid().row
int robotPos_col; // = kartesisch2grid().grid
double robot_yaw; // =  getRobotPosInMapFrame().

std::vector<std::vector<int> > filteredCostmap;


std::vector<std::vector<int> > filterMap(std::vector<std::vector<int> > filteredMap) {
    filteredCostmap = costmap;
    // std::vector<std::vector<int> > filteredMap = fullMap;
    ros::NodeHandle nh2;
    // für die visualisierung
    Visualizer myVisualize4;
    std::vector<geometry_msgs::Pose> myVizPos;
    geometry_msgs::Point myPoint;
    geometry_msgs::Pose dummyPos;

    double filteredCells = 0;

    // filter mechanismus
    int nhoodKnownCellCounter = 0; // number off neighbourhood-cells of an unknown cell
    for(int i = filteredMap.size() - 2; i >= 1 ; i--){      // durchsuche ganze Karte
        for(int j = 1; j < filteredMap[0].size() - 1 ; j++){

            if(filteredMap[i][j] == -1  && costmap[i][j] <= 0) { // Wenn Zelle unbekannt

                if(filteredMap[i+1][j] == 0) // prüfen ob der Nachbar bekannt und frei ist
                    nhoodKnownCellCounter++;
                if(filteredMap[i][j+1] == 0)
                    nhoodKnownCellCounter++;
                if(filteredMap[i-1][j] == 0)
                    nhoodKnownCellCounter++;
                if(filteredMap[i][j-1] == 0)
                    nhoodKnownCellCounter++;

                if(filteredMap[i+1][j-1] == 0)
                    nhoodKnownCellCounter++;
                if(filteredMap[i+1][j+1] == 0)
                    nhoodKnownCellCounter++;
                if(filteredMap[i-1][j-1] == 0)
                    nhoodKnownCellCounter++;
                if(filteredMap[i-1][j+1] == 0)
                    nhoodKnownCellCounter++;

                if(nhoodKnownCellCounter >= 5) { // bei 5 oder mehr (max 8) bekannten benachbarten Zellen tue:
                    // Setze diese unbekannte Zelle als BEKANNT weil umgeben von bekanntem Gebiet
                    filteredMap[i][j] = 0;
                    // filteredCostmap[i][j] = 0;

                    // std::cout << "filteredMap[i][j] = " << filteredMap[i][j] << std::endl;
                    filteredCells++;

                    // fülle den vis marker 
                    myPoint = grid2Kartesisch(grid, j, i);
                    dummyPos.position.x = myPoint.x;
                    dummyPos.position.y = myPoint.y;
                    myVizPos.push_back(dummyPos);
                }
                nhoodKnownCellCounter = 0;
            }
        }
    }
    std::cout << "#filteredCells = " << filteredCells << std::endl;

    // myVisualize4.setMarkerArray(nh2, myVizPos, 1, 0, 0, 0);
    myVisualize4.setMarkerArray(nh2, myVizPos, 0.1, 0.6, 0.6, 0);
    myVizPos.clear();

    return filteredMap;
}


void freeRobotOrigin(std::vector<std::vector<int> > & searchRegion, int x, int y) {

    // für die visualisierung
    ros::NodeHandle nh3;
    Visualizer myVisualize6;
    std::vector<geometry_msgs::Pose> myVizPos;
    geometry_msgs::Point myPoint;
    geometry_msgs::Pose dummyPos;
    int squareSize = 10;

   for(int i = x - squareSize; i < x + squareSize ; i++)  {
        for(int j = y - squareSize; j < y + squareSize ; j++)  {
            searchRegion[i][j] = 0;
            myPoint = grid2Kartesisch(grid, i,j);
            dummyPos.position.x = myPoint.x;
            dummyPos.position.y = myPoint.y;
            myVizPos.push_back(dummyPos);
        }
   }

    // myVisualize6.setMarkerArray(nh3, myVizPos, 0.6, 0.1, 0.6, 0);
    myVizPos.clear();
}



void floodFill(std::vector<std::vector<int> > & searchRegion, int x,int y,int oldcolor,int newcolor)
{
    if(searchRegion[x][y] == oldcolor && costmap[x][y] < 40)
    {

        searchRegion[x][y] = newcolor;
        // 8er-Nachbarschaft!
        // so wird sicher jeder Bereich "angemalt"
        floodFill(searchRegion,x+1,y,oldcolor,newcolor);
        floodFill(searchRegion,x,y+1,oldcolor,newcolor);
        floodFill(searchRegion,x-1,y,oldcolor,newcolor);
        floodFill(searchRegion,x,y-1,oldcolor,newcolor);
        floodFill(searchRegion,x+1,y+1,oldcolor,newcolor);
        floodFill(searchRegion,x-1,y-1,oldcolor,newcolor);
        floodFill(searchRegion,x+1,y-1,oldcolor,newcolor);
        floodFill(searchRegion,x-1,y+1,oldcolor,newcolor);
    }
}



void floodFillIterativ(std::vector<std::vector<int> > & searchRegion, int x,int y) {

    int costFactor = 40;
    gridCell startCell;
    gridCell cell_i;
    gridCell cell_nbr_1;
    gridCell cell_nbr_2;
    gridCell cell_nbr_3;
    gridCell cell_nbr_4;
    std::stack<gridCell> myStack;
    startCell.row = y;
    startCell.col = x;
    myStack.push(startCell);
   // stack.push(x, y);

   while(!myStack.empty()) {
        
        cell_i = myStack.top();
        myStack.pop();

        // std::cout << "inside while" << std::endl;
       if(searchRegion[cell_i.row][cell_i.col] == 0 && costmap[cell_i.row + 1][cell_i.col] < costFactor) {

            searchRegion[cell_i.row][cell_i.col] = 1;

            cell_nbr_1.row = cell_i.row + 1;
            cell_nbr_1.col = cell_i.col;

            cell_nbr_2.row = cell_i.row - 1;
            cell_nbr_2.col = cell_i.col;

            cell_nbr_3.row = cell_i.row;
            cell_nbr_3.col = cell_i.col + 1;

            cell_nbr_4.row = cell_i.row;
            cell_nbr_4.col = cell_i.col - 1;

            myStack.push(cell_nbr_1);
            myStack.push(cell_nbr_2);
            myStack.push(cell_nbr_3);
            myStack.push(cell_nbr_4);
       }
   }
   return;
}







void floodFillNonRecursive(std::vector<std::vector<int> > & searchRegion, int x,int y) {
int costFactor = 150;
    // für die visualisierung des initialen Punktes
    ros::NodeHandle nh3;
    Visualizer myVisualize7;
    std::vector<geometry_msgs::Pose> myVizPos;
    geometry_msgs::Point myPoint;
    geometry_msgs::Pose dummyPos;


    myPoint = grid2Kartesisch(grid, x,y);
    dummyPos.position.x = myPoint.x;
    dummyPos.position.y = myPoint.y;
    myVizPos.push_back(dummyPos);

    myVisualize7.setMarkerArray(nh3, myVizPos, 1, 0, 0, 0);
    myVizPos.clear();

    gridCell cell_i;
    gridCell startCell;
    startCell.row = y;
    startCell.col = x;
    std::queue<gridCell> bfs;
    
    bfs.push(startCell);

    gridCell cell_nbr;

    while(!bfs.empty()) {

        // std::cout << "inside while" << std::endl;
        cell_i = bfs.front();

        searchRegion[cell_i.row][cell_i.col] = 1;

        bfs.pop();

        // std::cout << "searchRegion[cell_i.row + 1][cell_i.col] = " << searchRegion[cell_i.row + 1][cell_i.col]<< std::endl;
        // std::cout << "costmap[cell_i.row + 1][cell_i.col] = " << costmap[cell_i.row + 1][cell_i.col]<< std::endl;
        if(searchRegion[cell_i.row + 1][cell_i.col] == 0 && costmap[cell_i.row + 1][cell_i.col] < costFactor){ 
            cell_nbr.row = cell_i.row + 1;
            cell_nbr.col = cell_i.col;
            bfs.push(cell_nbr);
        }
        if(searchRegion[cell_i.row][cell_i.col + 1] == 0 && costmap[cell_i.row][cell_i.col + 1] < costFactor){
            cell_nbr.row = cell_i.row;
            cell_nbr.col = cell_i.col + 1;
            bfs.push(cell_nbr);
        }
        if(searchRegion[cell_i.row - 1][cell_i.col] == 0 && costmap[cell_i.row - 1][cell_i.col] < costFactor){
            cell_nbr.row = cell_i.row - 1;
            cell_nbr.col = cell_i.col;
            bfs.push(cell_nbr);
        }
        if(searchRegion[cell_i.row][cell_i.col - 1] == 0 && costmap[cell_i.row][cell_i.col - 1] < costFactor){
            cell_nbr.row = cell_i.row;
            cell_nbr.col = cell_i.col - 1;
            bfs.push(cell_nbr);
        }

    }

}













void maleFreieFlacheAus(std::vector<std::vector<int> > searchRegion) {
    
    // für die visualisierung
    ros::NodeHandle nh3;
    Visualizer myVisualize5;
    std::vector<geometry_msgs::Pose> myVizPos;
    geometry_msgs::Point myPoint;
    geometry_msgs::Pose dummyPos;

    int reso = 100;
    for(int i = searchRegion.size() - 1; i >= 0 ; i = i - searchRegion.size()/reso){// durchsuche Karte
        for(int j = 0; j < searchRegion[0].size(); j = j + searchRegion.size()/reso){

            if(searchRegion[i][j] == 1) { // prüfen ob Zelle frei ist

                    // fülle den vis marker 
                    myPoint = grid2Kartesisch(grid, j, i);
                    dummyPos.position.x = myPoint.x;
                    dummyPos.position.y = myPoint.y;
                    myVizPos.push_back(dummyPos);

            }
        }
    }
    myVisualize5.setMarkerArray(nh3, myVizPos, 1, 1, 0, 0);
    myVizPos.clear();
}
            




// searchRegion is for now the entiry Map
std::vector<gridCell> findFrontierCells(std::vector<std::vector<int> > searchRegion) {
    int k = 0;
    std::vector<gridCell> frontierCells;
    gridCell myFrontierCell;

    // std::cout << "suchregion.size() = "<< searchRegion.size() << std::endl;
    // std::cout << "suchregion[0].size() = "<< searchRegion[0].size() << std::endl;
    for(int i = searchRegion.size() - 1; i >= 0 ; i--){      // durchsuche ganze Karte
        for(int j = 0; j < searchRegion[0].size(); j++){

            // if(searchRegion[i][j] == 0 && filteredCostmap[i][j] <= 0) { // prüfen ob Zelle frei ist
            // prüfen ob Zelle auf searchRegion also auf der gridMap frei ist
            // und ob die Zelle auf der CostMap frei bwz unbekannt ist (beides valide)
            if(searchRegion[i][j] == 1 && costmap[i][j] <= 0) {

                // 4nhood
                if(searchRegion[i+1][j] == -1){ // prüfen ob der Nachbar unbekannt ist
                    myFrontierCell.row = j;
                    myFrontierCell.col = i;
                    frontierCells.push_back(myFrontierCell);
                    // std::cout << "frontier rechts: row " << j << " / col " << i << std::endl;
                    k++;
                }
                else if(searchRegion[i][j+1] == -1){
                    myFrontierCell.row = j;
                    myFrontierCell.col = i;
                    frontierCells.push_back(myFrontierCell);
                    // std::cout << "frontier oben: row " << j << " / col " << i << std::endl;
                    k++;
                }
                else if(searchRegion[i-1][j] == -1){
                    myFrontierCell.row = j;
                    myFrontierCell.col = i;
                    frontierCells.push_back(myFrontierCell);
                    // std::cout << "frontier links: row " << j << " / col " << i << std::endl;
                    k++;
                }
                else if(searchRegion[i][j-1] == -1){
                    myFrontierCell.row = j;
                    myFrontierCell.col = i;
                    frontierCells.push_back(myFrontierCell);
                    // std::cout << "frontier unten: row " << j << " / col " << i << std::endl;
                    k++;
                }
                
                //// 8 nhood ist die Falsche nhood an dieser Stelle
                //// weil sonst innrere Zellen mitgezählt werden
                //// und diese machen den conn factor kaputt!!!!

                // else if(searchRegion[i+1][j-1] == -1) {
                //     myFrontierCell.row = j;
                //     myFrontierCell.col = i;
                //     frontierCells.push_back(myFrontierCell);
                //     k++;
                // }
                //
                // else if(searchRegion[i+1][j+1] == -1) {
                //     myFrontierCell.row = j;
                //     myFrontierCell.col = i;
                //     frontierCells.push_back(myFrontierCell);
                //     // std::cout << "frontier unten: row " << j << " / col " << i << std::endl;
                //     k++;
                // }
                // else if(searchRegion[i-1][j-1] == -1) {
                //     myFrontierCell.row = j;
                //     myFrontierCell.col = i;
                //     frontierCells.push_back(myFrontierCell);
                //     // std::cout << "frontier unten: row " << j << " / col " << i << std::endl;
                //     k++;
                // }
                // else if(searchRegion[i-1][j+1] == -1) {
                //     myFrontierCell.row = j;
                //     myFrontierCell.col = i;
                //     frontierCells.push_back(myFrontierCell);
                //     // std::cout << "frontier unten: row " << j << " / col " << i << std::endl;
                //     k++;
                // }
            }
        }
    }
    std::cout << "#FrontierCells = " << k << std::endl;
    return frontierCells;
}



// Neue bessere Art und Weise die Nachbarn der frontierzellen zu finden 
std::vector<Frontier> buildFrontiers(std::vector<gridCell> frontierCells) {

    int oldFrontierSize;
    std::vector<gridCell> frontier; //TODO: Umbennen weil in dem vec sind alle vebundenen F-Zellen eines Frontiers enthalten
    std::vector<Frontier> frontier_list;  

    for(int i = 0; i < frontierCells.size(); i++) {
        bool root = false;
        for(int n = i+1; n < frontierCells.size(); n++) {
            if( (abs(frontierCells[i].row - frontierCells[n].row) <= 1) && 
                (abs(frontierCells[i].col - frontierCells[n].col) <= 1) ) {
            // if(isFrontierCellANeigbour(i,n)) {
                frontier.push_back(frontierCells[n]);
                frontierCells.erase(frontierCells.begin()+n);
                root = true;
                n--; // NOTE: ist nötig weil alle Elemente einen aufrutschen
            }
        }

        if(root == true) {
            // frontiercell i is the root (wurzel) of 'this' frontier, so insert it at the beginning

            // ---------------------------------------------------------------------------------------------------
            // ander würde es auch nicht funktionieren weil das erste Element in froniter übersprungen wird (c=1)
            frontier.insert(frontier.begin(), frontierCells[i]); 
            frontierCells.erase(frontierCells.begin()+i);
            i--;
        }

        // Ab hier stehen alle Nachbarn von frontierCells[i] in einem Vector "frontier" 
        // frontierCells[i] steht an erster Stelle in dem Vector "frontier"

        int c = 1;
        do{
            oldFrontierSize = frontier.size();
            for(c; c < oldFrontierSize; c++) {
                for(int d = 0; d < frontierCells.size(); d++){
                    if( (abs(frontier[c].row - frontierCells[d].row) <= 1) && 
                        (abs(frontier[c].col - frontierCells[d].col) <= 1) ) {
                    // if(isFrontierCellANeigbour(c,d)) {
                        frontier.push_back(frontierCells[d]);
                        frontierCells.erase(frontierCells.begin()+d);
                        d--;
                    }
                }
            }
        }while(frontier.size() > oldFrontierSize);


        if(frontier.size() >= frontierMinSize)  {
            // fülle das Frontier mit Eigenschaften und pushe es in den vector
            frontier_list.push_back(fillFrontier(frontier));
        }

        frontier.clear();
        oldFrontierSize = 0;

    }
    std::cout << "........................" << std::endl;
    std::cout << "frontier_list.size() = "<< frontier_list.size() << std::endl;
    std::cout << "........................" << std::endl;
    return frontier_list;
}


Frontier fillFrontier(std::vector<gridCell> frontier) {
    Frontier realFrontier;
    realFrontier.connected_f_cells = frontier;
    realFrontier.numberOfElements = frontier.size();
    std::cout << "___________________________________________" << std::endl;
    std::cout << "realFrontier.numberOfElements = "<< realFrontier.numberOfElements << std::endl;
    realFrontier.pseudoMidPoint = ceil(realFrontier.numberOfElements / 2); //TODO: anstatt ceil auf int casten 
    realFrontier.centroid.row = 0;
    realFrontier.centroid.col = 0;

    double oldDistance = 100000; // sehr hoch wählen damit die neue Distnz auf jedenfall kleiner ist 
    for(int m = 0; m < realFrontier.numberOfElements; m++) {
        double newDistance = sqrt( pow((realFrontier.connected_f_cells[m].row - robotPos_row),2) + 
                                   pow((realFrontier.connected_f_cells[m].col - robotPos_col),2) ) * 0.02;
        if(newDistance < oldDistance) {
            realFrontier.directMinDistance = newDistance;
            realFrontier.idxOfMinDistance = m;
            oldDistance = newDistance;
        }

        realFrontier.centroid.row += realFrontier.connected_f_cells[m].row;
        realFrontier.centroid.col += realFrontier.connected_f_cells[m].col;
    }

    std::cout << "realFrontier.directMinDistance = "<< realFrontier.directMinDistance << std::endl;
    // Berechne den centroid des Frontiers
    realFrontier.centroid.row  = realFrontier.centroid.row / realFrontier.numberOfElements;
    realFrontier.centroid.col  = realFrontier.centroid.col / realFrontier.numberOfElements;

    // Berechne den Winkelunterschied zwischen Zielfrontier (minDistande) und Roboterausrichtung (yaw)
    // steering_angle = atan2(goal_pos_y - jackal_position_y, goal_pos_x - jackal_position_x);
    double steering_angle = atan2(realFrontier.connected_f_cells[realFrontier.pseudoMidPoint].col - robotPos_col, 
                                  realFrontier.connected_f_cells[realFrontier.pseudoMidPoint].row - robotPos_row);
    // double steering_angle = atan2(realFrontier.connected_f_cells[realFrontier.idxOfMinDistance].col - robotPos_col, 
    //                               realFrontier.connected_f_cells[realFrontier.idxOfMinDistance].row - robotPos_row);
    realFrontier.rotationAngle = steering_angle - robot_yaw;
    std::cout << "realFrontier.rotationAngle = "<< realFrontier.rotationAngle << std::endl;

    // // Berechne den Winkelunterschied zwischen Zielfrontier (pseudoMidpoiont) und Roboterausrichtung (yaw)
    // // brauche das hier für die Kostenberechnung
    // steering_angle = atan2(realFrontier.connected_f_cells[realFrontier.pseudoMidPoint].col - robotPos_col, 
    //                        realFrontier.connected_f_cells[realFrontier.pseudoMidPoint].row - robotPos_row);
    // realFrontier.angleToGoalPoint = steering_angle - robot_yaw;
    // std::cout << "realFrontier.angleToGoalPoint = "<< realFrontier.angleToGoalPoint << std::endl;

    return realFrontier;
}



