#include <iostream>
#include "findFrontiers.h"
#include "angles/angles.h"
#include "math.h"



// Minimum size that a Frontier should have to be accepted as valid Frontier
// Size is the Number of neighbour-gridCells;
// TODO: Calculate the lenght out of Size -> with cell resolution
const unsigned int frontierMinSize = 20;
const double PI = 3.1415;

int robotPos_row; // = kartesisch2grid().row
int robotPos_col; // = kartesisch2grid().grid
double robot_yaw; // =  getRobotPosInMapFrame().

// searchRegion is for now the entiry Map
std::vector<gridCell> findFrontierCells(std::vector<std::vector<int> > searchRegion) {
int k = 0;
    std::vector<gridCell> frontierCells;
    gridCell myFrontierCell;

    for(int i = searchRegion.size() - 1; i>= 0 ; i--){      // durchsuche ganze Karte
        for(int j = 0; j < searchRegion[0].size(); j++){

            if(searchRegion[i][j] == 0) { // prüfen ob Zelle frei ist

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
                n--;
            }
        }

        if(root == true) {
            // frontiercell i is the root (wurzel) of 'this' frontier, so insert it at the beginning
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
    std::cout << "frontier_list.size() = "<< frontier_list.size() << std::endl;
    return frontier_list;
}


Frontier fillFrontier(std::vector<gridCell> frontier) {
            Frontier realFrontier;
            realFrontier.connected_f_cells = frontier;
            realFrontier.numberOfElements = frontier.size();
            realFrontier.pseudoMidPoint = ceil(realFrontier.numberOfElements / 2); //TODO: anstatt ceil auf int casten 
            realFrontier.centroid.row = 0;
            realFrontier.centroid.col = 0;

            for(int m = 0; m < realFrontier.numberOfElements; m++) {
                static double oldDistance = 10000000; // sehr hoch wählen damit die neue Distnz auf jedenfall kleiner ist 
                double newDistance = sqrt( pow( (realFrontier.connected_f_cells[m].row - robotPos_row),2) + 
                                           pow( (realFrontier.connected_f_cells[m].col - robotPos_col),2) );
                if(newDistance < oldDistance) {
                    realFrontier.directMinDistance = newDistance;
                    realFrontier.idxOfMinDistance = m;
                    oldDistance = newDistance;
                }

                realFrontier.centroid.row += realFrontier.connected_f_cells[m].row;
                realFrontier.centroid.col += realFrontier.connected_f_cells[m].col;
            }
            
            std::cout << "realFrontier.directMinDistance = "<< realFrontier.directMinDistance  << std::endl;
            // Berechne den centroid des Frontiers
            realFrontier.centroid.row  = realFrontier.centroid.row / realFrontier.numberOfElements;
            realFrontier.centroid.col  = realFrontier.centroid.col / realFrontier.numberOfElements;
            
            // Berechne den Winkelunterschied zwischen zielfrontier und Roboterausrichtung (yaw)
            // steering_angle = atan2(goal_pos_y - jackal_position_y, goal_pos_x - jackal_position_x);
            double steering_angle = atan2(realFrontier.connected_f_cells[realFrontier.idxOfMinDistance].col - robotPos_col, 
                                          realFrontier.connected_f_cells[realFrontier.idxOfMinDistance].row - robotPos_row);
            int steering_angle_normalized_deg = ((int) radiand2degrees(steering_angle)+360)%360;
            double steering_angle_normalized_rad = degrees2rad(steering_angle_normalized_deg);
            realFrontier.rotationAngle = steering_angle_normalized_rad - robot_yaw;

            return realFrontier;

}


double radiand2degrees(double angle_in_radiand){

            return angle_in_radiand * 180 / PI;
}

double degrees2rad(double angle_in_degrees){

            return angle_in_degrees * PI/180;
}



//
//
// bool isFrontierCellANeigbour(unsigned int idx1, unsigned int idx2) {
//      return (abs(frontier[idx1].row - frontierCells[idx2].row) <= 1) &&
//              (abs(frontier[idx1].col - frontierCells[idx2].col) <= 1);
// }

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------


// ALTE ART (SEHR LANGSAM) die Nachbarn der Frontierzellen zu finden
#if 0
std::vector<std::vector<gridCell> > buildFrontiers(std::vector<gridCell> frontierCells) {
    // std::cout << "HalliHallo 1" << std::endl;

    int oldFrontierSize;
    std::vector<int> visited;
    std::vector<gridCell> frontier;
    std::vector<std::vector<gridCell> > frontier_list;

    for(int i = 0; i < frontierCells.size(); i++) {
        bool root = false;
            if( ! (std::find(visited.begin(), visited.end(), i) != visited.end()) ) {
            
            for(int n = i+1; n < frontierCells.size(); n++) {
                if( ! (std::find(visited.begin(), visited.end(), n) != visited.end()) ) {

                    if( (abs(frontierCells[i].row - frontierCells[n].row) <= 1) && (abs(frontierCells[i].col - frontierCells[n].col) <= 1) ) {
                        visited.push_back(n);
                        if(root == false) {
                            frontier.push_back(frontierCells[i]);
                            visited.push_back(i);
                            root = true;
                        }
                        frontier.push_back(frontierCells[n]);
                    }
                }

            }

            int c = 1;
            do{
                oldFrontierSize = frontier.size();
                // std::cout << "oldFrontierSize = " << oldFrontierSize << std::endl;
                for(c; c < oldFrontierSize; c++) {
                    // std::cout << "frontier[" <<c<< "]" << ".row = " << frontier[c].row << " / frontier[" <<c<< "]" << ".col = " << frontier[c].col << std::endl;
                    for(int d = 0; d < frontierCells.size(); d++){
                        if( ! (std::find(visited.begin(), visited.end(), d) != visited.end()) ) {
                            // std::cout << "vergleiche mit frontier[" << d << "]" << ".row = " << frontierCells[d].row << " / frontier[" <<d<< "]" << ".col = " << frontierCells[d].col << std::endl;
                            if( (abs(frontier[c].row - frontierCells[d].row) <= 1) && (abs(frontier[c].col - frontierCells[d].col) <= 1) ) {
                                visited.push_back(d);
                                frontier.push_back(frontierCells[d]);
                            }
                        }
                    }
                }
                // std::cout << "durchlauf der while schleife Nr. " << loop << std::endl;
            }while(frontier.size() > oldFrontierSize);

            if(frontier.size() >= 1) 
                frontier_list.push_back(frontier);

            frontier.clear();
            oldFrontierSize = 0;
        }

    }
    std::cout << "frontier_list.size() = "<< frontier_list.size() << std::endl;
    return frontier_list;
}
#endif


        
