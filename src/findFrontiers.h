#ifndef findFrontiers_
#define findFrontiers_

#include "myGetMap.h"

// double robotPos_x; // =  getRobotPosInMapFrame().getOrigin().x(); 
// double robotPos_y; // =  getRobotPosInMapFrame().getOrigin().y();
extern int robotPos_col; // = kartesisch2grid(grid, rob)
extern int robotPos_row; // = kartesisch2grid(grid, rob)
extern double robot_yaw; // =  getRobotPosInMapFrame().
extern std::vector<std::vector<int> > filteredCostmap;


struct Frontier {
    std::vector<gridCell> connected_f_cells;
    int numberOfElements; // = connected_f_cells.size();
    int pseudoMidPoint; // = ceil(numberOfElements / 2);
    gridCell centroid; // = der Schwerpunkt aller Zellen

    double directMinDistance; // der Minimale Abstand zum Roboter
                              // nach euklidischer Formel berechnen
    int idxOfMinDistance; // index of cell that contains the min distance to robot

    double rotationAngle; // angle between closest point and robot view direction (Rotation)

    double angleToGoalPoint; // winkel zwisch. Roboter und Zielpunkt (anfahren)
    bool shouldRotate;
    double cost;

    double goalSteeringAngle; // Richtung in die der Roboter bei erreichen des Ziel schaut! (nach fahrt)

    double distance2Frontier;
};

std::vector<gridCell> findFrontierCells(std::vector<std::vector<int> > searchRegion);

std::vector<Frontier> buildFrontiers(std::vector<gridCell> frontierCells);

Frontier fillFrontier(std::vector<gridCell> frontier);

std::vector<std::vector<int> > filterMap(std::vector<std::vector<int> > fullMap);


void floodFill(std::vector<std::vector<int> > & searchRegion, int x,int y,int oldcolor,int newcolor);

void maleFreieFlacheAus(std::vector<std::vector<int> > searchRegion);

void freeRobotOrigin(std::vector<std::vector<int> > & searchRegion, int x, int y);










#endif // end of findFrontiers_
