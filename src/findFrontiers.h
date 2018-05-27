#ifndef findFrontiers_
#define findFrontiers_

#include "myGetMap.h"

// double robotPos_x; // =  getRobotPosInMapFrame().getOrigin().x(); 
// double robotPos_y; // =  getRobotPosInMapFrame().getOrigin().y();
extern int robotPos_col; // = kartesisch2grid(grid, rob)
extern int robotPos_row; // = kartesisch2grid(grid, rob)
extern double robot_yaw; // =  getRobotPosInMapFrame().


struct Frontier {
    std::vector<gridCell> connected_f_cells;
    int numberOfElements; // = connected_f_cells.size();
    int pseudoMidPoint; // = ceil(numberOfElements / 2);
    gridCell centroid; // = der Schwerpunkt aller Zellen

    double directMinDistance; // der Minimale Abstand zum Roboter
                              // nach euklidischer Formel berechnen
    int idxOfMinDistance; // index of cell that contains the min distance to robot

    double rotationAngle; // angle between closest point and robot view direction

};

std::vector<gridCell> findFrontierCells(std::vector<std::vector<int> > searchRegion);

std::vector<Frontier> buildFrontiers(std::vector<gridCell> frontierCells);

Frontier fillFrontier(std::vector<gridCell> frontier);


double radiand2degrees(double angle_in_radiand);
double degrees2rad(double angle_in_degrees);














#endif // end of findFrontiers_
