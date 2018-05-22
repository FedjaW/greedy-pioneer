#include <iostream>
#include "findFrontiers.h"


// searchRegion is for now the entiry Map
std::vector<gridCell> findFrontierCells(std::vector<std::vector<int> > searchRegion) {
int k = 0;
    std::vector<gridCell> frontierCell;
    gridCell myFrontierCell;

    for(int i = searchRegion.size() - 1; i>= 0 ; i--){      // durchsuche ganze Karte
        for(int j = 0; j < searchRegion[0].size(); j++){

            if(searchRegion[i][j] == 0) { // prüfen ob Zelle frei ist

                if(searchRegion[i+1][j] == -1){ // prüfen ob der Nachbar unbekannt ist
                    myFrontierCell.row = j;
                    myFrontierCell.col = i;
                    frontierCell.push_back(myFrontierCell);
                    std::cout << "frontier rechts: row " << j << " / col " << i << std::endl;
                    // ROS_INFO("frontier rechts");
                    // ROS_INFO("frontier [%d]: x = %d",k, j);
                    // ROS_INFO("frontier [%d]: y = %d",k, i);
                    // ROS_INFO("--------------");
                    k++;
                }
                else if(searchRegion[i][j+1] == -1){
                    myFrontierCell.row = j;
                    myFrontierCell.col = i;
                    frontierCell.push_back(myFrontierCell);
                    std::cout << "frontier oben: row " << j << " / col " << i << std::endl;
                    // ROS_INFO("frontier oben");
                    // ROS_INFO("frontier [%d]: x = %d",k, j);
                    // ROS_INFO("frontier [%d]: y = %d",k, i);
                    // ROS_INFO("--------------");
                    k++;
                }
                else if(searchRegion[i-1][j] == -1){
                    myFrontierCell.row = j;
                    myFrontierCell.col = i;
                    frontierCell.push_back(myFrontierCell);
                    std::cout << "frontier links: row " << j << " / col " << i << std::endl;
                    // ROS_INFO("frontier links");
                    // ROS_INFO("frontier [%d]: x = %d",k, j);
                    // ROS_INFO("frontier [%d]: y = %d",k, i);
                    // ROS_INFO("--------------");
                    k++;
                }
                else if(searchRegion[i][j-1] == -1){
                    myFrontierCell.row = j;
                    myFrontierCell.col = i;
                    frontierCell.push_back(myFrontierCell);
                    std::cout << "frontier unten: row " << j << " / col " << i << std::endl;
                    // ROS_INFO("frontier unten");
                    // ROS_INFO("frontier [%d]: x = %d",k, j);
                    // ROS_INFO("frontier [%d]: y = %d",k, i);
                    // ROS_INFO("--------------");
                    k++;
                }
            }
        }
    }
    std::cout << "k = " << k << std::endl;
    return frontierCell;
}


