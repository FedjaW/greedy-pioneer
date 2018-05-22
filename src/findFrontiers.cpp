#include <iostream>
#include "findFrontiers.h"
#include <algorithm>


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



std::vector<std::vector<gridCell> > frontierCellNhood(std::vector<gridCell> frontierCells) {

    std::vector<int> visited;
    int oldFrontierSize;
    // std::vector<int> f_visited;
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
            // int loop = 0;
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

                                // std::cout << "d = " << d << std::endl;
                                visited.push_back(d);
                                frontier.push_back(frontierCells[d]);
                            }
                        }
                    }
                }
                // loop++;
                // std::cout << "durchlauf der while schleife Nr. " << loop << std::endl;
            }while(frontier.size() > oldFrontierSize);

            // std::cout << "frontier.size() = "<< frontier.size() << std::endl;
            if(frontier.size() >= 10) 
                frontier_list.push_back(frontier);

            // std::cout << "frontier.row = " << frontier.row << "frontier.col = " << frontier.col << std::endl;
            frontier.clear();
            oldFrontierSize = 0;
        }

    }

    // std::cout << "frontierCells[0].row  = " << frontierCells[10].row  << std::endl;
    // std::cout << "frontierCells[1].row  = " << frontierCells[10].row  << std::endl;
    // std::cout << "abs(frontierCells[1].row - frontierCells[0].row) = " << abs(frontierCells[10].row - frontierCells[10].row)  << std::endl;
   
    std::cout << "frontier_list.size() = "<< frontier_list.size() << std::endl;
    
    // std::cout << "frontier_list[0][0] = "<< frontier_list[0][0].row << std::endl;
    // std::cout << "frontier_list[0][0] = "<< frontier_list[0][0].col << std::endl;
    return frontier_list;
}




















