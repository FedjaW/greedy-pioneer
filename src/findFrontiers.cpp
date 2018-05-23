#include <iostream>
#include "findFrontiers.h"
#include <algorithm>

#if 0
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





#if 0


std::vector<unsigned int> nhood8(std::vector<gridCell> frontierCells) {
    frontierCells[0].row = 
    frontierCells[0].col = 
}

void breadthFirstSearch(std::vector<gridCell> frontierCells) {





    // push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    unsigned int = 0; // erstes Element im frontierCells Vector
    bfs.push(initial_cell);

  while (!bfs.empty()) {  // Solange bfs NICHT leer ist mache...
          unsigned int idx = bfs.front(); // übergebe das erste Element von bfs an idx
          bfs.pop(); // entferne das erste Element aus der queue

}


// std::vector<gridCell> HURENSOHN(std::vector<std::vector<int> > griMap) {
void HURENSOHN(std::vector<std::vector<int> > griMap) {

    // initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> frontier_flag(gridMap.size(), false);
    std::vector<std::vector<bool> > visited_flag(griMap.size(), false);


    // initialize breadth first search
    std::queue<gridCell> bfs;
    // starte die Suche von 
    gridCell initCell;
    initCell.row = 0;
    initCell.col = 0;
    bfs.push(initCell);

    visited_flag[bfs.front()] = true;
    



}











#endif




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



// std::vector<std::vector<gridCell> > frontierCellNhood(std::vector<gridCell> frontierCells) {
//     // std::cout << "HalliHallo 1" << std::endl;
//
//     int oldFrontierSize;
//     std::vector<int> visited;
//     std::vector<gridCell> frontier;
//     std::vector<std::vector<gridCell> > frontier_list;
//
//     // for(int i = 0; i < frontierCells.size(); i++) {
//     for(auto& frontierCell : frontierCells){
//         bool root = false;
//             // if( ! (std::find(visited.begin(), visited.end(), i) != visited.end()) ) {
//             
//             // for(int n = i+1; n < frontierCells.size(); n++) {
//             for(auto& frontierCellNext : frontierCells) {  // ab dem zweiten Element
//                 // if( ! (std::find(visited.begin(), visited.end(), n) != visited.end()) ) {
//
//                     if( (abs(frontierCell.row - frontierCellNext.row) <= 1) && (abs(frontierCell.col - frontierCellsNext.col) <= 1) ) {
//                         frontierCells.erase(frontierCellNext); // ab dem zweiten element
//                         if(root == false) {
//                             frontier.push_back(frontierCell);
//                             frontierCells.erase(frontierCell);
//                             root = true;
//                         }
//                         frontier.push_back(frontierCellNext);
//                     }
//                 // }
//
//             }
//
//             int c = 1;
//             do{
//                 oldFrontierSize = frontier.size();
//                 // std::cout << "oldFrontierSize = " << oldFrontierSize << std::endl;
//                 for(c; c < oldFrontierSize; c++) {
//                     // std::cout << "frontier[" <<c<< "]" << ".row = " << frontier[c].row << " / frontier[" <<c<< "]" << ".col = " << frontier[c].col << std::endl;
//                     for(int d = 0; d < frontierCells.size(); d++){
//                         if( ! (std::find(visited.begin(), visited.end(), d) != visited.end()) ) {
//                             // std::cout << "vergleiche mit frontier[" << d << "]" << ".row = " << frontierCells[d].row << " / frontier[" <<d<< "]" << ".col = " << frontierCells[d].col << std::endl;
//                             if( (abs(frontier[c].row - frontierCells[d].row) <= 1) && (abs(frontier[c].col - frontierCells[d].col) <= 1) ) {
//                                 visited.push_back(d);
//                                 frontier.push_back(frontierCells[d]);
//                             }
//                         }
//                     }
//                 }
//                 // std::cout << "durchlauf der while schleife Nr. " << loop << std::endl;
//             }while(frontier.size() > oldFrontierSize);
//
//             if(frontier.size() >= 1) 
//                 frontier_list.push_back(frontier);
//
//             frontier.clear();
//             oldFrontierSize = 0;
//         }
//
//     }
//     std::cout << "frontier_list.size() = "<< frontier_list.size() << std::endl;
//     return frontier_list;
// }
//

        





std::vector<std::vector<gridCell> > frontierCellNhood(std::vector<gridCell> frontierCells) {

    int oldFrontierSize;
    // std::vector<int> visited;
    std::vector<gridCell> frontier;
    std::vector<std::vector<gridCell> > frontier_list;

    for(int i = 0; i < frontierCells.size(); i++) {
        bool root = false;

        for(int n = i+1; n < frontierCells.size(); n++) {

            if( (abs(frontierCells[i].row - frontierCells[n].row) <= 1) && (abs(frontierCells[i].col - frontierCells[n].col) <= 1) ) {
                frontier.push_back(frontierCells[n]);
                frontierCells.erase(frontierCells.begin()+n);
                root = true;
                n--;
            }

        }

        if(root == true) {
            // frontier.push_back(frontierCells[i]); // TODO: an erste stelle pushen und nicht ans ende
            frontier.insert(frontier.begin(), frontierCells[i]); // TODO: an erste stelle pushen und nicht ans ende
            frontierCells.erase(frontierCells.begin()+i);
            i--;
        }


        int c = 1;
        do{
            oldFrontierSize = frontier.size();
            // std::cout << "frontier.size() = "<< frontier.size() << std::endl;
            // std::cout << "oldFrontierSize = " << oldFrontierSize << std::endl;
            for(c; c < oldFrontierSize; c++) {
                // std::cout << "frontier[" <<c<< "]" << ".row = " << frontier[c].row << " / frontier[" <<c<< "]" << ".col = " << frontier[c].col << std::endl;
                for(int d = 0; d < frontierCells.size(); d++){
                    // if( ! (std::find(visited.begin(), visited.end(), d) != visited.end()) ) {
                        // std::cout << "vergleiche mit frontier[" << d << "]" << ".row = " << frontierCells[d].row << " / frontier[" <<d<< "]" << ".col = " << frontierCells[d].col << std::endl;
                        if( (abs(frontier[c].row - frontierCells[d].row) <= 1) && (abs(frontier[c].col - frontierCells[d].col) <= 1) ) {
                            // visited.push_back(d);
                            frontier.push_back(frontierCells[d]);
                            frontierCells.erase(frontierCells.begin()+d);
                            d--;
                        }
                    // }
                }
            }
            // std::cout << "durchlauf der while schleife Nr. " << loop << std::endl;
        }while(frontier.size() > oldFrontierSize);

        // std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;

        if(frontier.size() >= 10) 
            frontier_list.push_back(frontier);

        frontier.clear();
        oldFrontierSize = 0;
        // }

    }
    std::cout << "frontier_list.size() = "<< frontier_list.size() << std::endl;
    return frontier_list;
}


