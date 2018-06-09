#ifndef EXPLORE_H_
#define EXPLORE_H_


struct color {
    int r;
    int g;
    int b;
};

bool startExploration(ros::NodeHandle& nh);
bool sortFunction(const Frontier& f1, const Frontier& f2);
void toggleColor(double &r, double &g, double &b);


#endif
