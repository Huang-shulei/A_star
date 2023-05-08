#ifndef A_STAR_PLANNER_H
#define A_STAR_PLANNER_H
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <algorithm>
#include "chrono"
#include <list>
#include <map>
#include <utility>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

enum NODE_STATUS //枚举，表示点的状态（NOT_VISITED = 0，IN_OPENSET = 1,  IN_CLOSESET = 2）
   {
       NOT_VISITED = 0, IN_OPENSET, IN_CLOSESET,IN_OBSTACLE
   };

struct a_star
{
     geometry_msgs::Point point;
     int F,G,H,O;
     a_star* parent;
     NODE_STATUS status;
     int gradient;
};



class A_star
{
  public:
     multimap<int, a_star*> openlist;
     int sites[90][162];
     a_star ***grid_node;
    

     void getNeighbourPoints(pair<int,int> point, pair<int,int> endpoint);//获取邻居节点

     list<a_star*> getpath(pair<int,int> startPoint, pair<int,int> endPoint);

     int getleastdistance(int& x,int& y);

     void initializeMap();

     void resetMap();
   
     void readMap(Mat& mat);

  private:
   
};
#endif