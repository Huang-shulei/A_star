#include <a_star/a_star.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <algorithm>
#include "chrono"
#include <nav_msgs/GridCells.h>
#include <list>
#include <map>
#include <utility>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace std::chrono;
using namespace cv;

int dir[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,-1},{1,1},{-1,1},{-1,-1}};


int A_star::getleastdistance(int& x,int& y)
{
   int x_distance;
   int y_distance;
   for(int k=0; k<((sizeof(this->sites)/sizeof(this->sites[0]))-1); k++)
   {
      if(this->sites[x+k][y] == -1 || this->sites[x-k][y] == -1)
      {
         x_distance = k;
         break;
      }
   }
   for(int p=0; p<((sizeof(this->sites[1])/sizeof(this->sites[0][0]))-1); p++)
   {
      if(this->sites[x][y+p] == -1 || this->sites[x][y-p] == -1)
      { 
         y_distance = p;
         break;
      }
   }
   int distance = min(x_distance, y_distance)*10;
   if(distance == 10)
   {
      distance = 1;
   }
   if(distance > 50)
   {
      distance = 50;
   }
   return distance;
}


void A_star::getNeighbourPoints(pair<int,int> point, pair<int,int> endpoint)//获取邻居节点
     {
        auto curpoint = this->grid_node[point.first][point.second];
        for(int i = 0; i < 8; i++)
        {
           int tempoint_x = point.first + dir[i][0];
           int tempoint_y = point.second + dir[i][1];
           auto tempoint = this->grid_node[tempoint_x][tempoint_y];
           int tempointCost = curpoint->G + ((i < 4) ? 10 : 14);
             if(tempoint->status == NODE_STATUS::NOT_VISITED)//当其不在openlist中时，将其加入
             {
                tempoint->parent = curpoint;
                tempoint->G = tempointCost;
                tempoint->H = (abs(tempoint_x - endpoint.first)+abs(tempoint_y - endpoint.second))*10;
                tempoint->F = tempoint->G + tempoint->H - tempoint->O;
                tempoint->status = NODE_STATUS::IN_OPENSET;
                this->openlist.insert(make_pair(tempoint->F,tempoint));
             }
             else if(tempoint->status == NODE_STATUS::IN_OPENSET && tempointCost < tempoint->G)//当其在openlist中且得到的G值小于原G值则更新G值
             {
                tempoint->parent = curpoint;
                tempoint->G = tempointCost;
                tempoint->F = tempoint->G + tempoint->H - tempoint->O;
             }
        }
     }


list<a_star*> A_star::getpath(pair<int,int> startPoint, pair<int,int> endPoint)
{
   this->resetMap();
   auto starttime1 = steady_clock::now();
   auto startpoint = this->grid_node[startPoint.first][startPoint.second];
   startpoint->G = 0;
   startpoint->H = (abs(startPoint.first - endPoint.first)+abs(startPoint.second - endPoint.second))*10;
   startpoint->F = startpoint->G + startpoint->H - startpoint->O;
   startpoint->parent = nullptr;
   this->openlist.insert(make_pair(startpoint->F,startpoint));
   while(!this->openlist.empty())
   {
      auto curpoint = this->openlist.begin()->second;
      openlist.erase(openlist.begin());
      curpoint->status = NODE_STATUS::IN_CLOSESET;
      if(curpoint->point.x == endPoint.first && curpoint->point.y == endPoint.second)//判断是否为终点
      {
         list<a_star*> path;
         cout<<"检测到终点"<<endl;
         while(curpoint != NULL)
         {
            path.push_back(curpoint);
            curpoint = curpoint->parent;
         }
         auto endtime1 = steady_clock::now();
         auto tt1 = duration_cast<microseconds>(endtime1 - starttime1);
         openlist.clear();
         cout<<"用时:"<<((double)tt1.count())/1000<<" 毫秒 over!"<<endl;
         return path;
      }
      int curpoint_x = (int)curpoint->point.x;
      int curpoint_y = (int)curpoint->point.y;
      this->getNeighbourPoints(make_pair(curpoint_x, curpoint_y), endPoint);
   } 
}


//初始化地图
void A_star::initializeMap()
{
   this->grid_node = new a_star **[112];
   for(int i = 0; i < ((sizeof(this->sites)/sizeof(this->sites[0]))-1);i++)
    {
        this->grid_node[i] = new a_star *[202];
        for(int j = 0; j < ((sizeof(this->sites[1])/sizeof(this->sites[0][0]))-1); j++)
        {
            a_star *tempt = new a_star;
            this->grid_node[i][j] = tempt;
        }
    } 
}


//重置地图
void A_star::resetMap()
{
   for(int i = 0; i < ((sizeof(this->sites)/sizeof(this->sites[0]))-1);i++)
    {
        for(int j = 0; j < ((sizeof(this->sites[1])/sizeof(this->sites[0][0]))-1); j++)
        {
           this->grid_node[i][j]->F = 0;
           this->grid_node[i][j]->G = 0;//初始G为0
           this->grid_node[i][j]->H = 0;//初始H为0
           this->grid_node[i][j]->point.x = i;
           this->grid_node[i][j]->point.y = j;
           this->grid_node[i][j]->parent = nullptr;
           if(this->sites[i][j] != -1)
            {
                this->grid_node[i][j]->status = NODE_STATUS::NOT_VISITED;
                this->grid_node[i][j]->O = this->sites[i][j];//初始O为符号距离场
            }
            else
            {
                this->grid_node[i][j]->status = NODE_STATUS::IN_OBSTACLE;
                this->grid_node[i][j]->O = -1;//初始O为-1
            }
        }
    }
}

void A_star::readMap(Mat &mat)
{
   for (int row = 0; row < mat.rows; row++)
   {
        for (int col = 0; col < mat.cols; col++)
      {
           if((int)(*(mat.data+mat.step[0]*row + mat.step[1]*col + 2)) == 0)
		      {
                  this->sites[row][col] = -1;
		      }
       }
   }
   for(int i = 0; i < ((sizeof(this->sites)/sizeof(this->sites[0]))-1);i++)
    {
        for(int j = 0; j < ((sizeof(this->sites[1])/sizeof(this->sites[0][0]))-1); j++)
        {
            if(this->sites[i][j] != -1)
            {
                this->sites[i][j] = 100000;
                int d = this->getleastdistance(i,j);
                if(d < this->sites[i][j])
                {
                   this->sites[i][j]=d;
                }
            }
        }
    }
}