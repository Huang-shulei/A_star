#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <algorithm>
#include "chrono"
#include <nav_msgs/GridCells.h>
#include <list>
#include <a_star/a_star.h>
#include <map>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace cv;
using namespace std::chrono;

pair<int,int> startpt(11,11);
pair<int,int> endpt;

void end_point_Callback(const geometry_msgs::PoseStamped &end_point)
{
    endpt.first = floor(end_point.pose.position.x);
    endpt.second = floor(end_point.pose.position.y);
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "a_star_plus");
    ros::NodeHandle n;
    ros::Publisher pub1 = n.advertise<nav_msgs::GridCells>("/GridCells1", 1);
    ros::Publisher pub = n.advertise<nav_msgs::GridCells>("/GridCells", 1);
    ros::Subscriber sub_end_point = n.subscribe("/move_base_simple/goal", 1, &end_point_Callback);
    nav_msgs::GridCells GC_msg1;
    geometry_msgs::Point obstacle;
    GC_msg1.header.frame_id = "GC";
    GC_msg1.cell_width = 1;
    GC_msg1.cell_height = 1;

    //读取图片并转化为数组
    Mat mat = imread("/home/hsl/catkin_ws/src/jps_test/map/map_lei.png");
    cout<<"高："<<mat.rows<<"宽："<<mat.cols<<endl;
    A_star A;
    A.readMap(mat);
     for (int row = 0; row < mat.rows; row++)
     {
        for (int col = 0; col < mat.cols; col++)
      {
           if((int)(*(mat.data+mat.step[0]*row + mat.step[1]*col + 2)) == 0)
		      {
                  obstacle.x = row;
                  obstacle.y = col;
                  GC_msg1.cells.push_back(obstacle);
		      }
       }
     }

    A.initializeMap();

while(ros::ok())
{   
    ros::spinOnce();
    list<a_star*> path;
    auto starttime = steady_clock::now();
    
    if(endpt.first>= 8 && endpt.second >= 8)
    {
        cout<<"don't present"<<endl;
        path = A.getpath(startpt, endpt);
    }
    
    auto endtime = steady_clock::now();
    auto tt = duration_cast<microseconds>(endtime - starttime);
    cout<<"用时:"<<((double)tt.count())/1000<<" 毫秒 over!"<<endl;
    cout<<"大小："<<path.size()<<endl;
    

    
    nav_msgs::GridCells GC_msg;
    GC_msg.header.frame_id = "GC";
    GC_msg.cell_width = 1;
    GC_msg.cell_height = 1;
    for(auto p : path)
    {
       GC_msg.cells.push_back(p->point);
    }
    
    pub.publish(GC_msg);
    pub1.publish(GC_msg1);
	
}

    return 0;
}