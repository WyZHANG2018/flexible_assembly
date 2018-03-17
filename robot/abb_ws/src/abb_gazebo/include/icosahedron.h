#include <iostream>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#define PHI (std::sqrt(5.0)+1.0)/2.0;



/////////////////////////////////////
/////////////////////////////////////
std::vector<geometry_msgs::Point> vertices_edges(int num_seg, float x,float y, float z, float xx,float yy, float zz);
/////////////////////////////////////
/////////////////////////////////////
std::vector<geometry_msgs::Point> vertices_faces(int num_seg,float xv, float yv,float zv,float x1,float y1,float z1,float x2,float y2,float z2);
/////////////////////////////////////
/////////////////////////////////////
std::vector<geometry_msgs::Point> all_vertices(int num_iter);
