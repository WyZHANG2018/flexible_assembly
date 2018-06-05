#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <utility>
#include <fstream>
using namespace cv;
using namespace std;


//Mat src; Mat src_gray;
//pair<vector<Point>,vector<int>> quickSort(vector<Point> contour,vector<int> sober);


/** @function main */
int main( int argc, char** argv )
{
/*
Mat src = imread( "/home/weiyizhang/grasp/robot/abb_ws/rgb1.jpg", 3 );
Mat mat=Mat(src.size(),CV_8UC2,cvScalar(0,0));
mat.at<Vec<uchar,2>>(Point(2,3))=Vec<uchar,2>(6,8);
cout<<(int)mat.at<Vec<uchar,2>>(Point(2,3))[0]<<endl<<(int)mat.at<Vec<uchar,2>>(Point(2,3))[1]<<endl;
*/

ofstream fs;
fs.open("/home/weiyizhang/grasp/robot/abb_ws/src/pose_estimation/normal.csv");
fs << "Column A" << "," << "Column B"  << "," << "Column C" << std::endl;
fs << 2.33 << "," << 6.66 << "," << 1.22 << std::endl;
fs.close();
  return(0); 
}











