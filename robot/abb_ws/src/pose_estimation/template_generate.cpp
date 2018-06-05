#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <utility>

using namespace cv;
using namespace std;

Mat src; Mat src_gray;
// find contours
int thresh = 3;
int max_thresh = 255;
RNG rng(12345);
//surface noramls
float row_med=239.5;
float col_med=319.5;
float pixel_size=0.0001;
float focal=1.047198;
int height=480;
int width=640;
cv::FileStorage file("/home/weiyizhang/grasp/robot/abb_ws/depth.xml", cv::FileStorage::READ);

/** @function main */
int main( int argc, char** argv )
{

	/// Load source image and convert it to gray
	Mat src = imread( "/home/weiyizhang/grasp/robot/abb_ws/rgb1.jpg", 3 );
	/// Convert image to gray and blur it
	cvtColor( src, src_gray, CV_BGR2GRAY );
	blur(src_gray, src_gray, Size(3,3) );
	//createTrackbar( " Canny thresh:", "Source", &thresh, max_thresh, thresh_callback);
	vector<vector<Point>> contours=thresh_callback(0, 0);
        //concatenate all the contours into one vector
	vector<Point> contour=contours[0];
	for(int i=1;i<contours.size();i++){
		contour.insert(contour.end(),contours[i].begin(),contours[i].end());
	}

	// contour gradients, horizontal and vertical, norm^2
	vector<vector<vector<int> >> sobers=color_gradient(contours,src);
	vector<int> sober_h=sobers[0][0];
	for(int i=1;i<sobers[0].size();i++){
		sober_h.insert(sober_h.end(),sobers[0][i].begin(),sobers[0][i].end());
	}
	vector<int> sober_v=sobers[0][0];
	for(int i=1;i<sobers[0].size();i++){
		sober_v.insert(sober_v.end(),sobers[0][i].begin(),sobers[0][i].end());
	}
	vector<int> sober_norm=sobers[0][0];
	for(int i=1;i<sobers[0].size();i++){
		sober_norm.insert(sober_norm.end(),sobers[0][i].begin(),sobers[0][i].end());
	}

      
	/*
	vector<vector<Point> > tmp;//contours with non null gradient
	vector<vector<int> > contours_grad;

	for(int i=0;i<contours.size();i++){
	vector<Point> c;
	for(int j=0;j<contours[i].size();j++){
	for(int n=0;n<sobers.size();n++){
	 contours_grad=sobers[n];
	 if(contours_grad[i][j]!=0) {
	     c.push_back(contours[i][j]);
	     break;
	 }
	}
	}
	if(c.size()!=0)tmp.push_back(c);
	}

	Mat drawing = Mat::zeros(src.size(), CV_8UC3);
	Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	drawContours( drawing, tmp, -1, color);
	namedWindow( "Contours3", CV_WINDOW_AUTOSIZE );
	imshow( "Contours3", drawing );
	waitKey(0);
	*/

	// convert to matrix 
	//matrix of quantized directions : 10 orientations in total represented by index of 1~10, 0 represent none
	Mat orient=Mat(src.size(),CV_8UC1,cvScalar(0));
	for(int i=0;i<contour.size();i++){
		int gh=sober_h[i];
		int gv=sober_v[i];
		int orient_idx=0;
			if(gh>=0&&gv>=0){
				orient_idx=1;
				float proj=gh*0.9510565+gv*0.3090170;
				if(gh*0.5877852+gv*0.8090170>proj){
					proj=gh*0.5877852+gv*0.8090170;
					orient_idx=2;
				}
				if(gv>proj){
					orient_idx=3;
				}
			}
			else if(gh<=0&&gv>=0){
				orient_idx=3;
				float proj=gv;
				if(-gh*0.5877852+gv*0.8090170>proj){
					proj=-gh*0.5877852+gv*0.8090170;
					orient_idx=4;
				}
				if(-gh*0.9510565+gv*0.3090170>proj){
					orient_idx=5;
				}
			}
			else if(gh<=0&&gv<=0){
				orient_idx=6;
				float proj=-gh*0.9510565-gv*0.3090170;
				if(-gh*0.5877852-gv*0.8090170>proj){
					proj=-gh*0.5877852-gv*0.8090170;
					orient_idx=7;
				}
				if(-gv>proj){
					orient_idx=8;
				}
			}
			else if(gh>=0&&gv<=0){
				orient_idx=8;
				float proj=-gv;
				if(gh*0.5877852-gv*0.8090170>proj){
					proj=gh*0.5877852-gv*0.8090170;
					orient_idx=9;
				}
				if(gh*0.9510565-gv*0.3090170>proj){
					orient_idx=10;
				}
			}
		orient_mat.at<uchar>(contour[i])=orient_idx;
	}
        // smooth voting in 3*3 matrix of orient
	Mat orient_smooth=Mat(src.size(),CV_8UC1,cvScalar(0));
	for(int i=0;i<contour.size();i++){
		int orient_voting=0;
		int votes[]={0,0,0,0,0,0,0,0,0,0}; //number of orients from 1~10
		Point x=contour[i];
		int orient_x=(int)orient_mat.at<uchar>(x);
		if((int)orient_mat.at<uchar>(x+Point(-1,-1))>0)votes[(int)orient_mat.at<uchar>(x+Point(-1,-1))-1]++;
		if((int)orient_mat.at<uchar>(x+Point(-1,0))>0)votes[(int)orient_mat.at<uchar>(x+Point(-1,0))-1]++;
		if((int)orient_mat.at<uchar>(x+Point(-1,1))>0)votes[(int)orient_mat.at<uchar>(x+Point(-1,1))-1]++;
		if((int)orient_mat.at<uchar>(x+Point(0,-1))>0)votes[(int)orient_mat.at<uchar>(x+Point(0,-1))-1]++;
		if(orient_x>0)votes[orient_x-1]++;
		if((int)orient_mat.at<uchar>(x+Point(0,1))>0)votes[(int)orient_mat.at<uchar>(x+Point(0,1))-1]++;
		if((int)orient_mat.at<uchar>(x+Point(1,-1))>0)votes[(int)orient_mat.at<uchar>(x+Point(1,-1))-1]++;
		if((int)orient_mat.at<uchar>(x+Point(1,0))>0)votes[(int)orient_mat.at<uchar>(x+Point(1,0))-1]++;
		if((int)orient_mat.at<uchar>(x+Point(1,1))>0)votes[(int)orient_mat.at<uchar>(x+Point(1,1))-1]++;
		for(int j=0;j<10;j++) if(votes[j]>4) {orient_voting=j+1; break;}
		if(orient_voting==0){
			int num_voter=0;
			for(int j=0;j<10;j++)num_voter+=votes[j];
			float orient_1_5=(1.0*votes[0]+2.0*votes[1]+3.0*votes[2]+4.0*votes[3]+5.0*votes[4])/num_voter;
			float orient_6_10=(1.0*votes[5]+2.0*votes[6]+3.0*votes[7]+4.0*votes[8]+5.0*votes[9])/num_voter;
			if(abs(orient_1_5-orient_6_10)<5){
				orient_voting=round((orient_1_5*(votes[0]+votes[1]+votes[2]+votes[3]+votes[4])+orient_6_10*(votes[5]+votes[6]+votes[7]+votes[8]+votes[9]))/num_voter);
			}else{
				orient_voting=(round((orient_1_5*(votes[0]+votes[1]+votes[2]+votes[3]+votes[4])+orient_6_10*(votes[5]+votes[6]+votes[7]+votes[8]+votes[9]))/num_voter)+5)%10;
			}
		}
		orient_smooth.at<uchar>(x)=orient_voting;
	}

	// quick sort
	pair<vector<Point>,vector<int>> sorted=quickSort(contour,sober_norm);
	vector<Point> contour_sorted=sorted.first;
	vector<int> sober_sorted=sorted.second;
	// reduce features
        Mat grad_reduced=Mat(src.size(),CV_8UC3,cvScalar(0));
	for(int i=0;i<contour.size();i++){
		grad_reduced.at<uchar>(contour[i])=1;//preserved gradient 
	}
	for(int i=contour_sorted.size()-1;i>=0;i--){
                //==0 when grad of this position is deleted because its great neighbor
		if(((int)grad_reduced.at<uchar>(contour_sorted[i]))!=0){
			grad_reduced.at<uchar>(contour_sorted[i]+Point(1,0))=0;
			grad_reduced.at<uchar>(contour_sorted[i]+Point(1,1))=0;
			grad_reduced.at<uchar>(contour_sorted[i]+Point(1,-1))=0;
			grad_reduced.at<uchar>(contour_sorted[i]+Point(-1,0))=0;
			grad_reduced.at<uchar>(contour_sorted[i]+Point(-1,1))=0;
			grad_reduced.at<uchar>(contour_sorted[i]+Point(-1,-1))=0;
			grad_reduced.at<uchar>(contour_sorted[i]+Point(0,1))=0;
			grad_reduced.at<uchar>(contour_sorted[i]+Point(0,-1))=0;
		}
	}
	vector<int> orient_reduced;
	vector<Point> contour_reduced;// no repeated points
	for(int r=0;r<src.rows;r++){
		for(int c=0;c<scr.cols;c++){
			if((int)grad_reduced.at<uchar>(Point(r,c))>0){
				orient_reduced.push_back((int)orient_smooth.at<uchar>(Point(r,c)));
				contour_reduced.push_back(Point(r,c));
			}
		}
	}

	//surface normal features
	Mat depth;
        file["depthMat1"]>>depth;
	tuple<vector<Point>,Mat,vector<float>> result_tuple=surface_normal(depth, contour);
        //reduced features
        pair<vector<Point>,vector<float>> normal_sorted=quickSort(get<0>(result_tuple),get<2>(result_tuple));
        vector<Point> contour_normal_sorted=normal_sorted.first;
        Mat normalf_reduced=Mat(src.size(),CV_8UC1,cvScalar(0));
	for(int i=0;i<contour_normal_sorted.size();i++){
		normalf_reduced.at<uchar>(contour[i])=1;//preserved gradient 
	}
	for(int i=contour_normal_sorted.size()-1;i>=0;i--){
                //==0 when grad of this position is deleted because its great neighbor
		if(((int)normalf_reduced.at<uchar>(contour_normal_sorted[i]))!=0){
			normalf_reduced.at<uchar>(contour_normal_sorted[i]+Point(1,0))=0;
			normalf_reduced.at<uchar>(contour_normal_sorted[i]+Point(1,1))=0;
			normalf_reduced.at<uchar>(contour_normal_sorted[i]+Point(1,-1))=0;
			normalf_reduced.at<uchar>(contour_normal_sorted[i]+Point(-1,0))=0;
			normalf_reduced.at<uchar>(contour_normal_sorted[i]+Point(-1,1))=0;
			normalf_reduced.at<uchar>(contour_normal_sorted[i]+Point(-1,-1))=0;
			normalf_reduced.at<uchar>(contour_normal_sorted[i]+Point(0,1))=0;
			normalf_reduced.at<uchar>(contour_normal_sorted[i]+Point(0,-1))=0;
		}
	}
        Mat normals=get<1>(result_tuple);
	vector<Vec<float,3>> normal_reduced;
	vector<Point> contour_normal_reduced;// no repeated points
	for(int r=0;r<src.rows;r++){
		for(int c=0;c<scr.cols;c++){
			if((int)normalf_reduced.at<uchar>(Point(r,c))>0){
				normal_reduced.push_back(normals.at<Vec<float,3>>(Point(r,c)));
				contour_normal_reduced.push_back(Point(r,c));
			}
		}
	}
file.release();
	return(0);
}
















