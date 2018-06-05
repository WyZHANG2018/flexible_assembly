#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <utility>
#include <fstream>
using namespace cv;
using namespace std;

//surface noramls
float row_med=239.5;
float col_med=319.5;
float pixel_size=0.0001;
float focal=1.047198;
int height=480;
int width=640;
tuple<vector<Point>,Mat,vector<float>,vector<Vec<float,3>>> surface_normal(Mat depth, vector<Point> contour);
vector<vector<Point> > thresh_callback(int, void*);
template <typename T> pair<vector<Point>,vector<T>> quickSort(vector<Point> contour,vector<T> sober);
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

	//surface normal features
	Mat depth;
        file["depthMat1"]>>depth;
	tuple<vector<Point>,Mat,vector<float>,vector<Vec<float,3>>> result_tuple=surface_normal(depth, contour);
        vector<Vec<float,3>> 3dpoints=get<3>(result_tuple);
        vector<Vec<float,3>> 3dnormals;
        vector<Point> ps=get<0>(result_tuple);
        Mat normalmat=get<1>(result_tuple);
        for(int i=0;i<ps.size();i++){
		3dnormals.push_back(normalmat.at<Vec<float,3>>(ps[i]));
	}

	ofstream fs;
	fs.open("/home/weiyizhang/grasp/robot/abb_ws/src/pose_estimation/normal.csv");
	for(int i=0;i<3dnormals.size();i++){
	fs << "a" << "," << "b" <<"," << "c" << ","  <<"d" << "," <<"e" << "," <<"f"<< std::endl;
	fs << 3dnormals[i][0] << "," << 3dnormals[i][1] << "," << 3dnormals[i][2] << 3dpoints[i][0] << "," << 3dpoints[i][1] << "," << 3dpoints[i][2] <<std::endl;
	}
	fs.close();
        
/*
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
*/
	return(0);
}

vector<vector<Point> > thresh_callback(int, void*)
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny( src_gray, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
 
   //add neignbor points to original contours
   vector<vector<Point> > thick_contours;

   for(int i=0;i<contours.size();i++){
        vector<Point> tmp;
	for(int j=0;j<contours[i].size();j++){
                //add point around contours
                int x=contours[i][j].x;
                int y=contours[i][j].y;
                tmp.push_back(Point(x,y));

                tmp.push_back(Point(x+1,y+1));
		tmp.push_back(Point(x+1,y-1));
		tmp.push_back(Point(x+1,y));
		tmp.push_back(Point(x-1,y+1));
		tmp.push_back(Point(x-1,y-1));
		tmp.push_back(Point(x-1,y));
		tmp.push_back(Point(x,y-1));
		tmp.push_back(Point(x,y+1));

                tmp.push_back(Point(x+2,y+1));
                tmp.push_back(Point(x+2,y+2));
                tmp.push_back(Point(x+2,y-1));
                tmp.push_back(Point(x+2,y-2));
                tmp.push_back(Point(x+2,y));
                tmp.push_back(Point(x-2,y+1));
                tmp.push_back(Point(x-2,y+2));
                tmp.push_back(Point(x-2,y-1));
                tmp.push_back(Point(x-2,y-2));
                tmp.push_back(Point(x-2,y));
                tmp.push_back(Point(x+1,y+2));
                tmp.push_back(Point(x+1,y-2));
                tmp.push_back(Point(x-1,y+2));
                tmp.push_back(Point(x-1,y-2));
                tmp.push_back(Point(x,y+2));
                tmp.push_back(Point(x,y-2));
       }
       thick_contours.push_back(tmp);
  }

template <typename T> pair<vector<Point>,vector<T>> quickSort(vector<Point> contour,vector<T> sober){
	int length=contour.size();
	int pos_median=(int)(length/2);//index
	T value_median=sober[pos_median];
	int l=0;//index
	int r=length-1;//index
	while(true){
		//left
		while(sober[l]<=value_median&&l<pos_median) l++;
		//right
		while(sober[r]>=value_median&&r>pos_median) r--;

		if(l<pos_median&&r>pos_median){
			int tmp=sober[l];
			sober[l]=sober[r];
			sober[r]=tmp;

			Point tmp2=contour[l];
			contour[l]=contour[r];
			contour[r]=tmp2;
			l++;
			r--;
		}else if(l<pos_median&&r==pos_median){
			sober.insert(sober.begin()+pos_median+1,sober[l]);
			sober.erase(sober.begin()+l);
			contour.insert(contour.begin()+pos_median+1,contour[l]);
			contour.erase(contour.begin()+l);
			pos_median--;
			r=pos_median;
		}else if(l==pos_median&&r>pos_median){
			sober.insert(sober.begin()+pos_median,sober[r]);
			sober.erase(sober.begin()+r+1);
			contour.insert(contour.begin()+pos_median,contour[r]);
			contour.erase(contour.begin()+r+1);
			pos_median++;
			l=pos_median;
		}else break;
	}
	//left
	vector<Point> cl;
	cl.reserve(pos_median+1);
	cl.insert(cl.begin(),contour.begin(),contour.begin()+pos_median+1);
	vector<T> sl;
	sl.reserve(pos_median+1);
	sl.insert(sl.begin(),sober.begin(),sober.begin()+pos_median+1);

	if(pos_median>1){
		pair<vector<Point>,vector<T>> result=quickSort(cl,sl);
		cl=result.first;
		sl=result.second;
	}
	//right
	vector<Point> cr;
	cr.reserve(length-pos_median);
	cr.insert(cr.begin(),contour.begin()+pos_median,contour.end());
	vector<T> sr;
	sr.reserve(length-pos_median);
	sr.insert(sr.begin(),sober.begin()+pos_median,sober.end());

	if(pos_median<length-2){
		pair<vector<Point>,vector<T>> result=quickSort(cr,sr);
		cr=result.first;
		sr=result.second;
	}

	//concatenate
	vector<Point> C;
	C.insert(C.begin(),cl.begin(),cl.end());
	if(cr.size()>1) C.insert(C.end(),cr.begin()+1,cr.end());

	vector<T> S;
	S.insert(S.begin(),sl.begin(),sl.end());
	if(sr.size()>1) S.insert(S.end(),sr.begin()+1,sr.end());

	return pair<vector<Point>,vector<T>>(C,S);
}

tuple<vector<Point>,Mat,vector<float>,vector<Vec<float,3>>> surface_normal(Mat depth, vector<Point> contour){

vector<Point> contour_normal;
vector<Vec<float,3>> 3dpoints;
Mat normals=Mat(depth.size(),CV_32FC3,cvScalar(0.0,0.0,0.0))
Point neignb1[]={Point(-1,-1),Point(-1,0),Point(-1,1),Point(0,-1),Point(0,1),Point(1,-1),Point(1,0),Point(1,1)}
Point neignb2[]={Point(-2,-2),Point(-2,-1),Point(-2,0),Point(-2,1),Point(-2,2),Point(-1,-2),Point(-1,2),Point(0,-2),Point(0,2),Point(1,-2),Point(1,2),Point(2,-2),Point(2,-1),Point(2,0),Point(2,1),Point(2,2)}
//1 2 3
//4 x 5
//6 7 8
//we consider only the pixels in the neignborhood of 8 centered on the pixel considered
for(int i=0;i<contour.size();i++){
	Point x=contour[i];
	float xdepth=(float)depth.at<float>(x);
	if (xdepth!=0.0){

		float coefx[11]={0,0,0,0,0,0,0,0,0,0,0};// dx,dy,x,1,2,3...8
		float coefy[11]={0,0,0,0,0,0,0,0,0,0,0};// dx,dy,x,1,2,3...8
		float neignbDepth[8]={0,0,0,0,0,0,0,0};
		int neignb_num=0;
                
		//1
		if((float)depth.at<float>(x+neignb1[0])!=0){
		neignb_num++;
		neignbDepth[0]=(float)depth.at<float>(x+neignb1[0]);
		coefx[0]+=2;
		coefx[1]+=2;
		coefx[2]+=-2;
		coefx[3]+=2;

		coefy[0]+=2;
		coefy[1]+=2;
		coefy[2]+=-2;
		coefy[3]+=2;
		}

		//2
		if((float)depth.at<float>(x+neignb1[1])!=0){
		neignb_num++;
		neignbDepth[1]=(float)depth.at<float>(x+neignb1[1]);
		coefx[0]+=2;
		coefx[2]+=-2;
		coefx[4]+=2;
		}

		//3
		if((float)depth.at<float>(x+neignb1[2])!=0){
		neignb_num++;
		neignbDepth[2]=(float)depth.at<float>(x+neignb1[2]);
		coefx[0]+=2;
		coefx[1]+=-2;
		coefx[2]+=-2;
		coefx[5]+=2;

		coefy[0]+=-2;
		coefy[1]+=2;
		coefy[2]+=2;
		coefy[5]+=-2;
		}

		//4
		if((float)depth.at<float>(x+neignb1[3])!=0){
		neignb_num++;
		neignbDepth[3]=(float)depth.at<float>(x+neignb1[3]);
		coefy[1]+=2;
		coefy[2]+=-2;
		coefy[6]+=2;
		}

		//5
		if((float)depth.at<float>(x+neignb1[4])!=0){
		neignb_num++;
		neignbDepth[4]=(float)depth.at<float>(x+neignb1[4]);
		coefy[1]+=2;
		coefy[2]+=2;
		coefy[7]+=-2;
		}

		//6
		if((float)depth.at<float>(x+neignb1[5])!=0){
		neignb_num++;
		neignbDepth[5]=(float)depth.at<float>(x+neignb1[5]);
		coefx[0]+=2;
		coefx[1]+=-2;
		coefx[2]+=2;
		coefx[8]+=-2;

		coefy[0]+=-2;
		coefy[1]+=2;
		coefy[2]+=-2;
		coefy[8]+=2;
		}

		//7
		if((float)depth.at<float>(x+neignb1[6])!=0){
		neignb_num++;
		neignbDepth[6]=(float)depth.at<float>(x+neignb1[6]);
		coefx[0]+=2;
		coefx[2]+=2;
		coefx[9]+=-2;
		}

		//8
		if((float)depth.at<float>(x+neignb1[7])!=0){
		neignb_num++;
		neignbDepth[7]=(float)depth.at<float>(x+neignb1[7]);
		coefx[0]+=2;
		coefx[1]+=2;
		coefx[2]+=2;
		coefx[10]+=-2;

		coefy[0]+=2;
		coefy[1]+=2;
		coefy[2]+=2;
		coefy[10]+=-2;
		}

		if(neignb_num>=2){
			float coefx_noy[10];// dx,x,1,2,3...8
			coefx_noy[0]=coefx[0]-coefy[0]/coefy[1]*coefx[1];
			coefx_noy[1]=coefx[2]-coefy[2]/coefy[1]*coefx[1];
			coefx_noy[2]=coefx[3]-coefy[3]/coefy[1]*coefx[1];
			coefx_noy[3]=coefx[4]-coefy[4]/coefy[1]*coefx[1];
			coefx_noy[4]=coefx[5]-coefy[5]/coefy[1]*coefx[1];
			coefx_noy[5]=coefx[6]-coefy[6]/coefy[1]*coefx[1];
			coefx_noy[6]=coefx[7]-coefy[7]/coefy[1]*coefx[1];
			coefx_noy[7]=coefx[8]-coefy[8]/coefy[1]*coefx[1];
			coefx_noy[8]=coefx[9]-coefy[9]/coefy[1]*coefx[1];
			coefx_noy[9]=coefx[10]-coefy[10]/coefy[1]*coefx[1];

			float coefy_nox[10];// dy,x,1,2,3...8
			coefy_nox[0]=coefy[1]-coefx[1]/coefx[0]*coefy[0];
			coefy_nox[1]=coefy[2]-coefx[2]/coefx[0]*coefy[0];
			coefy_nox[2]=coefy[3]-coefx[3]/coefx[0]*coefy[0];
			coefy_nox[3]=coefy[4]-coefx[4]/coefx[0]*coefy[0];
			coefy_nox[4]=coefy[5]-coefx[5]/coefx[0]*coefy[0];
			coefy_nox[5]=coefy[6]-coefx[6]/coefx[0]*coefy[0];
			coefy_nox[6]=coefy[7]-coefx[7]/coefx[0]*coefy[0];
			coefy_nox[7]=coefy[8]-coefx[8]/coefx[0]*coefy[0];
			coefy_nox[8]=coefy[9]-coefx[9]/coefx[0]*coefy[0];
			coefy_nox[9]=coefy[10]-coefx[10]/coefx[0]*coefy[0];

			float dx=coefx_noy[1]*xdepth;
			for(int c=0;c<8;c++) dx+=coefx_noy[2+c]*neignbDepth[c];
			dx=-dx/coefx_noy[0];

			float dy=coefy_nox[1]*xdepth;
			for(int c=0;c<8;c++) dy+=coefy_nox[2+c]*neignbDepth[c];
			dy=-dy/coefy_nox[0];

			//find three points not in the same
			int index_align[]={7,6,5,4,3,2,1,0};
			int projx[]={-1,-1,-1,0,0,1,1,1};
			int projy[]={-1,0,1,-1,1,-1,0,1};
			Point neignb[]={x+Point(-1,-1),x+Point(-1,0),x+Point(-1,1),x+Point(0,-1),x+Point(0,1),x+Point(1,-1),x+Point(1,0),x+Point(1,1)};
                        bool found=false;
			for(int n=0;n<8;n++){
				if (neignbDepth[n]!=0.0){
					for(int m=n+1;m<8;m++){
						if((m!=index_align[n])&&(neignbDepth[m]!=0.0)){
                                                        found=true;
							float ndepth=xdepth+dx*projx[n]+dy*projy[n];
							float mdepth=xdepth+dx*projx[m]+dy*projy[m];
							Point xn=neignb[n];
							Point xm=neignb[m];

							float xnx,xny,xnz,xmx,xmy,xmz,xx,xy,xz;

							xnx=(xn.x-row_med)*pixel_size;
							xny=(xn.y-col_med)*pixel_size;
							xnz=focal;
							float xnnorm=sqrt(xnx*xnx+xny*xny+xnz*xnz);
							xnx/=xnnorm*ndepth;
							xny/=xnnorm*ndepth;
							xnz/=xnnrom*ndepth;

							xmx=(xm.x-row_med)*pixel_size;
							xmy=(xm.y-col_med)*pixel_size;
							xmz=focal;
							float xmnorm=sqrt(xmx*xmx+xmy*xmy+xmz*xmz);
							xmx/=xmnorm*mdepth;
							xmy/=xmnorm*mdepth;
							xmz/=xmnrom*mdepth;
                                                        
							xx=(x.x-row_med)*pixel_size;
							xy=(x.y-col_med)*pixel_size;
							xz=focal;
							float xnorm=sqrt(xx*xx+xy*xy+xz*xz);
							xx/=xnorm*xdepth;
							xy/=xnorm*xdepth;
							xz/=xnrom*xdepth;
                                                        
                                                        //produit vectoriel
							float normalx,normaly,normalz;
							normalx=(xny-xy)*(xmz-xz)-(xmy-xy)*(xnz-xz);
							normaly=(xnz-xz)*(xmx-xx)-(xmz-xz)*(xnx-xx);
							normalz=(xnx-xx)*(xmy-xy)-(xmx-xx)*(xny-xy);
							float normalnorm=sqrt(normalx*normalx+normaly*normaly+normalz*normalz);
							int signe=normalx*xx+noramly*xy+normalz*xz;
							signe=(signe>0)?-1:1;
							normalx*=signe/normalnorm;
							normaly*=signe/normalnorm;
							normalz*=signe/normalnorm;
                                                        3dpoints.push_back(Vec<float,3>(xx,xy,xz));
							contour_normal.push_back(x);
							normals.at<Vec<float,3>>(x)[0]=normalx;
							normals.at<Vec<float,3>>(x)[1]=normaly;
							normals.at<Vec<float,3>>(x)[2]=normalz;
                                                        break;
						}
					}
                                        if(found==true) break;
				}
			}
		}
	}
}// for each point in the contour

//select the reliable normals
vector<float> normal_score;
int score=0;
for(int n=0;n<contour_normal.size();n++){
	Point x=contour_normal[n];

	for(int n1=0;n1<8;n1++){
		float similarity=normals.at<Vec<float,3>>(x+neignb1[n1])[0]*normals.at<Vec<float,3>>(x)[0]+normals.at<Vec<float,3>>(x+neignb1[n1])[1]*normals.at<Vec<float,3>>(x)[1]+normals.at<Vec<float,3>>(x+neignb1[n1])[2]*normals.at<Vec<float,3>>(x)[2];
		if(similarity>=similarity_thresh1)score++;
	}

	for(int n2=0;n2<16;n2++){
		float similarity=normals.at<Vec<float,3>>(x+neignb2[n2])[0]*normals.at<Vec<float,3>>(x)[0]+normals.at<Vec<float,3>>(x+neignb2[n2])[1]*normals.at<Vec<float,3>>(x)[1]+normals.at<Vec<float,3>>(x+neignb2[n2])[2]*normals.at<Vec<float,3>>(x)[2];
		if(similarity>=similarity_thresh2)score++;
	}

        normal_score.push_back(score);
}

return tuple<vector<Point>,Mat,vector<float>>(contour_normal,normals,normal_score,3dpoints);
}
