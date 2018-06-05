#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <utility>
using namespace cv;
using namespace std;

//surface noramls
float row_med=239.5;
float col_med=319.5;
float pixel_size=0.0001;
float focal=1.047198;
int height=480;
int width=640;

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

  /// Draw contours
/*
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< thick_contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, thick_contours, i, color, 2, 8, hierarchy, 0, Point() );
     }
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
  waitKey(0);
*/
  return thick_contours;
}


vector<vector<vector<int> >> color_gradient (vector<vector<Point> > contours,Mat img) {
	vector<vector<vector<int> >> sobers;
/*doesn't work cause lose negative grad
	Mat grad(img.size(),img.type());
	Sobel(img, grad, -1, 1,0,3);//grad type 16,horizontal
        cv::Mat m;
	cv::reduce(grad.reshape(1, grad.total()), m, 1, CV_REDUCE_MAX);
	grad = m.reshape(0, grad.rows); 
*/
/*
      cv::FileStorage file("/home/weiyizhang/grasp/robot/abb_ws/src/pose_estimation/sober.xml", cv::FileStorage::WRITE);
      file<<"sober"<<grad;
      file.release();
*/

/*
        cout<<grad.type()<<endl;
	namedWindow( "Contours1", CV_WINDOW_AUTOSIZE );
	imshow( "Contours1", grad );
	waitKey(0);
*/

	vector<vector<int> > contours_grad_h;
        vector<vector<int> > contours_grad_v;
        vector<vector<int> > contours_grad_norm;
	for(int i=0;i<contours.size();i++){
		vector<int> cg_h,cg_v,cg_norm;
		for(int j=0;j<contours[i].size();j++){
			//horizontal
			//-1 0 1
			//-2 0 0
			//-1 0 1
			int ch1_h=(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,-1))[0]+
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(0,-1))[0]*2+
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,-1))[0]-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,1))[0]-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(0,1))[0]*2-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,1))[0];
			int ch2_h=(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,-1))[1]+
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(0,-1))[1]*2+
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,-1))[1]-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,1))[1]-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(0,1))[1]*2-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,1))[1];
			int ch3_h=(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,-1))[2]+
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(0,-1))[2]*2+
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,-1))[2]-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,1))[2]-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(0,1))[2]*2-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,1))[2];
			//vertical
			//-1 -2 -1
			//0  0  0
			//1  2  1
			int ch1_v=(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,-1))[0]+
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,0))[0]*2+
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,1))[0]-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,-1))[0]-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,0))[0]*2-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,1))[0];
			int ch2_v=(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,-1))[1]+
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,0))[1]*2+
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,1))[1]-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,-1))[1]-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,0))[1]*2-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,1))[1];
			int ch3_v=(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,-1))[2]+
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,0))[2]*2+
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(-1,1))[2]-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,-1))[2]-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,0))[2]*2-
				(int)img.at<Vec<uchar,3>>(contours[i][j]+Point(1,1))[2];
                        
                        int norm1=ch1_h*ch1_h+ch1_v*ch1_v;//norm^2
			int norm2=ch2_h*ch2_h+ch2_v*ch2_v;
			int norm3=ch3_h*ch3_h+ch3_v*ch3_v;
			int norm_max=norm1,h_max=ch1_h,v_max=ch1_v;
			if(norm2>norm_max){
				norm_max=norm2;
				h_max=ch2_h;
				v_max=ch2_v;
			}
			if(norm3>norm_max){
				norm_max=norm3;
				h_max=ch3_h;
				v_max=ch3_v;
			}
			cg_h.push_back(h_max);
			cg_v.push_back(v_max);
			cg_norm.push_back(norm_max);
			
	        }
		contours_grad_h.push_back(cg_h);
		contours_grad_v.push_back(cg_v);
		contours_grad_norm.push_back(cg_norm);
	}

/*
        //contours and sober contours correspond
	Mat drawing = Mat::zeros(img.size(), CV_8UC3);
	Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	drawContours( drawing, contours, -1, color);
	namedWindow( "Contours1", CV_WINDOW_AUTOSIZE );
	imshow( "Contours1", drawing+grad );
	waitKey(0);
*/
	sobers.push_back(contours_grad_h);
	sobers.push_back(contours_grad_v);
	sobers.push_back(contours_grad_norm);

	return sobers;//two vector of two direction gradients and the other vector of norm^2
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




tuple<vector<Point>,Mat,vector<float>> surface_normal(Mat depth, vector<Point> contour){

vector<Point> contour_normal;
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

return tuple<vector<Point>,Mat,vector<float>>(contour_normal,normals,normal_score);
}

