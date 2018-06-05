#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <utility>

using namespace cv;
using namespace std;

Point neignb1[]={Point(-1,-1),Point(-1,0),Point(-1,1),Point(0,-1),Point(0,1),Point(1,-1),Point(1,0),Point(1,1)}
Point neignb2[]={Point(-2,-2),Point(-2,-1),Point(-2,0),Point(-2,1),Point(-2,2),Point(-1,-2),Point(-1,2),Point(0,-2),Point(0,2),Point(1,-2),Point(1,2),Point(2,-2),Point(2,-1),Point(2,0),Point(2,1),Point(2,2)}

Vec<float,3> point_normal(Mat depth, Point x);
int point_orient(Mat rgb, Point x);
pair<vector<int>,vector<float>> quickSort(vector<int> temp_num,vector<float> scores);

/** @function main */
int main( int argc, char** argv )
{
//template information
vector<vector<Point>> normal_points;
vector<vector<Vec<float,3>>> normals;
vector<vector<Point>> grad_points;
vector<vector<int>>  grads;
int num_template;
//input image
Mat depth;
Mat rgb;
//....

//line-mod similarity 
vector<float> scores;
vector<int> temp_num;
for(int temp=0;temp<num_template;temp++){

	vector<Point> npts=normal_points[temp];
	vector<Vec<float,3>> ns=normals[temp];
	vector<Point> gdpts=grad_points[temps];
	vector<int> gds=grads[temp];
	float score=0.0;

	//check normals similarity
	for(int i=0;i<npts.size();i++){

		Vec<float,3> normali=point_normal(depth,npts[i]);
		float similarity_max=normali[0]*ns[i][0]+normali[1]*ns[i][1]+normali[2]*ns[i][2];
		float tmp;
		//neignb1
		for(int nb=0;nb<8;nb++){
		Vec<float,3> normal_nb=point_normal(depth,npts[i]+neignb1[nb]);
		tmp=normal_nb[0]*ns[i][0]+normal_nb[1]*ns[i][1]+normal_nb[2]*ns[i][2];
		if(tmp>similarity_max) similarity_max=tmp;
		}
		//neignb2
		for(int nb=0;nb<16;nb++){
		Vec<float,3> normal_nb=point_normal(depth,npts[i]+neignb2[nb]);
		tmp=normal_nb[0]*ns[i][0]+normal_nb[1]*ns[i][1]+normal_nb[2]*ns[i][2];
		if(tmp>similarity_max) similarity_max=tmp;
		}
		//max
		score+=similarity_max;
	}

	//check gradient similarity
	float orient_similarity[]={1.0,0.8090170,0.3090170,-0.3090170,-0.8090170,-1.0};
	for(int i=0;i<gdpts.size();i++){
		int orienti=point_orient(rgb,gdpts[i]);
		float similarity_max= orient_similarity[ (abs(gds[i]-orienti)>5)?(10-abs(gds[i]-orienti)):abs(gds[i]-orienti) ];
		//neignb1
		for(int nb=0;nb<8;nb++){
		int orient_nb=point_orient(rgb,gdpts[i]+neignb1[nb]);
		tmp=orient_similarity[ (abs(gds[i]-orient_nb)>5)?(10-abs(gds[i]-orient_nb)):abs(gds[i]-orient_nb) ];
		if(tmp>similarity_max) similarity_max=tmp;
		}
		//neignb2
		for(int nb=0;nb<16;nb++){
		int orient_nb=point_orient(rgb,gdpts[i]+neignb2[nb]);
		tmp=orient_similarity[ (abs(gds[i]-orient_nb)>5)?(10-abs(gds[i]-orient_nb)):abs(gds[i]-orient_nb) ];
		if(tmp>similarity_max) similarity_max=tmp;
		}
		//max
		score+=similarity_max;
	}

	scores.push_back(score);
	temp_num.push_back(temp);
}

//sorted scores
pair<vector<int>,vector<float>> sorted_temp=quickSort(temp_num,scores);
sorted_temp_num=sorted_temp.first;
//post processing detection
//template image
vector<Mat> depths;
vector<Mat> rgbs;
//Coarse Outlier Removal by Color

Mat hsv=Mat(rgbs[sorted_temp_num[num_template-1]].size(),rgbs[sorted_temp_num[num_template-1]].type());
cvtColor(rgbs[sorted_temp_num[num_template-1]], hsv,cv.COLOR_BGR2HSV);
Mat hsv_input=Mat(rgb.size(),rgb.type());
cvtColor(rgb, hsv_input,cv.COLOR_BGR2HSV);
int hue_thresh;
int total_pix=0;
int pix_ok=0;
for(int i=0;i<hsv.rows;i++){
	for(int j=0;j<hsv.cols;j++){
		if((int)hsv.at<Vec<uchar,3>>(Point(i,j))[0]>0){
			total_pix++;
			if(abs((int)hsv.at<Vec<uchar,3>>(Point(i,j))[0]-(int)hsv_input.at<Vec<uchar,3>>(Point(i,j))[0])<hue_thresh) pix_ok++;	
		}
	}
}
if((float)pix_ok/pix_total)
//Fast Pose Estimation and Outlier Rejection based on Depth

return 0;
}


Vec<float,3> point_normal(Mat depth, Point x){
	Vec<float,3> normal(0.0,0.0,0.0);
	
	//1 2 3
	//4 x 5
	//6 7 8
	//we consider only the pixels in the neignborhood of 8 centered on the pixel considered
	float xdepth=(float)depth.at<float>(x);
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

		//
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

						xnx=(xn.x-239.5)*pixel_size;
						xny=(xn.y-319.5)*pixel_size;
						xnz=focal;
						float xnnorm=sqrt(xnx*xnx+xny*xny+xnz*xnz);
						xnx/=xnnorm*ndepth;
						xny/=xnnorm*ndepth;
						xnz/=xnnrom*ndepth;

						xmx=(xm.x-239.5)*pixel_size;
						xmy=(xm.y-319.5)*pixel_size;
						xmz=focal;
						float xmnorm=sqrt(xmx*xmx+xmy*xmy+xmz*xmz);
						xmx/=xmnorm*mdepth;
						xmy/=xmnorm*mdepth;
						xmz/=xmnrom*mdepth;

						xx=(x.x-239.5)*pixel_size;
						xy=(x.y-319.5)*pixel_size;
						xz=focal;
						float xnorm=sqrt(xx*xx+xy*xy+xz*xz);
						xx/=xnorm*xdepth;
						xy/=xnorm*xdepth;
						xz/=xnrom*xdepth;

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

						normal[0]=normalx;
						normal[1]=normaly;
						normal[2]=normalz;
		                                break;
					}
				}
		                if(found==true) break;
			}
		}
	}

	return normal;

}

//------------------------------------------------------------------------------------------------------------------
int point_orient(Mat rgb, Point x){

	//horizontal
	//-1 0 1
	//-2 0 0
	//-1 0 1
	int ch1_h=(int)img.at<Vec<uchar,3>>(x+Point(-1,-1))[0]+
		(int)img.at<Vec<uchar,3>>(x+Point(0,-1))[0]*2+
		(int)img.at<Vec<uchar,3>>(x+Point(1,-1))[0]-
		(int)img.at<Vec<uchar,3>>(x+Point(-1,1))[0]-
		(int)img.at<Vec<uchar,3>>(x+Point(0,1))[0]*2-
		(int)img.at<Vec<uchar,3>>(x+Point(1,1))[0];
	int ch2_h=(int)img.at<Vec<uchar,3>>(x+Point(-1,-1))[1]+
		(int)img.at<Vec<uchar,3>>(x+Point(0,-1))[1]*2+
		(int)img.at<Vec<uchar,3>>(x+Point(1,-1))[1]-
		(int)img.at<Vec<uchar,3>>(x+Point(-1,1))[1]-
		(int)img.at<Vec<uchar,3>>(x+Point(0,1))[1]*2-
		(int)img.at<Vec<uchar,3>>(x+Point(1,1))[1];
	int ch3_h=(int)img.at<Vec<uchar,3>>(x+Point(-1,-1))[2]+
		(int)img.at<Vec<uchar,3>>(x+Point(0,-1))[2]*2+
		(int)img.at<Vec<uchar,3>>(x+Point(1,-1))[2]-
		(int)img.at<Vec<uchar,3>>(x+Point(-1,1))[2]-
		(int)img.at<Vec<uchar,3>>(x+Point(0,1))[2]*2-
		(int)img.at<Vec<uchar,3>>(x+Point(1,1))[2];
	//vertical
	//-1 -2 -1
	//0  0  0
	//1  2  1
	int ch1_v=(int)img.at<Vec<uchar,3>>(x+Point(-1,-1))[0]+
		(int)img.at<Vec<uchar,3>>(x+Point(-1,0))[0]*2+
		(int)img.at<Vec<uchar,3>>(x+Point(-1,1))[0]-
		(int)img.at<Vec<uchar,3>>(x+Point(1,-1))[0]-
		(int)img.at<Vec<uchar,3>>(x+Point(1,0))[0]*2-
		(int)img.at<Vec<uchar,3>>(x+Point(1,1))[0];
	int ch2_v=(int)img.at<Vec<uchar,3>>(x+Point(-1,-1))[1]+
		(int)img.at<Vec<uchar,3>>(x+Point(-1,0))[1]*2+
		(int)img.at<Vec<uchar,3>>(x+Point(-1,1))[1]-
		(int)img.at<Vec<uchar,3>>(x+Point(1,-1))[1]-
		(int)img.at<Vec<uchar,3>>(x+Point(1,0))[1]*2-
		(int)img.at<Vec<uchar,3>>(x+Point(1,1))[1];
	int ch3_v=(int)img.at<Vec<uchar,3>>(x+Point(-1,-1))[2]+
		(int)img.at<Vec<uchar,3>>(x+Point(-1,0))[2]*2+
		(int)img.at<Vec<uchar,3>>(x+Point(-1,1))[2]-
		(int)img.at<Vec<uchar,3>>(x+Point(1,-1))[2]-
		(int)img.at<Vec<uchar,3>>(x+Point(1,0))[2]*2-
		(int)img.at<Vec<uchar,3>>(x+Point(1,1))[2];
        
        float norm1=sqrt(ch1_h*ch1_h+ch1_v*ch1_v);
	float norm2=sqrt(ch2_h*ch2_h+ch2_v*ch2_v);
	float norm3=sqrt(ch3_h*ch3_h+ch3_v*ch3_v);
	float norm_max=norm1;
	int h_max=ch1_h,v_max=ch1_v;
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
        // quantized orientation
	int orient_idx=0;
	if(h_max>=0&&v_max>=0){
		orient_idx=1;
		float proj=h_max*0.9510565+v_max*0.3090170;
		if(h_max*0.5877852+v_max*0.8090170>proj){
			proj=h_max*0.5877852+v_max*0.8090170;
			orient_idx=2;
		}
		if(v_max>proj){
			orient_idx=3;
		}
	}
	else if(h_max<=0&&v_max>=0){
		orient_idx=3;
		float proj=v_max;
		if(-h_max*0.5877852+v_max*0.8090170>proj){
			proj=-h_max*0.5877852+v_max*0.8090170;
			orient_idx=4;
		}
		if(-h_max*0.9510565+v_max*0.3090170>proj){
			orient_idx=5;
		}
	}
	else if(h_max<=0&&v_max<=0){
		orient_idx=6;
		float proj=-h_max*0.9510565-v_max*0.3090170;
		if(-h_max*0.5877852-gv*0.8090170>proj){
			proj=-h_max*0.5877852-v_max*0.8090170;
			orient_idx=7;
		}
		if(-v_max>proj){
			orient_idx=8;
		}
	}
	else if(h_max>=0&&v_max<=0){
		orient_idx=8;
		float proj=-v_max;
		if(h_max*0.5877852-v_max*0.8090170>proj){
			proj=h_max*0.5877852-v_max*0.8090170;
			orient_idx=9;
		}
		if(h_max*0.9510565-v_max*0.3090170>proj){
			orient_idx=10;
		}
	}
	return orient_idx;


}
//------------------------------------------------------------------------------------------------------------------
pair<vector<int>,vector<float>> quickSort(vector<int> temp_num,vector<float> scores){
	int length=temp_num.size();
	int pos_median=(int)(length/2);//index
	float value_median=scores[pos_median];
	int l=0;//index
	int r=length-1;//index
	while(true){
		//left
		while(scores[l]<=value_median&&l<pos_median) l++;
		//right
		while(scores[r]>=value_median&&r>pos_median) r--;

		if(l<pos_median&&r>pos_median){
			float tmp=scores[l];
			scores[l]=scores[r];
			scores[r]=tmp;

			int tmp2=temp_num[l];
			temp_num[l]=temp_num[r];
			temp_num[r]=tmp2;
			l++;
			r--;
		}else if(l<pos_median&&r==pos_median){
			scores.insert(scores.begin()+pos_median+1,scores[l]);
			scores.erase(scores.begin()+l);
			temp_num.insert(temp_num.begin()+pos_median+1,temp_num[l]);
			temp_num.erase(temp_num.begin()+l);
			pos_median--;
			r=pos_median;
		}else if(l==pos_median&&r>pos_median){
			scores.insert(scores.begin()+pos_median,scores[r]);
			scores.erase(scores.begin()+r+1);
			temp_num.insert(temp_num.begin()+pos_median,temp_num[r]);
			temp_num.erase(temp_num.begin()+r+1);
			pos_median++;
			l=pos_median;
		}else break;
	}
	//left
	vector<int> numl;
	numl.reserve(pos_median+1);
	numl.insert(numl.begin(),temp_num.begin(),temp_num.begin()+pos_median+1);
	vector<float> sl;
	sl.reserve(pos_median+1);
	sl.insert(sl.begin(),scores.begin(),scores.begin()+pos_median+1);

	if(pos_median>1){
		pair<vector<int>,vector<float>> result=quickSort(numl,sl);
		numl=result.first;
		sl=result.second;
	}
	//right
	vector<int> numr;
	numr.reserve(length-pos_median);
	numr.insert(numr.begin(),temp_num.begin()+pos_median,temp_num.end());
	vector<int> sr;
	sr.reserve(length-pos_median);
	sr.insert(sr.begin(),scores.begin()+pos_median,scores.end());

	if(pos_median<length-2){
		pair<vector<int>,vector<float>> result=quickSort(numr,sr);
		numr=result.first;
		sr=result.second;
	}

	//concatenate
	vector<int> num;
	num.insert(num.begin(),numl.begin(),numl.end());
	if(numr.size()>1) num.insert(num.end(),numr.begin()+1,numr.end());

	vector<float> S;
	S.insert(S.begin(),sl.begin(),sl.end());
	if(sr.size()>1) S.insert(S.end(),sr.begin()+1,sr.end());

	return pair<vector<int>,vector<float>>(num,S);
}











