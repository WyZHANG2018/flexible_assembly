#include "icosahedron.h"


/////////////////////////////////////
/////////////////////////////////////
std::vector<geometry_msgs::Point> vertices_edges(int num_seg, float x,float y, float z, float xx,float yy, float zz){
float r=std::sqrt((5.0+std::sqrt(5.0))/2.0);
std::vector<geometry_msgs::Point> v_edg;
float ratio,x_tmp,y_tmp,z_tmp;
for (int i=1;i<num_seg;i++){
x_tmp= x*(float)(i/num_seg)+xx*(1.0-(float)(i/num_seg));
y_tmp= y*(float)(i/num_seg)+yy*(1.0-(float)(i/num_seg));
z_tmp= z*(float)(i/num_seg)+zz*(1.0-(float)(i/num_seg));
ratio=r/sqrt(x_tmp*x_tmp+y_tmp*y_tmp+z_tmp*z_tmp);
geometry_msgs::Point p;
p.x=x_tmp*ratio;
p.y=y_tmp*ratio;
p.z=z_tmp*ratio;
v_edg.push_back(p);
}

return v_edg;

}

/////////////////////////////////////
/////////////////////////////////////
std::vector<geometry_msgs::Point> vertices_faces(int num_seg,float xv, float yv,float zv,float x1,float y1,float z1,float x2,float y2,float z2){

std::vector<geometry_msgs::Point> v_face;
float ratio,x_tmp,y_tmp,z_tmp;
float r=std::sqrt((5.0+std::sqrt(5.0))/2.0);
for (int i=1;i<num_seg-1;i++){
for (int j=0;j<i;j++){
x_tmp=xv+(x1-xv)*(float)((i-j)/num_seg)+(x2-xv)*(float)((j+1)/num_seg);
y_tmp=yv+(y1-yv)*(float)((i-j)/num_seg)+(y2-yv)*(float)((j+1)/num_seg);
z_tmp=zv+(z1-zv)*(float)((i-j)/num_seg)+(z2-zv)*(float)((j+1)/num_seg);
ratio=r/sqrt(x_tmp*x_tmp+y_tmp*y_tmp+z_tmp*z_tmp);
geometry_msgs::Point p;
p.x=x_tmp*ratio;
p.y=y_tmp*ratio;
p.z=z_tmp*ratio;
v_face.push_back(p);
}
}

return v_face;
}


/////////////////////////////////////
/////////////////////////////////////
std::vector<geometry_msgs::Point> all_vertices(int num_iter){
std::vector<geometry_msgs::Point> vp;
// std::vector push back a copy of the object
//12 vertices

////
geometry_msgs::Point p;
p.x=1.0;
p.y=PHI;
p.z=0.0;
vp.push_back(p);

p.x=1.0;
p.y=-PHI;
p.z=0.0;
vp.push_back(p);

p.x=-1.0;
p.y=PHI;
p.z=0.0;
vp.push_back(p);

p.x=-1.0;
p.y=-PHI;
p.z=0.0;
vp.push_back(p);
////
p.x=PHI;
p.y=1.0;
p.z=0.0;
vp.push_back(p);

p.x=PHI;
p.y=-1.0;
p.z=0.0;
vp.push_back(p);

p.x=-PHI;
p.y=1.0;
p.z=0.0;
vp.push_back(p);

p.x=-PHI;
p.y=-1.0;
p.z=0.0;
vp.push_back(p);
////
p.x=0.0;
p.y=1.0;
p.z=PHI;
vp.push_back(p);

p.x=0.0;
p.y=1.0;
p.z=-PHI;
vp.push_back(p);

p.x=0.0;
p.y=-1.0;
p.z=PHI;
vp.push_back(p);

p.x=0.0;
p.y=-1.0;
p.z=-PHI;
vp.push_back(p);

//30 edges
int num_seg=pow(2,num_iter);
float x,y,z,xx,yy,zz;
std::vector<geometry_msgs::Point> v_edg;

////1
x=0.0;y=1.0;z=PHI;
xx=-1.0;yy=PHI;zz=0.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////2
x=0.0;y=1.0;z=PHI;
xx=-PHI;yy=0.0;zz=1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////3
x=-1.0;y=PHI;z=0.0;
xx=-PHI;yy=0.0;zz=1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////4
x=0.0;y=-1.0;z=PHI;
xx=-1.0;yy=-PHI;zz=0.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////5
x=0.0;y=-1.0;z=PHI;
xx=-PHI;yy=-0.0;zz=1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////6
x=-1.0;y=-PHI;z=0.0;
xx=-PHI;yy=-0.0;zz=1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////7
x=-0.0;y=-1.0;z=PHI;
xx=1.0;yy=-PHI;zz=0.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////8
x=-0.0;y=-1.0;z=PHI;
xx=PHI;yy=-0.0;zz=1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////9
x=1.0;y=-PHI;z=0.0;
xx=PHI;yy=-0.0;zz=1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////10
x=0.0;y=1.0;z=PHI;
xx=-1.0;yy=PHI;zz=0.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////11
x=0.0;y=1.0;z=PHI;
xx=-PHI;yy=0.0;zz=1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////12
x=-1.0;y=PHI;z=0.0;
xx=-PHI;yy=0.0;zz=1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////13
x=0.0;y=1.0;z=-PHI;
xx=-1.0;yy=PHI;zz=0.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////14
x=0.0;y=1.0;z=-PHI;
xx=-PHI;yy=0.0;zz=1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////15
x=-1.0;y=PHI;z=0.0;
xx=-PHI;yy=0.0;zz=-1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////16
x=0.0;y=1.0;z=-PHI;
xx=1.0;yy=PHI;zz=0.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////17
x=0.0;y=1.0;z=-PHI;
xx=PHI;yy=0.0;zz=1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////18
x=1.0;y=PHI;z=0.0;
xx=PHI;yy=0.0;zz=-1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////19
x=0.0;y=-1.0;z=-PHI;
xx=1.0;yy=-PHI;zz=0.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////20
x=0.0;y=-1.0;z=-PHI;
xx=PHI;yy=0.0;zz=1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////21
x=1.0;y=-PHI;z=0.0;
xx=PHI;yy=0.0;zz=-1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////22
x=0.0;y=-1.0;z=-PHI;
xx=-1.0;yy=-PHI;zz=0.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////23
x=0.0;y=-1.0;z=-PHI;
xx=-PHI;yy=0.0;zz=1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////24
x=-1.0;y=-PHI;z=0.0;
xx=-PHI;yy=0.0;zz=-1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////25
x=0.0;y=1.0;z=PHI;
xx=0.0;yy=-1.0;zz=PHI;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////26
x=0.0;y=1.0;z=-PHI;
xx=0.0;yy=-1.0;zz=-PHI;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////27
x=1.0;y=PHI;z=0.0;
xx=-1.0;yy=PHI;zz=0.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////28
x=1.0;y=-PHI;z=0.0;
xx=-1.0;yy=-PHI;zz=0.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////29
x=-PHI;y=0.0;z=1.0;
xx=-PHI;yy=0.0;zz=-1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

////30
x=PHI;y=0.0;z=1.0;
xx=PHI;yy=0.0;zz=-1.0;
v_edg=vertices_edges(num_seg, x, y, z, xx, yy, zz);
vp.insert(vp.end(), v_edg.begin(), v_edg.end());

//20 faces
std::vector<geometry_msgs::Point> v_face;
float xv,yv,zv,x1,y1,z1,x2,y2,z2;

////1
xv=0.0;yv=1.0;zv=PHI;
x1=-1.0;y1=PHI;z1=0.0;
x2=-PHI;y2=0.0;z2=1.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////2
xv=0.0;yv=1.0;zv=PHI;
x1=1.0;y1=PHI;z1=0.0;
x2=PHI;y2=0.0;z2=1.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////3
xv=-PHI;yv=0.0;zv=1.0;
x1=0.0;y1=1.0;z1=PHI;
x2=0.0;y2=-1.0;z2=1.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////4
xv=PHI;yv=0.0;zv=1.0;
x1=0.0;y1=1.0;z1=PHI;
x2=0.0;y2=-1.0;z2=1.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////5
xv=0.0;yv=-1.0;zv=PHI;
x1=-PHI;y1=0.0;z1=1.0;
x2=-1.0;y2=-PHI;z2=0.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////6
xv=0.0;yv=-1.0;zv=PHI;
x1=PHI;y1=0.0;z1=1.0;
x2=1.0;y2=-PHI;z2=0.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////7
xv=0.0;yv=1.0;zv=PHI;
x1=1.0;y1=PHI;z1=0.0;
x2=-1.0;y2=PHI;z2=0.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////8
xv=0.0;yv=-1.0;zv=PHI;
x1=1.0;y1=-PHI;z1=0.0;
x2=-1.0;y2=-PHI;z2=0.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////9
xv=0.0;yv=1.0;zv=-PHI;
x1=-1.0;y1=PHI;z1=0.0;
x2=-PHI;y2=0.0;z2=-1.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////10
xv=0.0;yv=1.0;zv=-PHI;
x1=1.0;y1=PHI;z1=0.0;
x2=PHI;y2=0.0;z2=-1.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////11
xv=-PHI;yv=0.0;zv=-1.0;
x1=0.0;y1=1.0;z1=-PHI;
x2=0.0;y2=-1.0;z2=-1.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////12
xv=PHI;yv=0.0;zv=-1.0;
x1=0.0;y1=1.0;z1=-PHI;
x2=0.0;y2=-1.0;z2=-1.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////13
xv=0.0;yv=-1.0;zv=-PHI;
x1=-PHI;y1=0.0;z1=-1.0;
x2=-1.0;y2=-PHI;z2=0.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////14
xv=0.0;yv=-1.0;zv=-PHI;
x1=PHI;y1=0.0;z1=-1.0;
x2=1.0;y2=-PHI;z2=0.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////15
xv=0.0;yv=1.0;zv=-PHI;
x1=1.0;y1=PHI;z1=0.0;
x2=-1.0;y2=PHI;z2=0.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////16
xv=0.0;yv=-1.0;zv=-PHI;
x1=1.0;y1=-PHI;z1=0.0;
x2=-1.0;y2=-PHI;z2=0.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////17
xv=-PHI;yv=0.0;zv=1.0;
x1=-1.0;y1=PHI;z1=0.0;
x2=-PHI;y2=0.0;z2=-1.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////18
xv=-PHI;yv=0.0;zv=1.0;
x1=-1.0;y1=-PHI;z1=0.0;
x2=-PHI;y2=0.0;z2=-1.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////19
xv=PHI;yv=0.0;zv=1.0;
x1=1.0;y1=PHI;z1=0.0;
x2=PHI;y2=0.0;z2=-1.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

////20
xv=PHI;yv=0.0;zv=1.0;
x1=1.0;y1=-PHI;z1=0.0;
x2=PHI;y2=0.0;z2=-1.0;
v_face=vertices_faces(num_seg,xv,yv,zv,x1,y1,z1,x2,y2,z2);
vp.insert(vp.end(), v_face.begin(), v_face.end());

return vp;
}
