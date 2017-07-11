#include"head.h"


void reconstuct_3D(vector<Point3f>lightdotl,vector<Point3f>& xyz)
{
  xyz.clear();
  Mat M = (Mat_<double>(4,4)<<
    1.18899069e+03 ,  0.00000000e+00  , 7.24555716e+02,0,
    0.00000000e+00 ,  1.18782173e+03  , 5.39395564e+02, 0,
    0., 					0., 					1.,0,
    0,					0,					0,1);

  Mat invM = M.inv();
//  cout << invM<< endl;

  double uv[4]={1};  
  Mat UV=Mat(4,1,CV_64FC1,uv);

  double xz[4]={1};  
  Mat XZ=Mat(4,1,CV_64FC1,xz); 
  int beishu = 750;

  for(int i=0;i<lightdotl.size();i++)
  {
   
    uv[0] = lightdotl[i].x;
    uv[1] = lightdotl[i].y;
    uv[2] = 1;

    XZ = invM*UV;

    float x = xz[0]*3*beishu;
    float y = xz[1]*700;

    uv[0] = lightdotl[i].z;
		uv[1] = lightdotl[i].y;
		uv[2] =1;
    

    XZ = invM*UV;
    float z = xz[0]*beishu;
    xyz.push_back(Point3f(x,y,z));
    
    
  } 
  

}