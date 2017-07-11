#include"head.h"
#include"BlobMatch.h"
#include"MarkerRecognizer.h"



int ImgNum = 100;
int gNowPos = 0;

MarkerRecognizer m_recognizer;
vector<PointLine> gLines;
int sum_points;

int imgRead(int id,Mat &gray,Mat & img)
{
  char path[256] = "/0";
  sprintf(path,"data/%d.jpg",id);
  Mat src = imread(path);
  if(src.empty()){
    LOGPP("imread pic is error %s",path);
    return -1;
  }
  src.copyTo(img);
  cvtColor(src,gray,CV_BGR2GRAY);
  return 0;
}

void imgRotate(Mat src,Mat &dst,float angle)
{
 
  Point2f center;
  center.x=float (src.cols/2.0+0.5);  
  center.y=float (src.rows/2.0+0.5);  
  Mat M(2,3,CV_32FC1);
  M = getRotationMatrix2D(center,angle,1.0);
  warpAffine(src,dst, M, src.size() );
}

int reconstuct(int id)
{

  Mat gray,wrap_img,img;
  int ret = imgRead(id,gray,img);
  if (ret ==-1)
    return -1;
  imgRotate(img,wrap_img,180.0);
  if(id ==1)
  {
    if(logg) imwrite("1.jpg",wrap_img);
  }

//  m_recognizer.update(wrap_img, 100);
//  float angle1 = m_recongizer.getAngle();

//  Point2f center = m_recongizer.getCenter();

  
  // 关于mark 设置
  vector<Point3f> lightdotl;
  int ret1 = light_extraction(wrap_img,lightdotl);
  if(ret1 <0 )
    return -4;
  LOGPP("ligth_extraction size is :%d",lightdotl.size());

  vector<Point3f> xyz;
  xyz.clear();
  int len = lightdotl.size();
  xyz.resize(len);
  
  reconstuct_3D(lightdotl,xyz);
 
  PointLine aLine;
  for(int i = 0;i< len;i++)
  {
    
    int r = 255;
    int g = 255;
    int b = 255;
    float v = 1.0;
    float u = 1.0;
    aLine.point.push_back(PointCloud(xyz[i].x,xyz[i].y,xyz[i].z,r,g,b,u,v));
    
  }
 
  gLines.push_back(aLine);
  sum_points += len;
  
  return 0;
}

int savePLY()
{
  char file[256] = "out.ply";
  FILE *fp = fopen(file,"w+");
  if(fp==NULL){
    LOGPP("open ply file is error");
    return -1;
  }
  
  fprintf(fp,"ply\n");
	fprintf(fp,"format ascii 1.0\n");
	fprintf(fp,"element vertex %d\n", sum_points);
	fprintf(fp,"property float x\n");
	fprintf(fp,"property float y\n");
	fprintf(fp,"property float z\n");
	fprintf(fp,"property uchar red\n");
	fprintf(fp,"property uchar green\n");
	fprintf(fp,"property uchar blue\n");
	fprintf(fp,"end_header\n");
  for(int i = 0;i<gLines.size();i++)
  {
    for(int j = 0;j<gLines[i].point.size();j++)
    {
      fprintf(fp,"%f %f %f %d %d %d\n",gLines[i].point[j].x,gLines[i].point[j].y,gLines[i].point[j].z,255,255,255);
    }
  }
  fclose(fp);
  return 0;

}

int main()
{

  blob_detector_init();
  int ret;
  for(int i= 1;i<ImgNum;i++)
  {
    gNowPos = i;
    ret = reconstuct(i);
  }
  cout << gLines.size()<< endl;

  savePLY();

 ret = surface_recon();
  
//  ret = save_recon();

  return ret;

}