#include"head.h"

int light_extraction(Mat wrap_img,vector<Point3f>& lightdotl)
{
  Mat img_rgb[3] ;
  split(wrap_img,img_rgb);
  Mat src = img_rgb[2];
 

  int startX = 5;
  int endX = 645;
  int startY = 5;
  int endY = 700;

  Rect roi(startX,startY,endX-startX,endY-startY);
  Mat roi_img;
  src(roi).copyTo(roi_img);
  
  Mat filter_img;
  medianBlur(roi_img,filter_img,3);
 
  Mat thresh_img;
 // threshold(filter_img,thresh_img,220,255,THRESH_OTSU);
  threshold(filter_img,thresh_img,220,255,CV_THRESH_BINARY); 
  if(logg) imwrite("thresh_img.jpg",thresh_img);

  int dilation_type;
  int dilation_size = 3;
  int dilation_elem = 0;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  dilate(thresh_img, thresh_img, element); // 做膨胀操作，消除目标的不连续空洞

  if(logg) imwrite("dilate.jpg",thresh_img);

  vector<vector<Point> >contours;
  vector<Vec4i> hierarchy;  
      
  findContours( thresh_img, contours, hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,  Point(0,0) );  
  if(contours.size() == 0||hierarchy.size() == 0 ){
    LOGPP("cannot find contoures");
    return -4;
  }
  
  thresh_img.setTo(0);
  
  for (int i = 0; i >= 0; i = hierarchy[i][0])
  {
    double area = contourArea(contours[i]);
    if (area > 5)
    {
      drawContours(thresh_img, contours, i, Scalar(255), CV_FILLED, 8, hierarchy, 0);
    }
  }
  if(logg) imwrite("contours.jpg",thresh_img);
 
  vector<Point2i> position;
  Mat pos;
  findNonZero(thresh_img,pos);
  position = Mat_<Point2i>(pos);
  
  int curr_y = position[0].y;
  int sum_x =  position[0].x;
  int num = 0;
  Mat final_image = Mat::zeros(thresh_img.size(),CV_8UC1);
  for(int i=1;i<position.size();i++)
  {
    
    if(position[i].y == curr_y)
    {
      sum_x += position[i].x;
      num +=1;
    }else{
      int x = sum_x/num+0.5;
      int y =  curr_y;
      lightdotl.push_back(Point3f(x,y,0));
      sum_x = position[i].x;
      num = 1;
      curr_y = position[i].y ; 
      final_image.at<uchar>(y,x) = 255;
    }
    
  }
  if(logg) imwrite("final.jpg",final_image);
  return 0;

}