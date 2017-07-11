#ifndef  TEMPLATE_MATCH
#define TEMPLATE_MATCH
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include<opencv2/imgproc/imgproc.hpp>  
#include <head.h>
#include <ffdefine.h>
#include <fflog.h>

using namespace cv;
using namespace std;


int  match_template(Mat &img,int id, int *x,int *y)  ;
int  match_orb(Mat  &ima,Mat &imb,int id, int *x, int *y, double  *angle) ;

void blob_detector_init();
int blob_detector(Mat &ima,Mat  &ima_gray,PointLine  &aline);
int blob_detector_off(Mat &ima,Mat  &ima_gray,PointLine  &aline);



#endif
