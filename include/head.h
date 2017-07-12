#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <errno.h>
#include <iostream>
#include<vector>
#include"fflog.h"

using namespace std;
using namespace cv;

#define logg 0
extern int gNowPos;

struct  PointCloud{
	float x;
	float y;
	float z;
	uint8_t r;
	uint8_t g;
	uint8_t b;
	float  ux;
	float  uy;
	int  ref;
	PointCloud(){}
	PointCloud(float x1,float y1,float z1,float r1 ,float g1,float b1,float u1,float v1){
		x = x1;
		y = y1;
		z = z1;
		r = r1;
		g =g1;
		b = b1;
		ux = u1;
		uy = v1;
		ref = 0;
	}
	PointCloud(float x1,float y1,float z1,int r1 ,int g1,int b1,float u1,float v1){
		x = x1;
		y = y1;
		z = z1;
		r = r1;
		g =g1;
		b = b1;
		ux = u1;
		uy = v1;
		ref = 0;
	}
	PointCloud(double  &x1,double &y1,double &z1,double &r1 ,double &g1,double &b1,double &u1,double &v1){
		
		x = x1;
		y = y1;
		z = z1;
		
		r = r1;
		g =g1;
		b = b1;
		
		ux = u1;
		uy = v1;
		ref = 0;
	}	

	void swap(PointCloud p,int a){
		x = p.x;
		y = p.y;
		z = p.z;
		r = p.r;
		g = p.g;
		b = p.b;
		ux = p.ux;
		uy = p.uy;
		ref = p.ref;
		
	}

	
	
};


struct  PointLine{

	int id;
	double angle;
	vector<PointCloud>  point;
	//store  all   small  black point
	vector<KeyPoint>  keypoints;
	int  keyPoint_max_y;
	//center of   six big  black  point
	Point2f  offset_points_center;
	int  postion;
	double offset;
	long  sum_of_prevline;
	PointLine(){ id = -1;}
	PointLine(int idx, vector<PointCloud>  p){
		id =idx;
		point = p;
		offset = 0;

	}
	void swap(PointLine p){
		id = p.id;
		offset = p.offset;
		point.swap(p.point);
		keypoints.swap(p.keypoints);
		angle = p.angle;
		
	}
	void clear(){
		point.clear();
		keypoints.clear();
	}
	
};

extern vector<PointLine> gLines;

int light_extraction(Mat wrap_img, vector<Point3f>  & lightdotl);
void reconstuct_3D(vector<Point3f>lightdotl,vector<Point3f>&xyz);
int surface_recon();

