#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <errno.h>
#include<algorithm>
#include<numeric>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <ImgRead.h>
#include <fflog.h>

#include <BlobMatch.h>
#include <head.h>

struct   BlobKey{
	double k;
	int id;
	BlobKey(){}
	BlobKey(int i,double s){
		k = s;
		id = i;
	}
};

static SimpleBlobDetector *gDetector;


bool KeyPoint_sort_function(KeyPoint i, KeyPoint j) {
	return (i.pt.x < j.pt.x);
}

bool KeyPoint_sort_function_size(KeyPoint i, KeyPoint j) {
	return (i.size < j.size);
}

bool KeyPoint_sort_function_y(KeyPoint i, KeyPoint j) {
	return (i.pt.y < j.pt.y);
}

bool BlobKey_sort_function(BlobKey i, BlobKey j) {
	return (i.k < j.k);
}


//need  modify,     from  middle to  both sides
static int  statistical_distribution(std::vector<KeyPoint>  &xyz, vector < vector<KeyPoint> >  &point, double range){

    if(xyz.size() < 2) {
		LOGPP("vector   less than 2!");
		return -1;
    }

    LOGPP("range  %f",range);
	
    for(size_t i = 0; i< xyz.size(); i++){
	vector<KeyPoint> vpt;
	for(size_t j = 0; j  < xyz.size() ; j++){
		if(abs (xyz[i].size  - xyz[j].size) < range) {
			vpt.push_back(xyz[j]); 	
			//continue;
		}else{
			//LOGPP("[%d][%d]%f %f",i,j,xyz[i].size  , xyz[j].size);
		}
		
	}
	point.push_back(vpt);
	vpt.clear();
    }
    return 0;
}



void blob_detector_init(){

	SimpleBlobDetector::Params params;    
	params.minThreshold = 40;    
	params.maxThreshold = 160;    
	params.thresholdStep = 5;    

	params.filterByArea = true;
	params.minArea = 100;    
	params.maxArea = 5000;    
	
	params.minConvexity = .05f;    
	params.minInertiaRatio = .05f;    
	
	params.filterByCircularity = true;
	params.minCircularity = 0.85f;  
      gDetector = SimpleBlobDetector::create(params);  

}

int blob_detector_off(Mat &ima,Mat  &ima_gray,PointLine  &aline){

	int picId = 0;
	int id  = aline.id;
	char file[256];
    
	vector<KeyPoint> key_points;  

	gDetector->detect(ima_gray,key_points);  
	LOGPP("key_points  %d",key_points.size());

	if(1){
		Mat output_img;  
		drawKeypoints( ima_gray, key_points, output_img, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );  

		memset(file,0,256);
		sprintf(file,"pic/blob/%d_%d_off_KeyPoints.jpg",id,picId++);
		imwrite(file,output_img);
	}
	sort(key_points.begin(),key_points.end(),KeyPoint_sort_function_y);

    vector <vector<KeyPoint> >lists_points;  
	
	if( statistical_distribution(key_points,lists_points,1) != 0) return -2;

	int select_num = 0;
	int best_id = 0;

	vector<int>  select_id;
	
	for(size_t i = 0; i < lists_points.size();i++){
		for(size_t j = 0; j < lists_points[i].size();j++){
	//		LOGPP("[%d][%d]  %f  ",i,j,lists_points[i][j].size)
	 	}

		if(lists_points[i].size() == 6){
			select_num = lists_points[i].size();
			select_id.push_back(i);
		}
	 }


		float max_size = 0;
	
		for (int  i = 0; i < select_id.size(); i++){

			vector<KeyPoint> good_points = lists_points[select_id[i]];  
			float sum_size = 0;
			for(int j = 0 ; j < good_points.size();j++){
				sum_size +=  good_points[j].size;
			}
			float aver_size = sum_size/6;
			LOGPP("%d  aver_size  %f, %f",select_id[i] ,aver_size,max_size);
			if(aver_size > max_size){
			 	max_size = aver_size;
				best_id = select_id[i];
			}
			
		}
	
	
	select_num = lists_points[best_id].size();
	LOGPP("max_id: %d  max_num  %d",best_id,select_num);

	if(select_num <  6){

		return -2;
	}
	
	vector<KeyPoint> good_points = lists_points[best_id];  

	float  sum_x =0;
	float  sum_y = 0;
	for(size_t i = 0; i < good_points.size();i++){
		sum_x += good_points[i].pt.x;
		sum_y += good_points[i].pt.y;
		LOGPP("[%2d]  %f  %f  %f",i,good_points[i].pt.x,good_points[i].pt.y,good_points[i].size);
	}


	aline.offset_points_center.x = sum_x/good_points.size()+0.5;
	aline.offset_points_center.y = sum_y/good_points.size()+0.5;	
	
	return 0;

	
	sort(good_points.begin(),good_points.end(),KeyPoint_sort_function);


	for(size_t i = 0; i < good_points.size();i++){
	//	LOGPP("[%2d]  %f  %f  %f",i,good_points[i].pt.x,good_points[i].pt.y,good_points[i].size);
	}


	vector<Point2f>  line;
	line.clear();
	
	for(size_t i = 0; i < good_points.size();i++){

		line.push_back(good_points[i].pt);

	}

	Vec4f res_line;  
	fitLine(Mat(line),res_line,CV_DIST_L2,0,0.01,0.01);  

	float x0= res_line[2];  
	float y0= res_line[3];  
	
	float k0 = res_line[0];
	float k1 = res_line[1];
	
	double angle = 90 -fabs(atan(k1/k0)*180/PI );

	Mat ori_point = ima.clone();
	for(size_t i = 0; i < line.size();  i++){
		int y1 = line[i].y;
		float  s1 = (y0-y1)/res_line[1];
		int  x1 = x0-s1*res_line[0];  
		y1 += 150;
		circle(ori_point, Point2f(x1,y1), 4, Scalar(0,255,0));
	}	
	
	memset(file,0,256);
	sprintf(file,"pic/blob/%d_%d_off_linePoint.jpg",id,picId++);
	imwrite(file,ori_point);


}



int blob_detector(Mat &ima,Mat  &ima_gray,PointLine  &aline){

	int picId = 0;
	int id = aline.id;
	float  angle = -360;
	char file[256];
	vector<KeyPoint> key_points;  
	
	gDetector->detect(ima_gray,key_points);  
	LOGPP("key_points  %d",key_points.size());

	if(1){	
		Mat output_img;  
		drawKeypoints( ima, key_points, output_img, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );  
		memset(file,0,256);
		sprintf(file,"pic/blob/%d_%d_KeyPoints.jpg",id,picId++);
		imwrite(file,output_img);
	}
	
	sort(key_points.begin(),key_points.end(),KeyPoint_sort_function_size);

	for(size_t i = 0; i < key_points.size();  i++){
	 }

	vector <vector<KeyPoint> >lists_points;  
	
	statistical_distribution(key_points,lists_points,0.6);

	int max_num = 0;
	int max_id = 0;
	
	for(size_t i = 0; i < lists_points.size();i++){
		for(size_t j = 0; j < lists_points[i].size();j++){
		//	LOGPP("[%d][%d]  %f  ",i,j,lists_points[i][j].size)
	 	}
		if(max_num < lists_points[i].size()){
			max_num = lists_points[i].size();
			max_id = i;
		}
	 }
	
	LOGPP("id: %d  %d",max_id,max_num);

	if(max_num < 4) {
		 aline.angle =  -360;
		return -1;
	}
	
	vector<KeyPoint> good_points = lists_points[max_id];  
	sort(good_points.begin(),good_points.end(),KeyPoint_sort_function);

	vector<BlobKey>  pointk;
	
	for(size_t j = 0; j < good_points.size()-1;j++){
			
			double slope = (good_points[j+1].pt.y-good_points[j].pt.y)/ (good_points[j+1].pt.x-good_points[j].pt.x);
			pointk.push_back(BlobKey(j,slope));
		//	line.push_back(key_points[j].pt);
	 }	

	sort(pointk.begin(),pointk.end(),BlobKey_sort_function);

	for(size_t i = 0; i < pointk.size();i++){
	//	LOGPP("[%2d] slope  %f",i,pointk[i].k);
	}

	double sum_slope = 0;
	for(size_t i = 1; i < pointk.size()-1;i++){
		sum_slope += pointk[i].k;
	}
	double aver_slope = sum_slope/(pointk.size()-2);


	vector<Point2f>  line;
	line.clear();

	vector<KeyPoint> good_points2 ;
	double  max_y = 0;
	for(size_t i = 0; i < pointk.size();i++){
		double  dis = fabs( pointk[i].k - aver_slope);
		if(dis <  0.15){
	//		LOGPP("[%2d] slope  %f(%f  %f)",i,dis,good_points[pointk[i].id].pt.x,good_points[pointk[i].id].pt.y);
			line.push_back(good_points[pointk[i].id].pt);
			good_points2.push_back(good_points[pointk[i].id]);

			if(max_y < good_points[pointk[i].id].pt.y )  max_y = good_points[pointk[i].id].pt.y;
		}
	}

	LOGPP("max_y:  %f",max_y);
	line.push_back(good_points[pointk[pointk.size()-1].id+1].pt);
	good_points2.push_back(good_points[pointk[pointk.size()-1].id+1]);
	aline.keypoints.swap(good_points2);
	aline.keyPoint_max_y = (int)(max_y+10);


	for(size_t i = 0; i < line.size();i++){
//		LOGPP("line[%d] %f  %f",i,line[i].x,line[i].y);
	}

	LOGPP("line  %d",line.size());
	
	Vec4f res_line;  
	fitLine(Mat(line),res_line,CV_DIST_L2,0,0.01,0.01);  

	float x0= res_line[2];  
	float y0= res_line[3];  
	
	float k0 = res_line[0];
	float k1 = res_line[1];
	
	angle = atan(k1/k0)*180/PI;

	 aline.angle = angle;

	if(0){
		Mat ori_point = ima.clone();
		for(size_t j = 0; j < line.size();j++){
			int x1 = line[j].x;
			float s1 = (x0-x1)/k0;
			int  y1   =  y0 - s1*k1 ;	
		//	LOGPP("%d %d",x1,y1);
			circle(ori_point, Point2f(x1,y1), 4, Scalar(0,255,0));
		}	
		
		memset(file,0,256);
		sprintf(file,"pic/blob/%d_%d_linePoint.jpg",id,picId++);
		imwrite(file,ori_point);
	}

	return 0;
}
