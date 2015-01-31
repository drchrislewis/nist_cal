#include <stdlib.h>
#include <ostream>
#include <stdio.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <vector>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <iostream>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/generator_iterator.hpp>
#include "opencv2/core/core.hpp"
#include <circle/circledetector.hpp>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;
using std::ifstream;
using std::string;
using std::vector;
using cv::Mat;
using cv::CircleDetector;


Mat create_circle_image(int m, int n, double x, double y, double r, float i_color, float e_color, double s )
{
  // check bounds

  int num_interior = 0;
  int num_exterior = 0;
  int num_transition = 0;
  Mat image(m,n,CV_32FC1); 
  if(x-r<0 || y-r<0 || x+r>n || y+r>m) return(image);
  for(int i=0;i<n;i++){
    for(int j=0;j<m;j++){
      float color;
      double d = sqrt((i-x)*(i-x)+(j-y)*(j-y));
      if(d > r+s/2){
	color = e_color;	// exterior point
	num_exterior++;
      }
      else if(d < r-s/2){
	color = i_color;	// interior point
	num_interior++;
      }
      else{
	// transition point between   r - s/2 < d < r+s/2
	double alpha = ( d - (r-s/2))/s; // alpha = 0: interior alpha=1.0 exterior
    	color =  (1.0-alpha)*i_color + alpha*e_color;
	num_transition++;
      }
      image.at<float>(i,j) = color;
      //      image.at<Vec3b>(i,j)[1] = color;
      //      image.at<Vec3b>(i,j)[2] = color;
    }
  }
  //  printf("num_interior = %d num_exterior = %d num_transition = %d\n",num_interior,num_exterior,num_transition);
  return(image);
}

int main(int argc, char** argv)
{
  CircleDetector::Params params;
  params.thresholdStep = 10;
  params.minThreshold = 240;
  params.maxThreshold = 251;
  params.minRepeatability = 2;
  params.minDistBetweenCircles = 2.0;
  params.minRadiusDiff = 10;
  params.filterByColor = false;
  params.circleColor = 0;
  params.filterByArea = false;
  params.minArea = 25;
  params.maxArea = 5000;
  params.filterByCircularity = false;
  params.minCircularity = 0.8f;
  params.maxCircularity = std::numeric_limits<float>::max();
  params.filterByInertia = false;
  params.minInertiaRatio = 0.1f;
  params.maxInertiaRatio = std::numeric_limits<float>::max();
  params.filterByConvexity = false;
  params.minConvexity = 0.95f;
  params.maxConvexity = std::numeric_limits<float>::max();

  // set up and create the detector using the parameters
  cv::Ptr<cv::FeatureDetector> circle_detector = new cv::CircleDetector(params);
  circle_detector->create("Circle");

  std::vector<Point2d> circle_22;
  std::vector<Point2d> circle_41;
  std::vector<Point2d> circle_62;
  std::vector<Point2d> circle_78;
  std::vector<Point2d> circle_100;
  std::vector<Point2d> circle_116;
  std::vector<Point2d> circle_136;
  namedWindow("sub_image",CV_WINDOW_AUTOSIZE);
  for(int i=0;i<101;i++){
    char image_name[15];
    sprintf(image_name,"V%03d.tiff",i*5);
    printf("image_name = %s\n",image_name);
    Mat image = cv::imread(image_name,CV_LOAD_IMAGE_COLOR);
    Mat sub_image = image(Range(325,620),Range(640,1010));
    //    imshow("sub_image",sub_image);
    //    waitKey(30);
    
    // detect!
    vector<cv::KeyPoint> keypoints;
    circle_detector->detect(sub_image, keypoints);
    
    printf("keypoints size = %d\n",(int)keypoints.size());
    
    vector<cv::KeyPoint> tempkp;
    for(int i=0;i<(int)keypoints.size();i++){
      tempkp.push_back(keypoints[i]);
      tempkp[i].size *=2.0;
    }
    Mat kimage;
    cv::drawKeypoints(sub_image,tempkp,kimage,Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("sub_image",kimage);
    waitKey(1);

    // extract the x y coordinates of the keypoints: 
    for (int i=0; i<keypoints.size(); i++){
      float X=keypoints[i].pt.x; 
      float Y=keypoints[i].pt.y;
      printf("Circles at %f %f size = %f \n",Y,X,keypoints[i].size);
      if(keypoints[i].size>20.0 && keypoints[i].size<25.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_22.push_back(P);
      }
      if(keypoints[i].size>38.0 && keypoints[i].size<43.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_41.push_back(P);
      }
      if(keypoints[i].size>60.0 && keypoints[i].size<64.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_62.push_back(P);
      }
      if(keypoints[i].size>75.0 && keypoints[i].size<80.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_78.push_back(P);
      }
      if(keypoints[i].size>98.0 && keypoints[i].size<103.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_100.push_back(P);
      }
      if(keypoints[i].size>114.0 && keypoints[i].size<118.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_116.push_back(P);
      }
      if(keypoints[i].size>132.0 && keypoints[i].size<140.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_136.push_back(P);
      }
    }
    
  }
  printf("size of circle_22 = %d\n",circle_22.size());
  printf("size of circle_41 = %d\n",circle_41.size());
  printf("size of circle_62 = %d\n",circle_62.size());
  printf("size of circle_78 = %d\n",circle_78.size());
  printf("size of circle_100 = %d\n",circle_100.size());
  printf("size of circle_116 = %d\n",circle_116.size());
  printf("size of circle_136 = %d\n",circle_136.size());


  FILE *fp = fopen("bullseye_anal.m","w");
  fprintf(fp,"circle_22 = [");
  for(int i=0;i< (int) circle_22.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_22[i].x,circle_22[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"circle_41 = [");
  for(int i=0;i< (int) circle_41.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_41[i].x,circle_41[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"circle_62 = [");
  for(int i=0;i< (int) circle_62.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_62[i].x,circle_62[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"circle_78 = [");
  for(int i=0;i< (int) circle_78.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_78[i].x,circle_78[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"circle_100 = [");
  for(int i=0;i< (int) circle_100.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_100[i].x,circle_100[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"circle_116 = [");
  for(int i=0;i< (int) circle_116.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_116[i].x,circle_116[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"circle_136 = [");
  for(int i=0;i< (int) circle_136.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_136[i].x,circle_136[i].y);
  }
  fprintf(fp,"];\n");

  // do analysis 22
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_22.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_22[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_22.size(); i++){
    fprintf(fp,"%lf;\n",circle_22[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_22 = std(E)\n");



  // do analysis 41
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_41.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_41[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_41.size(); i++){
    fprintf(fp,"%lf;\n",circle_41[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_41 = std(E)\n");
  
  // do analysis 62
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_62.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_62[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_62.size(); i++){
    fprintf(fp,"%lf;\n",circle_62[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_62 = std(E)\n");


  // do analysis 78
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_78.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_78[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_78.size(); i++){
    fprintf(fp,"%lf;\n",circle_78[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_78 = std(E)\n");


  // do analysis 100
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_100.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_100[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_100.size(); i++){
    fprintf(fp,"%lf;\n",circle_100[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_100 = std(E)\n");


  // do analysis 116
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_116.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_116[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_116.size(); i++){
    fprintf(fp,"%lf;\n",circle_116[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_116 = std(E)\n");


  // do analysis 136
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_136.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_136[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_136.size(); i++){
    fprintf(fp,"%lf;\n",circle_136[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_136 = std(E)\n");

  fprintf(fp,"sizes = [ 22 41 62 78 100 116 136];\n");
  // NOTE since error is sum of squared y error. We assume similar error in x
  // therefore multiply sigmas by sqrt(2)
  fprintf(fp,"sigmas = [sigma_22 sigma_41 sigma_62 sigma_78 sigma_100 sigma_116 sigma_136]*sqrt(2);\n");
  fprintf(fp,"figure(1);\n");
  fprintf(fp,"hold off;\n");
  fprintf(fp,"plot(circle_22(:,1),circle_22(:,2),'r');\n");
  fprintf(fp,"hold on;\n");
  fprintf(fp,"plot(circle_41(:,1),circle_41(:,2),'g');\n");
  fprintf(fp,"plot(circle_62(:,1),circle_62(:,2),'b');\n");
  fprintf(fp,"plot(circle_78(:,1),circle_78(:,2),'c');\n");
  fprintf(fp,"plot(circle_100(:,1),circle_100(:,2),'y');\n");
  fprintf(fp,"plot(circle_116(:,1),circle_116(:,2),'k');\n");
  fprintf(fp,"plot(circle_136(:,1),circle_136(:,2),'m');\n");
  fprintf(fp,"title('Trajectories of Centers of Different Sizes');\n");
  fprintf(fp,"xlabel('Pixel X');\n");
  fprintf(fp,"ylabel('Pixel Y');\n");

  fprintf(fp,"figure(2);\n");
  fprintf(fp,"hold off;\n");
  fprintf(fp,"plot(sizes,sigmas,'r*');\n");
  fprintf(fp,"hold on;\n");
  fprintf(fp,"plot(sizes,sigmas,'b');\n");
  fprintf(fp,"title('Standard Deviation of Error');\n");
  fprintf(fp,"xlabel('Target Size (pixels)');\n");
  fprintf(fp,"ylabel('Standard Deviation of Error (pixels)');\n");
  fclose(fp);
}

