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
  params.maxThreshold = 255;
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

  std::vector<Point2d> circle_15;
  //  std::vector<Point2d> circle_23;
  std::vector<Point2d> circle_35;
  //  std::vector<Point2d> circle_45;
  std::vector<Point2d> circle_57;
  //  std::vector<Point2d> circle_67;
  std::vector<Point2d> circle_79;
  namedWindow("sub_image",CV_WINDOW_AUTOSIZE);
  for(int i=0;i<101;i++){
    char image_name[15];
    sprintf(image_name,"Vert_0.%03d.tiff",i*5);
    printf("image_name = %s\n",image_name);
    Mat image = cv::imread(image_name,CV_LOAD_IMAGE_COLOR);
    Mat sub_image = image(Range(440,630),Range(710,940));
    //    imshow("sub_image",sub_image);
    //    waitKey(30);
    
    // detect!
    vector<cv::KeyPoint> keypoints;
    circle_detector->detect(sub_image, keypoints);
   

    vector<cv::KeyPoint> tempkp;
    for(int i=0;i<(int)keypoints.size();i++){
      tempkp.push_back(keypoints[i]);
      tempkp[i].size *=2.0;
    }
    Mat kimage;
    cv::drawKeypoints(sub_image,tempkp,kimage,Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("sub_image",kimage);
    waitKey(1);
    /*
    char c='q';
    while(c != 'n'){
      scanf("%c",&c);
    }
    */
    
    printf("keypoints size = %d\n",(int)keypoints.size());
    
    // extract the x y coordinates of the keypoints: 
    for (int i=0; i<keypoints.size(); i++){
      float X=keypoints[i].pt.x; 
      float Y=keypoints[i].pt.y;
      printf("Circles at %f %f size = %f \n",Y,X,keypoints[i].size);
      if(keypoints[i].size>13.0 && keypoints[i].size<17.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_15.push_back(P);
      }
      /*
      if(keypoints[i].size>20.0 && keypoints[i].size<25.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_23.push_back(P);
      }
      */
      if(keypoints[i].size>34.0 && keypoints[i].size<38.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_35.push_back(P);
      }
      /*
      if(keypoints[i].size>42.0 && keypoints[i].size<47.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_45.push_back(P);
      }
      */
      if(keypoints[i].size>55.0 && keypoints[i].size<60.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_57.push_back(P);
      }
      /*
      if(keypoints[i].size>65.0 && keypoints[i].size<70.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_67.push_back(P);
      }
      */
      if(keypoints[i].size>76.0 && keypoints[i].size<81.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_79.push_back(P);
      }
    }
    
  }
  printf("size of circle_15 = %d\n",circle_15.size());
  //  printf("size of circle_23 = %d\n",circle_23.size());
  printf("size of circle_35 = %d\n",circle_35.size());
  //  printf("size of circle_45 = %d\n",circle_45.size());
  printf("size of circle_57 = %d\n",circle_57.size());
  //  printf("size of circle_67 = %d\n",circle_67.size());
  printf("size of circle_79 = %d\n",circle_79.size());

  FILE *fp = fopen("bullseye_anal.m","w");
  fprintf(fp,"circle_15 = [");
  for(int i=0;i< (int) circle_15.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_15[i].x,circle_15[i].y);
  }
  fprintf(fp,"];\n");
  /*
  fprintf(fp,"circle_23 = [");
  for(int i=0;i< (int) circle_23.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_23[i].x,circle_23[i].y);
  }
  fprintf(fp,"];\n");
  */
  fprintf(fp,"circle_35 = [");
  for(int i=0;i< (int) circle_35.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_35[i].x,circle_35[i].y);
  }
  fprintf(fp,"];\n");
  /*
  fprintf(fp,"circle_45 = [");
  for(int i=0;i< (int) circle_45.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_45[i].x,circle_45[i].y);
  }
  fprintf(fp,"];\n");
  */
  fprintf(fp,"circle_57 = [");
  for(int i=0;i< (int) circle_57.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_57[i].x,circle_57[i].y);
  }
  fprintf(fp,"];\n");
  /*
  fprintf(fp,"circle_67 = [");
  for(int i=0;i< (int) circle_67.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_67[i].x,circle_67[i].y);
  }
  fprintf(fp,"];\n");
  */
  fprintf(fp,"circle_79 = [");
  for(int i=0;i< (int) circle_79.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_79[i].x,circle_79[i].y);
  }
  fprintf(fp,"];\n");

  // do analysis 15
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_15.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_15[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_15.size(); i++){
    fprintf(fp,"%lf;\n",circle_15[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_15 = std(E)\n");


  /*
  // do analysis 23
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_23.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_23[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_23.size(); i++){
    fprintf(fp,"%lf;\n",circle_23[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_23 = std(E)\n");
  */
  
  // do analysis 35
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_35.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_35[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_35.size(); i++){
    fprintf(fp,"%lf;\n",circle_35[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_35 = std(E)\n");

  /*
  // do analysis 45
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_45.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_45[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_45.size(); i++){
    fprintf(fp,"%lf;\n",circle_45[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_45 = std(E)\n");
  */

  // do analysis 57
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_57.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_57[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_57.size(); i++){
    fprintf(fp,"%lf;\n",circle_57[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_57 = std(E)\n");

  /*
  // do analysis 67
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_67.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_67[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_67.size(); i++){
    fprintf(fp,"%lf;\n",circle_67[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_67 = std(E)\n");
  */

  // do analysis 79
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_79.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_79[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_79.size(); i++){
    fprintf(fp,"%lf;\n",circle_79[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_79 = std(E)\n");

  fprintf(fp,"sizes = [ 15 35 57 79];\n");
  // NOTE since error is sum of squared y error. We assume similar error in x
  // therefore multiply sigmas by sqrt(2)
  fprintf(fp,"sigmas = [sigma_15 sigma_35 sigma_57 sigma_79]*sqrt(2);\n");
  fprintf(fp,"figure(1);\n");
  fprintf(fp,"hold off;\n");
  fprintf(fp,"plot(circle_15(:,1),circle_15(:,2),'r');\n");
  fprintf(fp,"hold on;\n");
  //fprintf(fp,"plot(circle_23(:,1),circle_23(:,2),'g');\n");
  fprintf(fp,"plot(circle_35(:,1),circle_35(:,2),'b');\n");
  //  fprintf(fp,"plot(circle_45(:,1),circle_45(:,2),'c');\n");
  fprintf(fp,"plot(circle_57(:,1),circle_57(:,2),'y');\n");
  //  fprintf(fp,"plot(circle_67(:,1),circle_67(:,2),'k');\n");
  fprintf(fp,"plot(circle_79(:,1),circle_79(:,2),'m');\n");
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

