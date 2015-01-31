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
  params.minArea = 100;
  params.maxArea = 50000;
  params.filterByCircularity = false;
  params.minCircularity = 0.8f;
  params.maxCircularity = std::numeric_limits<float>::max();
  params.filterByInertia = false;
  params.minInertiaRatio = 0.1f;
  params.maxInertiaRatio = std::numeric_limits<float>::max();
  params.filterByConvexity = false;
  params.minConvexity = 0.95f;
  params.maxConvexity = std::numeric_limits<float>::max();
  bool pause = true;
  // set up and create the detector using the parameters
  cv::Ptr<cv::FeatureDetector> circle_detector = new cv::CircleDetector(params);
  circle_detector->create("Circle");

  std::vector<Point3d> circle_9;
  std::vector<Point3d> circle_24;
  std::vector<Point3d> circle_38;
  std::vector<Point3d> circle_53;

  namedWindow("sub_image",CV_WINDOW_AUTOSIZE);

  for(int i=1;i<51;i++){
    char image_name[15];
    sprintf(image_name,"v%02d.tiff",i);
    printf("image_name = %s\n",image_name);
    Mat image = cv::imread(image_name,CV_LOAD_IMAGE_COLOR);
    Mat sub_image = image(Range(500,620),Range(760,895));
    
    // detect!
    vector<cv::KeyPoint> keypoints;
    circle_detector->detect(sub_image, keypoints);
    
    printf("keypoints size = %d\n",(int)keypoints.size());
    //    for(int i=0;i<(int)keypoints.size();i++){
    //      keypoints[i].size *=2;
    //    }

    vector<cv::KeyPoint> tempkp;
    for(int i=0;i<(int)keypoints.size();i++){
      tempkp.push_back(keypoints[i]);
      tempkp[i].size *=2.0;
    }
    Mat kimage;
    cv::drawKeypoints(sub_image,tempkp,kimage,Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("sub_image",kimage);
    waitKey(1);

    char c='q';
    while(c != 'n' && pause){
      scanf("%c",&c);
      if(c=='q') pause = false;
    }

    // extract the x y coordinates of the keypoints: 
    for (int i=0; i<keypoints.size(); i++){
      float X=keypoints[i].pt.x; 
      float Y=keypoints[i].pt.y;
      float Z=keypoints[i].size;
      Point3d P;
      P.x = X;
      P.y = Y;
      P.z = Z;
      printf("Circles at %f %f size = %f \n",Y,X,keypoints[i].size);
      if(keypoints[i].size>8.0 && keypoints[i].size<12.0 && fabs(Y-59.6)<3.0 && fabs(X-64)<4.0){
        circle_9.push_back(P);
      }
      if(keypoints[i].size>22.0 && keypoints[i].size<26.0 && fabs(Y-59.6)<3.0 && fabs(X-64)<4.0){
        circle_24.push_back(P);
      }
      if(keypoints[i].size>36.0 && keypoints[i].size<40.0 && fabs(Y-59.6)<3.0 && fabs(X-64)<6.0){
        circle_38.push_back(P);
      }
      if(keypoints[i].size>51.0 && keypoints[i].size<55.0 && fabs(Y-59.6)<3.0 && fabs(X-66)<2.0){
        circle_53.push_back(P);
      }
    }
    
  }
  printf("size of circle_9 = %d\n",circle_9.size());
  printf("size of circle_24 = %d\n",circle_24.size());
  printf("size of circle_38 = %d\n",circle_38.size());
  printf("size of circle_53 = %d\n",circle_53.size());

  FILE *fp = fopen("bullseye_anal.m","w");
  fprintf(fp,"figure(1);\n");
  fprintf(fp,"hold off;\n");
  fprintf(fp,"circle_9 = [");
  for(int i=0;i< (int) circle_9.size(); i++){
    fprintf(fp,"%lf %lf %lf;\n",circle_9[i].x,circle_9[i].y,circle_9[i].z);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"plot3(circle_9(:,1),circle_9(:,2),circle_9(:,3),'r+')\n");
  fprintf(fp,"hold on;\n");

  fprintf(fp,"circle_24 = [");
  for(int i=0;i< (int) circle_24.size(); i++){
    fprintf(fp,"%lf %lf %lf;\n",circle_24[i].x,circle_24[i].y,circle_24[i].z);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"plot3(circle_24(:,1),circle_24(:,2),circle_24(:,3),'r+')\n");

  fprintf(fp,"circle_38 = [");
  for(int i=0;i< (int) circle_38.size(); i++){
    fprintf(fp,"%lf %lf %lf;\n",circle_38[i].x,circle_38[i].y,circle_38[i].z);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"plot3(circle_38(:,1),circle_38(:,2),circle_38(:,3),'r+')\n");

  fprintf(fp,"circle_53 = [");
  for(int i=0;i< (int) circle_53.size(); i++){
    fprintf(fp,"%lf %lf %lf;\n",circle_53[i].x,circle_53[i].y,circle_53[i].z);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"plot3(circle_53(:,1),circle_53(:,2),circle_53(:,3),'r+')\n");
  fprintf(fp,"xlabel('X-pixel')\n");
  fprintf(fp,"ylabel('Y-pixel')\n");
  fprintf(fp,"zlabel('size')\n");



  
  
  fprintf(fp,"sizes = [ mean(circle_9(:,3)) mean(circle_24(:,3)) mean(circle_38(:,3)) mean(circle_53(:,3))];\n");
  // NOTE since error is sum of squared y error. We assume similar error in x
  // therefore multiply sigmas by sqrt(2)
  fprintf(fp,"sigma_x = [ std(circle_9(:,1)) std(circle_24(:,1)) std(circle_38(:,1)) std(circle_53(:,1))];\n");
  fprintf(fp,"sigma_y = [ std(circle_9(:,2)) std(circle_24(:,2)) std(circle_38(:,2)) std(circle_53(:,2))];\n");
  fprintf(fp,"sigma_s = [ std(circle_9(:,3)) std(circle_24(:,3)) std(circle_38(:,3)) std(circle_53(:,3))];\n");

  fprintf(fp,"figure(2);\n");
  fprintf(fp,"hold off;\n");
  fprintf(fp,"plot(sizes,sigma_x,'r');\n");
  fprintf(fp,"hold on;\n");
  fprintf(fp,"plot(sizes,sigma_y,'g');\n");
  fprintf(fp,"title('Standard deviation in x(red) and y(green)');\n");
  fprintf(fp,"xlabel('Radius of circle');\n");
  fprintf(fp,"ylabel('Standard Deviation (pixels)');\n");

  fprintf(fp,"figure(3);\n");
  fprintf(fp,"hold off;\n");
  fprintf(fp,"plot(sizes,sigma_s,'r');\n");
  fprintf(fp,"title('Standard deviation of cirlce radius');\n");
  fprintf(fp,"xlabel('Radius of circle');\n");
  fprintf(fp,"ylabel('Standard Deviation (pixels)');\n");

  fclose(fp);
}

