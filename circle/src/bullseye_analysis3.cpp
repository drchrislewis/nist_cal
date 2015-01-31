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
  params.thresholdStep = 15;
  params.minThreshold = 220;
  params.maxThreshold = 236;
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

  // set up and create the detector using the parameters
  cv::Ptr<cv::FeatureDetector> circle_detector = new cv::CircleDetector(params);
  circle_detector->create("Circle");

  std::vector<Point2d> circle_26;
  std::vector<Point2d> circle_65;
  std::vector<Point2d> circle_105;
  std::vector<Point2d> circle_145;
  //  std::vector<Point2d> circle_37;
  //  std::vector<Point2d> circle_52;
  std::vector<Point3d> ss;
  //  std::vector<Point2d> circle_136;
  namedWindow("sub_image",CV_WINDOW_AUTOSIZE);

  for(int i=0;i<101;i++){
    char image_name[15];
    sprintf(image_name,"v%03d.tiff",i*5);
    printf("image_name = %s\n",image_name);
    Mat image = cv::imread(image_name,CV_LOAD_IMAGE_COLOR);
    Mat sub_image = image(Range(450,620),Range(730,890));
    
    // detect!
    vector<cv::KeyPoint> keypoints;
    circle_detector->detect(sub_image, keypoints);
    
    printf("keypoints size = %d\n",(int)keypoints.size());
    //    for(int i=0;i<(int)keypoints.size();i++){
    //      keypoints[i].size *=2;
    //    }

    Mat kimage;
    cv::drawKeypoints(sub_image,keypoints,kimage,Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("sub_image",kimage);
    waitKey(1);
    /*
    char c='q';
    while(c != 'n'){
      scanf("%c",&c);
    }
    */
    // extract the x y coordinates of the keypoints: 
    for (int i=0; i<keypoints.size(); i++){
      float X=keypoints[i].pt.x; 
      float Y=keypoints[i].pt.y;
      printf("Circles at %f %f size = %f \n",Y,X,keypoints[i].size);
      if(keypoints[i].size>25.0 && keypoints[i].size<27.0 ){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_26.push_back(P);
	Point3d P2;
	P2.x = X;
	P2.y = Y;
	P2.z = keypoints[i].size;
	ss.push_back(P2);
      }
      if(keypoints[i].size>64.0 && keypoints[i].size<67.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_65.push_back(P);
      }
      if(keypoints[i].size>104.0 && keypoints[i].size<107){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_105.push_back(P);
      }
      if(keypoints[i].size>144.0 && keypoints[i].size<147.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_145.push_back(P);
      }
      /*
      if(keypoints[i].size>35.0 && keypoints[i].size<40.0 && fabs(Y-59.6)<3.0 && fabs(X-66)<2.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_37.push_back(P);
      }
      if(keypoints[i].size>51.5 && keypoints[i].size<54 && fabs(Y-59.6)<0.5 && fabs(X-66)<2.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_52.push_back(P);
      }
      if(keypoints[i].size>132.0 && keypoints[i].size<140.0){
	Point2d P;
        P.x = X;
	P.y = Y;
        circle_136.push_back(P);
      }
      */
    }
    
  }
  printf("size of circle_26 = %d\n",circle_26.size());
  printf("size of circle_65 = %d\n",circle_65.size());
  printf("size of circle_105 = %d\n",circle_105.size());
  printf("size of circle_145 = %d\n",circle_145.size());
  /*
  printf("size of circle_37 = %d\n",circle_37.size());
  printf("size of circle_52 = %d\n",circle_52.size());
  */

  FILE *fp2 = fopen("junk.m","w");
  fprintf(fp2,"ss = [\n");
  for(int i=0;i< (int) ss.size(); i++){
    fprintf(fp2,"%lf %lf %lf;\n",ss[i].x,ss[i].y,ss[i].z);
  }
  fprintf(fp2,"];\n");
  fprintf(fp2,"plot3(ss(:,1),ss(:,2),ss(:,3),'r+')\n");
  fprintf(fp2,"xlabel('X-pixel')\n");
  fprintf(fp2,"ylabel('Y-pixel')\n");
  fprintf(fp2,"zlabel('size')\n");
  fclose(fp2);

  FILE *fp = fopen("bullseye_anal.m","w");
  fprintf(fp,"circle_26 = [");
  for(int i=0;i< (int) circle_26.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_26[i].x,circle_26[i].y);
  }
  fprintf(fp,"];\n");

  fprintf(fp,"circle_65 = [");
  for(int i=0;i< (int) circle_65.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_65[i].x,circle_65[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"circle_105 = [");
  for(int i=0;i< (int) circle_105.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_105[i].x,circle_105[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"circle_145 = [");
  for(int i=0;i< (int) circle_145.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_145[i].x,circle_145[i].y);
  }
  fprintf(fp,"];\n");
  /*
  fprintf(fp,"circle_37 = [");
  for(int i=0;i< (int) circle_37.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_37[i].x,circle_37[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"circle_52 = [");
  for(int i=0;i< (int) circle_52.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_52[i].x,circle_52[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"circle_136 = [");
  for(int i=0;i< (int) circle_136.size(); i++){
    fprintf(fp,"%lf %lf;\n",circle_136[i].x,circle_136[i].y);
  }
  fprintf(fp,"];\n");
  */
  // do analysis 9
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_26.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_26[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_26.size(); i++){
    fprintf(fp,"%lf;\n",circle_26[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_26 = std(E)\n");



  // do analysis 24
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_65.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_65[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_65.size(); i++){
    fprintf(fp,"%lf;\n",circle_65[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_65 = std(E)\n");
  
  // do analysis 38
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_105.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_105[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_105.size(); i++){
    fprintf(fp,"%lf;\n",circle_105[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_105 = std(E)\n");


  // do analysis 53
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_145.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_145[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_145.size(); i++){
    fprintf(fp,"%lf;\n",circle_145[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_145 = std(E)\n");

  /*
  // do analysis 37
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_37.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_37[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_37.size(); i++){
    fprintf(fp,"%lf;\n",circle_37[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_37 = std(E)\n");


  // do analysis 52
  fprintf(fp, "A = [");
  for(int i=0;i<(int)circle_52.size(); i++){
    fprintf(fp,"%lf 1.0;\n",circle_52[i].x);
  }
  fprintf(fp,"];\n");
  fprintf(fp, "B = [");
  for(int i=0;i<(int)circle_52.size(); i++){
    fprintf(fp,"%lf;\n",circle_52[i].y);
  }
  fprintf(fp,"];\n");
  fprintf(fp,"V = pinv(A)*B;\n");
  fprintf(fp,"E = A*V-B;\n");
  fprintf(fp,"sigma_52 = std(E)\n");


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
  */
  fprintf(fp,"sizes = [ 26 65 105 145];\n");
  // NOTE since error is sum of squared y error. We assume similar error in x
  // therefore multiply sigmas by sqrt(2)
  fprintf(fp,"sigmas = [sigma_26 sigma_65 sigma_105 sigma_145]*sqrt(2);\n");
  fprintf(fp,"figure(1);\n");
  fprintf(fp,"hold off;\n");
  fprintf(fp,"plot(circle_26(:,1),circle_26(:,2),'r');\n");
  fprintf(fp,"hold on;\n");
  fprintf(fp,"plot(circle_65(:,1),circle_65(:,2),'g');\n");
  fprintf(fp,"plot(circle_105(:,1),circle_105(:,2),'b');\n");
  fprintf(fp,"plot(circle_145(:,1),circle_145(:,2),'c');\n");
  //  fprintf(fp,"plot(circle_37(:,1),circle_37(:,2),'y');\n");
  //  fprintf(fp,"plot(circle_52(:,1),circle_52(:,2),'k');\n");
  //  fprintf(fp,"plot(circle_136(:,1),circle_136(:,2),'m');\n");
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

