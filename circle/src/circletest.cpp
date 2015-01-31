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
  double x = 110.2;
  double y = 120.5;
  double r = 20.0;
  double s = 8.0;
  if(argc == 5){
    sscanf(argv[1],"%lf",&x);
    sscanf(argv[2],"%lf",&y);
    sscanf(argv[3],"%lf",&r);
    sscanf(argv[4],"%lf",&s);
  }
  printf("using (x,y) = %6.3lf %6.3lf radius = %6.3lf sigma = %6.3lf\n",x,y,r,s);
  Mat image = create_circle_image(400, 400,x,y,r,1.0,0.0,s);
  //  Mat image2 = cv::imread("frame0006.jpg",CV_LOAD_IMAGE_COLOR);


  //  namedWindow("mycircle",CV_WINDOW_AUTOSIZE);
  //  imshow("mycircle",image);
  //  waitKey(30);

  Mat image2;
  image.convertTo(image2,16,128);
  cv::imwrite("test10.jpg",image2);
  //  Mat image2 = cv::imread("test10.jpg",CV_LOAD_IMAGE_COLOR);


  CircleDetector::Params params;
  params.thresholdStep = 10;
  params.minThreshold = 50;
  params.maxThreshold = 220;
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

  SimpleBlobDetector::Params sparams;
  sparams.minDistBetweenBlobs = 50.0f;
  sparams.filterByInertia = false;
  sparams.filterByConvexity = false;
  sparams.filterByColor = false;
  sparams.filterByCircularity = false;
  sparams.filterByArea = false;
  sparams.minArea = 20.0f;
  sparams.maxArea = 500.0f;


  // set up and create the detector using the parameters
  cv::Ptr<cv::FeatureDetector> circle_detector = new cv::CircleDetector(params);
  cv::Ptr<cv::FeatureDetector> simple_detector = new cv::SimpleBlobDetector(sparams);
  circle_detector->create("Circle");
  simple_detector->create("SimpleBlob");
  
  // detect!
  vector<cv::KeyPoint> keypoints;
  vector<cv::KeyPoint> skeypoints;
  circle_detector->detect(image2, keypoints);
  simple_detector->detect(image2, skeypoints);

  printf("keypoints size = %d\n",(int)keypoints.size());
  
  // extract the x y coordinates of the keypoints: 
  for (int i=0; i<keypoints.size(); i++){
    float X=keypoints[i].pt.x; 
    float Y=keypoints[i].pt.y;
    printf("Circles at %f %f size = %f \n",Y,X,keypoints[i].size);
  }

  // extract the x y coordinates of the keypoints: 
  for (int i=0; i<skeypoints.size(); i++){
    float X=skeypoints[i].pt.x; 
    float Y=skeypoints[i].pt.y;
    printf("Blobs   at %f %f size = %f \n",Y,X,skeypoints[i].size);
  }

  Mat bullseye = cv::imread("binarybullseye2.tiff",CV_LOAD_IMAGE_COLOR);
  if(!bullseye.data){
    printf("Could not read image\n");
    exit(1);
  }
  
  vector<cv::KeyPoint> bkeypoints;
  vector<cv::KeyPoint> bskeypoints;
  circle_detector->detect(bullseye, bkeypoints);
  simple_detector->detect(bullseye, bskeypoints);

  printf("bullseye keypoints size = %d\n",(int)bkeypoints.size());
  printf("bullseye skeypoints size = %d\n",(int)bskeypoints.size());
  
  // extract the x y coordinates of the keypoints: 
  for (int i=0; i<bkeypoints.size(); i++){
    float X=bkeypoints[i].pt.x; 
    float Y=bkeypoints[i].pt.y;
    printf("Circles at %f %f size = %f \n",Y,X,bkeypoints[i].size);
  }

  // extract the x y coordinates of the keypoints: 
  for (int i=0; i<bskeypoints.size(); i++){
    float X=bskeypoints[i].pt.x; 
    float Y=bskeypoints[i].pt.y;
    printf("Blobs   at %f %f size = %f \n",Y,X,bskeypoints[i].size);
  }
  Mat kpt_image;
  cv::drawKeypoints(bullseye,bkeypoints,kpt_image,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  namedWindow("bullseye",CV_WINDOW_AUTOSIZE);
  imshow("bullseye",kpt_image);
  waitKey(30);

  Mat skpt_image;
  cv::drawKeypoints(bullseye,bskeypoints,skpt_image,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
  namedWindow("bullseye2",CV_WINDOW_AUTOSIZE);
  imshow("bullseye2",skpt_image);
  waitKey(30);

  // now do a group of these to show the average performace
  double cx_error=0.0;
  double cy_error=0.0;
  double sx_error=0.0;
  double sy_error=0.0;
  int num_samps;
  for(double dx=0.0; dx<1.0;dx = dx+.01){
    for(double dy=0.0; dy<1.0;dy = dy+.01){
      image = create_circle_image(400, 400,x+dx,y+dy,r,1.0,0.0,s);
      image.convertTo(image2,16,128);
      circle_detector->detect(image2, keypoints);
      simple_detector->detect(image2, skeypoints);
      cx_error += fabs(keypoints[0].pt.y-(x+dx));
      cy_error += fabs(keypoints[0].pt.x-(y+dy));
      sx_error += fabs(skeypoints[0].pt.y-(x+dx));
      sy_error += fabs(skeypoints[0].pt.x-(y+dy));
      num_samps++;
      printf("E1 = %lf  E2 = %lf\n",fabs(keypoints[0].pt.y-(x+dx)),fabs(skeypoints[0].pt.y-(x+dx)));
    }
  }
  cx_error = cx_error/num_samps;
  cy_error = cy_error/num_samps;
  sx_error = sx_error/num_samps;
  sy_error = sy_error/num_samps;
  printf("circle error = %9.6lf %9.6lf\n",cx_error,cy_error);
  printf("blob   error = %9.6lf %9.6lf\n",sx_error,sy_error);
  while(1);

}
