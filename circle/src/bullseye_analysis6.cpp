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
  bool pause = true;
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
  FILE *fp = fopen("Circledata.m","w");
  fprintf(fp,"Circles = [");

  // set up and create the detector using the parameters
  cv::Ptr<cv::FeatureDetector> circle_detector = new cv::CircleDetector(params);
  circle_detector->create("Circle");

  std::vector< vector<Point3d> > circles;
  std::vector<double> radii;

  // define the regions of interest by looking at first image in each directory
  // the image should not change much
  cv::Mat rois(20,4,CV_32S);
  rois.at<int>(0,0)  = 736;
  rois.at<int>(1,0)  = 756;
  rois.at<int>(2,0)  = 758;
  rois.at<int>(3,0)  = 760;
  rois.at<int>(4,0)  = 762;
  rois.at<int>(5,0)  = 764;
  rois.at<int>(6,0)  = 764;
  rois.at<int>(7,0)  = 768;
  rois.at<int>(8,0)  = 772;
  rois.at<int>(9,0)  = 778;
  rois.at<int>(10,0) = 774;
  rois.at<int>(11,0) = 780;
  rois.at<int>(12,0) = 780;
  rois.at<int>(13,0) = 784;
  rois.at<int>(14,0) = 784;
  rois.at<int>(15,0) = 784;
  rois.at<int>(16,0) = 790;
  rois.at<int>(17,0) = 790;
  rois.at<int>(18,0) = 792;
  rois.at<int>(19,0) = 790;

  rois.at<int>(0,2)  = 460;
  rois.at<int>(1,2)  = 487;
  rois.at<int>(2,2)  = 494;
  rois.at<int>(3,2)  = 500;
  rois.at<int>(4,2)  = 504;
  rois.at<int>(5,2)  = 506;
  rois.at<int>(6,2)  = 506;
  rois.at<int>(7,2)  = 512;
  rois.at<int>(8,2)  = 520;
  rois.at<int>(9,2)  = 524;
  rois.at<int>(10,2) = 524;
  rois.at<int>(11,2) = 526;
  rois.at<int>(12,2) = 532;
  rois.at<int>(13,2) = 532;
  rois.at<int>(14,2) = 540;
  rois.at<int>(15,2) = 540;
  rois.at<int>(16,2) = 542;
  rois.at<int>(17,2) = 542;
  rois.at<int>(18,2) = 542;
  rois.at<int>(19,2) = 544;


  rois.at<int>(0,1)  = 1000;
  rois.at<int>(1,1)  = 1008;
  rois.at<int>(2,1)  = 988;
  rois.at<int>(3,1)  = 980;
  rois.at<int>(4,1)  = 968;
  rois.at<int>(5,1)  = 966;
  rois.at<int>(6,1)  = 960;
  rois.at<int>(7,1)  = 954;
  rois.at<int>(8,1)  = 950;
  rois.at<int>(9,1)  = 952;
  rois.at<int>(10,1) = 944;
  rois.at<int>(11,1) = 940;
  rois.at<int>(12,1) = 940;
  rois.at<int>(13,1) = 934;
  rois.at<int>(14,1) = 932;
  rois.at<int>(15,1) = 930;
  rois.at<int>(16,1) = 924;
  rois.at<int>(17,1) = 924;
  rois.at<int>(18,1) = 920;
  rois.at<int>(19,1) = 920;


  rois.at<int>(0,3)  = 700;
  rois.at<int>(1,3)  = 700;
  rois.at<int>(2,3)  = 694;
  rois.at<int>(3,3)  = 690;
  rois.at<int>(4,3)  = 682;
  rois.at<int>(5,3)  = 684;
  rois.at<int>(6,3)  = 680;
  rois.at<int>(7,3)  = 678;
  rois.at<int>(8,3)  = 672;
  rois.at<int>(9,3)  = 674;
  rois.at<int>(10,3) = 670;
  rois.at<int>(11,3) = 666;
  rois.at<int>(12,3) = 668;
  rois.at<int>(13,3) = 664;
  rois.at<int>(14,3) = 662;
  rois.at<int>(15,3) = 660;
  rois.at<int>(16,3) = 658;
  rois.at<int>(17,3) = 658;
  rois.at<int>(18,3) = 656;
  rois.at<int>(19,3) = 658;

  for(int i=0;i<20;i++) rois.at<int>(i,2) -=20;

  //  std::vector<Point3d> circle_136;
  namedWindow("sub_image",CV_WINDOW_AUTOSIZE);

  int q=0;
  for(int i=0;i<19;i++){// for each directory
    for(int j=0;j<50;j++){ // for each image in directory
      char image_name[39];
      sprintf(image_name,"./D%03d/P%04d.tiff",i,j);
      
      printf("image_name = %s\n",image_name);
      Mat image = cv::imread(image_name,CV_LOAD_IMAGE_COLOR);

      Range Rx(rois.at<int>(q,0),rois.at<int>(q,1));
      Range Ry(rois.at<int>(q,2),rois.at<int>(q,3));
      Mat sub_image = image(Ry,Rx);
    
      // detect!
      vector<cv::KeyPoint> keypoints;
      circle_detector->detect(sub_image, keypoints);
      //      printf("keypoints size = %d\n",(int)keypoints.size());

      vector<cv::KeyPoint> tkp;
      for(int k=0;k<(int)keypoints.size();k++){
	tkp.push_back(keypoints[k]);
      }
      Mat kimage;
      cv::drawKeypoints(sub_image,tkp,kimage,Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      imshow("sub_image",kimage);
      waitKey(1);


      char c='q';
      while(c != 'n' && pause == true){
	scanf("%c",&c);
	if(c=='q') pause=false;
      }

      Point3d P; // used for both halfs of next if
      if(j==0){ // The first image in each directory defines the number of circles
	radii.clear();
	for(int k=0;k<(int)circles.size();k++){
	  circles[k].clear();
	}
	circles.clear();
	//	printf("Initial sizes: ");
	for(int k=0;k<(int)keypoints.size();k++){
	  P.x = keypoints[k].pt.x; 
	  P.y = keypoints[k].pt.y;
	  P.z = keypoints[k].size;
	  radii.push_back(P.z);
	  vector<Point3d> C;
	  C.push_back(P);
	  circles.push_back(C);
	  //	  printf("%9.3lf ",P.z);
	}
	//	printf("\n");
      }// end first image in directory
      else{ // sort each new keypoint and add to previous detections
	for (int k=0; k<keypoints.size(); k++){
	  P.x = keypoints[k].pt.x; 
	  P.y = keypoints[k].pt.y;
	  P.z = keypoints[k].size;
	  for(int l =0; l<(int)radii.size(); l++){// figure out where the new circle belongs
	    double diff = fabs(P.z-radii[l]);
	    //	    printf("P.z = %9.3lf diff %d = %lf\n",P.z,l,diff);
	    if(fabs(P.z-radii[l])<2.0){// if circle belongs add it
	      int N = circles[l].size();
	      circles[l].push_back(P);
	      radii[l] = (radii[l]*N + P.z)/(N+1); // update average radii
	      continue;
	    }// end if belongs
	  }// end of for each of the existing radii
	}// end for each keypoint in image
      }// end for each image in directory
    }// end for each file in directory

    // analyse data from this directory
    //    printf(" we had %d circles\n",(int) circles.size());
    for(int k=0;k<(int)circles.size();k++){
      int Nk = (int)circles[k].size();
      //      printf("circle %d had %d samples\n",k,Nk);
      double mean_x = 0.0;
      double mean_y = 0.0;
      double mean_z = radii[k]; // aready computed
      for(int l=0;l<Nk;l++){
	mean_x += circles[k][l].x;
	mean_y += circles[k][l].y;
      }
      mean_x = mean_x/Nk;
      mean_y = mean_y/Nk;
      double std_x = 0.0;
      double std_y = 0.0;
      double std_z = 0.0;
      for(int l=0;l<(int)circles[k].size();l++){
	std_x += (mean_x - circles[k][l].x)*(mean_x - circles[k][l].x);
	std_y += (mean_y - circles[k][l].y)*(mean_y - circles[k][l].y);
	std_z += (mean_z - circles[k][l].z)*(mean_z - circles[k][l].z);
      }
      std_x = sqrt(std_x/(Nk-1));
      std_y = sqrt(std_x/(Nk-1));
      std_z = sqrt(std_z/(Nk-1));
      printf("Circle %10.4lf %10.4lf %10.4lf with %10.4lf %10.4lf %10.4lf\n",mean_x,mean_y,mean_z,std_x,std_y,std_z);
      if(std_z >.0001) fprintf(fp,"%10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf;\n",mean_x,mean_y,mean_z,std_x,std_y,std_z,sqrt(std_x*std_x + std_y*std_y));
    }

    q++;// use next roi for next directory
  }// end for each directory
  fprintf(fp,"];\n");
  fprintf(fp,"plot(Circles(:,3),Circles(:,7),'b+')\n");
  fprintf(fp,"xlabel('Size of Circle (pixels)');\n");
  fprintf(fp,"ylabel('Standard Deviation of it Center (pixels)');\n");
  fprintf(fp,"title('Standard Deviation of location vs Circle Size');\n");
  /*
  fprintf(fp,"figure(2);");
  fprintf(fp,"plot(Circles(:,3),Circles(:,6),'b+')\n");
  fprintf(fp,"xlabel('Size of Circle (pixels)');\n");
  fprintf(fp,"ylabel('Standard Deviation of radius (pixels)');\n");
  fprintf(fp,"title('Standard Deviation of radius vs Circle Size');\n");
  */
  fclose(fp);
}

