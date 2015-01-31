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
  std::vector<vector<Point3d> > pcircles;

  //  std::vector<Point3d> circle_136;
  namedWindow("sub_image",CV_WINDOW_AUTOSIZE);

  int q=0;
  for(int i=0;i<30;i++){// for each directory
    for(int j=0;j<30;j++){ // for each image in directory
      char image_name[39];
      sprintf(image_name,"./D%03d/P%04d.tiff",i,j);
      
      printf("image_name = %s\n",image_name);
      Mat image = cv::imread(image_name,CV_LOAD_IMAGE_COLOR);

      Range Rx(755,930);
      Range Ry(460,670);
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

      Point3d P; // used for both halfs of next if
      if(j==0){ // The first image in each directory defines the number of circles
	radii.clear();
	for(int k=0;k<(int)circles.size();k++){
	  circles[k].clear();
	}
	circles.clear();
	for(int k=0;k<(int)keypoints.size();k++){
	  P.x = keypoints[k].pt.x; 
	  P.y = keypoints[k].pt.y;
	  P.z = keypoints[k].size;
	  radii.push_back(P.z);
	  vector<Point3d> C;
	  C.push_back(P);
	  circles.push_back(C);
	  if(i==0)pcircles.push_back(C);// create circle vector for each size on first dir, first image
	}
      }// end first image in directory
      else{ // sort each new keypoint and add to previous detections
	for (int k=0; k<keypoints.size(); k++){
	  P.x = keypoints[k].pt.x; 
	  P.y = keypoints[k].pt.y;
	  P.z = keypoints[k].size;
	  for(int l =0; l<(int)radii.size(); l++){// figure out where the new circle belongs
	    double diff = fabs(P.z-radii[l]);
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
    for(int k=0;k<(int)circles.size();k++){
      int Nk = (int)circles[k].size();
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

      // add each mean location and size to plot the path of each circle 
      int index = -1;
      for(int l=0;l<(int)pcircles.size();l++){
	if(fabs(mean_z-pcircles[l][0].z)<3.0){
	  index = l;
	}
      }
      if(index>=0){
	Point3d P(mean_x,mean_y,mean_z);
	pcircles[index].push_back(P);
      }
      else{
	printf("Could not match this circle %lf %lf %lf\n",mean_x,mean_y,mean_z);
	printf("available sizes are: ");
	for(int l=0;l<(int)pcircles.size();l++) printf("%lf ",pcircles[l][0].z);
	printf("\n");
      }
    }// end of for each circle in this directory

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
  fprintf(fp,"figure(3);\n");
  fprintf(fp,"hold on\n");
  for(int l=0;l<(int)pcircles.size();l++){
    fprintf(fp,"pc%d = [\n",l);
    for(int k=0;k<(int)pcircles[l].size();k++){
      fprintf(fp,"%lf %lf %lf;\n",pcircles[l][k].x,pcircles[l][k].y,pcircles[l][k].z);
    }
    fprintf(fp,"];\n");
    if(l%7==0) fprintf(fp,"plot3(pc%d(:,1),pc%d(:,2),pc%d(:,3),'b+')\n",l,l,l);
    if(l%7==1) fprintf(fp,"plot3(pc%d(:,1),pc%d(:,2),pc%d(:,3),'r+')\n",l,l,l);
    if(l%7==2) fprintf(fp,"plot3(pc%d(:,1),pc%d(:,2),pc%d(:,3),'g+')\n",l,l,l);
    if(l%7==3) fprintf(fp,"plot3(pc%d(:,1),pc%d(:,2),pc%d(:,3),'c+')\n",l,l,l);
    //    if(l%7==4) fprintf(fp,"plot3(pc%d(:,1),pc%d(:,2),pc%d(:,3),'y+')\n",l,l,l);
    if(l%7==5) fprintf(fp,"plot3(pc%d(:,1),pc%d(:,2),pc%d(:,3),'k+')\n",l,l,l);
    if(l%7==6) fprintf(fp,"plot3(pc%d(:,1),pc%d(:,2),pc%d(:,3),'m+')\n",l,l,l);
  }
  fprintf(fp,"xlabel('Image locations x');\n");
  fprintf(fp,"ylabel('Image locations y');\n");
  fprintf(fp,"zlabel('Circle Size (pixels) y');\n");
  fprintf(fp,"title('Trajectories of Slanted Target .005in movements');\n");


  fprintf(fp,"figure(4);\n");
  fprintf(fp,"hold on\n");
  for(int l=0;l<(int)pcircles.size();l++){
    fprintf(fp,"pc%d = [\n",l);
    for(int k=0;k<(int)pcircles[l].size();k++){
      fprintf(fp,"%lf %lf %lf;\n",pcircles[l][k].x,pcircles[l][k].y,pcircles[l][k].z);
    }
    fprintf(fp,"];\n");
    if(l%7==0) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'b+')\n",l,l);
    if(l%7==1) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'r+')\n",l,l);
    if(l%7==2) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'g+')\n",l,l);
    if(l%7==3) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'c+')\n",l,l);
    //    if(l%7==4) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'y+')\n",l,l);
    if(l%7==5) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'k+')\n",l,l);
    if(l%7==6) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'m+')\n",l,l);

    if(l%7==0) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'b')\n",l,l);
    if(l%7==1) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'r')\n",l,l);
    if(l%7==2) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'g')\n",l,l);
    if(l%7==3) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'c')\n",l,l);
    //    if(l%7==4) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'y')\n",l,l);
    if(l%7==5) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'k')\n",l,l);
    if(l%7==6) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'m')\n",l,l);
  }
  fprintf(fp,"xlabel('Image locations x');\n");
  fprintf(fp,"ylabel('Image locations y');\n");
  fprintf(fp,"title('Trajectories of Slanted Target .005in movements');\n");

  fprintf(fp,"figure(5);\n");
  fprintf(fp,"hold on;\n");
  // line fit analysis
  for(int l = 0;l<(int)pcircles.size();l++){
    int n = pcircles[l].size();
    fprintf(fp,"for i=1:%d, A(i,1) = pc%d(i,1); A(i,2) = 1; B(i) = pc%d(i,2); end;\n",n,l,l);
    fprintf(fp,"y = pinv(A)*B';\n");
    fprintf(fp,"R = A*y-B';\n");
    if(l%7==0) fprintf(fp,"plot(R,'b')\n");
    if(l%7==1) fprintf(fp,"plot(R,'r')\n");
    if(l%7==2) fprintf(fp,"plot(R,'g')\n");
    if(l%7==3) fprintf(fp,"plot(R,'c')\n");
    //    if(l%7==4) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'y')\n");
    if(l%7==5) fprintf(fp,"plot(R,'k')\n");
    if(l%7==6) fprintf(fp,"plot(R,'m')\n");
  }
  fprintf(fp,"xlabel('Sample number (.005 inch increments)');\n");
  fprintf(fp,"ylabel('Error in y to linear fit (pixels)');\n");
  fprintf(fp,"title('Residual error trajectories for each circle');\n");

  
  fprintf(fp,"figure(6);\n");
  fprintf(fp,"hold on;\n");
  for(int i=0;i<(int)pcircles.size();i++){
    fprintf(fp,"delta_x%d = [\n",i);
    for(int j=2;j<(int)pcircles[i].size();j++){
      fprintf(fp,"%lf ",pcircles[i][j].x-pcircles[i][j-1].x);
    }
    fprintf(fp,"];\n",i);
    if(i%7==0) fprintf(fp,"plot(delta_x%d,'b')\n",i);
    if(i%7==1) fprintf(fp,"plot(delta_x%d,'r')\n",i);
    if(i%7==2) fprintf(fp,"plot(delta_x%d,'g')\n",i);
    if(i%7==3) fprintf(fp,"plot(delta_x%d,'c')\n",i);
    //    if(i%7==4) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'y')\n",i);
    if(i%7==5) fprintf(fp,"plot(delta_x%d,'k')\n",i);
    if(i%7==6) fprintf(fp,"plot(delta_x%d,'m')\n",i);

    if(i%7==0) fprintf(fp,"plot(delta_x%d,'b+')\n",i);
    if(i%7==1) fprintf(fp,"plot(delta_x%d,'r+')\n",i);
    if(i%7==2) fprintf(fp,"plot(delta_x%d,'g+')\n",i);
    if(i%7==3) fprintf(fp,"plot(delta_x%d,'c+')\n",i);
    //    if(i%7==4) fprintf(fp,"plot(pc%d(:,1),pc%d(:,2),'y')\n",i);
    if(i%7==5) fprintf(fp,"plot(delta_x%d,'k+')\n",i);
    if(i%7==6) fprintf(fp,"plot(delta_x%d,'m+')\n",i);
  }
  fprintf(fp,"xlabel('Sample number (.005 inch increments)');\n");
  fprintf(fp,"ylabel('Deltax');\n");
  fprintf(fp,"title('Measured incremental motion in X for all circles');\n");

  fclose(fp);
}

