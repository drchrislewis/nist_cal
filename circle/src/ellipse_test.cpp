#include <stdio.h>
#include <vector>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <iostream>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

//#include "industrial_extrinsic_cal/circle_cost_utils.hpp"
#include "industrial_extrinsic_cal/ceres_costs_utils.hpp"

double PI = atan(1.0)*4.0;

double edist(double cx, double cy, double e0, double e1, double psi, double qx, double qy, double theta)
{
  double c_psi     = cos(psi);
  double s_psi     = sin(psi);
  double c_theta   = cos(theta);
  double s_theta   = sin(theta);
  double x         = cx + e0 * c_theta * c_psi  - e1 * s_theta * s_psi;
  double y         = cy + e0 * c_theta * s_psi  + e1 * s_theta * c_psi;
  double dx        = x-qx;
  double dy        = y-qy;
  double dist      = sqrt(dx*dx + dy*dy);
  double d2center  = sqrt((cx-qx)*(cx-qx) + (cy-qy)*(cy-qy));
  double ed2center = sqrt((cx-x)*(cx-x) +(cy-y)*(cy-y));
  if(d2center<ed2center) {
    dist = -dist; // return negative distance to indicate point is inside ellipse
    //    printf("Point inside ellipse\n");
  }
  else{
    //    printf("Point outside ellipse\n");
  }
  //  printf("closest point = %lf %lf\n",x,y); 

  return(dist);

}

// function to allow ceres to find point on ellipse which is closest to a query point
struct EllipseDist
{
  EllipseDist(double cx, double cy, double e0, double e1, double psi, double qx, double qy) :
    cx_(cx), cy_(cy), e0_(e0), e1_(e1), psi_(psi), qx_(qx), qy_(qy)
  {
    c_psi = cos(psi_);
    s_psi = sin(psi_);
  }

  template<typename T>
  bool operator()(const T* const theta, /** angle from major axis of ellipse to point */
		  T* resid) const
  {
    T c_theta = ceres::cos(theta[0]);
    T s_theta = ceres::sin(theta[0]);
			    
    T x = cx_ + e0_ * c_theta * c_psi - e1_ * s_theta*s_psi;
    T y = cy_ + e0_ * c_theta * s_psi + e1_ * s_theta*c_psi;
    resid[0] = (qx_ - x)*(qx_ - x) + (qy_ - y)*(qy_ - y);

    return true;
  } /** end of operator() */

  /** Factory to hide the construction of the CostFunction object from */
  /** the client code. */
  static ceres::CostFunction* Create(const double cx,
				     const double cy,
				     const double e0, 
				     const double e1, 
				     const double psi,
				     const double qx,
				     const double qy)
  {
    return (new ceres::AutoDiffCostFunction<EllipseDist, 1, 1>(new EllipseDist(cx,cy,e0,e1,psi,qx,qy)));
  }
  double cx_; /*!< center of ellipse x coordinate*/
  double cy_; /*!< center of ellipse y coordinate */
  double e0_; /*!< length of major axis */
  double e1_; /*!< length of minor axis */
  double psi_; /*!< angle major axis makes with x axis*/
  double qx_; /*!< query point x coordinate */
  double qy_; /*!< query point y coordinate */
  double c_psi;    /*!< cosine of psi */
  double s_psi;    /*!< sine of psi */
};


double point2ellipsedistance(double x, double y, double cx, double cy, double e0,double e1, double psi)
{
  double theta;
  ceres::Problem problem;
  ceres::CostFunction* cost_function = EllipseDist::Create(cx,cy,e0,e1,psi,x,y);
  problem.AddResidualBlock(cost_function, NULL, &theta);

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;	// be quiet
  options.max_num_iterations = 1000;

  ceres::Solve(options, &problem, &summary);

  //  printf("theta = %lf\n",theta*180.0/3.1415926);
  return (edist(cx,cy,e0,e1,psi,x,y,theta));
  
}
cv::Mat create_ellipse_image(int m, int n, double cx, double cy, double e0, double e1, double psi, float i_color, float e_color, double s)
{
  cv::Mat image(m,n,CV_32FC1);
  int interior_pts=0;
  int exterior_pts=0;
  int transition_pts=0;
  int num_comp=0;
  double color;
  for(int i=0;i<m;i++){
    for(int j=0;j<n;j++){
      double d = sqrt((i-cx)*(i-cx) + (j-cy)*(j-cy));
      if(d>e0+s/2){		// do easy exterior ones without ceres
	color = e_color;
	exterior_pts++;
      }
      else if(d<e1-s/2){ // do easy interior ones without ceres
	color = i_color;
	interior_pts++;
      }
      else{
	num_comp++;
	d =  point2ellipsedistance(i,n-j,cx,cy,e0,e1,psi);
	if(d>0.0){
	  if(d>s/2){
	    color = e_color;
	    exterior_pts++;
	  }
	  else{
	    double alpha = 0.5 - d/s; // alpha = 0: interior alpha=1.0 exterior
	    color =  (1.0-alpha)*e_color + alpha*i_color;
	    transition_pts++;
	  }
	}// end exterior
	else {// d<0.0
	  if(-d>s/2){
	    color = i_color;
	    interior_pts++;
	  }
	  else{ // transition zone
	    double alpha = .5 - d/s; // alpha = 0: interior alpha=1.0 exterior
	    color =  (1.0-alpha)*e_color + alpha*i_color;
	    transition_pts++;
	  }
	}// end interior
      }
      image.at<float>(i,j) = color;
    }// end for cols
  }// end for rows
  return(image);
}

void R_eye(double A[9])
{
  A[0] = 1.0;  A[3] = 0.0;  A[6] = 0.0;
  A[1] = 0.0;  A[4] = 1.0;  A[7] = 0.0;
  A[2] = 0.0;  A[5] = 0.0;  A[8] = 1.0;
}

void R_inv(double A[9],double Ainv[9])
{
  Ainv[0] = A[0]; Ainv[3] = A[1]; Ainv[6] = A[2];
  Ainv[1] = A[3]; Ainv[4] = A[4]; Ainv[7] = A[5];
  Ainv[2] = A[6]; Ainv[5] = A[7]; Ainv[8] = A[8];
}


void Rc_mul(double A[9],double B[9],double AB[9])
{
  AB[0] = A[0]*B[0] + A[3]*B[1] + A[6]*B[2];  
  AB[3] = A[0]*B[3] + A[3]*B[4] + A[6]*B[5]; 
  AB[6] = A[0]*B[6] + A[3]*B[7] + A[6]*B[8];
  
  AB[1] = A[1]*B[0] + A[4]*B[1] + A[7]*B[2];  
  AB[4] = A[1]*B[3] + A[4]*B[4] + A[7]*B[5];
  AB[7] = A[1]*B[6] + A[4]*B[7] + A[7]*B[8];

  AB[2] = A[2]*B[0] + A[5]*B[1] + A[8]*B[2];  
  AB[5] = A[2]*B[3] + A[5]*B[4] + A[8]*B[5];  
  AB[8] = A[2]*B[6] + A[5]*B[7] + A[8]*B[8];
}

void Rc_mulv(double A[9],double x[3],double Ax[3])
{
  Ax[0] = A[0]*x[0] + A[3]*x[1] + A[6]*x[2];  
  Ax[1] = A[1]*x[0] + A[4]*x[1] + A[7]*x[2]; 
  Ax[2] = A[2]*x[0] + A[5]*x[1] + A[8]*x[2];
}

void Rc_Rotx(double theta, double R1[9])
{
  double c = cos(theta);
  double s = sin(theta);
  R1[0] = 1.0;  R1[3] = 0.0;  R1[6] = 0.0;
  R1[1] = 0.0;  R1[4] = c;    R1[7] = -s;
  R1[2] = 0.0;  R1[5] = s;    R1[8] = c;
}

void Rc_Roty(double alpha, double R1[9])
{
  double c = cos(alpha);
  double s = sin(alpha);
  R1[0] = c;   R1[3] = 0.0;  R1[6] = s;
  R1[1] = 0.0; R1[4] = 1.0;  R1[7] = 0.0;
  R1[2] = -s;   R1[5] = 0.0;  R1[8] = c;
}

void Rc_Rotz(double beta, double R1[9])
{
  double c = cos(beta);
  double s = sin(beta);
  R1[0] = c;   R1[3] = -s;   R1[6] = 0.0;
  R1[1] = s;   R1[4] = c;    R1[7] = 0.0;
  R1[2] = 0.0; R1[5] = 0.0;  R1[8] = 1;
}

void Rc_print(double R[9], std::string msg)
{
  printf("%s:\n",msg.c_str());
  printf("%6.3lf %6.3lf %6.3lf\n",R[0],R[3],R[6]);
  printf("%6.3lf %6.3lf %6.3lf\n",R[1],R[4],R[7]);
  printf("%6.3lf %6.3lf %6.3lf\n",R[2],R[5],R[8]);
}

void Rr_print(double R[9], char * msg)
{
  printf("%s:\n",msg);
  printf("%6.3lf %6.3lf %6.3lf\n",R[0],R[1],R[2]);
  printf("%6.3lf %6.3lf %6.3lf\n",R[3],R[4],R[5]);
  printf("%6.3lf %6.3lf %6.3lf\n",R[6],R[7],R[8]);
}

void Rr_mul(double A[9],double B[9],double AB[9])
{
  
  AB[0] = A[0]*B[0] + A[1]*B[3] + A[2]*B[6];  
  AB[3] = A[3]*B[0] + A[4]*B[3] + A[5]*B[6]; 
  AB[6] = A[6]*B[0] + A[7]*B[3] + A[8]*B[6];
  
  AB[1] = A[0]*B[1] + A[1]*B[4] + A[2]*B[7];  
  AB[4] = A[3]*B[1] + A[4]*B[4] + A[5]*B[7];
  AB[7] = A[6]*B[1] + A[7]*B[4] + A[8]*B[7];

  AB[2] = A[0]*B[2] + A[1]*B[5] + A[2]*B[8];  
  AB[5] = A[3]*B[2] + A[4]*B[5] + A[5]*B[8];  
  AB[8] = B[2]*A[6] + B[5]*A[7] + B[8]*A[8];
}


void Rr_Rotx(double theta, double R1[9])
{
  double c = cos(theta);
  double s = sin(theta);
  R1[0] = 1.0;  R1[1] = 0.0;  R1[2] = 0.0;
  R1[3] = 0.0;  R1[4] = c;    R1[5] = -s;
  R1[6] = 0.0;  R1[7] = s;    R1[8] = c;
}

void Rr_Roty(double alpha, double R1[9])
{
  double c = cos(alpha);
  double s = sin(alpha);
  R1[0] = c;   R1[1] = 0.0;  R1[2] = -s;
  R1[3] = 0.0; R1[4] = 1.0;  R1[5] = 0.0;
  R1[6] = s;   R1[7] = 0.0;  R1[8] = c;
}

void Rr_Rotz(double beta, double R1[9])
{
  double c = cos(beta);
  double s = sin(beta);
  R1[0] = c;   R1[1] = -s;   R1[2] = 0.0;
  R1[3] = s;   R1[4] = c;    R1[5] = 0.0;
  R1[6] = 0.0; R1[7] = 0.0;  R1[8] = 1;
}

void print_camera_int_params(double cparams[9])
{
  printf("fx = %6.3lf fy = %6.3lf\n",cparams[0],cparams[1]);
  printf("cx = %6.3lf cy = %6.3lf\n",cparams[2],cparams[3]);
  printf("k1 = %6.3lf k2 = %6.3lf k3 = %6.3lf\n",cparams[4],cparams[5],cparams[6]);
  printf("p1 = %6.3lf p2 = %6.3lf\n",cparams[7],cparams[8]);
}
  
#define PI 3.1415926
int main(int argc, char** argv)
{
  double cx  = 75.0;		// center of ellipse
  double cy  = 75.0;
  double e0  = 30.0;		// length of major axis
  double e1  = 20.0;		// length of minor axis
  double psi = 45*PI/180.0;	// angle between major axis and x axis

  double qx = 21.0;		// query point x coordinate
  double qy = 22.0;		// query point y coordinate

  double d = point2ellipsedistance(qx,qy,cx,cy,e0,e1,psi);
  printf("distance = %lf\n",d);

  double s = 15.0;
  float i_color = 0.0;
  float e_color = 1.0;
  cv::Mat image;
  image = create_ellipse_image(150, 150, cx, cy, e0, e1, psi, i_color, e_color, s);
  
  cv::namedWindow("Ellipse",CV_WINDOW_AUTOSIZE);
  cv::imshow("Ellipse",image);
  cv::waitKey(30);

  // use these variables to 
  //     define a camera with some intrinsics
  //     define a target at a location
  //     define a circle point on the target
  double p1[6];			// camera extrinsics
  double p2[6];			// target transform
  double p3[9];			// camera intrinsics
  double p4[3];			// point target coordinates
  double r[2];			// residual
  double Obs_x = 966.01519;	// Observed center of ellipse x
  double Obs_y = 516.59866;	// Observed center of ellipse y
  double c_dia = .2;		// circle diameter
  double target_height = 1.5;
  double target_width  = 2.0;

  // Camera transform
  double R_camera[9]; 
  double cp[3];  
  double R_camera_inv[9]; 

  // target transform
  double R_target[9];
  double tp[3];
  
  double C_ex[6];		// camera extrinsic parameters (transforms pts from world to camera)
  double *C_aa = &C_ex[0];	// angle axis portion
  double *C_tx = &C_ex[3];	// translation portion

  double C_in[9];		// camera intrinsic parameters 
  int q=0;
  double *C_fx_ptr = &C_in[q++]; // focal length x
  double *C_fy_ptr = &C_in[q++]; // focal length y
  double *C_cx_ptr = &C_in[q++]; // central point x
  double *C_cy_ptr = &C_in[q++]; // central point y
  double *C_k1_ptr = &C_in[q++]; // distortion k1
  double *C_k2_ptr = &C_in[q++]; // distortion k2
  double *C_k3_ptr = &C_in[q++]; // distortion k3
  double *C_p1_ptr = &C_in[q++]; // distortion p1
  double *C_p2_ptr = &C_in[q++]; // distortion p2

  *C_fx_ptr = 892.33958;	// taken from a basler camera (left stereo camera)
  *C_fy_ptr = 891.15427;
  *C_cx_ptr = 966.01519;
  *C_cy_ptr = 516.59866;
  *C_k1_ptr =  -0.01124;
  *C_k2_ptr =   0.01366;
  *C_k3_ptr =  -0.00164;
  *C_p1_ptr =  -0.00145;
  *C_p2_ptr =   0.00000;

  double T_ex[6];		// Target extrinsic parameters (transforms pts from camera to world)
  double *T_aa = &T_ex[0];	// angle axis portion
  double *T_tx = &T_ex[3];	// translation portion
    
  // target lying down in xy plane with its x axis along world positve y, z along world +z
  double RTz[9];
  Rc_Rotz(PI/2.0,RTz);
  double RTx[9];
  Rc_Roty(-PI/4,RTx);
  Rc_mul(RTx,RTz,R_target);
  ceres::RotationMatrixToAngleAxis(R_target,T_aa);
  
  // origin of target
  //  T_tx[0] = target_height;
  //  T_tx[1] = 0.0;
  T_tx[0] = target_height/2.0;
  T_tx[1] = target_width/2.0;
  T_tx[2] = 0.0;
 
  // location of circle of interest within target frame
  double point[3];
  //  point[0] = target_width/2.0;
  //  point[1] = target_height/2.0;
  point[0] = 0.0;
  point[1] = 0.0;
  point[2] = 0.0;

  // camera centered over target looking straight down on it
  //  (remember, its world origin as seen in camera frame)
  C_tx[0] = -target_width/2.0;
  C_tx[1] = -target_height/2.0;
  C_tx[2] = 3.0; // distance to target


  double Rz[9],Rx[9];
  Rc_Rotz(PI/2.0,Rz);
  Rc_Rotx(PI,Rx);
  Rc_mul(Rz,Rx,R_camera_inv);
  R_inv(R_camera_inv,R_camera);
  ceres::RotationMatrixToAngleAxis(R_camera,C_aa);

  double wp[3];
  Rc_mulv(R_target,point,wp);
  wp[0] = wp[0] + T_tx[0];
  wp[1] = wp[1] + T_tx[1];
  wp[2] = wp[2] + T_tx[2];
  Rc_mulv(R_camera,wp,cp);
  cp[0] = cp[0] + C_tx[0];
  cp[1] = cp[1] + C_tx[1];
  cp[2] = cp[2] + C_tx[2];
  double R_CtoT[9];
  Rc_mul(R_camera,R_target,R_CtoT);
  Rc_print(R_CtoT,"R_CtoT");
  Rc_print(R_camera_inv,"R_camera_inv");
  Rc_print(R_camera,"R_camera");
  printf("world location in camera: %6.3lf %6.3lf %6.3lf\n",C_tx[0],C_tx[1],C_tx[2]);
  Rc_print(R_target,"R_target");
  printf("target location : %6.3lf %6.3lf %6.3lf\n",T_tx[0],T_tx[1],T_tx[2]);
  printf("target point location in target:%6.3lf %6.3lf %6.3lf\n",point[0],point[1],point[2]);
  printf("target point location in world:%6.3lf %6.3lf %6.3lf\n",wp[0],wp[1],wp[2]);
  printf("target point location in camera:%6.3lf %6.3lf %6.3lf\n",cp[0],cp[1],cp[2]);
  print_camera_int_params(C_in);

  double fx =  892.33958;	// taken from a basler camera (left stereo camera)
  double fy = 891.15427;
  cx = 966.01519;
  cy = 516.59866;
  double k1 =  -0.01124;
  double k2 =   0.01366;
  double k3 =  -0.00164;

  industrial_extrinsic_cal::CircleTargetCameraReprjError K(Obs_x,Obs_y,c_dia, fx, fy, cx, cy);
  bool rtn;
  if((rtn = K(C_ex,T_ex,point,r))==true){
    printf("rtn = true \n r = %lf %lf\n",r[0],r[1]);
  }
  double testaa[3];
  testaa[0] = .5;
  testaa[1] = .8;
  testaa[2] = -.8;
  double testR[9];
  ceres::AngleAxisToRotationMatrix(testaa,testR);
  Rc_print(testR,"testR");
  testaa[0] = -.5;
  testaa[1] = -.8;
  testaa[2] = .8;
  ceres::AngleAxisToRotationMatrix(testaa,testR);
  Rc_print(testR,"testR_again");
  while(1);
  return 0;
}
