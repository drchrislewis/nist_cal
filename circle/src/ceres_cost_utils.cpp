/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include <industrial_extrinsic_cal/basic_types.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"


using industrial_extrinsic_cal::CameraParameters;
using industrial_extrinsic_cal::Point3d;
using industrial_extrinsic_cal::Pose6d;

namespace circle
{
  class Observation
  {
  public:
    Observation()
    {
      point_id = 0;
      image_loc_x = 0.0;
      image_loc_y = 0.0;
    };
    ~Observation(){};
    int point_id;
    double image_loc_x;
    double image_loc_y;
  };

  Observation projectPoint(CameraParameters C, Point3d P)
  {
    double p[3];
    double pt[3];
    pt[0] = P.x;
    pt[1] = P.y;
    pt[2] = P.z;

    /* transform point into camera frame */
    /* note, camera transform takes points from camera frame into world frame */
    ceres::AngleAxisRotatePoint(C.angle_axis, pt, p);

    p[0] += C.position[0];
    p[1] += C.position[1];
    p[2] += C.position[2];

    double xp = p[0] / p[2];
    double yp = p[1] / p[2];

    double r2 = xp * xp + yp * yp;
    double r4 = r2 * r2;
    double r6 = r2 * r4;

    double xp2 = xp * xp; /* temporary variables square of others */
    double yp2 = yp * yp;

    /* apply the distortion coefficients to refine pixel location */
    double xpp = xp + C.distortion_k1 * r2 * xp 
      + C.distortion_k2 * r4 * xp  
      + C.distortion_k3 * r6 * xp  
      + C.distortion_p2 * (r2 + 2 * xp2) 
      + C.distortion_p1 * xp * yp * 2.0;
    double ypp = yp + C.distortion_k1 * r2 * yp 
      + C.distortion_k2 * r4 * yp 
      + C.distortion_k3 * r6 * yp 
      + C.distortion_p1 * (r2 + 2 * yp2) 
      + C.distortion_p2 * xp * yp * 2.0;

    /* perform projection using focal length and camera center into image plane */
    Observation O;
    O.point_id = 0;
    O.image_loc_x = C.focal_length_x * xpp + C.center_x;
    O.image_loc_y = C.focal_length_y * ypp + C.center_y;
    return (O);
  }

  // HELPER TEMPLATES  

  template<typename T> inline void rotationProduct(const T R1[9], const T R2[9], T R3[9]);
  template<typename T> inline void rotationProduct(const T R1[9], const T R2[9], T R3[9])
  {
    // x column
    R3[0] = R1[0]*R2[0] +  R1[3]*R2[1] +  R1[6]*R2[2];
    R3[1] = R1[1]*R2[0] +  R1[4]*R2[1] +  R1[7]*R2[2];
    R3[2] = R1[2]*R2[0] +  R1[5]*R2[1] +  R1[8]*R2[2];
    // y column
    R3[3] = R1[0]*R2[3] +  R1[3]*R2[4] +  R1[6]*R2[5];
    R3[4] = R1[1]*R2[3] +  R1[4]*R2[4] +  R1[7]*R2[5];
    R3[5] = R1[2]*R2[3] +  R1[5]*R2[4] +  R1[8]*R2[5];
    // z column
    R3[6] = R1[0]*R2[6] +  R1[3]*R2[7] +  R1[6]*R2[8];
    R3[7] = R1[1]*R2[6] +  R1[4]*R2[7] +  R1[7]*R2[8];
    R3[8] = R1[2]*R2[6] +  R1[5]*R2[7] +  R1[8]*R2[8];
  }

  template<typename T> inline void extractCameraIntrinsics(const T intrinsics[9], T fx, T fy, T cx, T cy, T k1, T k2, T k3, T p1, T p2);
  template<typename T> inline void extractCameraIntrinsics(const T intrinsics[9], T fx, T fy, T cx, T cy, T k1, T k2, T k3, T p1, T p2)
  {
    fx  = intrinsics[0]; /** focal length x */
    fy  = intrinsics[1]; /** focal length y */
    cx  = intrinsics[2]; /** central point x */
    cy  = intrinsics[3]; /** central point y */
    k1  = intrinsics[4]; /** distortion k1  */
    k2  = intrinsics[5]; /** distortion k2  */
    k3  = intrinsics[6]; /** distortion k3  */
    p1  = intrinsics[7]; /** distortion p1  */
    p2  = intrinsics[8]; /** distortion p2  */
  }

  template<typename T> inline void rotationInverse(const T R[9], const T RI[9]);
  template<typename T> inline void rotationInverse(const T R[9], T RI[9])
  {
    RI[0] = R[0]; RI[3] = R[1];  RI[6] = R[2];
    RI[1] = R[3]; RI[4] = R[4];  RI[7] = R[5];
    RI[2] = R[6]; RI[5] = R[7];  RI[8] = R[8];
  }
  template<typename T> inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3]);
  template<typename T> inline void transformPoint(const T angle_axis[3], const T tx[3], const T point[3], T t_point[3])
  {
    ceres::AngleAxisRotatePoint(angle_axis, point, t_point);
    t_point[0] = t_point[0] + tx[0];
    t_point[1] = t_point[1] + tx[1];
    t_point[2] = t_point[2] + tx[2];
  }

  template<typename T> inline void poseTransformPoint(const Pose6d pose, const T point[3], T t_point[3]);
  template<typename T> inline void poseTransFormPoint(const Pose6d pose, const T point[3], T t_point[3])
  {
    T angle_axis[3];
    angle_axis[0]  = T(pose.ax);
    angle_axis[1]  = T(pose.ay);
    angle_axis[2]  = T(pose.az);
    ceres::AngleAxisRotatePoint(angle_axis, point, t_point);
    t_point[0] = t_point[0] + T(pose.x);
    t_point[1] = t_point[1] + T(pose.y);
    t_point[2] = t_point[2] + T(pose.z);
  }

  template<typename T> inline void transformPoint(const T angle_axis[3], const T tx[3], const Point3d &point, T t_point[3]);
  template<typename T> inline void transformPoint(const T angle_axis[3], const T tx[3], const Point3d &point, T t_point[3])
  {
    T point_[3];
    point_[0] = T(point.x);
    point_[1] = T(point.y);
    point_[2] = T(point.z);
    ceres::AngleAxisRotatePoint(angle_axis, point_, t_point);
    t_point[0] = t_point[0] + tx[0];
    t_point[1] = t_point[1] + tx[1];
    t_point[2] = t_point[2] + tx[2];
  }

  template<typename T> inline void poseRotationMatrix(const Pose6d pose, T R[9]);
  template<typename T> inline void poseRotationMatrix(const Pose6d pose, T R[9])
  {
    T angle_axis[3];
    angle_axis[0] = T(pose.ax);
    angle_axis[1] = T(pose.ay);
    angle_axis[2] = T(pose.az);
    ceres::AngleAxisToRotationMatrix(angle_axis, R);
  }
  
  template<typename T> inline void cameraPntResidualDist(T point[3], T k1, T k2, T k3, T p1, T p2, T fx, T fy, T cx, T cy, T ox, T oy, T resid[2]);
  template<typename T> inline void cameraPntResidualDist(T point[3], T k1, T k2, T k3, T p1, T p2, T fx, T fy, T cx, T cy, T ox, T oy, T resid[2])
  {
    T xp1 = point[0];
    T yp1 = point[1];
    T zp1 = point[2];

    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1;
    T yp = yp1 / zp1;

    /** calculate terms for polynomial distortion */
    T r2 = xp * xp + yp * yp;
    T r4 = r2 * r2;
    T r6 = r2 * r4;

    T xp2 = xp * xp; /** temporary variables square of others */
    T yp2 = yp * yp;

    /*apply the distortion coefficients to refine pixel location */
    T xpp = xp + k1 * r2 * xp + k2 * r4 * xp + k3 * r6 * xp + p2 * (r2 + T(2.0) * xp2) + T(2.0) * p1 * xp * yp;
    T ypp = yp + k1 * r2 * yp + k2 * r4 * yp + k3 * r6 * yp + p1 * (r2 + T(2.0) * yp2) + T(2.0) * p2 * xp * yp;

    /** perform projection using focal length and camera center into image plane */
    resid[0] = fx * xpp + cx;
    resid[1] = fy * ypp + cy;
  }
  template<typename T> inline void cameraCircResidualDist(T point[3], T circle_diameter, T R_TtoC[9], 
							  T k1, T k2, T k3, T p1, T p2, 
							  T fx, T fy, T cx, T cy, T ox, T oy, T resid[2]);
  template<typename T> inline void cameraCircResidualDist(T point[3], T circle_diameter, T R_TtoC[9],
							  T k1, T k2, T k3, T p1, T p2, 
							  T fx, T fy, T cx, T cy, T ox, T oy, T resid[2])
  {
    T xp1 = point[0];
    T yp1 = point[1];
    T zp1 = point[2];
    
    // Circle Delta is the difference between the projection of the center of the circle
    // and the center of the projected ellipse 

    // The 3 columns of R_TtoC represent:
    // 1. the x-axis of the target in camera coordinates
    // 2. the y-axis of the target in camera coordinates
    // 3. the z-axis (normal of the target plane) in camera coordinates
    // NOTE: we assume target is an XY planar target (all points on target have nominal z=0)
    
    // Find projection of distance vector D = [xp1 yp1 zp1] on to plane of target
    T D_targetx = xp1*R_TtoC[0] + yp1*R_TtoC[1] + zp1*R_TtoC[2];
    T D_targety = xp1*R_TtoC[3] + yp1*R_TtoC[4] + zp1*R_TtoC[5];
    
    // projection of D onto target xy plane expressed in camera frame is given by 
    // D_targetx * R_TtoC(1stcol) + D_targety * R_TtoC(2ndcol)
    
    // The vector of interest "Vperp" is orthogonal to this in the target xy plane
    // Vperp = -D_targety * R_TtoC(1stcol) + D_targetx * R_TtoC(2ndcol)
    // However we want Vperp to be in the direction with a negative z component
    
    T Vperp[3]; 
    Vperp[0] = -D_targety*R_TtoC[0] + D_targetx*R_TtoC[3] ;
    Vperp[1] = -D_targety*R_TtoC[1] + D_targetx*R_TtoC[4] ;
    Vperp[2] = -D_targety*R_TtoC[2] + D_targetx*R_TtoC[5] ;
    
    // Vector direction of Vperp is arbitrary, but need to specify direction closer to camera
    T mysign = -abs(Vperp[2])/Vperp[2]; // Warning, division by zero could happen
    Vperp[0] = mysign*Vperp[0];
    Vperp[1] = mysign*Vperp[1];
    Vperp[2] = mysign*Vperp[2];
    
    
    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1;
    T yp = yp1 / zp1;
    
    if(zp1+Vperp[2] !=0.0){	// adjust only focal plan not parallel to target's xy plane
      T Vpx = (xp1+Vperp[0])/(zp1+Vperp[2]);
      T Vpy = (yp1+Vperp[1])/(zp1+Vperp[2]);
      T Vnorm = sqrt(Vpx*Vpx+Vpy*Vpy);
      if(Vnorm!=0.0){
	// find scale of motion
	// Delta = (r*sin(theta)/(D-rcos(theta)) - r*sin(theta)/(D+rcos(theta)))/2
	// where r is the radius of the circle being projected
	//       D is the distance between camera and circle center
	//       theta is the angle between D vector an target xy plane
	Vpx = Vpx/Vnorm;
	Vpy = Vpy/Vnorm;
	T D = sqrt(xp1*xp1 + yp1*yp1 + zp1*zp1);
	T s_theta = (R_TtoC[6]*xp1 + R_TtoC[7]*yp1 + R_TtoC[8]*zp1)/D;
	T c_theta = sqrt( T(1.0) - s_theta*s_theta);
	T r = T(circle_diameter/2.0);
	T Delta = r*s_theta*(T(1.0)/(D-r*c_theta) - T(1.0)/(D+r*c_theta))/T(2.0);
	xp = xp + Delta*Vpx;
	yp = yp + Delta*Vpy;
      }
    }
    
    /* temporary variables for distortion model */
    T xp2 = xp * xp;		/* x^2 */
    T yp2 = yp * yp;		/* y^2 */
    T r2  = xp2 + yp2;	/* r^2 radius squared */
    T r4  = r2 * r2;		/* r^4 */
    T r6  = r2 * r4;		/* r^6 */
    
    /* apply the distortion coefficients to refine pixel location */
    T xpp = xp 
      + k1 * r2 * xp		// 2nd order term
      + k2 * r4 * xp		// 4th order term
      + k3 * r6 * xp		// 6th order term
      + p2 * (r2 + T(2.0) * xp2) // tangential
      + p1 * xp * yp * T(2.0); // other tangential term
    T ypp = yp 
      + k1 * r2 * yp		// 2nd order term
      + k2 * r4 * yp		// 4th order term
      + k3 * r6 * yp		// 6th order term
      + p1 * (r2 + T(2.0) * yp2) // tangential term
      + p2 * xp * yp * T(2.0); // other tangential term
    
    /** perform projection using focal length and camera center into image plane */
    resid[0] = fx * xpp + cx - ox;
    resid[1] = fy * ypp + cy - oy;
  }

  template<typename T> inline void cameraCircResidual(T point[3], T circle_diameter, T R_TtoC[9],
						      T fx, T fy, T cx, T cy, T ox, T oy, T resid[2]);
  template<typename T> inline void cameraCircResidual(T point[3], T circle_diameter, T R_TtoC[9],
						      T fx, T fy, T cx, T cy, T ox, T oy, T resid[2])
  {
    T xp1 = point[0];
    T yp1 = point[1];
    T zp1 = point[2];
    
    // Circle Delta is the difference between the projection of the center of the circle
    // and the center of the projected ellipse 
    
    // find rotation from target coordinates into camera coordinates
    // The 3 columns represent:
    // 1. the x-axis of the target in camera coordinates
    // 2. the y-axis of the target in camera coordinates
    // 3. the z-axis (normal of the target plane) in camera coordinates
    // NOTE: we assume target is an XY planar target (all points on target have nominal z=0)
    
    // Find projection of distance vector D = [xp1 yp1 zp1] on to plane of target
    T D_targetx = xp1*R_TtoC[0] + yp1*R_TtoC[1] + zp1*R_TtoC[2];
    T D_targety = xp1*R_TtoC[3] + yp1*R_TtoC[4] + zp1*R_TtoC[5];
    
    // projection of D onto target xy plane expressed in camera frame is given by 
    // D_targetx * R_TtoC(1stcol) + D_targety * R_TtoC(2ndcol)
    
    // The vector of interest "Vperp" is orthogonal to this in the target xy plane
    // Vperp = -D_targety * R_TtoC(1stcol) + D_targetx * R_TtoC(2ndcol)
    // However we want Vperp to be in the direction with a negative z component
    
    T Vperp[3]; 
    Vperp[0] = -D_targety*R_TtoC[0] + D_targetx*R_TtoC[3] ;
    Vperp[1] = -D_targety*R_TtoC[1] + D_targetx*R_TtoC[4] ;
    Vperp[2] = -D_targety*R_TtoC[2] + D_targetx*R_TtoC[5] ;
    
    // Vector direction of Vperp is arbitrary, but need to specify direction closer to camera
    T mysign = -abs(Vperp[2])/Vperp[2]; // Warning, division by zero could happen
    Vperp[0] = mysign*Vperp[0];
    Vperp[1] = mysign*Vperp[1];
    Vperp[2] = mysign*Vperp[2];
    
    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1;
    T yp = yp1 / zp1;
    
    if(zp1+Vperp[2] !=0.0){	// adjust only focal plan not parallel to target's xy plane
      T Vpx = (xp1+Vperp[0])/(zp1+Vperp[2]);
      T Vpy = (yp1+Vperp[1])/(zp1+Vperp[2]);
      T Vnorm = sqrt(Vpx*Vpx+Vpy*Vpy);
      if(Vnorm!=0.0){
	// find scale of motion
	// Delta = (r*sin(theta)/(D-rcos(theta)) - r*sin(theta)/(D+rcos(theta)))/2
	// where r is the radius of the circle being projected
	//       D is the distance between camera and circle center
	//       theta is the angle between D vector an target xy plane
	Vpx = Vpx/Vnorm;
	Vpy = Vpy/Vnorm;
	T D = sqrt(xp1*xp1 + yp1*yp1 + zp1*zp1);
	T s_theta = (R_TtoC[6]*xp1 + R_TtoC[7]*yp1 + R_TtoC[8]*zp1)/D;
	T c_theta = sqrt( T(1.0) - s_theta*s_theta);
	T r = T(circle_diameter/2.0);
	T Delta = r*s_theta*(T(1.0)/(D-r*c_theta) - T(1.0)/(D+r*c_theta))/T(2.0);
	xp = xp + Delta*Vpx;
	yp = yp + Delta*Vpy;
      }
    }
    
    /** perform projection using focal length and camera center into image plane */
    resid[0] = fx * xp + cx - ox;
    resid[1] = fy * yp + cy - oy;
  }

  template<typename T> inline void cameraPntResidual(T point[3], T fx, T fy, T cx, T cy, T ox, T oy, T resid[2]);
  template<typename T> inline void cameraPntResidual(T point[3], T fx, T fy, T cx, T cy, T ox, T oy, T resid[2])
  {
    T xp1 = point[0];
    T yp1 = point[1];
    T zp1 = point[2];

    /** scale into the image plane by distance away from camera */
    T xp = xp1 / zp1;
    T yp = yp1 / zp1;

    /** perform projection using focal length and camera center into image plane */
    resid[0] = fx * xp + cx - ox;
    resid[1] = fy * yp + cy - oy;
  }

  class CameraReprjErrorWithDistortion
  {
  public:
    CameraReprjErrorWithDistortion(double ob_x, double ob_y) :
      ox_(ob_x), oy_(ob_y)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* c_p2, /** intrinsic parameters */
                    const T* point, /** point being projected, has 3 parameters */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T fx, fy, cx, cy, k1, k2, k3, p1, p2;
      extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      T camera_point[3]; /** point in camera coordinates*/

      /** transform point into camera coordinates */
      transformPoint(camera_aa, camera_tx, point, camera_point);

      /** compute project point into image plane and compute residual */
      cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, T(ox_), T(oy_),  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y)
    {
      return (new ceres::AutoDiffCostFunction<CameraReprjErrorWithDistortion, 2, 6, 9, 3>(new CameraReprjErrorWithDistortion(o_x, o_y)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
  };

  // reprojection error of a single simple point observed by a camera with lens distortion
  // typically used for intrinsic calibration
  // location of target is known, and fixed, point's location within target is also known
  // both extrinsic and intrinsic parameters of camera are being computed
  class CameraReprjErrorWithDistortionPK
  {
  public:
    CameraReprjErrorWithDistortionPK(double ob_x, double ob_y, Point3d point) :
      ox_(ob_x), oy_(ob_y), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* c_p2, /** intrinsic parameters */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T fx, fy, cx, cy, k1, k2, k3, p1, p2;
      extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      T camera_point[3];

      /** transform point into camera coordinates */
      transformPoint(camera_aa, camera_tx, point_, camera_point);

      /** compute project point into image plane and compute residual */
      cameraPntResidualDist(camera_point, k1, k2, k3, p1, p2, fx, fy, cx, cy, T(ox_), T(oy_),  resid);
      
      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<CameraReprjErrorWithDistortionPK, 2, 6, 9>(new CameraReprjErrorWithDistortionPK(o_x, o_y, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    Point3d point_; /*! location of point in target coordinates */
  };

  // reprojection error of a single simple point observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  class CameraReprjError
  {
  public:
    CameraReprjError(double ob_x, double ob_y, double fx, double fy, double cx, double cy) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* point, /** point being projected, yes this is has 3 parameters */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(camera_aa, camera_tx, point, camera_point);

      /** compute project point into image plane and compute residual */
      cameraPntResidual(camera_point, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_),  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, 
				       const double fx, const double fy, 
				       const double cx, const double cy)
    {
      return (new ceres::AutoDiffCostFunction<CameraReprjError, 2, 6, 3>(new CameraReprjError(o_x, o_y, fx, fy, cx, cy)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
  };

  // reprojection error of a single simple point observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  class CameraReprjErrorPK
  {
  public:
    CameraReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Point3d point) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(camera_aa, camera_tx, point_, camera_point);

      /** compute project point into image plane and compute residual */
      cameraPntResidual(camera_point, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_),  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, 
				       const double fx, const double fy, 
				       const double cx, const double cy,
				       const Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<CameraReprjErrorPK, 2, 6>(new CameraReprjErrorPK(o_x, o_y, fx, fy, cx, cy, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    Point3d point_; /*! location of point in target coordinates */

  };

  // reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  //
  class TargetCameraReprjError
  {
  public:
    TargetCameraReprjError(double ob_x, double ob_y, double fx, double fy, double cx, double cy) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* const c_p2, /** 6Dof transform of target points into world frame */
                    const T* const point, /** point described in target frame */
                    T* resid) const
    {

      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      /** compute project point into image plane and compute residual */
      cameraPntResidual(camera_point, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_),  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y,
				       const double fx, const double fy, 
				       const double cx, const double cy)

    {
      return (new ceres::AutoDiffCostFunction<TargetCameraReprjError, 2, 6, 6, 3>(new TargetCameraReprjError(o_x, o_y, fx, fy, cx, cy)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
  };

  // reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  //
  class TargetCameraReprjErrorPK
  {
  public:
    TargetCameraReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Point3d point) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* const c_p2, /** 6Dof transform of target points into world frame */
                    T* resid) const
    {

      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point_, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      /** compute project point into image plane and compute residual */
      cameraPntResidual(camera_point, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_),  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y,
				       const double fx, const double fy, 
				       const double cx, const double cy,
				       const Point3d point)

    {
      return (new ceres::AutoDiffCostFunction<TargetCameraReprjErrorPK, 2, 6, 6>(new TargetCameraReprjErrorPK(o_x, o_y, fx, fy, cx, cy, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    Point3d point_; /*! location of point in target coordinates */
  };

  // reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  //
  class LinkTargetCameraReprjError
  {
  public:
    LinkTargetCameraReprjError(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d link_pose) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* const c_p2, /** 6Dof transform of target points into world frame */
                    const T* const point, /** point described in target frame that is being seen */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T link_point[3]; /** point in link coordinates */
      T world_point[3]; /** point in world coordinates */
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, link_point);
      poseTransformPoint(link_pose_, link_point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      /** compute project point into image plane and compute residual */
      cameraPntResidual(camera_point,  T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_),  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double ox, const double oy,
				       const double fx, const double fy, 
				       const double cx, const double cy,
				       Pose6d pose)

    {
      return (new ceres::AutoDiffCostFunction<LinkTargetCameraReprjError, 2, 6, 6, 3>(new LinkTargetCameraReprjError(ox, oy, fx, fy, cx, cy, pose)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    Pose6d link_pose_; /*!< transform from world to link coordinates */ 
  };

  // reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  //
  class LinkTargetCameraReprjErrorPK
  {
  public:
    LinkTargetCameraReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d link_pose, Point3d point) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* const c_p2, /** 6Dof transform of target points into world frame */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T link_point[3]; /** point in link coordinates */
      T world_point[3];/** point in worls coordinates */			   
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point_, link_point);
      poseTransformPoint(link_pose_, link_point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      /** compute project point into image plane and compute residual */
      cameraPntResidual(camera_point, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_),  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y,
				       const double fx, const double fy, 
				       const double cx, const double cy,
				       Pose6d pose, Point3d point)

    {
      return (new ceres::AutoDiffCostFunction<LinkTargetCameraReprjErrorPK, 2, 6, 6>( new LinkTargetCameraReprjErrorPK(o_x, o_y, fx, fy, cx, cy, pose, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    Pose6d link_pose_; /*!< transform from world to link coordinates */ 
    Point3d point_; /*! location of point in target coordinates */
  };

  // reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  //
  class LinkCameraTargetReprjError
  {
  public:
    LinkCameraTargetReprjError(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d link_pose) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose)
    {
      link_posei_ = link_pose_.getInverse();
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* const c_p2, /** 6Dof transform of target points into world frame */
                    const T* const point, /** point described in target frame that is being seen */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T link_point[3]; /** point in link coordinates */
      T camera_point[3];/** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, world_point);
      poseTransformPoint(link_posei_, world_point, link_point);
      transformPoint(camera_aa, camera_tx, link_point, camera_point);

      /** compute project point into image plane and compute residual */
      cameraPntResidual(camera_point, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_),  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double ox, const double oy,
				       const double fx, const double fy, 
				       const double cx, const double cy,
				       Pose6d pose)

    {
      return (new ceres::AutoDiffCostFunction<LinkCameraTargetReprjError, 2, 6, 6, 3>(new LinkCameraTargetReprjError(ox, oy, fx, fy, cx, cy, pose)));
    }
    double ox_; /*!< observed x location of object in image */
    double oy_; /*!< observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    Pose6d link_pose_; /*!< transform from world to link coordinates */ 
    Pose6d link_posei_; /*!< transform from link to world coordinates */ 
  };

  // reprojection error of a single point attatched to a target observed by a camera with NO lens distortion
  // should subscribe to a rectified image when using the error function
  //
  class LinkCameraTargetReprjErrorPK
  {
  public:
    LinkCameraTargetReprjErrorPK(double ob_x, double ob_y, double fx, double fy, double cx, double cy, Pose6d link_pose, Point3d point) :
      ox_(ob_x), oy_(ob_y), fx_(fx), fy_(fy), cx_(cx), cy_(cy), link_pose_(link_pose), point_(point)
    {
      link_posei_ = link_pose_.getInverse();
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters */
                    const T* const c_p2, /** 6Dof transform of target points into world frame */
                    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T link_point[3]; /** point in link coordinates */
      T camera_point[3]; /** point in camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point_, world_point);
      poseTransformPoint(link_posei_, world_point, link_point);
      transformPoint(camera_aa, camera_tx, link_point, camera_point);

      /** compute project point into image plane and compute residual */
      cameraPntResidual(camera_point, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_),  resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y,
				       const double fx, const double fy, 
				       const double cx, const double cy,
				       Pose6d pose, Point3d pnt)

    {
      return (new ceres::AutoDiffCostFunction<LinkCameraTargetReprjErrorPK, 2, 6, 6>( new LinkCameraTargetReprjErrorPK(o_x, o_y, fx, fy, cx, cy, pose, pnt)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double fx_; /*!< known focal length of camera in x */
    double fy_; /*!< known focal length of camera in y */
    double cx_; /*!< known optical center of camera in x */
    double cy_; /*!< known optical center of camera in y */
    Pose6d link_pose_; /*!< transform from world to link coordinates */ 
    Pose6d link_posei_; /*!< transform from link to world coordinates */ 
    Point3d point_; /*! location of point in target coordinates */
  };

  // WARNING, ASSUMES CIRCLE LIES IN XY PLANE OF WORLD
  class  CircleCameraReprjErrorWithDistortion
  {
  public:
    CircleCameraReprjErrorWithDistortion(double ob_x, double ob_y, double c_dia) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
		    const T* const c_p2, /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9]*/
		    const T* const point, /** point described in target frame that is being seen [3]*/
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T fx, fy, cx, cy, k1, k2, k3, p1, p2;      
      extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      T camera_point[3];  /** point in camera coordinates*/       
      T R_TtoC[9];

      /** transform point into camera coordinates */
      transformPoint(camera_aa, camera_tx, point, camera_point);
      
      // find rotation from target to camera frame
      ceres::AngleAxisToRotationMatrix(camera_aa,R_TtoC);

      /** compute project point into image plane and compute residual */
      cameraCircResidualDist(camera_point, T(circle_diameter_), R_TtoC, k1, k2, k3, p1, p2, fx, fy, cx, cy, T(ox_), T(oy_), resid);

      return true;
    } /** end of operator() */
    
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia)
    {
      return (new ceres::AutoDiffCostFunction<CircleCameraReprjErrorWithDistortion, 2, 6, 9, 3>(new CircleCameraReprjErrorWithDistortion(o_x, o_y, c_dia)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
  };

  // WARNING, ASSUMES CIRCLE LIES IN XY PLANE OF WORLD
  class  CircleCameraReprjErrorWithDistortionPK
  {
  public:
    CircleCameraReprjErrorWithDistortionPK(double ob_x, double ob_y, double c_dia, Point3d point) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
		    const T* const c_p2, /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9] */
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T fx, fy, cx, cy, k1, k2, k3, p1, p2;      
      extractCameraIntrinsics(c_p2, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      T camera_point[3]; /** point in camera coordinates */
      T R_TtoC[9]; /** rotation from target to camera coordinates 

      /** find point in camera coordinates */
      transformPoint(camera_aa, camera_tx, point_, camera_point);

      // find rotation from target to camera coordinates
      ceres::AngleAxisToRotationMatrix(camera_aa, R_TtoC);

      /** compute project point into image plane and compute residual */
      cameraCircResidualDist(camera_point, T(circle_diameter_), R_TtoC, k1, k2, k3, p1, p2, fx, fy, cx, cy, T(ox_), T(oy_), resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<CircleCameraReprjErrorWithDistortionPK, 2, 6, 9>(											       new CircleCameraReprjErrorWithDistortionPK(o_x, o_y, c_dia, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    Point3d point_;
  };

  class  CircleCameraReprjError
  {
  public:
    CircleCameraReprjError(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
		    const T* const point, /** point described in target frame that is being seen [3]*/
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T camera_point[3];  /** point in camera coordinates */ 
      T R_TtoC[9]; /** rotation from target to camera coordinates */

      /** transform point into camera coordinates */
      transformPoint(camera_aa, camera_tx, point, camera_point);
      
      // find rotation from target to camera coordinates
      ceres::AngleAxisToRotationMatrix(camera_aa,R_TtoC);

      /** compute project point into image plane and compute residual */
      cameraCircResidual(camera_point, T(circle_diameter_), R_TtoC, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_), resid);

      return true;
    } /** end of operator() */
    
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, 
				       const double fx,  const double fy, const double cx, const double cy)
    {
      return (new ceres::AutoDiffCostFunction<CircleCameraReprjError, 2, 6, 3>(new CircleCameraReprjError(o_x, o_y, c_dia, fx, fy, cx, cy)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; /** diameter of circle being observed */
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */
  };

  class  CircleCameraReprjErrorPK
  {
  public:
    CircleCameraReprjErrorPK(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy,  Point3d point) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      T camera_point[3]; /* point in camera coordinates */
      T R_TtoC[9]; /** rotation from target to camera coordinates */

      /** rotate and translate point into camera coordinates*/
      transformPoint(camera_aa, camera_tx, point_, camera_point);

      // find rotation from target to camera coordinates
      ceres::AngleAxisToRotationMatrix(camera_aa, R_TtoC);

      /** compute project point into image plane and compute residual */
      cameraCircResidual(camera_point, T(circle_diameter_), R_TtoC, T(fx_),T(fy_), T(cx_), T(cy_), T(ox_), T(oy_), resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
  static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, 
				     const double fx, const double fy, const double cx, const double cy,
				     Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<CircleCameraReprjErrorPK, 2, 6>(new CircleCameraReprjErrorPK(o_x, o_y, c_dia, fx, fy, cx, cy, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */
    Point3d point_; /** location of point in target coordinates */
  };

  class  CircleTargetCameraReprjErrorWithDistortion
  {
  public:
    CircleTargetCameraReprjErrorWithDistortion(double ob_x, double ob_y, double c_dia) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
		    const T* const c_p2, /** 6Dof transform of target into world frame [6]*/
		    const T* const c_p3, /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9]*/
		    const T* const point, /** point described in target frame that is being seen [3]*/
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T fx, fy, cx, cy, k1, k2, k3, p1, p2;      
      extractCameraIntrinsics(c_p3, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      T world_point[3]; /** point in world coordinates */
      T camera_point[3];  /** point in camera coordinates*/ 
      T R_WtoC[9]; // rotation from world to camera coordinates
      T R_TtoW[9]; // rotation from target to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** compute necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);  
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      // find rotation from target to camera coordinates
      rotationProduct(R_WtoC, R_TtoW, R_TtoC); // R_WtoC*R_TtoW = R_TtoC

      /** compute project point into image plane and compute residual */
      cameraCircResidualDist(camera_point, T(circle_diameter_), R_TtoC, k1, k2, k3, p1, p2, fx, fy, cx, cy, T(ox_), T(oy_), resid);

      return true;
    } /** end of operator() */
    
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia)
    {
      return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjErrorWithDistortion, 2, 6, 6, 9, 3>(new CircleTargetCameraReprjErrorWithDistortion(o_x, o_y, c_dia)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
  };

  class  CircleTargetCameraReprjErrorWithDistortionPK
  {
  public:
    CircleTargetCameraReprjErrorWithDistortionPK(double ob_x, double ob_y, double c_dia, Point3d point) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
		    const T* const c_p2,  /** 6Dof transform of target into world frame [6] */
		    const T* const c_p3, /** intrinsic parameters of camera fx,fy,cx,cy,k1,k2,k2,p1,p2 [9] */
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T fx, fy, cx, cy, k1, k2, k3, p1, p2;      
      extractCameraIntrinsics(c_p3, fx, fy, cx, cy, k1, k2, k3, p1, p2);
      T world_point[3]; /** point in world coordinates */
      T camera_point[3]; /** point in world coordinates */
      T R_WtoC[9]; // rotation from world to camera coordinates
      T R_TtoW[9]; // rotation from target to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** compute necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);  
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point_, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      // find rotation from target to camera coordinates
      rotationProduct(R_WtoC, R_TtoW, R_TtoC); // R_WtoC*R_TtoW = R_TtoC

      /** compute project point into image plane and compute residual */
      cameraCircResidualDist(camera_point, T(circle_diameter_), R_TtoC, k1, k2, k3, p1, p2, fx, fy, cx, cy, T(ox_), T(oy_), resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjErrorWithDistortionPK, 2, 6, 6, 9>(											       new CircleTargetCameraReprjErrorWithDistortionPK(o_x, o_y, c_dia, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    Point3d point_;
  };

  class  CircleTargetCameraReprjError
  {
  public:
    CircleTargetCameraReprjError(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
		    const T* const c_p2, /** 6Dof transform of target into world frame [6]*/
		    const T* const point, /** point described in target frame that is being seen [3]*/
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T camera_point[3];  /** point in camera coordinates*/ 
      T R_WtoC[9]; // rotation from world to camera coordinates
      T R_TtoW[9]; // rotation from target to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** compute necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);  
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);
      
      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);
      
      /** find rotation from target to camera coordinates */
      rotationProduct(R_WtoC, R_TtoW, R_TtoC); // R_WtoC*R_TtoW = R_TtoC
      
      /** compute project point into image plane and compute residual */
      cameraCircResidual(camera_point, T(circle_diameter_), R_TtoC, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_), resid);

      return true;
    } /** end of operator() */
    
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, 
				       const double fx, const double fy, const double cx, const double cy)
    {
      return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjError, 2, 6, 6, 3>(new CircleTargetCameraReprjError(o_x, o_y, c_dia, fx, fy, cx, cy)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */
  };

  class  CircleTargetCameraReprjErrorPK
  {
  public:
    CircleTargetCameraReprjErrorPK(double ob_x, double ob_y, double c_dia, double fx, double fy, double cx, double cy, Point3d point) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), fx_(fx), fy_(fy), cx_(cx), cy_(cy), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
		    const T* const c_p2,  /** 6Dof transform of target into world frame [6] */
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T camera_point[3]; /** point in camera coordinates */
      T R_WtoC[9]; // rotation from world to camera coordinates
      T R_TtoW[9]; // rotation from target to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** compute necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);  
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

      /** transform point into camera frame */
      transformPoint(target_aa, target_tx, point_, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      /** find rotation from target to camera coordinates */
      rotationProduct(R_WtoC, R_TtoW, R_TtoC); // R_WtoC*R_TtoW = R_TtoC
      
      /** compute project point into image plane and compute residual */
      cameraCircResidual(camera_point, T(circle_diameter_), R_TtoC, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_), resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia,
				       const double fx, const double fy, const double cx, const double cy,
				       Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<CircleTargetCameraReprjErrorPK, 2, 6, 6>(											       new CircleTargetCameraReprjErrorPK(o_x, o_y, c_dia, fx, fy, cx, cy, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */
    Point3d point_;
  };

  class  LinkCircleTargetCameraReprjError
  {
  public:
    LinkCircleTargetCameraReprjError(double ob_x, double ob_y, double c_dia, Pose6d link_pose) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), link_pose_(link_pose)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
		    const T* const c_p2, /** 6Dof transform of target into world frame [6]*/
		    const T* const point, /** point described in target frame that is being seen [3]*/
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T link_point[3]; /** point in link coordinates */
      T world_point[3]; /** point in world coordinates */
      T camera_point[3];  /** point in camera coordinates*/ 
      T R_WtoC[9]; // rotation from world to camera coordinates
      T R_TtoL[9]; // rotation from target to linkcoordinates
      T R_LtoW[9]; // rotation from link to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)
      T R_LtoC[9]; // rotation from link to camera coordinates

      /** get necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);  
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoL);
      poseRotationMatrix(link_pose_, R_LtoW);

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, link_point);
      poseTransformPoint(link_pose_, link_point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);
      
      /** find rotation from target to camera coordinates */
      rotationProduct(R_WtoC, R_LtoW, R_LtoC);
      rotationProduct(R_LtoC, R_TtoL, R_TtoC); // R_WtoC*R_LtoW*R_TtoL = R_TtoC
      
      /** compute project point into image plane and compute residual */
      cameraCircResidual(camera_point, T(circle_diameter_), R_TtoC, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_), resid);

      return true;
    } /** end of operator() */
    
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, const Pose6d pose)
    {
      return (new ceres::AutoDiffCostFunction<LinkCircleTargetCameraReprjError, 2, 6, 6, 3>(new LinkCircleTargetCameraReprjError(o_x, o_y, c_dia, pose)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; /** diameter of circle being observed */
    Pose6d link_pose_; /** transform from link to world coordinates*/
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */ 
 };

  class  LinkCircleTargetCameraReprjErrorPK
  {
  public:
    LinkCircleTargetCameraReprjErrorPK(double ob_x, double ob_y, double c_dia, Pose6d link_pose, Point3d point) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), link_pose_(link_pose), point_(point)
    {
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
		    const T* const c_p2,  /** 6Dof transform of target into world frame [6] */
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T link_point[3]; /** point in link coordinates */
      T world_point[3];/** point in world coordinates */
      T camera_point[3]; /** point in camera coordinates */
      T R_WtoC[9]; // rotation from world to camera coordinates
      T R_TtoL[9]; // rotation from target to linkcoordinates
      T R_LtoW[9]; // rotation from link to world coordinates
      T R_LtoC[9]; // rotation from link to camera coordinataes
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** compute necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_WtoC);  
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoL);
      poseRotationMatrix(link_pose_,R_LtoW);

      /** transform point into camera frame */
      transformPoint(target_aa, target_tx, point_, link_point);
      poseTransformPoint(link_pose_, link_point, world_point);
      transformPoint(camera_aa, camera_tx, world_point, camera_point);

      /** find rotation from target to camera coordinates */
      rotationProduct(R_WtoC, R_LtoW, R_LtoC);
      rotationProduct(R_LtoC, R_TtoL, R_TtoC); // R_WtoC*R_LtoW*R_TtoL = R_TtoC
      
      /** compute project point into image plane and compute residual */
      cameraCircResidual(camera_point, T(circle_diameter_), R_TtoC, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_), resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, const Pose6d pose, Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<LinkCircleTargetCameraReprjErrorPK, 2, 6, 6>(											       new LinkCircleTargetCameraReprjErrorPK(o_x, o_y, c_dia, pose, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    Pose6d link_pose_; /** transform from link to world coordinates*/
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */
    Point3d point_; /** point expressed in target coordinates */
  };


  class  LinkCameraCircleTargetReprjError
  {
  public:
    LinkCameraCircleTargetReprjError(double ob_x, double ob_y, double c_dia, Pose6d link_pose) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), link_pose_(link_pose)
    {
      link_posei_ = link_pose_.getInverse();
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6]*/
		    const T* const c_p2, /** 6Dof transform of target into world frame [6]*/
		    const T* const point, /** point described in target frame that is being seen [3]*/
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(& c_p2[0]);
      const T *target_tx(& c_p2[3]);
      T world_point[3]; /** point in world coordinates */
      T link_point[3]; /** point in link coordinates */
      T camera_point[3];  /** point in camera coordinates*/ 
      T R_LtoC[9]; // rotation from link to camera coordinates
      T R_WtoL[9]; // rotation from world to link coordinates
      T R_WtoC[9]; // rotation from world to camera coordinates, and intermediate transform
      T R_TtoW[9]; // rotation from target to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** compute necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_LtoC);  
      poseRotationMatrix(link_pose_, R_WtoL);
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);
      
      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, world_point);
      poseTransformPoint(link_posei_, world_point, link_point);
      transformPoint(camera_aa, camera_tx, link_point, camera_point);
      
      /** find rotation from target to camera coordinates */
      rotationProduct(R_LtoC,R_WtoL, R_WtoC);
      rotationProduct(R_WtoC, R_TtoW, R_TtoC); 
      
      /** compute project point into image plane and compute residual */
      cameraCircResidual(camera_point, T(circle_diameter_), R_TtoC, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_), resid);

      return true;
    } /** end of operator() */
    
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, const Pose6d pose)
    {
      return (new ceres::AutoDiffCostFunction<LinkCameraCircleTargetReprjError, 2, 6, 6, 3>(new LinkCameraCircleTargetReprjError(o_x, o_y, c_dia, pose)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; /** diameter of circle being observed */
    Pose6d link_pose_; /** transform from link to world coordinates*/
    Pose6d link_posei_; /** transform from world to link coordinates*/
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */

  };

  class  LinkCameraCircleTargetReprjErrorPK
  {
  public:
    LinkCameraCircleTargetReprjErrorPK(double ob_x, double ob_y, double c_dia, Pose6d link_pose, Point3d point) :
      ox_(ob_x), oy_(ob_y), circle_diameter_(c_dia), link_pose_(link_pose), point_(point)
    {
      link_posei_ = link_pose_.getInverse();
    }

    template<typename T>
    bool operator()(const T* const c_p1, /** extrinsic parameters [6] */
		    const T* const c_p2,  /** 6Dof transform of target into world frame [6] */
		    T* resid) const
    {
      const T *camera_aa(&c_p1[0]);
      const T *camera_tx(&c_p1[3]);
      const T *target_aa(&c_p2[0]);
      const T *target_tx(&c_p2[3]);
      T point[3]; /** point in target coordinates */
      T world_point[3]; /** point in world coordinates */
      T link_point[3]; /** point in link coordinates */
      T camera_point[3];  /** point in camera coordinates*/ 
      T R_LtoC[9]; // rotation from link to camera coordinates
      T R_WtoL[9]; // rotation from world to link coordinates
      T R_WtoC[9]; // rotation from world to camera coordinates, and intermediate transform
      T R_TtoW[9]; // rotation from target to world coordinates
      T R_TtoC[9];  // rotation from target to camera coordinates (assume circle lies in x-y plane of target coordinates)

      /** get necessary rotation matrices */
      ceres::AngleAxisToRotationMatrix(camera_aa, R_LtoC);  
      poseRotationMatrix(link_posei_,R_WtoL);
      ceres::AngleAxisToRotationMatrix(target_aa, R_TtoW);

      /** transform point into camera coordinates */
      transformPoint(target_aa, target_tx, point, world_point);
      poseTransformPoint(link_posei_, world_point, link_point);
      transformPoint(camera_aa, camera_tx, link_point, camera_point);

      /** find rotation from target to camera coordinates */
      rotationProduct(R_LtoC, R_WtoL, R_WtoC);
      rotationProduct(R_WtoC, R_TtoW, R_TtoC); 

      /** compute project point into image plane and compute residual */
      cameraCircResidual(camera_point, T(circle_diameter_), R_TtoC, T(fx_), T(fy_), T(cx_), T(cy_), T(ox_), T(oy_), resid);

      return true;
    } /** end of operator() */

    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(const double o_x, const double o_y, const double c_dia, const Pose6d pose, Point3d point)
    {
      return (new ceres::AutoDiffCostFunction<LinkCameraCircleTargetReprjErrorPK, 2, 6, 6>(											       new LinkCameraCircleTargetReprjErrorPK(o_x, o_y, c_dia, pose, point)));
    }
    double ox_; /** observed x location of object in image */
    double oy_; /** observed y location of object in image */
    double circle_diameter_; //** diameter of circle being observed */
    Pose6d link_pose_; /** transform from link to world coordinates*/
    Pose6d link_posei_; /** transform from world to link coordinates*/
    double fx_; /** focal length of camera in x (pixels) */
    double fy_; /** focal length of camera in y (pixels) */
    double cx_; /** focal center of camera in x (pixels) */
    double cy_; /** focal center of camera in y (pixels) */
    Point3d point_; /** point expressed in target coordinates */

  };


  double edist(double cx, double cy, double e0, double e1, double psi, double qx, double qy, double theta)
  {
    double c_psi   = cos(psi);
    double s_psi   = sin(psi);
    double c_theta = cos(theta);
    double s_theta = sin(theta);
    double dx = cx + e0 * c_theta * c_psi  - e1 * s_theta * s_psi - qx;
    double dy = cy + e0 * c_theta * s_psi  + e1 * s_theta * c_psi - qy;
    return( sqrt(dx*dx + dy*dy) );
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
			    
      T ellipse_x = cx_ + e0_ * c_theta * c_psi - e1_ * s_theta*s_psi;
      T ellipse_y = cy_ + e0_ * c_theta * s_psi + e1_ * s_theta*c_psi;
      resid[0] = (qx_-ellipse_x)*(qx_-ellipse_x) + (qy_-ellipse_y)*(qy_-ellipse_y);

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

  void printQTasH(double qx, double qy, double qz, double qw, double tx, double ty, double tz)
  {
    double Rs11 = qw * qw + qx * qx - qy * qy - qz * qz;
    double Rs21 = 2.0 * qx * qy + 2.0 * qw * qz;
    double Rs31 = 2.0 * qx * qz - 2.0 * qw * qy;

    double Rs12 = 2.0 * qx * qy - 2.0 * qw * qz;
    double Rs22 = qw * qw - qx * qx + qy * qy - qz * qz;
    double Rs32 = 2.0 * qy * qz + 2.0 * qw * qx;

    double Rs13 = 2.0 * qx * qz + 2.0 * qw * qy;
    double Rs23 = 2.0 * qy * qz - 2.0 * qw * qx;
    double Rs33 = qw * qw - qx * qx - qy * qy + qz * qz;

    printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", Rs11, Rs12, Rs13, tx);
    printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", Rs21, Rs22, Rs23, ty);
    printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", Rs31, Rs32, Rs33, tz);
    printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);
  }

  void printAATasH(double x, double y, double z, double tx, double ty, double tz)
  {
    double R[9];
    double aa[3];
    aa[0] = x;
    aa[1] = y;
    aa[2] = z;
    ceres::AngleAxisToRotationMatrix(aa, R);
    printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[0], R[3], R[6], tx);
    printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[1], R[4], R[7], ty);
    printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[2], R[5], R[8], tz);
    printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);
  }

  void printAATasHI(double x, double y, double z, double tx, double ty, double tz)
  {
    double R[9];
    double aa[3];
    aa[0] = x;
    aa[1] = y;
    aa[2] = z;
    ceres::AngleAxisToRotationMatrix(aa, R);
    double ix = -(tx * R[0] + ty * R[1] + tz * R[2]);
    double iy = -(tx * R[3] + ty * R[4] + tz * R[5]);
    double iz = -(tx * R[6] + ty * R[7] + tz * R[8]);
    printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[0], R[1], R[2], ix);
    printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[3], R[4], R[5], iy);
    printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", R[6], R[7], R[8], iz);
    printf("%6.3lf %6.3lf %6.3lf %6.3lf\n", 0.0, 0.0, 0.0, 1.0);
  }

  void printAAasEuler(double x, double y, double z)
  {
    double R[9];
    double aa[3];
    aa[0] = x;
    aa[1] = y;
    aa[2] = z;
    ceres::AngleAxisToRotationMatrix(aa, R);
    double rx = atan2(R[7], R[8]);
    double ry = atan2(-R[6], sqrt(R[7] * R[7] + R[8] * R[8]));
    double rz = atan2(R[3], R[0]);
    printf("rpy = %8.4f %8.4f %8.4f\n", rx, ry, rz);
  }

  void printCameraParameters(CameraParameters C, std::string words)
  {
    printf("%s\n", words.c_str());
    printf("Camera to World Transform:\n");
    printAATasHI(C.angle_axis[0], C.angle_axis[1], C.angle_axis[2], C.position[0], C.position[1], C.position[2]);

    printf("World to Camera\n");
    printAATasH(C.angle_axis[0], C.angle_axis[1], C.angle_axis[2], C.position[0], C.position[1], C.position[2]);
    printAAasEuler(C.angle_axis[0], C.angle_axis[1], C.angle_axis[2]);
    printf("fx = %8.3lf fy = %8.3lf\n", C.focal_length_x, C.focal_length_y);
    printf("k1 = %8.3lf k2 = %8.3lf k3 = %8.3lf\n", C.distortion_k1, C.distortion_k2, C.distortion_k3);
    printf("p1 = %8.3lf p2 = %8.3lf\n", C.distortion_p1, C.distortion_p2);
    printf("cx = %8.3lf cy = %8.3lf\n", C.center_x, C.center_y);
  }
} // end of namespace
