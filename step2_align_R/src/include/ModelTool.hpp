/*
This code is the extra evaluation tool of our paper "TextSLAM: Visual SLAM with Semantic Planar Text Features."

Author: Boying Li   < LeeBY2016@outlook.com >

If you use any code of this repo in your research, please cite our papers:
[1] Li Boying, Zou Danping, et al. "TextSLAM: Visual SLAM with Semantic Planar Text Features." 
[2] Li Boying, Zou Danping, et al. "TextSLAM: Visual slam with planar text features." 
*/

#ifndef MODELTOOL_HPP_
#define MODELTOOL_HPP_
#include <iostream>
#include "ceres/ceres.h"
#include <opencv2/core.hpp>
#include <math.h>
using namespace std;
using namespace ceres;

// for Sim3 ---------
inline Eigen::Vector3d deltaR(const Eigen::Matrix3d& R)
{
  Eigen::Vector3d v;
  v(0)=R(2,1)-R(1,2);
  v(1)=R(0,2)-R(2,0);
  v(2)=R(1,0)-R(0,1);
  return v;
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d&v)
{
  Eigen::Matrix3d m;
  m.fill(0.);
  m(0,1)  = -v(2);
  m(0,2)  =  v(1);
  m(1,2)  = -v(0);
  m(1,0)  =  v(2);
  m(2,0) = -v(1);
  m(2,1) = v(0);
  return m;
}

inline Eigen::Matrix <double, 7, 1> logSim3(const Eigen::Quaterniond &r, const Eigen::Vector3d &t, const double &s)
{
    Eigen::Matrix <double, 7, 1> res;

    double sigma = std::log(s);

    Eigen::Vector3d omega;
    Eigen::Vector3d upsilon;

    Eigen::Matrix3d R = r.toRotationMatrix();
    double d =  0.5*(R(0,0)+R(1,1)+R(2,2)-1);

    Eigen::Matrix3d Omega;

    double eps = 0.00001;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    double A,B,C;
    if (fabs(sigma)<eps)
    {
      C = 1;
      if (d>1-eps)
      {
        omega=0.5*deltaR(R);
        Omega = skew(omega);
        A = 1./2.;
        B = 1./6.;
      }
      else
      {
        double theta = acos(d);
        double theta2 = theta*theta;
        omega = theta/(2*sqrt(1-d*d))*deltaR(R);
        Omega = skew(omega);
        A = (1-cos(theta))/(theta2);
        B = (theta-sin(theta))/(theta2*theta);
      }
    }
    else
    {
      C=(s-1)/sigma;
      if (d>1-eps)
      {

        double sigma2 = sigma*sigma;
        omega=0.5*deltaR(R);
        Omega = skew(omega);
        A = ((sigma-1)*s+1)/(sigma2);
        B = ((0.5*sigma2-sigma+1)*s)/(sigma2*sigma);
      }
      else
      {
        double theta = acos(d);
        omega = theta/(2*sqrt(1-d*d))*deltaR(R);
        Omega = skew(omega);
        double theta2 = theta*theta;
        double a=s*sin(theta);
        double b=s*cos(theta);
        double c=theta2 + sigma*sigma;
        A = (a*sigma+ (1-b)*theta)/(theta*c);
        B = (C-((b-1)*sigma+a*theta)/(c))*1./(theta2);
      }
    }

    Eigen::Matrix3d W = A*Omega + B*Omega*Omega + C*I;

    upsilon = W.lu().solve(t);

    for (int i=0; i<3; i++)
      res[i] = omega[i];

     for (int i=0; i<3; i++)
      res[i+3] = upsilon[i];

    res[6] = sigma;

    return res;

}


#endif // MODELTOOL_HPP_
