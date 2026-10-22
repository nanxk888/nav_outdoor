// Navigation adapter for the retained FAST-LIVO2 state; not a replacement estimator.
#pragma once
#include "common_lib.h"
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Eigenvalues>
#include <stdexcept>

namespace d1h_navigation {
inline M3D cross_matrix(const V3D &v) {
  M3D m; m << 0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0; return m;
}

inline nav_msgs::msg::Odometry make_odometry(const StatesGroup &s,
                                            const V3D &gyro, double gyro_variance) {
  if (!s.pos_end.allFinite() || !s.vel_end.allFinite() || !s.rot_end.allFinite() ||
      !s.cov.allFinite() || !gyro.allFinite() || !std::isfinite(gyro_variance) ||
      gyro_variance <= 0 || (s.rot_end.transpose()*s.rot_end-M3D::Identity()).norm()>1e-5 ||
      s.rot_end.determinant()<0.99)
    throw std::runtime_error("Invalid navigation state");
  // State ordering: right-local rotation, world position, exposure,
  // world velocity, gyro bias, acceleration bias, world gravity.
  Eigen::Matrix<double,6,DIM_STATE> pose_j=Eigen::Matrix<double,6,DIM_STATE>::Zero();
  pose_j.block<3,3>(0,3)=M3D::Identity();
  pose_j.block<3,3>(3,0)=s.rot_end;
  Eigen::Matrix<double,6,DIM_STATE> twist_j=Eigen::Matrix<double,6,DIM_STATE>::Zero();
  const V3D velocity=s.rot_end.transpose()*s.vel_end;
  twist_j.block<3,3>(0,7)=s.rot_end.transpose();
  twist_j.block<3,3>(0,0)=cross_matrix(velocity);
  twist_j.block<3,3>(3,10)=-M3D::Identity();
  Eigen::Matrix<double,6,6> pose_cov=pose_j*s.cov*pose_j.transpose();
  Eigen::Matrix<double,6,6> twist_cov=twist_j*s.cov*twist_j.transpose();
  twist_cov.block<3,3>(3,3)+=gyro_variance*M3D::Identity();
  pose_cov=(pose_cov+pose_cov.transpose()).eval()*.5;
  twist_cov=(twist_cov+twist_cov.transpose()).eval()*.5;
  if (Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>>(pose_cov).eigenvalues().minCoeff() < -1e-8 ||
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>>(twist_cov).eigenvalues().minCoeff() < -1e-8)
    throw std::runtime_error("Non-positive navigation covariance");
  nav_msgs::msg::Odometry out;
  out.pose.pose.position.x=s.pos_end.x();out.pose.pose.position.y=s.pos_end.y();out.pose.pose.position.z=s.pos_end.z();
  const Eigen::Quaterniond q(s.rot_end);
  out.pose.pose.orientation.x=q.x();out.pose.pose.orientation.y=q.y();
  out.pose.pose.orientation.z=q.z();out.pose.pose.orientation.w=q.w();
  const V3D omega=gyro-s.bias_g;
  out.twist.twist.linear.x=velocity.x();out.twist.twist.linear.y=velocity.y();out.twist.twist.linear.z=velocity.z();
  out.twist.twist.angular.x=omega.x();out.twist.twist.angular.y=omega.y();out.twist.twist.angular.z=omega.z();
  for(int r=0;r<6;++r) for(int c=0;c<6;++c) {
    out.pose.covariance[r*6+c]=pose_cov(r,c);
    out.twist.covariance[r*6+c]=twist_cov(r,c);
  }
  return out;
}
}  // namespace d1h_navigation
