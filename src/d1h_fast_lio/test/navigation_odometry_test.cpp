#include "navigation_odometry.h"
#include "voxel_map.h"
#include <iostream>

void require(bool value,const char *message) {
  if(!value) throw std::runtime_error(message);
}
int main() {
  StatesGroup state;
  state.rot_end=Eigen::AngleAxisd(0.7,V3D::UnitZ()).toRotationMatrix();
  state.vel_end=V3D(1.,2.,.1);state.bias_g=V3D(.01,.02,.03);
  state.cov=Eigen::Matrix<double,DIM_STATE,DIM_STATE>::Identity()*.01;
  auto out=d1h_navigation::make_odometry(state,V3D(.1,.2,.3),.02);
  const V3D expected=state.rot_end.transpose()*state.vel_end;
  require(std::abs(out.twist.twist.linear.x-expected.x())<1e-12,"velocity frame");
  require(std::abs(out.twist.twist.angular.z-.27)<1e-12,"gyro bias");
  require(out.pose.covariance[0]>.0 && out.twist.covariance[35]>.02,"nonzero covariance");
  const double eps=1e-7;
  for(int i=0;i<3;++i) {
    V3D delta=V3D::Zero();delta[i]=eps;
    const M3D perturbed=state.rot_end*Eigen::AngleAxisd(eps,V3D::Unit(i)).toRotationMatrix();
    const V3D derivative=(perturbed.transpose()*state.vel_end-expected)/eps;
    require((derivative-d1h_navigation::cross_matrix(expected).col(i)).norm()<1e-5,"rotation/velocity Jacobian");
  }
  bool rejected=false;
  state.cov(3,3)=-1.;
  try {d1h_navigation::make_odometry(state,V3D::Zero(),.02);} catch(...) {rejected=true;}
  require(rejected,"negative covariance rejected");
  state.cov.setIdentity();state.rot_end(0,0)=2.;rejected=false;
  try {d1h_navigation::make_odometry(state,V3D::Zero(),.02);} catch(...) {rejected=true;}
  require(rejected,"invalid rotation rejected");
  VoxelOctoTree tree(2,0,5,50,.0025);
  std::vector<pointWithVar> identical(8);
  for(auto &point:identical) {point.point_w=V3D(1.,1.,1.);point.var.setIdentity();}
  tree.init_plane(identical,tree.plane_ptr_);
  require(!tree.plane_ptr_->is_plane_,"degenerate point cluster rejected without eigen-index crash");
  std::cout<<"PASS: sensor velocity frame, gyro bias, covariance and Jacobian checks\n";
}
