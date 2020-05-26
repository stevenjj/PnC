#include <PnC/ValkyriePnC/StateMachines/DoubleSupportStand.hpp>
#include <PnC/ValkyriePnC/CtrlArchitectures/ValkyrieControlArchitecture.hpp>

DoubleSupportStand::DoubleSupportStand(const StateIdentifier state_identifier_in, ValkyrieControlArchitecture* _ctrl_arch, RobotSystem* _robot) : 
               StateMachine(state_identifier_in, _robot) {
  myUtils::pretty_constructor(2, "SM: Double Support Stand");

  // Set Pointer to Control Architecture
  val_ctrl_arch_ = ((ValkyrieControlArchitecture*) _ctrl_arch);
  taf_container_ = val_ctrl_arch_->taf_container_;

  // Get State Provider
  sp_ = ValkyrieStateProvider::getStateProvider(robot_);


  // To Do: Belongs to trajectory manager.
  // COM
  ini_com_pos_ = Eigen::VectorXd::Zero(3); 
  des_com_pos_ = Eigen::VectorXd::Zero(3);
  des_com_vel_ = Eigen::VectorXd::Zero(3);
  des_com_acc_ = Eigen::VectorXd::Zero(3);
  com_pos_dev_ = Eigen::VectorXd::Zero(3);
}

DoubleSupportStand::~DoubleSupportStand(){
}


void DoubleSupportStand::firstVisit(){
  ctrl_start_time_ = sp_->curr_time;
  // =========================================================================
  // Set CoM Trajectory
  // =========================================================================
  ini_com_pos_ = robot_->getCoMPosition();
  des_com_pos_ = ini_com_pos_ + com_pos_dev_;
  _SetBspline(ini_com_pos_,des_com_pos_);

  ini_pelvis_quat_ = Eigen::Quaternion<double>(robot_->getBodyNodeIsometry(ValkyrieBodyNode::pelvis).linear());

  // =========================================================================
  // Pelvis Ori Task: Maintain Starting Orientation
  // =========================================================================
  Eigen::VectorXd des_pelvis_quat = Eigen::VectorXd::Zero(4);
  des_pelvis_quat << ini_pelvis_quat_.w(),ini_pelvis_quat_.x(), ini_pelvis_quat_.y(),
                      ini_pelvis_quat_.z();
  taf_container_->pelvis_ori_task_->updateDesired(des_pelvis_quat, Eigen::VectorXd::Zero(3),Eigen::VectorXd::Zero(3));

  // =========================================================================
  // Set Angular Momentum Tasks
  // =========================================================================
  Eigen::VectorXd zero3 = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_ang_momentum = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_ang_momentum_rate = Eigen::VectorXd::Zero(3);
  taf_container_->ang_momentum_task_->updateDesired(zero3, des_ang_momentum, des_ang_momentum_rate);

  // =========================================================================
  // Joint Pos Task
  // =========================================================================
  Eigen::VectorXd jpos_des = sp_->jpos_ini;
  taf_container_->upper_body_task_->updateDesired(jpos_des.tail(taf_container_->upper_body_joint_indices_.size()),
                                                  Eigen::VectorXd::Zero(Valkyrie::n_adof),
                                                  Eigen::VectorXd::Zero(Valkyrie::n_adof));

}

void DoubleSupportStand::_taskUpdate(){
  // =========================================================================
  // Update CoM Task
  // =========================================================================
  _GetBsplineTrajectory();
  for (int i = 0; i < 3; ++i) {
      (sp_->com_pos_des)[i] = des_com_pos_[i];
      (sp_->com_vel_des)[i] = des_com_vel_[i];
  }
  taf_container_->com_task_->updateDesired(des_com_pos_, des_com_vel_, des_com_acc_);

  // =========================================================================
  // Set Foot Motion Tasks
  // =========================================================================
  Eigen::VectorXd foot_pos_des(3); foot_pos_des.setZero();
  Eigen::VectorXd foot_vel_des(3); foot_vel_des.setZero();    
  Eigen::VectorXd foot_acc_des(3); foot_acc_des.setZero();    

  Eigen::VectorXd foot_ori_des(4); foot_ori_des.setZero();
  Eigen::VectorXd foot_ang_vel_des(3); foot_ang_vel_des.setZero();    
  Eigen::VectorXd foot_ang_acc_des(3); foot_ang_acc_des.setZero();

  // Set Right Foot Task
  foot_pos_des = robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::rightCOP_Frame).translation();
  Eigen::Quaternion<double> rfoot_ori_act(robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::rightCOP_Frame).linear());
  foot_ori_des[0] = rfoot_ori_act.w();
  foot_ori_des[1] = rfoot_ori_act.x();
  foot_ori_des[2] = rfoot_ori_act.y();
  foot_ori_des[3] = rfoot_ori_act.z();

  taf_container_->rfoot_center_pos_task_->updateDesired(foot_pos_des, foot_vel_des, foot_acc_des);
  taf_container_->rfoot_center_ori_task_->updateDesired(foot_ori_des, foot_ang_vel_des, foot_ang_acc_des);

  // Set Left Foot Task
  foot_pos_des = robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::leftCOP_Frame).translation();
  Eigen::Quaternion<double> lfoot_ori_act(robot_->getBodyNodeCoMIsometry(ValkyrieBodyNode::leftCOP_Frame).linear());
  foot_ori_des[0] = lfoot_ori_act.w();
  foot_ori_des[1] = lfoot_ori_act.x();
  foot_ori_des[2] = lfoot_ori_act.y();
  foot_ori_des[3] = lfoot_ori_act.z();

  taf_container_->lfoot_center_pos_task_->updateDesired(foot_pos_des, foot_vel_des, foot_acc_des);
  taf_container_->lfoot_center_ori_task_->updateDesired(foot_ori_des, foot_ang_vel_des, foot_ang_acc_des);
}

void DoubleSupportStand::oneStep(){  
  state_machine_time_ = sp_->curr_time - ctrl_start_time_;
  _taskUpdate();
}

void DoubleSupportStand::lastVisit(){  
}

bool DoubleSupportStand::endOfState(){  
  return false;
} 

StateIdentifier DoubleSupportStand::getNextState(){
}


void DoubleSupportStand::initialization(const YAML::Node& node){
}

void DoubleSupportStand::_SetBspline(const Eigen::VectorXd st_pos,
                          const Eigen::VectorXd des_pos) {
  // Trajectory Setup
  double init[9];
  double fin[9];
  double** middle_pt = new double*[1];
  middle_pt[0] = new double[3];
  Eigen::Vector3d middle_pos;

  middle_pos = (st_pos + des_pos) / 2.;

  // Initial and final position & velocity & acceleration
  for (int i(0); i < 3; ++i) {
    // Initial
    init[i] = st_pos[i];
    init[i + 3] = 0.;
    init[i + 6] = 0.;
    // Final
    fin[i] = des_pos[i];
    fin[i + 3] = 0.;
    fin[i + 6] = 0.;
    // Middle
    middle_pt[0][i] = middle_pos[i];
  }
  // TEST
  //fin[5] = amplitude_[] * omega_;
  com_traj_.SetParam(init, fin, middle_pt, end_time_/2.0);

  delete[] * middle_pt;
  delete[] middle_pt;
}

void DoubleSupportStand::_GetBsplineTrajectory(){
  double pos[3];
  double vel[3];
  double acc[3];

  com_traj_.getCurvePoint(state_machine_time_, pos);
  com_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
  com_traj_.getCurveDerPoint(state_machine_time_, 2, acc);

  for (int i(0); i < 3; ++i) {
    des_com_pos_[i] = pos[i];
    des_com_vel_[i] = vel[i];
    des_com_acc_[i] = acc[i];
  }
}