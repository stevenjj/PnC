#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <Utils/IO/DataManager.hpp>

ValkyrieStateProvider* ValkyrieStateProvider::getStateProvider(
    RobotSystem* _robot) {
    static ValkyrieStateProvider state_provider_(_robot);
    return &state_provider_;
}

ValkyrieStateProvider::ValkyrieStateProvider(RobotSystem* _robot) {
    myUtils::pretty_constructor(1, "Valkyrie State Provider");

    // API related parameters
    b_walking = false;
    ft_length = 0.;
    r_ft_width = 0.;
    l_ft_width = 0.;
    ft_ori_inc = 0.;
    num_total_step = 0;
    num_residual_step = 0;

    num_step_copy = 0;
    phase_copy = 0;
    robot_ = _robot;
    stance_foot = ValkyrieBodyNode::leftFoot;
    curr_time = 0.;
    prev_state_machine_time = 0.;
    planning_moment = 0.;

    q = Eigen::VectorXd::Zero(Valkyrie::n_dof);
    qdot = Eigen::VectorXd::Zero(Valkyrie::n_dof);

    b_rfoot_contact = 0;
    b_lfoot_contact = 0;

    foot_target_list.clear();
    com_des_list.clear();

    com_pos = Eigen::VectorXd::Zero(3);
    com_vel = Eigen::VectorXd::Zero(3);
    mom = Eigen::VectorXd::Zero(6);

    com_pos_des = Eigen::VectorXd::Zero(3);
    com_vel_des = Eigen::VectorXd::Zero(3);
    mom_des = Eigen::VectorXd::Zero(6);

    rf_pos = Eigen::VectorXd::Zero(3);
    rf_vel = Eigen::VectorXd::Zero(3);
    lf_pos = Eigen::VectorXd::Zero(3);
    lf_vel = Eigen::VectorXd::Zero(3);

    rf_pos_des = Eigen::VectorXd::Zero(3);
    rf_vel_des = Eigen::VectorXd::Zero(3);
    lf_pos_des = Eigen::VectorXd::Zero(3);
    lf_vel_des = Eigen::VectorXd::Zero(3);

    rf_ori_quat = Eigen::Quaternion<double>::Identity();
    rf_ang_vel = Eigen::VectorXd::Zero(3);
    lf_ori_quat = Eigen::Quaternion<double>::Identity();
    lf_ang_vel = Eigen::VectorXd::Zero(3);

    rf_ori_quat_des = Eigen::Quaternion<double>::Identity();
    rf_ang_vel_des = Eigen::VectorXd::Zero(3);
    lf_ori_quat_des = Eigen::Quaternion<double>::Identity();
    lf_ang_vel_des = Eigen::VectorXd::Zero(3);

    pelvis_ori = Eigen::Quaternion<double>::Identity();
    pelvis_ang_vel = Eigen::VectorXd::Zero(3);
    torso_ori = Eigen::Quaternion<double>::Identity();
    torso_ang_vel = Eigen::VectorXd::Zero(3);

    pelvis_ori_des = Eigen::Quaternion<double>::Identity();
    pelvis_ang_vel_des = Eigen::VectorXd::Zero(3);
    torso_ori_des = Eigen::Quaternion<double>::Identity();
    torso_ang_vel_des = Eigen::VectorXd::Zero(3);

    r_rf = Eigen::VectorXd::Zero(6);
    l_rf = Eigen::VectorXd::Zero(6);
    r_rf_des = Eigen::VectorXd::Zero(6);
    l_rf_des = Eigen::VectorXd::Zero(6);

    des_jacc_cmd = Eigen::VectorXd::Zero(Valkyrie::n_adof);

    DataManager* data_manager = DataManager::GetDataManager();
    data_manager->RegisterData(&curr_time, DOUBLE, "time");
    data_manager->RegisterData(&q, VECT, "q", Valkyrie::n_dof);
    data_manager->RegisterData(&qdot, VECT, "qdot", Valkyrie::n_dof);
    data_manager->RegisterData(&b_rfoot_contact, INT, "rfoot_contact", 1);
    data_manager->RegisterData(&b_lfoot_contact, INT, "lfoot_contact", 1);

    data_manager->RegisterData(&com_pos, VECT, "com_pos", 3);
    data_manager->RegisterData(&com_vel, VECT, "com_vel", 3);
    data_manager->RegisterData(&mom, VECT, "cm", 6);

    data_manager->RegisterData(&com_pos_des, VECT, "com_pos_des", 3);
    data_manager->RegisterData(&com_vel_des, VECT, "com_vel_des", 3);
    data_manager->RegisterData(&mom_des, VECT, "cm_des", 6);

    data_manager->RegisterData(&rf_pos, VECT, "rf_pos", 3);
    data_manager->RegisterData(&rf_vel, VECT, "rf_vel", 3);
    data_manager->RegisterData(&lf_pos, VECT, "lf_pos", 3);
    data_manager->RegisterData(&lf_vel, VECT, "lf_vel", 3);

    data_manager->RegisterData(&rf_pos_des, VECT, "rf_pos_des", 3);
    data_manager->RegisterData(&rf_vel_des, VECT, "rf_vel_des", 3);
    data_manager->RegisterData(&lf_pos_des, VECT, "lf_pos_des", 3);
    data_manager->RegisterData(&lf_vel_des, VECT, "lf_vel_des", 3);

    data_manager->RegisterData(&rf_ori_quat, QUATERNION, "rf_ori_quat", 4);
    data_manager->RegisterData(&rf_ang_vel, VECT, "rf_ang_vel", 3);
    data_manager->RegisterData(&lf_ori_quat, QUATERNION, "lf_ori_quat", 4);
    data_manager->RegisterData(&lf_ang_vel, VECT, "lf_ang_vel", 3);

    data_manager->RegisterData(&rf_ori_quat_des, QUATERNION, "rf_ori_quat_des",
                               4);
    data_manager->RegisterData(&rf_ang_vel_des, VECT, "rf_ang_vel_des", 3);
    data_manager->RegisterData(&lf_ori_quat_des, QUATERNION, "lf_ori_quat_des",
                               4);
    data_manager->RegisterData(&lf_ang_vel_des, VECT, "lf_ang_vel_des", 3);

    data_manager->RegisterData(&pelvis_ori, QUATERNION, "pelvis_ori", 4);
    data_manager->RegisterData(&pelvis_ang_vel, VECT, "pelvis_ang_vel", 3);
    data_manager->RegisterData(&torso_ori, QUATERNION, "torso_ori", 4);
    data_manager->RegisterData(&torso_ang_vel, VECT, "torso_ang_vel", 3);

    data_manager->RegisterData(&pelvis_ori_des, QUATERNION, "pelvis_ori_des",
                               4);
    data_manager->RegisterData(&pelvis_ang_vel_des, VECT, "pelvis_ang_vel_des",
                               3);
    data_manager->RegisterData(&torso_ori_des, QUATERNION, "torso_ori_des", 4);
    data_manager->RegisterData(&torso_ang_vel_des, VECT, "torso_ang_vel_des",
                               3);

    data_manager->RegisterData(&r_rf_des, VECT, "r_rf_des", 6);
    data_manager->RegisterData(&l_rf_des, VECT, "l_rf_des", 6);
    data_manager->RegisterData(&r_rf, VECT, "r_rf", 6);
    data_manager->RegisterData(&l_rf, VECT, "l_rf", 6);

    data_manager->RegisterData(&des_jacc_cmd, VECT, "des_jacc_cmd",
                               Valkyrie::n_adof);
}

void ValkyrieStateProvider::saveCurrentData() {
    for (int i = 0; i < 3; ++i) {
        com_pos[i] = robot_->getCoMPosition()[i];
        com_vel[i] = robot_->getCoMVelocity()[i];
    }
    mom = robot_->getCentroidMomentum();
    rf_pos = robot_->getBodyNodeIsometry(ValkyrieBodyNode::rightCOP_Frame)
                 .translation();
    rf_vel =
        robot_->getBodyNodeSpatialVelocity(ValkyrieBodyNode::rightCOP_Frame)
            .tail(3);
    lf_pos = robot_->getBodyNodeIsometry(ValkyrieBodyNode::leftCOP_Frame)
                 .translation();
    lf_vel = robot_->getBodyNodeSpatialVelocity(ValkyrieBodyNode::leftCOP_Frame)
                 .tail(3);

    rf_ori_quat = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::rightCOP_Frame).linear());
    rf_ang_vel =
        robot_->getBodyNodeSpatialVelocity(ValkyrieBodyNode::rightCOP_Frame)
            .head(3);
    lf_ori_quat = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::leftCOP_Frame).linear());
    lf_ang_vel =
        robot_->getBodyNodeSpatialVelocity(ValkyrieBodyNode::leftCOP_Frame)
            .head(3);

    pelvis_ori = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::pelvis).linear());

    pelvis_ang_vel =
        robot_->getBodyNodeSpatialVelocity(ValkyrieBodyNode::pelvis).head(3);

    torso_ori = Eigen::Quaternion<double>(
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::torso).linear());

    torso_ang_vel =
        robot_->getBodyNodeSpatialVelocity(ValkyrieBodyNode::torso).head(3);
}
