#include <array>

#include <PnC/PlannerSet/ContactSequenceGenerator/FootstepSequenceGenerator.hpp>
#include <PnC/ValkyriePnC/ContactSet/ContactSet.hpp>
#include <PnC/ValkyriePnC/CtrlSet/CtrlSet.hpp>
#include <PnC/ValkyriePnC/TaskSet/TaskSet.hpp>
#include <PnC/ValkyriePnC/ValkyrieDefinition.hpp>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <PnC/WBC/WBLC/KinWBC.hpp>
#include <PnC/WBC/WBLC/WBLC.hpp>
#include <Utils/IO/DataManager.hpp>
#include <Utils/Math/MathUtilities.hpp>

DoubleSupportCtrl::DoubleSupportCtrl(RobotSystem* robot, Planner* planner,
                                     FootstepSequenceGenerator* fsg)
    : Controller(robot),
      planner_(planner),
      foot_sequence_gen_(fsg),
      b_reached_(false) {
    myUtils::pretty_constructor(2, "Double Support Ctrl");
    dsp_dur_ = 1000.;
    ctrl_start_time_ = 0.;
    des_jpos_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    des_jvel_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    des_jacc_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    Kp_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    Kd_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);

    centroid_task_ = new BasicTask(robot, BasicTaskType::CENTROID, 6);
    com_task_ = new BasicTask(robot, BasicTaskType::COM, 3);
    total_joint_task_ =
        new BasicTask(robot, BasicTaskType::JOINT, Valkyrie::n_adof);
    pelvis_ori_task_ = new BasicTask(robot, BasicTaskType::LINKORI, 3,
                                     ValkyrieBodyNode::pelvis);
    torso_ori_task_ = new BasicTask(robot, BasicTaskType::LINKORI, 3,
                                    ValkyrieBodyNode::torso);

    std::vector<bool> act_list;
    act_list.resize(Valkyrie::n_dof, true);
    for (int i(0); i < Valkyrie::n_vdof; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    rfoot_contact_ = new SurfaceContactSpec(
        robot_, ValkyrieBodyNode::rightCOP_Frame, 0.135, 0.08, 0.7);
    lfoot_contact_ = new SurfaceContactSpec(
        robot_, ValkyrieBodyNode::leftCOP_Frame, 0.135, 0.08, 0.7);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    wblc_data_->W_qddot_ = Eigen::VectorXd::Constant(Valkyrie::n_dof, 100.0);
    wblc_data_->W_rf_ = Eigen::VectorXd::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = Eigen::VectorXd::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[rfoot_contact_->getDim() + lfoot_contact_->getFzIndex()] =
        0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = Eigen::VectorXd::Constant(Valkyrie::n_adof, -2500.);
    wblc_data_->tau_max_ = Eigen::VectorXd::Constant(Valkyrie::n_adof, 2500.);

    sp_ = ValkyrieStateProvider::getStateProvider(robot);
}

DoubleSupportCtrl::~DoubleSupportCtrl() {
    delete centroid_task_;
    delete com_task_;
    delete total_joint_task_;
    delete pelvis_ori_task_;
    delete torso_ori_task_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;

    delete rfoot_contact_;
    delete lfoot_contact_;
}

void DoubleSupportCtrl::oneStep(void* _cmd) {
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time - ctrl_start_time_;
    sp_->prev_state_machine_time = state_machine_time_;

    if (b_do_plan_ && (!b_reached_)) {
        PlannerUpdate_();
        if (!b_replan_) b_do_plan_ = false;
    }

    Eigen::VectorXd gamma = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for (int i(0); i < Valkyrie::n_adof; ++i) {
        ((ValkyrieCommand*)_cmd)->jtrq[i] = gamma[i];
        ((ValkyrieCommand*)_cmd)->q[i] = des_jpos_[i];
        ((ValkyrieCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void DoubleSupportCtrl::PlannerUpdate_() {
    sp_->clock.start();
    PlannerInitialization_();
    planner_->DoPlan();
    ((CentroidPlanner*)planner_)
        ->SaveResult("DS_" + std::to_string(sp_->num_step_copy));
    sp_->planning_moment = sp_->curr_time;
    std::cout << "Saved DS_" + std::to_string(sp_->num_step_copy) << std::endl;
    std::cout << "(ds) planning takes : " << sp_->clock.stop() << " (ms)"
              << std::endl;
    ((CentroidPlanner*)planner_)
        ->GetSolution(com_traj_, lmom_traj_, amom_traj_, cop_local_traj_,
                      frc_world_traj_, trq_local_traj_);
    sp_->com_des_list.clear();
    for (int i = 0; i < com_traj_.cols(); ++i) {
        sp_->com_des_list.push_back(
            (Eigen::VectorXd)(com_traj_.block(0, i, 3, 1)));
    }
}

void DoubleSupportCtrl::PlannerInitialization_() {
    CentroidPlannerParameter* _param =
        ((CentroidPlanner*)planner_)->GetCentroidPlannerParameter();
    // =========================================================================
    // update initial state
    // =========================================================================
    Eigen::Vector3d r, l, k;
    r = robot_->getCoMPosition();
    Eigen::VectorXd lk = robot_->getCentroidMomentum();
    for (int i = 0; i < 3; ++i) {
        l[i] = lk[i + 3];
        // k[i] = lk[i];
        // l[i] = 0.;
        k[i] = 0.;
    }
    std::array<int, CentroidModel::numEEf> actv;
    std::array<Eigen::Vector3d, CentroidModel::numEEf> eef_frc;
    std::array<Eigen::Isometry3d, CentroidModel::numEEf> iso;
    std::array<Eigen::VectorXd, CentroidModel::numEEf> quat;
    for (int eef_id = 0; eef_id < CentroidModel::numEEf; ++eef_id) {
        actv[eef_id] = 0;
        eef_frc[eef_id].setZero();
    }
    actv[static_cast<int>(CentroidModel::EEfID::rightFoot)] = 1;
    actv[static_cast<int>(CentroidModel::EEfID::leftFoot)] = 1;
    // TODO : reaction force distribution
    eef_frc[static_cast<int>(CentroidModel::EEfID::rightFoot)] << 0., 0., 0.5;
    eef_frc[static_cast<int>(CentroidModel::EEfID::leftFoot)] << 0., 0., 0.5;

    iso[static_cast<int>(CentroidModel::EEfID::rightFoot)] =
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::rightCOP_Frame);
    iso[static_cast<int>(CentroidModel::EEfID::leftFoot)] =
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::leftCOP_Frame);
    iso[static_cast<int>(CentroidModel::EEfID::rightHand)] =
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::rightPalm);
    iso[static_cast<int>(CentroidModel::EEfID::leftHand)] =
        robot_->getBodyNodeIsometry(ValkyrieBodyNode::leftPalm);

    for (int i = 0; i < CentroidModel::numEEf; ++i) {
        Eigen::VectorXd tmp_vec = Eigen::VectorXd::Zero(4);
        Eigen::Quaternion<double> tmp_quat =
            Eigen::Quaternion<double>(iso[i].linear());
        tmp_vec << tmp_quat.w(), tmp_quat.x(), tmp_quat.y(), tmp_quat.z();
        quat[i] = tmp_vec;
    }

    _param->UpdateInitialState(r, l, k, actv, eef_frc, iso);

    // =========================================================================
    // update contact plan interface
    // =========================================================================

    std::array<std::vector<Eigen::VectorXd>, CentroidModel::numEEf> c_seq;
    for (int i = 0; i < CentroidModel::numEEf; ++i) {
        c_seq[i].clear();
    }
    double rem_time(0.);
    if (sp_->num_step_copy == 0) {
        rem_time = ini_dsp_dur_ - state_machine_time_;
    } else {
        rem_time = dsp_dur_ - state_machine_time_;
    }
    Eigen::VectorXd rf_ct_pos =
        iso[static_cast<int>(CentroidModel::EEfID::rightFoot)].translation();
    Eigen::VectorXd lf_ct_pos =
        iso[static_cast<int>(CentroidModel::EEfID::leftFoot)].translation();
    Eigen::VectorXd f_ori = Eigen::VectorXd::Zero(4);
    f_ori[0] = 1.;
    double rf_st_time(0.);
    double lf_st_time(0.);
    CentroidModel::EEfID dummy;
    if (sp_->phase_copy == 0) {
        // stance : left foot, // swing : right foot
        double rf_end_time(rem_time);
        double lf_end_time(rem_time + ssp_dur_ + dsp_dur_);
        foot_sequence_gen_->Update(CentroidModel::EEfID::leftFoot, rf_ct_pos,
                                   lf_ct_pos);

        int n_step_hori(2);
        for (int i = 0; i < n_step_hori; ++i) {
            // update right foot
            AddContactSequence_(CentroidModel::EEfID::rightFoot, rf_st_time,
                                rf_end_time, rf_ct_pos, f_ori, c_seq);
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)].push_back(
            // Eigen::VectorXd::Zero(10));
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][0] =
            // rf_st_time;
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][1] =
            // rf_end_time;
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i]
            //.segment<3>(2) = rf_ct_pos;
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i]
            //.segment<4>(5) = Eigen::VectorXd::Zero(4);
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][5]
            // = 1.;
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][9] =
            // static_cast<double>(ContactType::FlatContact);
            rf_st_time = rf_end_time + ssp_dur_;
            if (i == n_step_hori - 1) {
                rf_end_time += ssp_dur_ + fin_dsp_dur_;
            } else {
                rf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + dsp_dur_;
            }
            foot_sequence_gen_->GetNextFootStep(dummy, rf_ct_pos);

            // update left foot
            AddContactSequence_(CentroidModel::EEfID::leftFoot, lf_st_time,
                                lf_end_time, lf_ct_pos, f_ori, c_seq);
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)].push_back(
            // Eigen::VectorXd::Zero(10));
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][0] =
            // lf_st_time;
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][1] =
            // lf_end_time;
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i]
            //.segment<3>(2) = lf_ct_pos;
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i]
            //.segment<4>(5) = Eigen::VectorXd::Zero(4);
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][5]
            // = 1.;
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][9] =
            // static_cast<double>(ContactType::FlatContact);
            lf_st_time = lf_end_time + ssp_dur_;
            if (i == n_step_hori - 2) {
                lf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + fin_dsp_dur_;
                foot_sequence_gen_->GetNextFootStep(dummy, lf_ct_pos);
            } else if (i == n_step_hori - 1) {
            } else {
                lf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + dsp_dur_;
                foot_sequence_gen_->GetNextFootStep(dummy, lf_ct_pos);
            }
        }
        // update final right foot
        rf_ct_pos[0] = lf_ct_pos[0];
        AddContactSequence_(CentroidModel::EEfID::rightFoot, rf_st_time,
                            rf_end_time, rf_ct_pos, f_ori, c_seq);
        // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)].push_back(
        // Eigen::VectorXd::Zero(10));
        // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][n_step_hori]
        //[0] = rf_st_time;
        // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][n_step_hori]
        //[1] = rf_end_time;
        // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][n_step_hori]
        //.segment<3>(2) = rf_ct_pos;
        // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][n_step_hori]
        //.segment<4>(5) = Eigen::VectorXd::Zero(4);
        // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][n_step_hori]
        //[5] = 1.;
        // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][n_step_hori]
        //[9] = static_cast<double>(ContactType::FlatContact);

        _param->UpdateTimeHorizon(rf_end_time);
    } else {
        // stance : right foot, // swing : left foot
        double lf_end_time(rem_time);
        double rf_end_time(rem_time + ssp_dur_ + dsp_dur_);
        foot_sequence_gen_->Update(CentroidModel::EEfID::rightFoot, rf_ct_pos,
                                   lf_ct_pos);

        int n_step_hori(2);
        for (int i = 0; i < n_step_hori; ++i) {
            // update left foot
            AddContactSequence_(CentroidModel::EEfID::leftFoot, lf_st_time,
                                lf_end_time, lf_ct_pos, f_ori, c_seq);
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)].push_back(
            // Eigen::VectorXd::Zero(10));
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][0] =
            // lf_st_time;
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][1] =
            // lf_end_time;
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i]
            //.segment<3>(2) = lf_ct_pos;
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i]
            //.segment<4>(5) = Eigen::VectorXd::Zero(4);
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][5]
            // = 1.;
            // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][i][9] =
            // static_cast<double>(ContactType::FlatContact);
            lf_st_time = lf_end_time + ssp_dur_;
            if (i == n_step_hori - 1) {
                lf_end_time += ssp_dur_ + fin_dsp_dur_;
            } else {
                lf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + dsp_dur_;
            }
            foot_sequence_gen_->GetNextFootStep(dummy, lf_ct_pos);

            // update right foot
            AddContactSequence_(CentroidModel::EEfID::rightFoot, rf_st_time,
                                rf_end_time, rf_ct_pos, f_ori, c_seq);
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)].push_back(
            // Eigen::VectorXd::Zero(10));
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][0] =
            // rf_st_time;
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][1] =
            // rf_end_time;
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i]
            //.segment<3>(2) = rf_ct_pos;
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i]
            //.segment<4>(5) = Eigen::VectorXd::Zero(4);
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][5]
            // = 1.;
            // c_seq[static_cast<int>(CentroidModel::EEfID::rightFoot)][i][9] =
            // static_cast<double>(ContactType::FlatContact);
            rf_st_time = rf_end_time + ssp_dur_;
            if (i == n_step_hori - 2) {
                rf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + fin_dsp_dur_;
                foot_sequence_gen_->GetNextFootStep(dummy, rf_ct_pos);
            } else if (i == n_step_hori - 1) {
            } else {
                rf_end_time += ssp_dur_ + dsp_dur_ + ssp_dur_ + dsp_dur_;
                foot_sequence_gen_->GetNextFootStep(dummy, rf_ct_pos);
            }
        }
        // update final left foot
        lf_ct_pos[0] = rf_ct_pos[0];
        AddContactSequence_(CentroidModel::EEfID::leftFoot, lf_st_time,
                            lf_end_time, lf_ct_pos, f_ori, c_seq);
        // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)].push_back(
        // Eigen::VectorXd::Zero(10));
        // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][n_step_hori]
        //[0] = lf_st_time;
        // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][n_step_hori]
        //[1] = lf_end_time;
        // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][n_step_hori]
        //.segment<3>(2) = lf_ct_pos;
        // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][n_step_hori]
        //.segment<4>(5) = Eigen::VectorXd::Zero(4);
        // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][n_step_hori]
        //[5] = 1.;
        // c_seq[static_cast<int>(CentroidModel::EEfID::leftFoot)][n_step_hori]
        //[9] = static_cast<double>(ContactType::FlatContact);

        _param->UpdateTimeHorizon(rf_end_time);
    }
    _param->UpdateContactPlanInterface(c_seq);

    // =========================================================================
    // update reference dynamics state sequence
    // =========================================================================
    _param->UpdateRefDynamicsStateSequence();

    // =========================================================================
    // update goal state
    // =========================================================================
    Eigen::Vector3d com_goal;
    Eigen::Vector3d com_displacement;
    for (int i = 0; i < 2; ++i) {
        com_goal[i] = (lf_ct_pos[i] + rf_ct_pos[i]) / 2.0;
        com_displacement[i] = com_goal[i] - r[i];
    }
    com_displacement[2] = com_height_ - r[2];
    _param->UpdateTerminalState(com_displacement);

    // =========================================================================
    // save contact sequence for plotting
    // =========================================================================
    sp_->foot_target_list.clear();
    for (int eef_id = 0; eef_id < CentroidModel::numEEf; ++eef_id) {
        for (int c_id = 1; c_id < c_seq[eef_id].size(); ++c_id) {
            Eigen::Isometry3d tmp;
            tmp.translation() = c_seq[eef_id][c_id].segment<3>(2);
            tmp.linear() = Eigen::Quaternion<double>(
                               c_seq[eef_id][c_id][5], c_seq[eef_id][c_id][6],
                               c_seq[eef_id][c_id][7], c_seq[eef_id][c_id][8])
                               .toRotationMatrix();
            sp_->foot_target_list.push_back(tmp);
        }
    }
}

void DoubleSupportCtrl::_compute_torque_wblc(Eigen::VectorXd& gamma) {
    // WBLC
    wblc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    Eigen::VectorXd des_jacc_cmd =
        des_jacc_ +
        Kp_.cwiseProduct(des_jpos_ -
                         sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof)) +
        Kd_.cwiseProduct(des_jvel_ - sp_->qdot.tail(Valkyrie::n_adof));

    sp_->des_jacc_cmd = des_jacc_cmd;
    wblc_->makeWBLC_Torque(des_jacc_cmd, contact_list_, gamma, wblc_data_);
    sp_->r_rf = wblc_data_->Fr_.head(6);
    sp_->l_rf = wblc_data_->Fr_.tail(6);
}

void DoubleSupportCtrl::_task_setup() {
    // =========================================================================
    // Centroid Task
    // =========================================================================
    Eigen::VectorXd cen_pos_des = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd cen_vel_des = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd cen_acc_des = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd dummy = Eigen::VectorXd::Zero(6);
    // planner_->EvalTrajectory(state_machine_time_, cen_pos_des, cen_vel_des,
    // dummy);
    planner_->EvalTrajectory(sp_->curr_time - sp_->planning_moment, cen_pos_des,
                             cen_vel_des, dummy);

    // for (int i = 0; i < 3; ++i) {
    // sp_->com_pos_des[i] = cen_pos_des[i + 3];
    // sp_->com_vel_des[i] = cen_vel_des[i + 3] / robot_->getRobotMass();
    // sp_->mom_des[i] = cen_vel_des[i];
    // sp_->mom_des[i + 3] = cen_vel_des[i + 3];
    //}

    centroid_task_->updateTask(cen_pos_des, cen_vel_des, cen_acc_des);

    // =========================================================================
    // COM Task
    // =========================================================================
    Eigen::VectorXd com_pos_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_vel_des = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd com_acc_des = Eigen::VectorXd::Zero(3);

    for (int i = 0; i < 3; ++i) {
        com_pos_des[i] = cen_pos_des[i + 3];
        com_vel_des[i] = cen_vel_des[i + 3] / robot_->getRobotMass();
    }

    for (int i = 0; i < 3; ++i) {
        sp_->com_pos_des[i] = com_pos_des[i];
        sp_->com_vel_des[i] = com_vel_des[i];
    }

    com_task_->updateTask(com_pos_des, com_vel_des, com_acc_des);

    // =========================================================================
    // Pelvis & Torso Ori Task
    // =========================================================================
    Eigen::VectorXd des_quat = Eigen::VectorXd::Zero(4);
    des_quat << 1., 0., 0., 0.;
    Eigen::VectorXd des_so3 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd ori_acc_des = Eigen::VectorXd::Zero(3);
    pelvis_ori_task_->updateTask(des_quat, des_so3, ori_acc_des);
    torso_ori_task_->updateTask(des_quat, des_so3, ori_acc_des);

    // =========================================================================
    // Joint Pos Task
    // =========================================================================
    Eigen::VectorXd jpos_des = sp_->jpos_ini;
    Eigen::VectorXd jvel_des(Valkyrie::n_adof);
    jvel_des.setZero();
    Eigen::VectorXd jacc_des(Valkyrie::n_adof);
    jacc_des.setZero();
    total_joint_task_->updateTask(jpos_des, jvel_des, jacc_des);

    // =========================================================================
    // Task List Update
    // =========================================================================
    // task_list_.push_back(centroid_task_);
    task_list_.push_back(com_task_);
    task_list_.push_back(pelvis_ori_task_);
    task_list_.push_back(torso_ori_task_);
    task_list_.push_back(total_joint_task_);

    // =========================================================================
    // Solve Inv Kinematics
    // =========================================================================
    kin_wbc_->FindConfiguration(sp_->q, task_list_, contact_list_, des_jpos_,
                                des_jvel_, des_jacc_);
}

void DoubleSupportCtrl::_contact_setup() {
    rfoot_contact_->updateContactSpec();
    lfoot_contact_->updateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void DoubleSupportCtrl::firstVisit() {
    jpos_ini_ = sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    ctrl_start_time_ = sp_->curr_time;
    b_do_plan_ = true;
}

void DoubleSupportCtrl::lastVisit() {}

bool DoubleSupportCtrl::endOfPhase() {
    if (sp_->num_step_copy == 0) {
        if (state_machine_time_ > ini_dsp_dur_) {
            return true;
        }
    } else {
        if (state_machine_time_ > dsp_dur_) {
            return true;
        }
    }
    return false;
}

void DoubleSupportCtrl::ctrlInitialization(const YAML::Node& node) {
    jpos_ini_ = sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    try {
        myUtils::readParameter(node, "kp", Kp_);
        myUtils::readParameter(node, "kd", Kd_);

        Eigen::VectorXd tmp_vec1, tmp_vec2;
        myUtils::readParameter(node, "com_kp", tmp_vec1);
        myUtils::readParameter(node, "com_kd", tmp_vec2);
        com_task_->setGain(tmp_vec1, tmp_vec2);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

void DoubleSupportCtrl::AddContactSequence_(
    CentroidModel::EEfID eef_id, double ini, double fin, Eigen::VectorXd f_pos,
    Eigen::VectorXd f_ori,
    std::array<std::vector<Eigen::VectorXd>, CentroidModel::numEEf>& c_seq) {
    c_seq[static_cast<int>(eef_id)].push_back(Eigen::VectorXd::Zero(10));
    int idx = c_seq[static_cast<int>(eef_id)].size() - 1;
    c_seq[static_cast<int>(eef_id)][idx][0] = ini;
    c_seq[static_cast<int>(eef_id)][idx][1] = fin;
    c_seq[static_cast<int>(eef_id)][idx].segment<3>(2) = f_pos;
    c_seq[static_cast<int>(eef_id)][idx].segment<4>(5) = f_ori;
    c_seq[static_cast<int>(eef_id)][idx][9] =
        static_cast<double>(ContactType::FlatContact);
}
