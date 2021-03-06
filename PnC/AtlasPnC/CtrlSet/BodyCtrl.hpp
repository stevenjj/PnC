#pragma once

#include <PnC/Controller.hpp>

class AtlasStateProvider;
class RobotSystem;
class WBLC;
class WBLC_ExtraData;
class KinWBC;
class ContactSpec;

class BodyCtrl : public Controller {
   public:
    BodyCtrl(RobotSystem*);
    virtual ~BodyCtrl();

    virtual void oneStep(void* _cmd);
    virtual void firstVisit();
    virtual void lastVisit();
    virtual bool endOfPhase();
    virtual void ctrlInitialization(const YAML::Node& node);

    void setStanceTime(double time) { end_time_ = time; }
    void setStanceHeight(double height) {
        target_body_height_ = height;
        b_set_height_target_ = true;
    }

   protected:
    Eigen::VectorXd Kp_, Kd_;
    Eigen::VectorXd des_jpos_;
    Eigen::VectorXd des_jvel_;
    Eigen::VectorXd des_jacc_;

    Eigen::VectorXd jpos_ini_;
    bool b_set_height_target_;
    double end_time_;
    int dim_contact_;

    std::vector<int> selected_jidx_;

    Task* total_joint_task_;
    Task* body_pos_task_;  // pelvis
    Task* body_ori_task_;
    Task* torso_ori_task_;
    Task* selected_joint_task_;

    ContactSpec* rfoot_contact_;
    ContactSpec* lfoot_contact_;

    KinWBC* kin_wbc_;
    WBLC* wblc_;
    WBLC_ExtraData* wblc_data_;

    // Task specification
    double target_body_height_;
    Eigen::VectorXd ini_body_pos_;

    Eigen::Quaternion<double> ini_body_quat_;
    Eigen::Quaternion<double> body_delta_quat_;
    Eigen::Vector3d body_delta_so3_;

    Eigen::Quaternion<double> ini_torso_quat_;
    Eigen::Quaternion<double> torso_delta_quat_;
    Eigen::Vector3d torso_delta_so3_;

    void _task_setup();
    void _contact_setup();
    void _compute_torque_wblc(Eigen::VectorXd& gamma);

    double ctrl_start_time_;
    AtlasStateProvider* sp_;
};
