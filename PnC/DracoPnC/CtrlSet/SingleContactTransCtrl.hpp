#pragma once

#include <PnC/Controller.hpp>

class RobotSystem;
class DracoStateProvider;
class ContactSpec;
class WBLC;
class KinWBC;
class WBLC_ExtraData;

class SingleContactTransCtrl: public Controller{
    public:
        SingleContactTransCtrl(RobotSystem* robot, std::string moving_foot, bool b_increase);
        virtual ~SingleContactTransCtrl();

        virtual void oneStep(void* _cmd);
        virtual void firstVisit();
        virtual void lastVisit();
        virtual bool endOfPhase();

        virtual void ctrlInitialization(const std::string & setting_file_name);

        void setTransitionTime(double time){ end_time_ = time; }
        void setStanceHeight(double height) {
            des_base_height_ = height;
            b_set_height_target_ = true;
        }

    protected:
        bool b_set_height_target_;
        double des_base_height_;
        double ini_base_height_;
        Eigen::Vector3d ini_base_pos_;
        int dim_contact_;
        // [right_front, right_back, left_front, left_back]
        std::vector<int> fz_idx_in_cost_;

        double end_time_;
        std::string moving_foot_;
        bool b_increase_; // Increasing or decreasing reaction force
        double max_rf_z_;
        double min_rf_z_;

        std::vector<int> selected_jidx_;
        Task* selected_joint_task_;
        Task* base_task_;

        ContactSpec* rfoot_front_contact_;
        ContactSpec* lfoot_front_contact_;
        ContactSpec* rfoot_back_contact_;
        ContactSpec* lfoot_back_contact_;

        KinWBC* kin_wbc_;
        WBLC* wblc_;
        WBLC_ExtraData* wblc_data_;

        Eigen::VectorXd des_jpos_;
        Eigen::VectorXd des_jvel_;
        Eigen::VectorXd des_jacc_;

        Eigen::VectorXd Kp_;
        Eigen::VectorXd Kd_;

        void _task_setup();
        void _contact_setup();
        void _compute_torque_wblc(Eigen::VectorXd & gamma);

        DracoStateProvider* sp_;
        double ctrl_start_time_;
};
