#include <PnC/PlannerSet/ContactSequenceGenerator/FootstepSequenceGenerator.hpp>

void FootstepSequenceGenerator::GetNextFootStep(
    CentroidModel::EEfID &next_stance, Eigen::Isometry3d &next_foot_iso) {
    next_foot_iso = Eigen::Isometry3d::Identity();
    Eigen::VectorXd global_vec = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd local_vec = Eigen::VectorXd::Zero(3);

    if (curr_stance_ == CentroidModel::EEfID::rightFoot) {
        // right stance
        local_vec << ft_length_, l_ft_spread_, 0;
        next_stance = CentroidModel::EEfID::leftFoot;

        global_vec = curr_rf_iso_.linear() * local_vec;
        next_foot_iso.linear() =
            curr_rf_iso_.linear() * ft_ori_.toRotationMatrix();

        for (int i = 0; i < 2; ++i) {
            next_foot_iso.translation()[i] =
                curr_rf_iso_.translation()[i] + global_vec[i];
        }
        next_foot_iso.translation()[2] = 0;

        // update for the next step
        curr_stance_ = CentroidModel::EEfID::leftFoot;
        curr_lf_iso_ = next_foot_iso;
    } else {
        // left stance
        local_vec << ft_length_, -r_ft_spread_, 0;
        next_stance = CentroidModel::EEfID::rightFoot;

        global_vec = curr_lf_iso_.linear() * local_vec;
        next_foot_iso.linear() =
            curr_lf_iso_.linear() * ft_ori_.toRotationMatrix();
        for (int i = 0; i < 2; ++i) {
            next_foot_iso.translation()[i] =
                curr_lf_iso_.translation()[i] + global_vec[i];
        }
        next_foot_iso.translation()[2] = 0;
        // update for the next step
        curr_stance_ = CentroidModel::EEfID::rightFoot;
        curr_rf_iso_ = next_foot_iso;
    }
}

void FootstepSequenceGenerator::GetFinalFootStep(
    CentroidModel::EEfID &next_stance, Eigen::Isometry3d &next_foot_iso) {
    next_foot_iso = Eigen::Isometry3d::Identity();
    Eigen::VectorXd global_vec = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd local_vec = Eigen::VectorXd::Zero(3);

    if (curr_stance_ == CentroidModel::EEfID::rightFoot) {
        // right stance
        local_vec << 0., l_ft_spread_, 0;
        next_stance = CentroidModel::EEfID::leftFoot;

        global_vec = curr_rf_iso_.linear() * local_vec;
        next_foot_iso.linear() = curr_rf_iso_.linear();

        for (int i = 0; i < 2; ++i) {
            next_foot_iso.translation()[i] =
                curr_rf_iso_.translation()[i] + global_vec[i];
        }
        next_foot_iso.translation()[2] = 0;

        // update for the next step
        curr_stance_ = CentroidModel::EEfID::leftFoot;
        curr_lf_iso_ = next_foot_iso;
    } else {
        // left stance
        local_vec << 0., -r_ft_spread_, 0;
        next_stance = CentroidModel::EEfID::rightFoot;

        global_vec = curr_lf_iso_.linear() * local_vec;
        next_foot_iso.linear() = curr_lf_iso_.linear();
        for (int i = 0; i < 2; ++i) {
            next_foot_iso.translation()[i] =
                curr_lf_iso_.translation()[i] + global_vec[i];
        }
        next_foot_iso.translation()[2] = 0;
        // update for the next step
        curr_stance_ = CentroidModel::EEfID::rightFoot;
        curr_rf_iso_ = next_foot_iso;
    }
}
