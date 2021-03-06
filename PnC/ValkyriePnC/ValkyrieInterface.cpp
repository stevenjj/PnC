#include <math.h>
#include <stdio.h>
#include <PnC/RobotSystem/RobotSystem.hpp>
#include <PnC/ValkyriePnC/TestSet/BalanceTest.hpp>
#include <PnC/ValkyriePnC/TestSet/WalkingTest.hpp>
#include <PnC/ValkyriePnC/ValkyrieInterface.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateEstimator.hpp>
#include <PnC/ValkyriePnC/ValkyrieStateProvider.hpp>
#include <Utils/IO/IOUtilities.hpp>
#include <Utils/Math/MathUtilities.hpp>
#include <string>

ValkyrieInterface::ValkyrieInterface() : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    myUtils::color_print(myColor::BoldCyan, border);
    myUtils::pretty_constructor(0, "Valkyrie Interface");

    robot_ = new RobotSystem(
        6, THIS_COM "RobotModel/Robot/Valkyrie/ValkyrieSim_Dart.urdf");
    // robot_->printRobotInfo();
    state_estimator_ = new ValkyrieStateEstimator(robot_);
    sp_ = ValkyrieStateProvider::getStateProvider(robot_);

    sp_->stance_foot = ValkyrieBodyNode::leftCOP_Frame;

    count_ = 0;
    waiting_count_ = 2;
    cmd_jpos_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);

    prev_planning_moment_ = 0.;

    _ParameterSetting();

    myUtils::color_print(myColor::BoldCyan, border);

    DataManager* data_manager = DataManager::GetDataManager();
    data_manager->RegisterData(&cmd_jpos_, VECT, "jpos_des", Valkyrie::n_adof);
    data_manager->RegisterData(&cmd_jvel_, VECT, "jvel_des", Valkyrie::n_adof);
    data_manager->RegisterData(&cmd_jtrq_, VECT, "command", Valkyrie::n_adof);
}

ValkyrieInterface::~ValkyrieInterface() {
    delete robot_;
    delete state_estimator_;
    delete test_;
}

void ValkyrieInterface::getCommand(void* _data, void* _command) {
    ValkyrieCommand* cmd = ((ValkyrieCommand*)_command);
    ValkyrieSensorData* data = ((ValkyrieSensorData*)_data);

    if (!Initialization_(data, cmd)) {
        state_estimator_->Update(data);
        test_->getCommand(cmd);
        CropTorque_(cmd);
    }

    cmd_jtrq_ = cmd->jtrq;
    cmd_jvel_ = cmd->qdot;
    cmd_jpos_ = cmd->q;

    ++count_;
    running_time_ = (double)(count_)*ValkyrieAux::servo_rate;
    sp_->curr_time = running_time_;
    sp_->phase_copy = test_->getPhase();
}

void ValkyrieInterface::CropTorque_(ValkyrieCommand* cmd) {
    // cmd->jtrq = myUtils::CropVector(cmd->jtrq,
    // robot_->GetTorqueLowerLimits(), robot_->GetTorqueUpperLimits(), "clip trq
    // in interface");
}

void ValkyrieInterface::_ParameterSetting() {
    try {
        YAML::Node cfg =
            YAML::LoadFile(THIS_COM "Config/Valkyrie/INTERFACE.yaml");
        std::string test_name =
            myUtils::readParameter<std::string>(cfg, "test_name");
        if (test_name == "balance_test") {
            test_ = new BalanceTest(robot_);
        } else if (test_name == "walking_test") {
            test_ = new WalkingTest(robot_);
        } else {
            printf(
                "[Valkyrie Interface] There is no test matching test with "
                "the name\n");
            exit(0);
        }
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

bool ValkyrieInterface::Initialization_(ValkyrieSensorData* _sensor_data,
                                        ValkyrieCommand* _command) {
    static bool test_initialized(false);
    if (!test_initialized) {
        test_->TestInitialization();
        test_initialized = true;
    }
    if (count_ < waiting_count_) {
        state_estimator_->Initialization(_sensor_data);
        DataManager::GetDataManager()->start();
        return true;
    }
    return false;
}

bool ValkyrieInterface::IsTrajectoryUpdated() {
    if (prev_planning_moment_ == sp_->planning_moment) {
        prev_planning_moment_ = sp_->planning_moment;
        return false;
    } else {
        prev_planning_moment_ = sp_->planning_moment;
        return true;
    }
}

void ValkyrieInterface::GetCoMTrajectory(
    std::vector<Eigen::VectorXd>& com_des_list) {
    com_des_list = sp_->com_des_list;
}
void ValkyrieInterface::GetContactSequence(
    std::vector<Eigen::Isometry3d>& foot_target_list) {
    foot_target_list = sp_->foot_target_list;
}

void ValkyrieInterface::Walk(double ft_length, double r_ft_width,
                             double l_ft_width, double ori_inc, int num_step) {
    if (sp_->b_walking) {
        std::cout
            << "Still Walking... Please Wait Until Ongoing Walking is Done"
            << std::endl;
    } else {
        sp_->b_walking = true;
        sp_->ft_length = ft_length;
        sp_->r_ft_width = r_ft_width;
        sp_->l_ft_width = l_ft_width;
        sp_->ft_ori_inc = ori_inc;
        sp_->num_total_step = num_step;
        sp_->num_residual_step = num_step;

        ((WalkingTest*)test_)->ResetWalkingParameters();
        ((WalkingTest*)test_)->InitiateWalkingPhase();
    }
}
