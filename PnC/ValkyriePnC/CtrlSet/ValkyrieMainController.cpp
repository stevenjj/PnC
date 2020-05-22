#include <PnC/ValkyriePnC/CtrlSet/ValkyrieMainController.hpp>

ValkyrieMainController::ValkyrieMainController(ValkyrieTaskAndForceContainer* _taf_container, RobotSystem* _robot) : Controller(_robot){
    myUtils::pretty_constructor(2, "Valkyrie Main Controller");
    // Initialize Flag
    b_first_visit_ = true;

    // Initialize Pointer to the Task and Force Container
    taf_container_ = _taf_container;

    // Initialize State Provider
    sp_ = ValkyrieStateProvider::getStateProvider(robot_);

    // Initialize IHWBC
    std::vector<bool> act_list;
    act_list.resize(Valkyrie::n_dof, true);
    for (int i(0); i < Valkyrie::n_vdof; ++i) act_list[i] = false;
    ihwbc_ = new IHWBC(act_list);

    tau_cmd_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    qddot_cmd_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);

    // Initialize Joint Integrator
    ihwbc_dt_ = ValkyrieAux::servo_rate;
    ihwbc_joint_integrator_ = new IHWBC_JointIntegrator(Valkyrie::n_adof, ihwbc_dt_);

    // Initialize desired pos, vel, acc containers
    des_jpos_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    des_jvel_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);
    des_jacc_ = Eigen::VectorXd::Zero(Valkyrie::n_adof);    
}

ValkyrieMainController::~ValkyrieMainController(){
    delete ihwbc_;
    delete ihwbc_joint_integrator_;
}

void ValkyrieMainController::_PreProcessing_Command(){
    // Update Dynamic Terms
    A_ = robot_->getMassMatrix();
    Ainv_ = robot_->getInvMassMatrix();
    grav_ = robot_->getGravity();
    coriolis_ = robot_->getCoriolis();

    // Clear out local pointers
    task_list_.clear();
    contact_list_.clear();

    // Grab Variables from the container.
    // Update task and contact list pointers from container object
    for (int i = 0; i < taf_container_->task_list_.size(); i++){
        task_list_.push_back(taf_container_->task_list_[i]);
    }
    for (int i = 0; i < taf_container_->contact_list_.size(); i++){
        contact_list_.push_back(taf_container_->contact_list_[i]);
    }
    Fd_des_ = taf_container_->Fd_des_;

    // Update Task Jacobians and commands
    for(int i = 0; i < task_list_.size(); i++){
        task_list_[i]->updateJacobians();
        task_list_[i]->computeCommands();
    }
    // Update Contact Spec
    for(int i = 0; i < contact_list_.size(); i++){
        contact_list_[i]->updateContactSpec();
    }   
}

void ValkyrieMainController::getCommand(void* _cmd){   
    // Perform First time visit Initialization
    if (b_first_visit_){
        firstVisit();
        b_first_visit_ = false;
    }

    // Update Dynamic Terms, Task Jacobians, and Contact Jacobians
    _PreProcessing_Command();

    // Update Task Hierarchy
    Eigen::VectorXd w_task_hierarchy_ = Eigen::VectorXd::Zero(task_list_.size());
    for(int i = 0; i < task_list_.size(); i++){
        w_task_hierarchy_[i] = task_list_[i]->getHierarchyWeight();        
    }

    // Set QP weights   
    double local_w_contact_weight = w_contact_weight_/(robot_->getRobotMass()*9.81);
    ihwbc_->setQPWeights(w_task_hierarchy_, local_w_contact_weight);
    ihwbc_->setRegularizationTerms(lambda_qddot_, lambda_Fr_);

    // QP dec variable results
    Eigen::VectorXd qddot_res;
    Eigen::VectorXd Fr_res;

    // Update QP and solve
    ihwbc_->updateSetting(A_, Ainv_, coriolis_, grav_);
    ihwbc_->solve(task_list_, contact_list_, Fd_des_, tau_cmd_, qddot_cmd_);

    // Get Results
    ihwbc_->getQddotResult(qddot_res);
    ihwbc_->getFrResult(Fr_res);

    // Integrate Joint Velocities and Positions
    des_jacc_= qddot_cmd_;
    ihwbc_joint_integrator_->integrate(des_jacc_, sp_->qdot.segment(Valkyrie::n_vdof, Valkyrie::n_adof),
                                                  sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof),
                                                  des_jvel_,
                                                  des_jpos_);
    // Set Command
    for (int i(0); i < Valkyrie::n_adof; ++i) {
        ((ValkyrieCommand*)_cmd)->jtrq[i] = tau_cmd_[i];
        ((ValkyrieCommand*)_cmd)->q[i] = des_jpos_[i];
        ((ValkyrieCommand*)_cmd)->qdot[i] = des_jvel_[i];
    }

}

void ValkyrieMainController::ctrlInitialization(const YAML::Node& node){    
    // Defaults
    ihwbc_dt_ = ValkyrieAux::servo_rate;
    lambda_qddot_ = 1e-8;  // Generalized Coord Acceleration 
    lambda_Fr_ = 1e-8;     // Reaction Force Regularization
    w_contact_weight_ = 1e-3;  // Contact Weight

    // Enable Torque Limits
    ihwbc_->enableTorqueLimits(true); 
    Eigen::VectorXd tau_min = robot_->GetTorqueLowerLimits().segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    Eigen::VectorXd tau_max = robot_->GetTorqueUpperLimits().segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    ihwbc_->setTorqueLimits(tau_min, tau_max);
}

void ValkyrieMainController::firstVisit(){
    // Initialize joint integrator
    Eigen::VectorXd jpos_ini = sp_->q.segment(Valkyrie::n_vdof, Valkyrie::n_adof);
    ihwbc_joint_integrator_->initializeStates(Eigen::VectorXd::Zero(Valkyrie::n_adof), jpos_ini);
}

// Parent Functions not used
void ValkyrieMainController::oneStep(void* _cmd){}
void ValkyrieMainController::lastVisit(){}
bool ValkyrieMainController::endOfPhase(){}
void ValkyrieMainController::_PostProcessing_Command(){}