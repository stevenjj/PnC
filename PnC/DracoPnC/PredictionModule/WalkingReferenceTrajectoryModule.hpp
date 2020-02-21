
// Given:
//	  - starting time
//	  - starting starting configuration:
//			- starting com
//			- starting ori
//			- starting foot config
//    - sequence of footsteps to take,
//
// This class provides information about:
//     Contact schedule c(t): (binary value if contact is active or not)
//	   State schedule s(t): (left leg single support, right leg single support, double support)	
//	   MPC CoM and orientation reference: x_com(t), x_ori(t)
//     Reaction force schedule: f(t)
//	   Left and Right Foot location x_rf(t), x_lf(t)
//
// There is also an interface to provide early contact times.
// 

#include <PnC/DracoPnC/PredictionModule/DracoFootstep.hpp>
#include <PnC/DracoPnC/PredictionModule/WalkingReactionForceSchedule.hpp>
#include <Eigen/Dense>
#include <memory>

#include <stdio.h>

// Interpolators
#include <Utils/Math/hermite_curve_vec.hpp>
#include <Utils/Math/hermite_quaternion_curve.hpp>

// states
#define DRACO_STATE_DS 0 // double support state
#define DRACO_STATE_LLS 1 // left leg swing state
#define DRACO_STATE_RLS 2 // right leg swing state

#define QUERY_BEFORE_TRAJECTORY -1 // if the query occurs before the trajectories
#define QUERY_AFTER_TRAJECTORY -2  // if the query occurs after the trajectories

class WalkingReferenceTrajectoryModule{
public:
	// Initialize by assigning the contact indices to a robot side.
	WalkingReferenceTrajectoryModule(const std::vector<int> & index_to_side_in);
	~WalkingReferenceTrajectoryModule();

	void setStartingConfiguration(const Eigen::Vector3d x_com_start_in,
								  const Eigen::Quaterniond x_ori_start_in,
								  const DracoFootstep & left_foot_start_in, 
								  const DracoFootstep & right_foot_start_in);
	// function which assigns a contact index to a robot side.
	void setContactIndexToSide(const std::vector<int> & index_to_side_in);

	void setFootsteps(double t_walk_start_in, const std::vector<DracoFootstep> & footstep_list_in);

	std::shared_ptr<ReactionForceSchedule> reaction_force_schedule_ptr;
	std::shared_ptr<WalkingReactionForceSchedule> walking_rfs_ptr;

	// gets the references 
	int getState(const double time);
	void getMPCRefCom(const double time, Eigen::Vector3d & x_com);
	void getMPCRefOri(const double time, Eigen::Quaterniond & x_ori);
	double getMaxNormalForce(int index, double time);


	// set that a particular contact was hit early
	// index: DRACO_LEFT_FOOTSTEP or DRACO_RIGHT_FOOTSTEP
	// time: time of early contact
	void setEarlyFootContact(const int index, const double time);


	// helper function to identify which footstep is in swing
	// if false. the foot is in not in swing for the time queried
	bool whichFootstepIndexInSwing(const double time, int & footstep_index);


private:
	std::vector<int> index_to_side_;

	double t_walk_start_ = 0.0;

	Eigen::Vector3d x_com_start_;
	Eigen::Quaterniond x_ori_start_;
	DracoFootstep left_foot_start_;
	DracoFootstep right_foot_start_;

	// List of footsteps
	std::vector<DracoFootstep> footstep_list_; // list of footsteps
	std::map<int, double> early_contact_times_;

	// Containers for trajectories
	std::shared_ptr<HermiteCurveVec> foot_pos_traj;
	std::shared_ptr<HermiteQuaternionCurve> foot_ori_traj_0;
	std::shared_ptr<HermiteQuaternionCurve> foot_ori_traj_1;




};
