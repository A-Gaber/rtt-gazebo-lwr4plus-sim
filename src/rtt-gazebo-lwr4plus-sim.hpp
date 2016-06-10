#ifndef LWR_4_PLUS_SIM_HPP
#define LWR_4_PLUS_SIM_HPP
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <Eigen/Dense>

#include <vector>

#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>

#include <thread>
#include <memory>

//#include <rci/dto/JointAngles.h>
//#include <rci/dto/JointTorques.h>
//#include <rci/dto/JointVelocities.h>

// KDL includes
#include <kdl/tree.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainfksolver.hpp>

// BOOST includes
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
// Parser include convert URDF/SDF into KDL::Chain
#include "parsertools/KDLParser.hpp"

// RST-RT includes
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>

namespace lwr {

class LWR4plusSim: public RTT::TaskContext {
public:
	LWR4plusSim(std::string const& name);bool configureHook();
	void updateHook();
	void WorldUpdateBegin();
	void WorldUpdateEnd();
	virtual ~LWR4plusSim() {
	}
	;
protected:
	bool getModel(const std::string& model_name);

	void setUrdfPath(const std::string& urdf_path);

	std::string getUrdfPath();

	bool parseURDFforKDL(std::string urdfString);

	void checkActivateBrakes();

	void gazeboUpdateHook(gazebo::physics::ModelPtr model);

	void initKDLTools();

	void setControlMode(const std::string& controlMode);

	bool gazeboConfigureHook(gazebo::physics::ModelPtr model);

	gazebo::physics::ModelPtr model;
	gazebo::event::ConnectionPtr world_begin;
	gazebo::event::ConnectionPtr world_end;

	RTT::SendHandle<gazebo::physics::ModelPtr(const std::string&, double)> get_model_handle;

	gazebo::physics::Joint_V gazebo_joints_;
	gazebo::physics::Link_V model_links_;
	std::vector<std::string> joint_names_;
	std::vector<int> joints_idx_;

	RTT::InputPort<rstrt::kinematics::JointAngles> port_JointPositionCommand;
	RTT::InputPort<rstrt::dynamics::JointTorques> port_JointTorqueCommand;

	RTT::FlowStatus jnt_trq_cmd_fs, jnt_pos_cmd_fs;

	RTT::OutputPort<rstrt::kinematics::JointAngles> port_JointPosition;
	RTT::OutputPort<rstrt::kinematics::JointVelocities> port_JointVelocity;
	RTT::OutputPort<rstrt::dynamics::JointTorques> port_JointTorque;

	rstrt::kinematics::JointAngles jnt_pos_cmd_, jnt_pos_;
	rstrt::dynamics::JointTorques jnt_trq_, jnt_trq_cmd_, jnt_trq_gazebo_cmd_;
	rstrt::kinematics::JointVelocities jnt_vel_, jnt_vel_cmd_;
	rstrt::kinematics::JointAngles jnt_pos_no_dyn_;

	// contains the urdf string for the associated model.
	std::string urdf_string;

	// KDL::Tree needed for KDL::Chain creation
	KDL::Tree kdl_tree_;
	// Needed for the different solvers
	KDL::Chain kdl_chain_;

	// Helper tools for KDL
	KDLParser p;
	// for conversion to use KDL
//	Eigen::VectorXd jnt_pos_eig, jnt_vel_eig, jnt_trq_eig, jnt_pos_cmd_eig;
	Eigen::VectorXd sumTorquesOutput, kp_default_;

	// Contains the gravity information about the environment
	KDL::Vector gravity_vector;

	KDL::JntArrayVel jntPosConfigPlusJntVelConfig_q;
	KDL::JntSpaceInertiaMatrix M_;
	KDL::JntArray G_;
	KDL::JntArray C_;
	KDL::Jacobian jac_;
	KDL::Jacobian jac_dot_;

	// KDL solver
	boost::scoped_ptr<KDL::ChainDynParam> id_dyn_solver;
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
	boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jac_dot_solver;

	boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_cart_pos_solver;
	boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> jnt_to_cart_vel_solver;

	enum ControlModes {
		JointPositionCtrl, JointTorqueCtrl, JointImpedanceCtrl
	};
	ControlModes currentControlMode;

private:
	bool is_configured;bool set_brakes_;bool set_joint_pos_no_dynamics_;
	bool old_brakes_;
	unsigned int nb_loops_;

	int DEFAULT_NR_JOINTS;

	double fbTrq;

	bool once;
};
}

#endif
