#include "rtt-gazebo-lwr4plus-sim.hpp"
#include <Eigen/Dense>
#include <kdl/kdl.hpp>
#include <rtt/Operation.hpp>

#include <string>
#include <fstream>
#include <streambuf>

using namespace lwr;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

LWR4plusSim::LWR4plusSim(std::string const& name) :
		TaskContext(name), is_configured(false), set_brakes_(false), old_brakes_(false), set_joint_pos_no_dynamics_(
		true), nb_loops_(0), gravity_vector(0., 0., -9.81289), DEFAULT_NR_JOINTS(7), currentControlMode(JointPositionCtrl), once(true) {

	this->provides("gazebo")->addOperation("WorldUpdateBegin",
			&LWR4plusSim::WorldUpdateBegin, this, RTT::ClientThread);
	this->provides("gazebo")->addOperation("WorldUpdateEnd",
			&LWR4plusSim::WorldUpdateEnd, this, RTT::ClientThread);

	this->addOperation("getModel", &LWR4plusSim::getModel, this, ClientThread);

	this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc(
			"Input for JointPosition-cmds from Orocos to Gazebo world.");
	this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc(
			"Input for JointTorque-cmds from Orocos to Gazebo world.");

	this->ports()->addPort("JointVelocity", port_JointVelocity).doc(
			"Output for JointVelocity-fbs from Gazebo to Orocos world.");
	this->ports()->addPort("JointTorque", port_JointTorque).doc(
			"Output for JointTorques-fbs from Gazebo to Orocos world.");
	this->ports()->addPort("JointPosition", port_JointPosition).doc(
			"Output for JointPosition-fbs from Gazebo to Orocos world.");

	this->addAttribute("fbTrq", fbTrq);

	this->provides("misc")->addAttribute("urdf_string", urdf_string);
	this->provides("misc")->addOperation("setUrdfPath", &LWR4plusSim::setUrdfPath,
			this, RTT::OwnThread);
	this->provides("misc")->addOperation("getUrdfPath", &LWR4plusSim::getUrdfPath,
			this, RTT::ClientThread);

	this->addOperation("setControlMode", &LWR4plusSim::setControlMode,
				this, RTT::ClientThread);

	// using boost instead of std??
	world_begin = gazebo::event::Events::ConnectWorldUpdateBegin(
			boost::bind(&LWR4plusSim::WorldUpdateBegin, this));
	world_end = gazebo::event::Events::ConnectWorldUpdateEnd(
			boost::bind(&LWR4plusSim::WorldUpdateEnd, this));

}

void LWR4plusSim::setControlMode(const std::string& controlMode) {
	if (controlMode == "JointPositionCtrl") {
		currentControlMode = JointPositionCtrl;
	} else if (controlMode == "JointTorqueCtrl") {
		currentControlMode = JointTorqueCtrl;
	}
}

void LWR4plusSim::setUrdfPath(const std::string& urdf_path) {
	std::ifstream ifs(urdf_path);
	urdf_string.assign((std::istreambuf_iterator<char>(ifs)),
			(std::istreambuf_iterator<char>()));
	parseURDFforKDL(urdf_string);
}

std::string LWR4plusSim::getUrdfPath() {
	return urdf_string;
}

void LWR4plusSim::initKDLTools() {
	// initialize KDL fields
	jntPosConfigPlusJntVelConfig_q.resize(DEFAULT_NR_JOINTS);
	G_.resize(DEFAULT_NR_JOINTS);
	jac_.resize(DEFAULT_NR_JOINTS);
	jac_dot_.resize(DEFAULT_NR_JOINTS);
	jac_dot_.data.setZero();
	jac_.data.setZero();
	M_.resize(DEFAULT_NR_JOINTS);
	C_.resize(DEFAULT_NR_JOINTS);

//	jnt_pos_eig.resize(DEFAULT_NR_JOINTS);
//	jnt_pos_eig.setZero();
//	jnt_vel_eig.resize(DEFAULT_NR_JOINTS);
//	jnt_vel_eig.setZero();
//	jnt_trq_eig.resize(DEFAULT_NR_JOINTS);
//	jnt_trq_eig.setZero();
//
//	jnt_pos_cmd_eig.resize(DEFAULT_NR_JOINTS);
//	jnt_pos_cmd_eig.setZero();

	kp_default_.resize(DEFAULT_NR_JOINTS);

	sumTorquesOutput.resize(DEFAULT_NR_JOINTS);
	sumTorquesOutput.setZero();
}

bool LWR4plusSim::parseURDFforKDL(std::string urdfString) {
	std::string chain_root_link_name = "lwr_arm_base_link";
	// name of the tip link for the KDL chain
	std::string chain_tip_link_name = "lwr_arm_7_link";


	if (!p.initTreeAndChainFromURDFString(urdfString, chain_root_link_name,
			chain_tip_link_name, kdl_tree_, kdl_chain_)) {
		log(Error) << "[" << this->getName() << "] URDF could not be parsed !"
				<< endlog();
		return false;
	}

	if (kdl_tree_.getNrOfJoints() > 0) {
		log(Info) << "URDF parsed !" << endlog();

		log(Info) << "NrOfJoints: " << kdl_tree_.getNrOfJoints()
				<< ", NrOfSegments: " << kdl_tree_.getNrOfSegments()
				<< endlog();

		KDL::SegmentMap::const_iterator it;
		for (it = kdl_tree_.getSegments().begin();
				it != kdl_tree_.getSegments().end(); it++) {
			log(Info) << "Parsed Joints in KDL-Tree " << it->first << endlog();
		}
		// initialize solvers
		id_dyn_solver.reset(new KDL::ChainDynParam(kdl_chain_, gravity_vector));
		jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
		jnt_to_jac_dot_solver.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));

		jnt_to_cart_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
		jnt_to_cart_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));

		return true;
	} else {
		log(Error) << "[" << this->getName() << "] URDF could not be parsed !"
				<< endlog();
		return false;
	}
}

bool LWR4plusSim::getModel(const std::string& gazebo_comp_name,
		const std::string& model_name, double timeout_s) {
	if (model) {
		log(Warning) << "Model [" << model_name << "] already loaded !"
				<< endlog();
		return true;
	}
	gazebo::printVersion();
	if (!gazebo::physics::get_world()) {
		log(Error) << "getWorldPtr does not seem to exists" << endlog();
		return false;
	}
	model = gazebo::physics::get_world()->GetModel(model_name);
	if (model) {
		log(Info) << "Model [" << model_name << "] successfully loaded !"
				<< endlog();
		return true;
	}
	return bool(model);
}

void LWR4plusSim::updateHook() {
}

bool LWR4plusSim::gazeboConfigureHook(gazebo::physics::ModelPtr model) {
	if (model.get() == NULL) {
		RTT::log(RTT::Error) << "No model could be loaded" << RTT::endlog();
		return false;
	}

	// Get the joints
	gazebo_joints_ = model->GetJoints();
	model_links_ = model->GetLinks();

	RTT::log(RTT::Info) << "Model has " << gazebo_joints_.size() << " joints"
			<< RTT::endlog();
	RTT::log(RTT::Info) << "Model has " << model_links_.size() << " links"
			<< RTT::endlog();

	//NOTE: Get the joint names and store their indices
	// Because we have base_joint (fixed), j0...j6, ati_joint (fixed)
	int idx = 0;
	joints_idx_.clear();
	for (gazebo::physics::Joint_V::iterator jit = gazebo_joints_.begin();
			jit != gazebo_joints_.end(); ++jit, ++idx) {

		const std::string name = (*jit)->GetName();
		// NOTE: Remove fake fixed joints (revolute with upper==lower==0
		// NOTE: This is not used anymore thanks to <disableFixedJointLumping>
		// Gazebo option (ati_joint is fixed but gazebo can use it )

		if ((*jit)->GetLowerLimit(0u) == (*jit)->GetUpperLimit(0u)) {
			RTT::log(RTT::Warning) << "Not adding (fake) fixed joint [" << name
					<< "] idx:" << idx << RTT::endlog();
			continue;
		}
		joints_idx_.push_back(idx);
		joint_names_.push_back(name);
		RTT::log(RTT::Info) << "Adding joint [" << name << "] idx:" << idx
				<< RTT::endlog();
	}

	if (joints_idx_.size() == 0) {
		RTT::log(RTT::Error) << "No Joints could be added, exiting"
				<< RTT::endlog();
		return false;
	}

	RTT::log(RTT::Info) << "Gazebo model found " << joints_idx_.size()
			<< " joints " << RTT::endlog();

	jnt_pos_cmd_ = rstrt::kinematics::JointAngles(7);
	jnt_pos_cmd_.angles.setZero();
	jnt_pos_ = rstrt::kinematics::JointAngles(7);
	jnt_pos_.angles.setZero();

	jnt_pos_no_dyn_ = rstrt::kinematics::JointAngles(7);
	jnt_pos_no_dyn_.angles.setConstant(-0.3);

	jnt_trq_gazebo_cmd_ = rstrt::dynamics::JointTorques(7);
	jnt_trq_gazebo_cmd_.torques.setZero();
	jnt_trq_cmd_ = rstrt::dynamics::JointTorques(7);
	jnt_trq_cmd_.torques.setZero();
	jnt_trq_ = rstrt::dynamics::JointTorques(7);
	jnt_trq_.torques.setZero();

	jnt_vel_cmd_ = rstrt::kinematics::JointVelocities(7);
	jnt_vel_cmd_.velocities.setZero();
	jnt_vel_ = rstrt::kinematics::JointVelocities(7);
	jnt_vel_.velocities.setZero();

	port_JointPosition.setDataSample(jnt_pos_);
	port_JointVelocity.setDataSample(jnt_vel_);
	port_JointTorque.setDataSample(jnt_trq_);

	// TODO #1) convert urdf to kdl and provide it!

	RTT::log(RTT::Warning) << "Done configuring gazebo" << RTT::endlog();
	return true;
}

bool LWR4plusSim::configureHook() {
	this->is_configured = gazeboConfigureHook(model);
	initKDLTools();
	return is_configured;
}

ORO_CREATE_COMPONENT(lwr::LWR4plusSim)
