#include "rtt-gazebo-lwr4plus-sim.hpp"
#include <Eigen/Dense>
#include <kdl/kdl.hpp>

#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

using namespace lwr;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

void LWR4plusSim::WorldUpdateBegin() {
	if (!is_configured && !isRunning())
		return;

	// Get state
	for (unsigned j = 0; j < joints_idx_.size(); j++) {
		jnt_pos_.angles(j) = gazebo_joints_[joints_idx_[j]]->GetAngle(0).Radian();
		jnt_vel_.velocities(j) = gazebo_joints_[joints_idx_[j]]->GetVelocity(0);

		gazebo::physics::JointWrench w =
				gazebo_joints_[joints_idx_[j]]->GetForceTorque(0u);
		gazebo::math::Vector3 a = gazebo_joints_[joints_idx_[j]]->GetLocalAxis(
				0u);
		jnt_trq_.torques(j) = a.Dot(w.body1Torque); // perhaps change to GetForceTorque

		// Reset commands from users
		jnt_trq_cmd_.torques.setZero();
		jnt_trq_gazebo_cmd_.torques.setZero();
	}

	// TODO debug test
	fbTrq = jnt_trq_.torques(1);

	// Do set pos if asked
	if (set_joint_pos_no_dynamics_) {
		for (unsigned j = 0; j < joints_idx_.size(); j++) {
#ifdef GAZEBO_GREATER_6
			gazebo_joints_[joints_idx_[j]]->SetPosition(0, (double) jnt_pos_no_dyn_.angles(j));
#else
			gazebo_joints_[joints_idx_[j]]->SetAngle(0,
					(double) jnt_pos_no_dyn_.angles(j));
#endif
		}

		jnt_pos_.angles = jnt_pos_no_dyn_.angles; // might be a reference issue here... also below!?? TODO
		jnt_pos_cmd_.angles = jnt_pos_no_dyn_.angles;
		set_joint_pos_no_dynamics_ = false;
	}

	jnt_trq_cmd_fs = port_JointTorqueCommand.readNewest(jnt_trq_cmd_);
	jnt_pos_cmd_fs = port_JointPositionCommand.readNewest(jnt_pos_cmd_);

//	jnt_trq_cmd_fs = NewData;
//	jnt_trq_cmd_->setFromNm(0, 300.0);
//	jnt_trq_cmd_->setFromNm(1, 100.0);
//	jnt_trq_cmd_->setFromNm(2, -0.1868);
//	jnt_trq_cmd_->setFromNm(3, -1.5733);
//	jnt_trq_cmd_->setFromNm(4, 0.1194);
//	jnt_trq_cmd_->setFromNm(5, 1.0685);
//	jnt_trq_cmd_->setFromNm(6, 0.0);

//	if (once) {
		jnt_pos_cmd_fs = NewData;
		jnt_pos_cmd_.angles(0) = -0.1192;
		jnt_pos_cmd_.angles(1) = 0.3114;
		jnt_pos_cmd_.angles(2) = -0.1868;
		jnt_pos_cmd_.angles(3) = -1.5733;
		jnt_pos_cmd_.angles(4) = 0.1194;
		jnt_pos_cmd_.angles(5) = 1.0685;
		jnt_pos_cmd_.angles(6) = 0.0;
//		once = false;
//	} else {
//		jnt_pos_cmd_fs = OldData;
//	}

	// IF old data, dont break but follow the controllers previous target!!! TODO

	// calculate dynamics:
	/* ### Convert Data to be used with KDL solvers */
//	p.convertRealVectorToEigenVectorXd(jnt_pos_->radVector(), jnt_pos_eig);
//	p.convertRealVectorToEigenVectorXd(jnt_vel_->rad_sVector(), jnt_vel_eig);
//	p.convertRealVectorToEigenVectorXd(jnt_trq_->NmVector(), jnt_trq_eig);
//
//	p.convertRealVectorToEigenVectorXd(jnt_pos_cmd_->radVector(),
//			jnt_pos_cmd_eig);

	/* ### initialize strange stuff for solvers */
	jntPosConfigPlusJntVelConfig_q.q.data = jnt_pos_.angles;
	jntPosConfigPlusJntVelConfig_q.qdot.data = jnt_vel_.velocities;

	/* ### execute solvers for inv.Dynamics */
	// calculate matrices H (inertia),C(coriolis) and G(gravitation)
	id_dyn_solver->JntToMass(jntPosConfigPlusJntVelConfig_q.q, M_);

	id_dyn_solver->JntToGravity(jntPosConfigPlusJntVelConfig_q.q, G_);
	id_dyn_solver->JntToCoriolis(jntPosConfigPlusJntVelConfig_q.q,
			jntPosConfigPlusJntVelConfig_q.qdot, C_);

	/* ### execute solver for Jacobian based on velocities */
	jnt_to_jac_solver->JntToJac(jntPosConfigPlusJntVelConfig_q.q, jac_,
			kdl_chain_.getNrOfSegments());

	jnt_to_jac_dot_solver->JntToJacDot(jntPosConfigPlusJntVelConfig_q, jac_dot_,
			kdl_chain_.getNrOfSegments());

	// check if brakes need to be activated
	if (jnt_pos_cmd_fs != NewData && jnt_trq_cmd_fs != NewData) {
		if (!set_brakes_) {
			jnt_pos_no_dyn_.angles = jnt_pos_.angles; // could be an reference issue here TODO
			set_brakes_ = true;
		}
	} else {
		old_brakes_ = set_brakes_;
		//check current control mode:
		switch (currentControlMode) {
		case JointPositionCtrl:
			if ((set_brakes_ = !(jnt_pos_cmd_fs == NewData)) == true) {
				break;
			}
			if (jnt_pos_cmd_fs == NewData) {
				// basic gravity compensation
				sumTorquesOutput = G_.data;
				// position command
				kp_default_.setConstant(1);
				sumTorquesOutput += kp_default_.asDiagonal()
						* (jnt_pos_cmd_.angles - jnt_pos_.angles); // - kd_default_.asDiagonal()*jnt_vel_eig;

				RTT::log(RTT::Error) << "sumTorquesOutput: " << sumTorquesOutput
						<< RTT::endlog();
				RTT::log(RTT::Error) << "jnt_pos_cmd_eig: " << jnt_pos_cmd_
						<< RTT::endlog();
				RTT::log(RTT::Error) << "jnt_pos_eig: " << jnt_pos_
						<< RTT::endlog();
				RTT::log(RTT::Error) << "G_: " << G_ << RTT::endlog();

				for (int k = 0; k < DEFAULT_NR_JOINTS; k++) {
					jnt_trq_gazebo_cmd_.torques = sumTorquesOutput;
				}
			}
			break;
		case JointTorqueCtrl:
			if ((set_brakes_ = !(jnt_trq_cmd_fs == NewData)) == true) {
							break;
			}
			jnt_trq_gazebo_cmd_.torques = jnt_trq_cmd_.torques; // TODO could be a reference issue here...
			break;

		case JointImpedanceCtrl:
			// TODO
			set_brakes_ = true;
			break;
		}

		if ((!old_brakes_) && (set_brakes_)) {
			jnt_pos_no_dyn_.angles = jnt_pos_.angles; // TODO could be a reference issue here...
			set_brakes_ = true;
		}
	}

	// write feedback to Orocos
	if (port_JointPosition.connected())
		port_JointPosition.write(jnt_pos_);

	if (port_JointVelocity.connected())
		port_JointVelocity.write(jnt_vel_);

	if (port_JointTorque.connected())
		port_JointTorque.write(jnt_trq_);

}

void LWR4plusSim::checkActivateBrakes() {
	if (!set_brakes_) {
		set_brakes_ = true;
	}
}

void LWR4plusSim::WorldUpdateEnd() {
	if (!is_configured && !isRunning())
		return;

	if (!set_brakes_) {
//		if (jnt_pos_cmd_fs == NewData) {
//			for (unsigned j = 0; j < joints_idx_.size(); j++) {
//#ifdef GAZEBO_GREATER_6
//				gazebo_joints_[joints_idx_[j]]->SetPosition(0, jnt_pos_cmd_->rad(j));
//#else
//				gazebo_joints_[joints_idx_[j]]->SetAngle(0,
//						jnt_pos_cmd_->rad(j));
//#endif
//			}
//		} else if (jnt_trq_cmd_fs == NewData) {
		for (unsigned j = 0; j < joints_idx_.size(); j++) {
			gazebo_joints_[joints_idx_[j]]->SetForce(0,
					(double) jnt_trq_gazebo_cmd_.torques(j));
		}
//		}
	} else {
		// brakes are active so brake!
		for (unsigned j = 0; j < joints_idx_.size(); j++) {
#ifdef GAZEBO_GREATER_6
			gazebo_joints_[joints_idx_[j]]->SetPosition(0, (double) jnt_pos_no_dyn_.angles(j));
#else
			gazebo_joints_[joints_idx_[j]]->SetAngle(0,
					(double) jnt_pos_no_dyn_.angles(j));
#endif
		}
	}

}
