#ifndef LEG_ODOMETRY_HPP
#define LEG_ODOMETRY_HPP

// #include <iit/commons/dog/forward_kinematics.h>
// #include <iit/commons/dog/feet_jacobians.h>
// #include "state_estimator/Dog.hpp"

// #include <pronto_anymal_b_commons/feet_jacobians.hpp>
// #include <pronto_anymal_b_commons/forward_kinematics.hpp>

// namespace state_estimator 
// {

// 	class LegOdometry {
// 	public:
// 		// ANYMAL
// 		LegOdometry (pronto::quadruped::FeetJacobians *feet_jacs, pronto::quadruped::ForwardKinematics *fwd_kin) 
// 		: 
// 		feet_jacs_(feet_jacs), fwd_kin_(fwd_kin)
// 		{ }	

// 		Eigen::Vector3d getAnymalFootVelocity
// 		(
// 			const pronto::quadruped::JointState& q,
// 			const pronto::quadruped::JointState& qd,
// 			const Eigen::Vector3d& omega,
// 			const pronto::quadruped::LegID& leg
// 		)
// 		{
// 			Eigen::Vector3d vel, pos;
// 			pos = this->fwd_kin_->getFootPos(q,leg);
// 			vel = this->feet_jacs_->getFootJacobian(q,leg)*qd.block<3,1>(leg*3,0) + omega.cross(pos);
// 			return vel;
// 		}

// 		pronto::quadruped::LegDataMap<Eigen::Vector3d> getAnymalFeetVelocity
// 		(
// 			const pronto::quadruped::JointState& q, 
// 			const pronto::quadruped::JointState& qd, 
// 			const Eigen::Vector3d& omega
// 		) 
// 		{
// 			pronto::quadruped::LegDataMap<Eigen::Vector3d> vel;
// 			for (pronto::quadruped::LegID leg : state_estimator::Dog::ANYMAL_LEGS)
// 			{
// 				vel[leg] = - getAnymalFootVelocity(q,qd,omega,leg);
// 			}
// 			return vel;
// 		}

// 	private:
// 		pronto::quadruped::FeetJacobians *feet_jacs_;
// 		pronto::quadruped::ForwardKinematics *fwd_kin_;

// 	}; // end class LegOdometry

// } //namespace state_estimator

#endif
