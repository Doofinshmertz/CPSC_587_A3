#pragma once

#include <givr.h>
#include <array>
#include <glm/fwd.hpp>
#include "skinned_model.hpp"

namespace rigging {
	// -----------------------------------------------------------------------------
	// A minimal articulated "arm", currently implemented for only 2 bones. The arm 
	// is a *kinematic chain* (pitch joints with an additional root yaw), assumed to 
	// be rooted at (0,0,0).
	// 
	// What students must do:
	//   1) Change kBones from 2 -> 3 or more (this automatically updates kJoints/kAngles).
	//   2) Update defaults (angles/bounds/lengths) to match the new sizes.
	//   3) Implement the FK matrix functions:
	//        - globalJointM()
	//        - globalEndEffectorM()
	//
	// Conventions assumed when completed:
	// 1) "joint i" refers to the *base* of bone i
	//    - joint 0 is the root joint (base of bone 0)
	//    - joint 1 is the base of bone 1
	//    - ...
	//    - joint kJoints-1 is the base of the last bone
	//
	// 2) Bone i connects:
	//      joint i  -->  joint i+1
	//	  except for the last bone, which connects:
	//      joint kJoints-1  -->  end effector
	//
	// 3) The end effector is NOT a joint.
	//    - It is defined as the *tip* of the last bone.
	//    - It has no angle DOF associated with it.
	//
	// 4) globalJointM(i) returns a *frame transform* for joint i.
	//
	//    By �frame transform� we mean a 4�4 matrix that represents the position
	//    and orientation of joint i�s *local coordinate frame* expressed in
	//    world coordinates. The origin of this frame is located at the base of
	//    the joint.
	//
	//    This matrix converts points expressed in joint-local coordinates into
	//    world-space coordinates.
	//
	// 5) globalEndEffectorM() returns a *frame transform* for the end effector.
	//
	//    The end effector frame is defined at the *tip of the final bone* and
	//    represents the position and orientation of that tip in world space.
	//    It is not associated with a joint and has no angle DOF of its own.
	// -----------------------------------------------------------------------------
	struct SimpleArm {
		static constexpr size_t kBones  = 3;			// TODO: Students change this to 3.
		static constexpr size_t kJoints = kBones;		// Joint at base of each bone (NOT TIP)
		static constexpr size_t kAngles = kBones + 1;	// Root joint has 2 DOF, rest have 1 DOF

		struct Bounds {
			enum class Type { Clamp, Wrap } type = Type::Clamp;
			float min = -std::numeric_limits<float>::infinity();
			float max = +std::numeric_limits<float>::infinity();
			float apply(float v) const;
		};

		using joint_angles			= glm::vec<kAngles, float>;	// Our degrees of freedom of IK
		using joint_angle_bounds	= std::array<Bounds, kAngles>;	// How to treat the angles on the bounds (clamp vs wrap)
		using bone_lengths			= std::array<float, kBones>;	// Not DOFs, just defining bone lengths
		using jacobian				= glm::mat<kAngles, 3, float>; // the jacobian matrix data structure
		using jacobian_inv 			= glm::mat<kAngles, kAngles, float>; // for holding the matrix in the damped least squares approximation
		// TODO: Update these when changing kBones
		static joint_angles			defaultJointAngles();
		static joint_angle_bounds	defaultJointAngleBounds();
		static bone_lengths			defaultBoneLengths();

		SimpleArm(
			joint_angles init_angles		= defaultJointAngles(),
			joint_angle_bounds init_bounds	= defaultJointAngleBounds(),
			bone_lengths init_lengths		= defaultBoneLengths()
		);

		// TODO: Implement these 2 functions
		glm::mat4 globalJointM(size_t joint_index) const;	// joint_index in [0..kJoints-1]
		glm::mat4 globalEndEffectorM() const;

		glm::vec3 jointPosition(size_t joint_index) const;	// joint_index in [0..kJoints-1]
		glm::vec3 endEffectorPosition() const;

		/**
		 * jacobian transpose inverse kinematics function
		 * uses the jacobian transpose to attempt to move the end effector to the target position
		 */
		void moveToPositionJT(glm::vec3 e_t, float epsilon, float tolerance, float alpha_max, size_t max_iterations);

		/**
		 * use the damped least squares to attempt to move the end effector to  the target position
		 */
		void moveToPositionDLS(glm::vec3 e_t, float epsilon, float tolerance, float lambda, size_t max_iterations);


		/**
		 * project the target so that is a maximum distance of "max_delta" away from the current position
		 */
		glm::vec3 getProjectedTarget(glm::vec3 e_t, glm::vec3 e, float max_delta);

		/**
		 *  Use the jacobian transpose to solve for the angles 
		 */
		joint_angles solveAnglesJT(glm::vec3 delta_e, float epsilon, float alpha_max);

		/**
		 * Use the damped least squares methode to solve for the angles
		 */
		joint_angles solveAnglesDLS(glm::vec3 delta_e, float epsilon, float lambda);

		/**
		 * calculates the jacobian of the system using the current arm position and given epsilon value
		 * @param epsilon the differencing amount for taking the numerical deriviative of the system with respect to the angles
		 */
		jacobian calcJacobian(float epsilon);


		/**
		 * pre-computes the rest inverted position matrices for the bones
		 */
		void SetupRestPositionMatrices();

		/**
		 * deforms the mesh around the bones according to the given bone weights
		 */
		void DeformMeshToBones(skinning::SkinnedModel *model);

		// debuggin utility
		void printJacobian(jacobian j);
		void printJacobianInv(jacobian_inv j);

		void applyConstraints();

		// Render Helper
		std::array<givr::geometry::Cylinder, kBones> armGeometry(float radius) const;

		// Data
		joint_angles angles;
		joint_angle_bounds angle_bounds;
		bone_lengths lengths;
		std::vector<glm::mat4> inv_rest_mats; // the inverted rest position transformation matrices (pre-computed for skinning)
	};
} // namespace rigging