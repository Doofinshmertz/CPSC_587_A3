/**
 * CPSC 587 W26 Assignment 3
 * @name Holden Holzer
 * @email holden.holzer@ucalgary.ca
 *
 * Modified from provided Assignment 3 - Boilerplate
 * @authors Copyright 2019 Lakin Wecker, Jeremy Hart, Andrew Owens and Others (see AUTHORS)
 */

#pragma once

#include <imgui/imgui.h>
#include <simple_arm.hpp>

namespace imgui_panel {
	extern bool showPanel;
	extern ImVec4 clear_color;

	// rig
	extern rigging::SimpleArm::bone_lengths bone_lengths;
	extern rigging::SimpleArm::joint_angles joint_angles;
	extern rigging::SimpleArm::joint_angle_bounds joint_angle_bounds;
	void pushArmState(const rigging::SimpleArm& arm);

	// kinematics
	extern bool isIK;

	// skinning
	extern bool isLBS;

	// animation
	enum class TargetType {Specific, Animated, Cursor};
	extern TargetType target_type;
	extern glm::vec3 specific_target_pos;
	extern float animated_target_speed;

	// lambda function
	extern std::function<void(void)> draw;


	// added stuff
	// slover
	enum class SolverType{JT, DLS};
	extern SolverType solver_type;
	extern int max_iterations;
	extern float tolerance;
	extern float epsilon;
	extern float max_alpha;
	extern float lambda;
	extern bool use_custom_weights;
} // namespace panel