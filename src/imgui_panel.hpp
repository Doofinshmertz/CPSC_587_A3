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
} // namespace panel