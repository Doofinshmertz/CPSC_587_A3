#include "imgui_panel.hpp"

namespace imgui_panel {
	// default values
	bool showPanel = true;
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

	// solver stuff
	int max_iterations = 100;
	float tolerance = 0.001f;
	float epsilon = 0.001f;
	float max_alpha = 10.0f;
	float lambda = 0.5f;
	bool use_custom_weights = false;
	SolverType solver_type = SolverType::JT;

	// rig
	rigging::SimpleArm::bone_lengths bone_lengths				= rigging::SimpleArm::defaultBoneLengths();
	rigging::SimpleArm::joint_angles joint_angles				= rigging::SimpleArm::defaultJointAngles();
	rigging::SimpleArm::joint_angle_bounds joint_angle_bounds	= rigging::SimpleArm::defaultJointAngleBounds();
	void pushArmState(const rigging::SimpleArm& arm)
	{
		bone_lengths		= arm.lengths;
		joint_angles		= arm.angles;
		joint_angle_bounds	= arm.angle_bounds;
	}
	//Kinematics
	bool isIK = false;

	// skinning
	bool isLBS = false;

	// animation
	TargetType target_type = TargetType::Specific;
	glm::vec3 specific_target_pos = { 0.0f, 0.0f, 0.0f };
	float animated_target_speed = 1.f;

	std::function<void(void)> draw = [](void) {
		if (showPanel && ImGui::Begin("Panel", &showPanel, ImGuiWindowFlags_MenuBar)) {
			ImGui::Spacing();
			ImGui::Separator();

			ImGui::ColorEdit3("Clear color", (float*)&clear_color);

			ImGui::Spacing();
			ImGui::Separator();

			ImGui::Checkbox("Use Inverse Kinematics", &isIK);

			// if using inverse kinematics then show the controll pannel for it
			if(isIK)
			{
				// controll the maximum number of iterations
				ImGui::DragInt("Max Iterations", &max_iterations, 1, 1, 1000);
				// controll the tolerance
				ImGui::DragFloat("Tolerance", &tolerance, 0.001f, 0.001f, 1.0f, "%.4f");
				// controll the finite step
				ImGui::DragFloat("Finite step (rad)", &epsilon, 0.001f, 0.0001f, 0.1f, "%.4f");
				// select the solver type
				static constexpr const char* solverTypeLables[]={"Jacobian Transpose", "Damped Least Squares"};
				int solver = static_cast<int>(solver_type);
				if(ImGui::Combo("Solver Type", &solver, solverTypeLables, 2))
				{
					solver_type = static_cast<SolverType>(solver);
				}

				// give different options depending on the solver type
				if(solver_type == SolverType::JT)
				{
					ImGui::DragFloat("Alpha max", &max_alpha, 0.001f, 0.001f, 100.0f, "%0.4f"); 
				}
				else
				{
					ImGui::DragFloat("lambda", &lambda, 0.01f, 0.01f, 10.0f, "%0.4f");
				}
			}


			ImGui::Checkbox("Use Skinning Model", &isLBS);
			ImGui::Checkbox("Use Custom Calculated Weights", &use_custom_weights);

			ImGui::Spacing();
			ImGui::Separator();

			static constexpr const char* targetTypeLabels[] = {
				"Specific",
				"Animated",
				"Cursor"
			};    
			int current = static_cast<int>(target_type);
			if (ImGui::Combo("Target Type", &current, targetTypeLabels, 3)) {
				target_type = static_cast<TargetType>(current);
			}

			switch (target_type) {
			case TargetType::Specific: {
				ImGui::DragFloat3(
					"Specific Target Pos", (float*)&specific_target_pos, 0.05f, -20.f, 20.f, "%.2f"
				);
			} break;
			case TargetType::Animated: {
				ImGui::DragFloat(
					"Animated Target Speed", &animated_target_speed, 0.05f, 0.f, 2.f, "%.2f"
				);
			} break;
			case TargetType::Cursor: { /* Nothing */ } break;
			}

			ImGui::Spacing();
			ImGui::Separator();

			// Model has a specific bone length to use and thus the input should be disabled
			if (!isLBS) {
				for (size_t i = 0; i < bone_lengths.size(); ++i) {
					ImGui::PushID(static_cast<int>(i));
					ImGui::SliderFloat("Bone Length", &bone_lengths[i], 0.f, 12.f);
					ImGui::PopID();
				}
			}

			// No forward kinematics under IK and thus the input should be disabled
			if (!isIK) {
				for (size_t i = 0; i < joint_angles.length(); ++i) {
					const auto& bmin = joint_angle_bounds[i].min;
					const auto& bmax = joint_angle_bounds[i].max;

					// Convert from radians to degrees for UI
					const float min_deg = std::isfinite(bmin) ? glm::degrees(bmin) : -180.f;
					const float max_deg = std::isfinite(bmax) ? glm::degrees(bmax) : +180.f;
					float angle_deg = glm::degrees(joint_angles[i]);

					ImGui::PushID(static_cast<int>(i));
					switch (joint_angle_bounds[i].type) {
					case rigging::SimpleArm::Bounds::Type::Wrap: {
						ImGui::DragFloat("Angle (deg)", &angle_deg, 1.f, min_deg, max_deg, "%.1f");
					} break;
					case rigging::SimpleArm::Bounds::Type::Clamp: {
						ImGui::SliderFloat("Angle (deg)", &angle_deg, min_deg, max_deg, "%.1f");
					} break;
					}
					ImGui::PopID();

					// Convert back
					joint_angles[i] = glm::radians(angle_deg);
				}
			}

			ImGui::Spacing();
			ImGui::Separator();
			float frame_rate = ImGui::GetIO().Framerate;
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
				1000.0f / frame_rate, frame_rate);

			ImGui::End();
		}
	};

} // namespace panel