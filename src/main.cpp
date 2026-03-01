#include <givio.h>
#include <givr.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>
#include <glm/ext/scalar_constants.hpp>

#include "picking_controls.h"
#include "turntable_controls.h"

#include "imgui_panel.hpp"
#include "simple_arm.hpp"
#include "skinned_model.hpp"
#include <cmath>

using namespace giv;
using namespace giv::io;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

class KeyboardTurnTableControls {
public:
	KeyboardTurnTableControls(Window& w, TurnTableCamera& c) 
		: m_window(w)
		, m_camera(c) 
		, m_cursor_pos({0.f,0.f})
	{
		m_window.keyboardCommands()
			| Key(GLFW_KEY_ESCAPE,	[&](auto e) { m_window.shouldClose();												})
			| Key(GLFW_KEY_LEFT,	[&](auto e) { if (e.action != GLFW_RELEASE) m_camera.rotateAroundX(-0.1f);			})
			| Key(GLFW_KEY_RIGHT,	[&](auto e) { if (e.action != GLFW_RELEASE) m_camera.rotateAroundX(+0.1f);			})
			| Key(GLFW_KEY_UP,		[&](auto e) { if (e.action != GLFW_RELEASE) m_camera.rotateAroundY(-0.1f);			})
			| Key(GLFW_KEY_DOWN,	[&](auto e) { if (e.action != GLFW_RELEASE) m_camera.rotateAroundY(+0.1f);			})
			| Key(GLFW_KEY_1,		[&](auto e) { imgui_panel::target_type = imgui_panel::TargetType::Specific;			})
			| Key(GLFW_KEY_2,		[&](auto e) { imgui_panel::target_type = imgui_panel::TargetType::Animated;			})
			| Key(GLFW_KEY_3,		[&](auto e) { imgui_panel::target_type = imgui_panel::TargetType::Cursor;			})
			| Key(GLFW_KEY_KP_1,	[&](auto e) { imgui_panel::target_type = imgui_panel::TargetType::Specific;			})
			| Key(GLFW_KEY_KP_2,	[&](auto e) { imgui_panel::target_type = imgui_panel::TargetType::Animated;			})
			| Key(GLFW_KEY_KP_3,	[&](auto e) { imgui_panel::target_type = imgui_panel::TargetType::Cursor;			})
			| Key(GLFW_KEY_I,		[&](auto e) { if (e.action == GLFW_PRESS) imgui_panel::isIK = !imgui_panel::isIK;	})
			| Key(GLFW_KEY_S,		[&](auto e) { if (e.action == GLFW_PRESS) imgui_panel::isLBS = !imgui_panel::isLBS;	});
		m_window.scrollWheelCommand() = [&](auto e) {
			m_camera.zoom(e.yOffset);
		};
		m_window.cursorCommand() = [&](auto const& cursor_pos) {
			m_cursor_pos = cursor_pos;
		};
	}

	int px() const { return static_cast<int>(m_cursor_pos.x); };
	int py() const { return static_cast<int>(m_cursor_pos.y); };

private:
	Window& m_window;
	TurnTableCamera& m_camera;
	CursorPosition m_cursor_pos;
};

// program entry point
int main(void) {
	// initialize OpenGL and window
	GLFWContext glContext;
	glContext.glMajorVesion(3)
		.glMinorVesion(3)
		.glForwardComaptability(true)
		.glCoreProfile()
		.glAntiAliasingSamples(4)
		.matchPrimaryMonitorVideoMode();
	std::cout << glfwVersionString() << '\n';

	// setup window (OpenGL context)
	ImGuiWindow window = glContext.makeImGuiWindow(Properties()
		.size(dimensions{ 1000, 1000 })
		.title("Inverse Kinematics")
		.glslVersionString("#version 330 core"));
	// set our imgui update function
	panel::update_lambda_function = imgui_panel::draw;

	ViewContext view = View(TurnTable(), Perspective());
	KeyboardTurnTableControls controls(window, view.camera);
	view.camera.latitude() = 0.25f * glm::pi<float>(); //So you can see downward initially

	rigging::SimpleArm arm; //Default Geometry
	skinning::SkinnedModel model = skinning::SkinnedModel::loadFromFile("./models/bone_mesh_weights.txt").value();

	// --------------------------------
	glm::vec3 light_pos = { 2.0f, 15.0f, 2.0f };

	Phong plate_s = Phong(Colour(1.0, 0.0, 0.1529), AmbientFactor(0.1), LightPosition(light_pos));
	Phong arm_s   = Phong(Colour(1.0, 1.0, 0.1529), AmbientFactor(0.1), LightPosition(light_pos));

	Mesh plate_g			= Mesh(Filename("./models/plate.obj"));
	Sphere sphere_g			= Sphere(Radius(0.2));
	TriangleSoup model_g	= model.makeMesh();

	RenderContext plate_rc				= createRenderable(plate_g, plate_s);
	InstancedRenderContext sphere_irc   = createInstancedRenderable(sphere_g, arm_s);
	RenderContext bone_rc				= createRenderable(Cylinder(Point1(0, 0, 0), Point2(0, 1, 0)), arm_s);
	RenderContext model_rc				= createRenderable(model_g, arm_s);
	// --------------------------------
	// Animation 
	float t = 0.f;
	float factor = 1.f;

	// Actual Target
	glm::vec3 target = { 0.f, 0.f, 0.f };

	// main loop
	mainloop(std::move(window), [&](float dt /*** Time since last frame ***/) {
		// -------------------- ------ -------------------- //
		// -------------------- Target -------------------- //
		// -------------------- ------ -------------------- //
		switch (imgui_panel::target_type) {
			case imgui_panel::TargetType::Specific: { 
				// Use specified
				target = imgui_panel::specific_target_pos;
			} break;
			case imgui_panel::TargetType::Animated: {
				// Simple function to animate the target in a interesting fashion
				t += factor * imgui_panel::animated_target_speed * dt;
				if (t < 0.f || t > 10.f) {
					t = std::clamp(t, 0.f, 10.f);
					factor *= -1.f;
				}
				target = glm::vec3{
					0.f + 9.f * std::cos(0.5f * glm::pi<float>() * t),
					5.f + 1.f * std::sin(5.0f * glm::pi<float>() * t),
					0.f - 3.f * std::sin(0.5f * glm::pi<float>() * t)
				};
				imgui_panel::specific_target_pos = target;
			} break;
			case imgui_panel::TargetType::Cursor: {
				//Move the target in the view plane by tracking the cursor
				glm::mat4 MVP = view.projection.projectionMatrix() * view.camera.viewMatrix();
				glm::mat4 invMVP = glm::inverse(MVP);
				glm::vec3 NDC = world3DToNDC(target, MVP);
				target = pixelToWorld3D(
					controls.px(), controls.py(), window.width(), window.height(), invMVP, NDC.z
				);
				imgui_panel::specific_target_pos = target;
			} break;
		}
		// -------------------- ------ -------------------- //
		// -------------------- ------ -------------------- //
		// -------------------- ------ -------------------- //



		// -------------------- ---------- -------------------- //
		// -------------------- Kinematics -------------------- //
		// -------------------- ---------- -------------------- //
		if (imgui_panel::isIK) {
			// TODO: your IK process(s) should be here
			if(imgui_panel::solver_type == imgui_panel::SolverType::JT)
			{
				arm.moveToPositionJT(target, imgui_panel::epsilon, imgui_panel::tolerance, imgui_panel::max_alpha, static_cast<size_t>(imgui_panel::max_iterations));
			}
			else
			{
				arm.moveToPositionDLS(target, imgui_panel::epsilon, imgui_panel::tolerance, imgui_panel::lambda, static_cast<size_t>(imgui_panel::max_iterations));
			}
			arm.applyConstraints();// Called somewhere in here
		}
		else { //Simply apply the inputs
			arm.angles = imgui_panel::joint_angles;
			arm.applyConstraints();
			imgui_panel::joint_angles = arm.angles;
		}
		// -------------------- ---------- -------------------- //
		// -------------------- ---------- -------------------- //
		// -------------------- ---------- -------------------- //



		// -------------------- -------- -------------------- //
		// -------------------- Skinning -------------------- //
		// -------------------- -------- -------------------- //
		if (imgui_panel::isLBS) {
			// The bones are defined by the model, fix these for arm + imgui
			const size_t n = std::min(model.bones.size(), arm.lengths.size());
			for(size_t i = 0; i < n; i++) arm.lengths[i] = model.bones[i].length;
			imgui_panel::bone_lengths = arm.lengths;

			// TODO: Calculate tranformations to use below in Update 
		}
		else { // Pass input to arm
			arm.lengths = imgui_panel::bone_lengths;
		}
		// -------------------- -------- -------------------- //
		// -------------------- -------- -------------------- //
		// -------------------- -------- -------------------- //



		// -------------------- --------- -------------------- //
		// -------------------- Rendering -------------------- //
		// -------------------- --------- -------------------- //
		auto color = imgui_panel::clear_color;
		glClearColor(color.x, color.y, color.z, color.z);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		view.projection.updateAspectRatio(window.width(), window.height());

		// ----------- Plate -------------
		draw(plate_rc, view, glm::scale(glm::vec3(arm.lengths[0]))); // Scale by root arm length
		
		// ----------- Spheres -----------
		if (!imgui_panel::isLBS) {
			for (int i = 0; i < arm.kJoints; i++) //Joints
				addInstance(sphere_irc, glm::translate(arm.jointPosition(i)));
			//End Effector
			addInstance(sphere_irc, glm::translate(arm.endEffectorPosition()));
		}
		addInstance(sphere_irc, glm::translate(target));
		draw(sphere_irc, view); //Target + possible (joints + end effector)
		
		// ----------- Bones -------------
		if (!imgui_panel::isLBS) {
			for (const Cylinder& bone_g : arm.armGeometry(0.1f)) {
				updateRenderable(bone_g, arm_s, bone_rc);
				draw(bone_rc, view);
			}
		}

		// ----------- Model -------------
		if (imgui_panel::isLBS) {
			// TODO: Update model_g (Triangle soup) for the articulated model
			updateRenderable(model_g, arm_s, model_rc);
			draw(model_rc, view);
		}
		
		// -------------------- --------- -------------------- //
		// -------------------- --------- -------------------- //
		// -------------------- --------- -------------------- //
	});
	return EXIT_SUCCESS;
}