// simple_arm.cpp
#include "simple_arm.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <cassert>

#define GLM_ENABLE_EXPERIMENTAL
#include <array>
#include <givr.h>
#include <glm/trigonometric.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/scalar_constants.hpp>

inline void assert_or_throw(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

namespace rigging {

    float SimpleArm::Bounds::apply(float v) const {
        // If bounds are infinite, do nothing.
        if (!std::isfinite(min) || !std::isfinite(max)) return v;

        // Degenerate interval: treat as fixed.
        if (!(max > min)) return min;

        switch (type) {
        case Type::Clamp: {
            return std::clamp(v, min, max);
        } break;
        case Type::Wrap: {
            const float span = max - min;
            float x = std::fmod(v - min, span);
            if (x < 0.f) x += span;
            return x + min;
        } break;
        }

        throw std::runtime_error("Should not be here, something bad happend");
        return v; // unreachable, keeps compiler happy
    }

    // TODO: Update when changing kBones
    SimpleArm::joint_angles SimpleArm::defaultJointAngles() {
        // Angle layout: [root_yaw, root_pitch, joint1_pitch]
        assert_or_throw(kAngles == 3, "Forgot to update default"); // Removed this (recommended) or update it when adding bone
        return { 0.f, 0.f, glm::radians(30.f) };
    }

    // TODO: Update when changing kBones
    SimpleArm::joint_angle_bounds SimpleArm::defaultJointAngleBounds() {
        static const float pi          = glm::pi<float>();
        static const float pi_buffered = pi - glm::radians(1.f);
        // Angle layout: [root_yaw, root_pitch, joint1_pitch]
        assert_or_throw(kAngles == 3, "Forgot to update default"); // Removed this (recommended) or update it when adding bone
        return { 
            Bounds{ Bounds::Type::Wrap,  -pi,          pi          },
            Bounds{ Bounds::Type::Clamp, 0.0f,         pi_buffered },
            Bounds{ Bounds::Type::Clamp, -pi_buffered, pi_buffered }
        };
    }

    // TODO: Update when changing kBones
    SimpleArm::bone_lengths SimpleArm::defaultBoneLengths() {
        // Bone layout: [bone_length_0, bone_length_1]
        assert_or_throw(kBones == 2, "Forgot to update default"); // Removed this (recommended) or update it when adding bone
        return {5.f, 4.f};
    }

    SimpleArm::SimpleArm(
        joint_angles init_angles, 
        joint_angle_bounds init_bounds, 
        bone_lengths init_lengths
    )
        : angles(init_angles)
        , angle_bounds(init_bounds)
        , lengths(init_lengths) 
    {
        applyConstraints();
    }

    glm::mat4 SimpleArm::globalJointM(size_t joint_index) const {
        if (joint_index >= kJoints)
            throw std::out_of_range("SimpleArm::globalJointM: joint_index out of range");

        glm::mat4 M(1.0f);

        // TODO: Implement Simple Arm Transformations with angles (Remove everything in scope "{}" below)
        {
            switch (joint_index) {
            case 0: { 
                return glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, 0.f)); 
            } break;
            case 1: {
                return glm::translate(glm::mat4(1.f), glm::vec3(lengths[0], 0.f, 0.f));
            }break;
            }
        }
        
        return M;
    }

    glm::mat4 SimpleArm::globalEndEffectorM() const {
        // Tip of the last bone: start at last joint base
        glm::mat4 M = globalJointM(kJoints - 1);

        // TODO: Implement Simple Arm Transformation with angles to get tip (Remove everything in scope "{}" below)
        {
            return glm::translate(glm::mat4(1.f), glm::vec3(lengths[0] + lengths[1], 0.f, 0.f));
        }

        return M;
    }

    glm::vec3 SimpleArm::jointPosition(size_t joint_index) const {
        return glm::vec3(globalJointM(joint_index)[3]); // Think about: Why does this work?
    }

    glm::vec3 SimpleArm::endEffectorPosition() const {
        return glm::vec3(globalEndEffectorM()[3]); // Think about: Why does this work?
    }

    void SimpleArm::applyConstraints() {
        for (size_t i = 0; i < kAngles; ++i)
            angles[i] = angle_bounds[i].apply(angles[i]);
    }

    std::array<givr::geometry::Cylinder, SimpleArm::kBones> SimpleArm::armGeometry(float radius) const {
        return [&]<std::size_t... I>(std::index_sequence<I...>) {
            return std::array<givr::geometry::Cylinder, kBones>{
                givr::geometry::Cylinder(
                    givr::geometry::Point1{ jointPosition(I) },
                    givr::geometry::Point2{
                        (I + 1 < kJoints) ? jointPosition(I + 1) : endEffectorPosition()
                    },
                    givr::geometry::Radius{ radius }
                )...
            };
        }(std::make_index_sequence<kBones>{});
    }

} // namespace rigging