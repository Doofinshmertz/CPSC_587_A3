/**
 * CPSC 587 W26 Assignment 3
 * @name Holden Holzer
 * @email holden.holzer@ucalgary.ca
 *
 * Modified from provided Assignment 3 - Boilerplate
 * @authors Copyright 2019 Lakin Wecker, Jeremy Hart, Andrew Owens and Others (see AUTHORS)
 */

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
        assert_or_throw(kAngles == 4, "Forgot to update default"); // Removed this (recommended) or update it when adding bone
        return { 0.f, 0.f,glm::radians(30.0f), glm::radians(30.f) };
    }

    // TODO: Update when changing kBones
    SimpleArm::joint_angle_bounds SimpleArm::defaultJointAngleBounds() {
        static const float pi          = glm::pi<float>();
        static const float pi_buffered = pi - glm::radians(1.f);
        // Angle layout: [root_yaw, root_pitch, joint1_pitch]
        assert_or_throw(kAngles == 4, "Forgot to update default"); // Removed this (recommended) or update it when adding bone
        return { 
            Bounds{ Bounds::Type::Wrap,  -pi,          pi          },
            Bounds{ Bounds::Type::Clamp, 0.0f,         pi_buffered },
            Bounds{ Bounds::Type::Clamp, -pi_buffered, pi_buffered },
            Bounds{ Bounds::Type::Clamp, -pi_buffered, pi_buffered }
        };
    }

    // TODO: Update when changing kBones
    SimpleArm::bone_lengths SimpleArm::defaultBoneLengths() {
        // Bone layout: [bone_length_0, bone_length_1]
        assert_or_throw(kBones == 3, "Forgot to update default"); // Removed this (recommended) or update it when adding bone
        return {1.f, 2.f, 4.0f};
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

        if(joint_index == 0)
        {
            M = glm::rotate(M, angles[0], glm::vec3(0.f, 1.f, 0.f));
            return glm::rotate(M, angles[1], glm::vec3(0.f,0.f,1.f));
        }
        else
        {
            M = glm::translate(M, glm::vec3(lengths[joint_index - 1], 0.f, 0.f));
            M = glm::rotate(M, angles[joint_index+1], glm::vec3(0.f,0.f,1.0f));
            return globalJointM(joint_index -1) * M;
        }
        return M;
    }

    glm::mat4 SimpleArm::globalEndEffectorM() const {
        // Tip of the last bone: start at last joint base
        glm::mat4 M(1.0f);
        return globalJointM(kJoints-1) * glm::translate(M, glm::vec3(lengths[kBones-1], 0.f, 0.f));

        return M;
    }

    glm::vec3 SimpleArm::jointPosition(size_t joint_index) const {
        return glm::vec3(globalJointM(joint_index)[3]); // Think about: Why does this work?: A: cuz the last column of an affine transformation matrix is the position
    }

    glm::vec3 SimpleArm::endEffectorPosition() const {
        return glm::vec3(globalEndEffectorM()[3]); // Think about: Why does this work?: A: cuz the last column of an affine transformation matrix is the position
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

    void SimpleArm::moveToPositionJT(glm::vec3 e_t, float epsilon, float tolerance, float alpha_max, size_t max_iterations)
    {
        // define the delta angle with a length greater than the tolerance
        SimpleArm::joint_angles delta_angles(tolerance * 10.0f);
        
        // the numbr of iterations that have occured
        size_t iteration = 0;
        // the displacement from the target
        glm::vec3 delta_e(1.0f);

        float length = 0;
        for(size_t i = 0; i < kBones; i++)
        {
            length += lengths[i];
        }

        // iteratively move the end until it is within tolerance of the target, or the maximum number of iterations has been exceeded
        while(tolerance < glm::length(delta_angles) && iteration < max_iterations)
        {
            // ensure that the angles are valid
            applyConstraints();
            // get the position of the end effector
            glm::vec3 e = endEffectorPosition();
            // ensure that the target is within reachable distance
            glm::vec3 e_e = getProjectedTarget(e_t, e, alpha_max);
            // get the displacement from the target
            glm::vec3 delta_e = e_e - e;
            // calculate the angles update step
            delta_angles = solveAnglesJT(delta_e, epsilon, alpha_max);
            // update the angles 
            angles = angles + delta_angles;
            iteration++;
        }
    }

    SimpleArm::joint_angles SimpleArm::solveAnglesJT(glm::vec3 delta_e, float epsilon, float alpha_max)
    {
        // calculate the jacobian
        SimpleArm::jacobian j = calcJacobian(epsilon);
        // intermediate result
        SimpleArm::joint_angles temp1 = glm::transpose(j) * delta_e;
        // intermediate result 2
        glm::vec3 temp2 = j * temp1;
        // calculate alpha
        float alpha = glm::dot(delta_e, temp2) / (glm::dot(temp2, temp2));

        if (std::isnan(alpha) || !std::isfinite(alpha))
        {
            alpha = 0;
        }

        if (alpha > alpha_max)
        {
            alpha = alpha_max;
        }

        // calculate the new angles
        SimpleArm::joint_angles delta_angles = alpha * temp1;
        return delta_angles;
    }

    void SimpleArm::moveToPositionDLS(glm::vec3 e_t, float epsilon, float tolerance, float lambda, size_t max_iterations)
    {

        // define the delta angle with a length greater than the tolerance
        SimpleArm::joint_angles delta_angles(tolerance * 10.0f);

        // the numbr of iterations that have occured
        size_t iteration = 0;
        // the displacement from the target
        glm::vec3 delta_e(1.0f);

        float length = 0;
        for (size_t i = 0; i < kBones; i++)
        {
            length += lengths[i];
        }

        e_t = getProjectedTarget(e_t, glm::vec3(0.0f, 0.0f, 0.0f), length);

        // iteratively move the end until it is within tolerance of the target, or the maximum number of iterations has been exceeded
        while (tolerance < glm::length(delta_angles) && iteration < max_iterations)
        {
            // ensure that the angles are valid
            applyConstraints();
            // get the position of the end effector
            glm::vec3 e = endEffectorPosition();
            // get the displacement from the target
            glm::vec3 delta_e = e_t - e;
            // calculate the angles update step
            delta_angles = solveAnglesDLS(delta_e, epsilon, lambda);
            // update the angles
            angles = angles + delta_angles;
            iteration++;
        }

    }

    SimpleArm::joint_angles SimpleArm::solveAnglesDLS(glm::vec3 delta_e, float epsilon, float lambda)
    {
        // get the jacobian of the system
        SimpleArm::jacobian j = calcJacobian(epsilon);

        SimpleArm::jacobian_inv m(1.0f);
        m = glm::transpose(j)*j + m * lambda*lambda;
        SimpleArm::joint_angles delta_angles = glm::inverse(m)*(glm::transpose(j)*delta_e);

        return delta_angles;
    }

    void SimpleArm::printJacobianInv(jacobian_inv j)
    {
        printf("\n\n\n");
        for(size_t i =0; i < kAngles; i++)
        {
            for(size_t k = 0; k < kAngles; k++)
            {
                printf(" %6.2f |", j[k][i]);
            }
            printf("\n");
        }
    }


    glm::vec3 SimpleArm::getProjectedTarget(glm::vec3 e_t, glm::vec3 e, float max_delta)
    {
        // This uses the algorithm outlined in the Assignment 03 - Technical Specifications
        // displacement vector
        glm::vec3 d = e_t - e;
        float l = glm::length(d);

        if(l > max_delta)
        {
            return e + d * (max_delta / l);
        }
        else
        {
            return e_t;
        }
    }


    SimpleArm::jacobian SimpleArm::calcJacobian(float epsilon)
    {
        // record the current angles (so we can restore them later)
        SimpleArm::joint_angles previous_angles = angles;

        // the output jacobian matrix
        jacobian j(1.0f);

        for(size_t i = 0; i < kAngles; i++)
        {
            // calculate the forward difference value
            angles[i] = previous_angles[i] + epsilon;
            applyConstraints();
            glm::vec3 ea = endEffectorPosition();

            // calculate the backward difference value
            angles[i] = previous_angles[i] - epsilon;
            applyConstraints();
            glm::vec3 eb = endEffectorPosition();

            // restore the angle
            angles[i] = previous_angles[i];

            // take the derivative
            glm::vec3 de_di = (ea - eb) / (2.0f * epsilon);

            // assign the column of the jacobian
            j[i] = de_di;
        }
        //printJacobian(j);
        return j;
    }

    void SimpleArm::SetupRestPositionMatrices()
    {
        // first set the angles to zero and record the resting position transformation matrices of each bone
        SimpleArm::joint_angles previous_angles = angles;
        for (size_t i = 0; i < kAngles; i++)
        {
            angles[i] = 0.0f;
        }

        inv_rest_mats.resize(kBones);

        for (size_t i = 0; i < kBones; i++)
        {
            inv_rest_mats[i] = glm::inverse(globalJointM(i));
        }

        // restore the angles
        angles = previous_angles;
    }

    void SimpleArm::DeformMeshToBones(skinning::SkinnedModel *model)
    {
        // pre-compute the local to world matrices of each bone
        std::vector<glm::mat4> matrices;
        matrices.resize(kBones); 

        for(size_t i = 0; i < kBones; i++)
        {
            matrices[i] = globalJointM(i);
        }

       
        // loop over each vertex and apply the transform
        for(size_t i = 0; i < model->vertices.size(); i++)
        {
            // convert the position to affine coordinate
            glm::vec4 pos(0.0f);
            glm::vec3 p = model->vertices[i].rest_pos;
            glm::vec4 p4;
            p4.x = p.x;
            p4.y = p.y;
            p4.z = p.z;
            p4.w = 1.0f;

            // calculate new vector for each weight
            for(size_t j = 0; j < model->vertices[i].bone_weights.size(); j++)
            {
               
                size_t bone_indx = model->vertices[i].bone_weights[j].bone_id;
                pos = pos + model->vertices[i].bone_weights[j].w * (matrices[bone_indx] * inv_rest_mats[bone_indx]) * p4;
               
            } 

            // ensures that this is an affine operation (sum of weights is 1)
            float correction = 1.0f / pos.w;
            p.x = pos.x * correction;
            p.y = pos.y * correction;
            p.z = pos.z * correction;

            model->vertices[i].def_pos = p;
        }
    }

    void SimpleArm::CalculateWeights(skinning::SkinnedModel *model)
    {
        // set the bones to the lengths required by the model
        const size_t n = std::min(model->bones.size(), lengths.size());
        for (size_t i = 0; i < n; i++)
        {
            lengths[i] = model->bones[i].length;
        }

        // first set the angles to zero and record the resting position transformation matrices of each bone
        SimpleArm::joint_angles previous_angles = angles;
        for (size_t i = 0; i < kAngles; i++)
        {
            angles[i] = 0.0f;
        }

        // record the position of the start and end of each bone
        std::vector<glm::vec3> bone_positions;
        // position i is the start of bone i and end of bone i-1, we need k+1 positions for the start and end of all k bones
        // the end effector is the end position of the last bone
        bone_positions.resize(kBones + 1);

        // the vector difference between the end and start of each bone
        std::vector<glm::vec3> bone_d_values;
        std::vector<float> bone_len_sq; // the length of the bones squared
        bone_d_values.resize(kBones);
        bone_len_sq.resize(kBones);

        // set the position of each bone (the position of the next bone is the end position of the current bone)
        for(size_t i = 0; i < kBones; i++)
        {
            bone_positions[i] = jointPosition(i);
        }

        // the end of the last bone is the end effector position
        bone_positions[kBones] = endEffectorPosition();

        // calculate the displacement values of each bone
        for(size_t i = 0; i < kBones; i++)
        {
            bone_d_values[i] = bone_positions[i+1] - bone_positions[i];
            bone_len_sq[i] = glm::dot(bone_d_values[i], bone_d_values[i]);
        }

        // for each vertex, calculate the effect from each bone
        for(size_t i = 0; i < model->vertices.size(); i++)
        {
            // the following algorithm is based on that which is shown in the Assignment 03 - Technical Specifications
            // ensure there are kBones number of weights for each vertex
            model->vertices[i].bone_weights.resize(kBones);

            // get the rest position
            glm::vec3 p = model->vertices[i].rest_pos;

            // the sum of the weights (will be used for normalization latter)
            float weight_sum = 0; 

            // get weight of each bone
            for(size_t j = 0; j < kBones; j++)
            {
                float w = 0.0f;
                // the position relative to the start of the bone
                glm::vec3 u = p - bone_positions[j];
                // the projected distance along the bone
                float t = glm::dot(u, bone_d_values[j]) / (bone_len_sq[j]);
                if(t < 0.0f)
                {
                    // behinde the bone, return the distance from the start of the bone
                    w = glm::length(u);
                }
                else if(t > 1.0f)
                {
                    // past the end, return the distance past the end of the bone
                    w = glm::length(p - bone_positions[j+1]);
                }
                else
                {
                    w = glm::length(u - t*bone_d_values[j]);
                }

                // apply dropoff function
                w = std::exp((w*w)*(-1.0f));
                weight_sum += w;
                // store the new weight
                model->vertices[i].bone_weights[j].w = w;
                model->vertices[i].bone_weights[j].bone_id = j;
            }

            // normalize the weights
            for(size_t j = 0; j < kBones; j++)
            {
                model->vertices[i].bone_weights[j].w = model->vertices[i].bone_weights[j].w / weight_sum;
            }
        }

        // reset the angles to their original values
        angles = previous_angles;
    }

    void SimpleArm::printJacobian(SimpleArm::jacobian j)
    {
        printf("\n\n\n");
        for(size_t i = 0; i < 3; i++)
        {
            for(size_t k = 0; k < kAngles; k++)
            {
                printf(" %6.2f |", j[k][i]);
            }

            printf("\n");
        }
    }

} // namespace rigging