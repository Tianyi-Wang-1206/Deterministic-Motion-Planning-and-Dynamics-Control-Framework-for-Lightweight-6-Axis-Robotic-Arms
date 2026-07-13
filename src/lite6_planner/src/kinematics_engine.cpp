#include "lite6_planner/kinematics_engine.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

// ROS2 ament_index to find mesh paths for Pinocchio
#include <ament_index_cpp/get_packages_with_prefixes.hpp>

// Pinocchio parsers and collision algorithms
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>

// HPP-FCL shapes for programmatically adding the ground
#include <coal/shape/geometric_shapes.h>

namespace lite6_planner
{

KinematicsEngine::KinematicsEngine() : singularity_threshold_(0.001), quadrant_jump_threshold_(0.5) {}

bool KinematicsEngine::load_limits(const std::string& yaml_path)
{
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);
        
        for (int i = 0; i < 6; ++i) {
            std::string j_name = "joint" + std::to_string(i + 1);
            joint_limits_[i].max_vel = config["joint_limits"][j_name]["max_vel"].as<double>();
            joint_limits_[i].max_acc = config["joint_limits"][j_name]["max_acc"].as<double>();
            joint_limits_[i].min_pos = config["joint_limits"][j_name]["min_pos"].as<double>();
            joint_limits_[i].max_pos = config["joint_limits"][j_name]["max_pos"].as<double>();
        }
        
        cart_limits_.max_trans_vel = config["cartesian_limits"]["max_trans_vel"].as<double>();
        cart_limits_.max_trans_acc = config["cartesian_limits"]["max_trans_acc"].as<double>();
        cart_limits_.max_rot_vel = config["cartesian_limits"]["max_rot_vel"].as<double>();
        cart_limits_.max_rot_acc = config["cartesian_limits"]["max_rot_acc"].as<double>();
        
        singularity_threshold_ = config["singularity_threshold"].as<double>();
        quadrant_jump_threshold_ = config["quadrant_jump_threshold"].as<double>();
        
        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "[KinematicsEngine] Failed to load limits YAML: " << e.what() << std::endl;
        return false;
    }
}

bool KinematicsEngine::init_robot_model(const std::string& urdf_path, const std::string& srdf_path)
{
    try {
        // 1. Build Kinematic Model
        pinocchio::urdf::buildModel(urdf_path, pin_model_);

        // 2. Resolve ROS 2 Package Paths for Mesh Loading
        // Pinocchio needs to know where "package://lite6_description/..." points to.
        std::vector<std::string> package_dirs;
        auto packages = ament_index_cpp::get_packages_with_prefixes();
        for (const auto& pkg : packages) {
            package_dirs.push_back(pkg.second + "/share");
        }

        // 3. Build Geometry Model from URDF (Collision meshes)
        pinocchio::urdf::buildGeom(pin_model_, urdf_path, pinocchio::COLLISION, geom_model_, package_dirs);

        // 4. Add all possible collision pairs, then prune them using SRDF
        geom_model_.addAllCollisionPairs();
        pinocchio::srdf::removeCollisionPairs(pin_model_, geom_model_, srdf_path);

        // 5. Programmatically build a safe ground floor using COAL (formerly HPP-FCL) Box
        // Size: 3m x 3m x 0.1m, positioned at Z = -0.05m
        auto ground_shape = std::make_shared<coal::Box>(3.0, 3.0, 0.1);
        pinocchio::SE3 ground_placement(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, -0.05));
        
        // Find the root frame (usually universe or link_base) to attach the ground
        pinocchio::FrameIndex base_frame_id = pin_model_.getFrameId("link_base");
        pinocchio::JointIndex base_joint_id = pin_model_.frames[base_frame_id].parentJoint;

        // NEW API in Pinocchio 3.x (ROS 2 Jazzy):
        // GeometryObject(name, parent_joint, parent_frame, placement, collision_geometry)
        pinocchio::GeometryObject ground_obj(
            "ground_plane", 
            base_joint_id, 
            base_frame_id, 
            ground_placement, 
            ground_shape
        );
        geom_model_.addGeometryObject(ground_obj);

        // Add collision pairs between the newly added ground and all other robot links
        size_t ground_id = geom_model_.geometryObjects.size() - 1;
        for(size_t i = 0; i < ground_id; ++i) {
            geom_model_.addCollisionPair(pinocchio::CollisionPair(i, ground_id));
        }

        std::cout << "[KinematicsEngine] Pinocchio + HPP-FCL Geometry initialized with SRDF pruning and Ground!" << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[KinematicsEngine] Fatal error during geometry init: " << e.what() << std::endl;
        return false;
    }
}

bool KinematicsEngine::check_collision(const std::array<double, 6>& q) const
{
    // Map std::array to Eigen Vector
    Eigen::VectorXd q_eig = Eigen::Map<const Eigen::VectorXd>(q.data(), 6);

    // Thread-safe local data structures for Pinocchio calculations
    pinocchio::Data local_data(pin_model_);
    pinocchio::GeometryData local_geom_data(geom_model_);

    // Compute forward kinematics for all joints and frames
    pinocchio::forwardKinematics(pin_model_, local_data, q_eig);
    pinocchio::updateGeometryPlacements(pin_model_, local_data, geom_model_, local_geom_data);

    // Perform Fast-Exit collision checking (stops at the first collision detected)
    bool is_colliding = pinocchio::computeCollisions(geom_model_, local_geom_data, true);
    
    return is_colliding;
}

double KinematicsEngine::check_singularity(const std::array<double, 6>& q) const
{
    // Jacobian determinant check for singularity (Maple generated)
    double q2 = q[1];
    double q3 = q[2];
    double q4 = q[3];
    double q5 = q[4];

    double t1 = std::sin(q2);
    double t3 = q2 - q3;
    double t4 = std::cos(t3);
    double t6 = std::sin(t3);
    double t9 = std::cos(q3);
    double t11 = std::sin(q3);
    double t15 = std::cos(q4);
    double t16 = std::sin(q5);
    
    double t19 = 0.2002 * t16 * t15 * (0.087 * t9 + 0.22761 * t11) * 
                 (0.2002 * t1 + 0.087 * t4 - 0.22761 * t6);
                 
    return t19;
}

void KinematicsEngine::raw_inverse_kinematics(const Eigen::Matrix4d& T, double cg0[8][6]) const
{
    double R11 = T(0,0), R12 = T(0,1), R13 = T(0,2), Xt = T(0,3);
    double R21 = T(1,0), R22 = T(1,1), R23 = T(1,2), Yt = T(1,3);
    double R31 = T(2,0), R32 = T(2,1), R33 = T(2,2), Zt = T(2,3);

    double t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;
    double t11, t12, t13, t14, t15, t16, t17, t18, t19, t20;
    double t21, t22, t23, t24, t25, t26, t27, t28, t29, t30;
    double t31, t32, t33, t34, t35, t36, t37, t38, t39, t40, t41;

    // Inverse kinematics calculations (Maple generated)
    t1 = 0.1e1 / 0.16e2;
    t2 = R23 * t1 - Yt;
    t3 = R13 * t1 - Xt;
    t4 = atan2(t2, t3);
    t5 = -0.32e2;
    t6 = 0.256e3;
    t5 = R13 * R13 + R23 * R23 + t5 * (R13 * Xt + R23 * Yt) + t6 * (Xt * Xt + Yt * Yt);
    t5 = pow(t5, -0.1e1 / 0.2e1);
    t7 = t5 * (-pow(t2, 0.2e1) - pow(t3, 0.2e1));
    t8 = -0.487e3 / 0.2000e4;
    t1 = -R33 * t1 + Zt + t8;
    t8 = pow(t7, 0.2e1);
    t9 = pow(t1, 0.2e1);
    t10 = 0.2500e4 / 0.1001e4 * t9;
    t11 = 0.640000e6 / 0.1001e4 * t8 + t10 - 0.14842517e8 / 0.308000000e9;
    t8 = -pow(t11, 0.2e1) + t6 * t8 + t9;

    if (t8 < -1e-5) {
        t8 = std::numeric_limits<double>::quiet_NaN(); 
    } else if (t8 < 0.0) {
        t8 = 0.0;
    }

    t8 = std::sqrt(t8);   // Ensure non-negative before sqrt
    t12 = atan2(t11, -t8);
    t13 = 0.16e2;
    t14 = atan2(t1, t13 * t7);
    t15 = t12 - t14;
    t16 = sin(t15);
    t17 = cos(t15);
    t18 = 0.87e2 / 0.16000e5 * R33;
    t19 = -0.87e2 / 0.1000e4 * Zt;
    t20 = -0.22761e5 / 0.6250e4 * t7;
    t21 = 0.22783761e8 / 0.500000000e9 * t16 + 0.87087e5 / 0.5000000e7 * t17 + 0.42369e5 / 0.2000000e7 + t18 + t19 + t20;
    t22 = 0.22761e5 / 0.1600000e7 * R33;
    t7 = 0.174e3 / 0.125e3 * t7;
    t23 = -0.22761e5 / 0.100000e6 * Zt;
    t16 = 0.22783761e8 / 0.500000000e9 * t17 - 0.87087e5 / 0.5000000e7 * t16 + 0.11084607e8 / 0.200000000e9 + t22 + t7 + t23;
    t12 = t12 - t14 - atan2(t21, t16);
    t17 = pow(t16, 0.2e1) + pow(t21, 0.2e1);
    t17 = pow(t17, -0.1e1 / 0.2e1);
    t24 = R13 * t3 + R23 * t2;
    t21 = t21 * t17;
    t25 = t13 * t16 * t17 * t24 * t5 - t21 * R33;
    t26 = t13 * t5 * (R13 * t2 - R23 * t3);
    t21 = t21 * t5;
    t16 = t16 * t17;
    t17 = t13 * t21 * t24 + t16 * R33;
    t27 = -pow(t17, 0.2e1) + 0.1e1;

    if (t27 < -1e-5) {
        t27 = std::numeric_limits<double>::quiet_NaN(); 
    } else if (t27 < 0.0) {
        t27 = 0.0;
    }

    t27 = std::sqrt(t27);
    t17 = -t17;
    t28 = R12 * t3 + R22 * t2;
    t29 = -t13 * t21 * t28 - t16 * R32;
    t30 = R11 * t3 + R21 * t2;
    t16 = t13 * t21 * t30 + t16 * R31;
    t8 = atan2(t11, t8);
    t11 = t8 - t14;
    t21 = sin(t11);
    t31 = cos(t11);
    t20 = 0.22783761e8 / 0.500000000e9 * t21 + 0.87087e5 / 0.5000000e7 * t31 + 0.42369e5 / 0.2000000e7 + t18 + t19 + t20;
    t7 = 0.22783761e8 / 0.500000000e9 * t31 - 0.87087e5 / 0.5000000e7 * t21 + 0.11084607e8 / 0.200000000e9 + t22 + t7 + t23;
    t8 = t8 - atan2(t20, t7) - t14;
    t14 = pow(t20, 0.2e1) + pow(t7, 0.2e1);
    t14 = pow(t14, -0.1e1 / 0.2e1);
    t20 = t20 * t14;
    t21 = t13 * t14 * t24 * t5 * t7 - t20 * R33;
    t20 = t20 * t5;
    t7 = t7 * t14;
    t14 = t13 * t20 * t24 + t7 * R33;
    t24 = -pow(t14, 0.2e1) + 0.1e1;

    if (t24 < -1e-5) {
        t24 = std::numeric_limits<double>::quiet_NaN(); 
    } else if (t24 < 0.0) {
        t24 = 0.0;
    }
    
    t24 = std::sqrt(t24);
    t14 = -t14;
    t28 = -t13 * t20 * t28 - t7 * R32;
    t7 = t13 * t20 * t30 + t7 * R31;
    t20 = atan2(-t2, -t3);
    t30 = t5 * (pow(t2, 0.2e1) + pow(t3, 0.2e1));
    t31 = pow(t30, 0.2e1);
    t10 = 0.640000e6 / 0.1001e4 * t31 + t10 - 0.14842517e8 / 0.308000000e9;
    t6 = -pow(t10, 0.2e1) + t31 * t6 + t9;

    if (t6 < -1e-5) {
        t6 = std::numeric_limits<double>::quiet_NaN(); 
    } else if (t6 < 0.0) {
        t6 = 0.0;
    }

    t6 = std::sqrt(t6);
    t9 = atan2(t10, -t6);
    t1 = atan2(t1, t13 * t30);
    t31 = t9 - t1;
    t32 = sin(t31);
    t33 = cos(t31);
    t34 = -0.22761e5 / 0.6250e4 * t30;
    t35 = 0.22783761e8 / 0.500000000e9 * t32 + 0.87087e5 / 0.5000000e7 * t33 + 0.42369e5 / 0.2000000e7 + t18 + t19 + t34;
    t30 = 0.174e3 / 0.125e3 * t30;
    t32 = 0.22783761e8 / 0.500000000e9 * t33 - 0.87087e5 / 0.5000000e7 * t32 + 0.11084607e8 / 0.200000000e9 + t22 + t30 + t23;
    t9 = atan2(t35, t32) - t9 + t1;
    t33 = pow(t32, 0.2e1) + pow(t35, 0.2e1);
    t33 = pow(t33, -0.1e1 / 0.2e1);
    t36 = -R13 * t3 - R23 * t2;
    t35 = t35 * t33;
    t37 = t13 * t32 * t33 * t36 * t5 - t35 * R33;
    t38 = t13 * t5 * (-R13 * t2 + R23 * t3);
    t35 = t35 * t5;
    t32 = t32 * t33;
    t33 = t13 * t35 * t36 + t32 * R33;
    t39 = -pow(t33, 0.2e1) + 0.1e1;

    if (t39 < -1e-5) {
        t39 = std::numeric_limits<double>::quiet_NaN(); 
    } else if (t39 < 0.0) {
        t39 = 0.0;
    }

    t39 = std::sqrt(t39);
    t33 = -t33;
    t40 = -R12 * t3 - R22 * t2;
    t41 = -t13 * t35 * t40 - t32 * R32;
    t2 = -R11 * t3 - R21 * t2;
    t3 = t13 * t2 * t35 + t32 * R31;
    t6 = atan2(t10, t6);
    t10 = t6 - t1;
    t32 = sin(t10);
    t35 = cos(t10);
    t18 = 0.22783761e8 / 0.500000000e9 * t32 + 0.87087e5 / 0.5000000e7 * t35 + 0.42369e5 / 0.2000000e7 + t18 + t19 + t34;
    t19 = 0.22783761e8 / 0.500000000e9 * t35 - 0.87087e5 / 0.5000000e7 * t32 + 0.11084607e8 / 0.200000000e9 + t22 + t30 + t23;
    t1 = atan2(t18, t19) - t6 + t1;
    t6 = pow(t18, 0.2e1) + pow(t19, 0.2e1);
    t6 = pow(t6, -0.1e1 / 0.2e1);
    t18 = t18 * t6;
    t22 = t13 * t19 * t36 * t5 * t6 - t18 * R33;
    t5 = t18 * t5;
    t6 = t19 * t6;
    t18 = t13 * t36 * t5 + t6 * R33;
    t19 = -pow(t18, 0.2e1) + 0.1e1;

    if (t19 < -1e-5) {
        t19 = std::numeric_limits<double>::quiet_NaN(); 
    } else if (t19 < 0.0) {
        t19 = 0.0;
    }
    
    t19 = std::sqrt(t19);
    t18 = -t18;
    t23 = -t13 * t40 * t5 - t6 * R32;
    t2 = t13 * t2 * t5 + t6 * R31;

    cg0[0][0] = t4;  cg0[0][1] = t15; cg0[0][2] = t12; cg0[0][3] = atan2(t26, t25);   cg0[0][4] = atan2(-t27, t17); cg0[0][5] = atan2(t29, t16);
    cg0[1][0] = t4;  cg0[1][1] = t15; cg0[1][2] = t12; cg0[1][3] = atan2(-t26, -t25); cg0[1][4] = atan2(t27, t17);  cg0[1][5] = atan2(-t29, -t16);
    cg0[2][0] = t4;  cg0[2][1] = t11; cg0[2][2] = t8;  cg0[2][3] = atan2(t26, t21);   cg0[2][4] = atan2(-t24, t14); cg0[2][5] = atan2(t28, t7);
    cg0[3][0] = t4;  cg0[3][1] = t11; cg0[3][2] = t8;  cg0[3][3] = atan2(-t26, -t21); cg0[3][4] = atan2(t24, t14);  cg0[3][5] = atan2(-t28, -t7);
    cg0[4][0] = t20; cg0[4][1] = t31; cg0[4][2] = -t9; cg0[4][3] = atan2(t38, t37);   cg0[4][4] = atan2(-t39, t33); cg0[4][5] = atan2(t41, t3);
    cg0[5][0] = t20; cg0[5][1] = t31; cg0[5][2] = -t9; cg0[5][3] = atan2(-t38, -t37); cg0[5][4] = atan2(t39, t33);  cg0[5][5] = atan2(-t41, -t3);
    cg0[6][0] = t20; cg0[6][1] = t10; cg0[6][2] = -t1; cg0[6][3] = atan2(t38, t22);   cg0[6][4] = atan2(-t19, t18); cg0[6][5] = atan2(t23, t2);
    cg0[7][0] = t20; cg0[7][1] = t10; cg0[7][2] = -t1; cg0[7][3] = atan2(-t38, -t22); cg0[7][4] = atan2(t19, t18);  cg0[7][5] = atan2(-t23, -t2);
}

IKResult KinematicsEngine::solve_optimal_ik(const Eigen::Matrix4d& T, 
                                            const std::array<double, 6>& q_ref, 
                                            std::vector<std::array<double, 6>>& valid_solutions_sorted,
                                            bool is_continuous_path)
{
    double cg0[8][6];
    raw_inverse_kinematics(T, cg0);

    std::vector<IKSolution> candidate_solutions;

    // Trackers for failure diagnostics
    bool has_valid_math = false;
    int collision_or_limit_count = 0;
    int quadrant_jump_count = 0;

    for (int i = 0; i < 8; ++i) {
        std::array<double, 6> current_sol;
        bool limits_violated = false;
        double dist = 0.0;
        bool quadrant_jumped = false;
        bool math_invalid = false;

        // 1. Process mathematical solutions and enforce physical joint limits
        for (int j = 0; j < 6; ++j) {
            double q_base = cg0[i][j];
            
            // If the math solver outputs NaN, this branch is mathematically unreachable
            if (std::isnan(q_base)) {
                math_invalid = true;
                break;
            }

            double best_q_j = q_base;
            double min_j_dist = std::numeric_limits<double>::max();
            
            // Search across -2pi to 2pi wraparounds to find the nearest valid angle
            for (int k = -2; k <= 2; ++k) {
                double test_q = q_base + k * 2.0 * M_PI;
                if (test_q >= joint_limits_[j].min_pos - 1e-4 && test_q <= joint_limits_[j].max_pos + 1e-4) {
                    double d = std::abs(test_q - q_ref[j]);
                    if (d < min_j_dist) {
                        min_j_dist = d;
                        best_q_j = test_q;
                    }
                }
            }

            if (min_j_dist == std::numeric_limits<double>::max()) {
                limits_violated = true;
                break;
            }

            // For continuous paths (MoveL/MoveC), prevent sudden >180 deg flips
            if (is_continuous_path && min_j_dist > quadrant_jump_threshold_) {
                quadrant_jumped = true;
                break;
            }

            current_sol[j] = best_q_j;
            double weights[6] = {1.5, 1.5, 1.2, 1.0, 1.0, 0.8};
            dist += weights[j] * min_j_dist * min_j_dist;
        }

        if (math_invalid) continue;
        has_valid_math = true;

        if (limits_violated) {
            collision_or_limit_count++;
            continue;
        }
        
        if (quadrant_jumped) {
            quadrant_jump_count++;
            continue;
        }

        // 2. Hardware limit valid, perform rigid HPP-FCL collision check (Self + Ground)
        if (check_collision(current_sol)) {
            collision_or_limit_count++;
            continue; // Rejected due to physical collision
        }

        IKSolution valid_sol;
        valid_sol.q = current_sol;
        valid_sol.cost_distance = dist;
        candidate_solutions.push_back(valid_sol);
    }

    // --- Diagnostic Output if no candidates survived ---
    if (candidate_solutions.empty()) {
        if (!has_valid_math) return IKResult::ERR_NO_MATH_SOLUTION;
        // Collision has higher diagnostic priority than quadrant jumps
        if (collision_or_limit_count > 0) return IKResult::ERR_LIMIT_OR_COLLISION;
        if (quadrant_jump_count > 0) return IKResult::ERR_QUADRANT_JUMP;
        
        return IKResult::ERR_NO_MATH_SOLUTION; // Fallback
    }

    // 3. Sort solutions based on minimum movement effort
    std::sort(candidate_solutions.begin(), candidate_solutions.end());

    // 4. Final filter: Reject solutions inside a singularity zone
    valid_solutions_sorted.clear();
    for (const auto& sol : candidate_solutions) {
        if (is_continuous_path && std::abs(check_singularity(sol.q)) < singularity_threshold_) {
            continue;
        }
        valid_solutions_sorted.push_back(sol.q);
    }

    if (valid_solutions_sorted.empty()) {
        return IKResult::ERR_SINGULARITY; // All otherwise valid solutions hit a singularity
    }

    return IKResult::SUCCESS;
}

} // namespace lite6_planner