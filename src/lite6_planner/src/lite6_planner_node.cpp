#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <shared_mutex>
#include <string>
#include <array>

// Include the action definition and kinematics engine header
#include "lite6_interfaces/action/industrial_motion.hpp"
#include "lite6_planner/kinematics_engine.hpp"

// Ruckig & Pinocchio (No Coal/HPP-FCL headers needed here, fully encapsulated in the engine!)
#include <ruckig/ruckig.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace lite6_planner
{

class IndustrialPlannerNode : public rclcpp::Node
{
public:
    using IndustrialMotion = lite6_interfaces::action::IndustrialMotion;
    using GoalHandleMotion = rclcpp_action::ServerGoalHandle<IndustrialMotion>;
    using FJT = control_msgs::action::FollowJointTrajectory;

    IndustrialPlannerNode() : Node("lite6_industrial_planner")
    {
        // 1. Initialize Kinematics Engine (Now includes URDF, SRDF and Collision geometry)
        std::string pkg_path = ament_index_cpp::get_package_share_directory("lite6_description");
        std::string planner_pkg_path = ament_index_cpp::get_package_share_directory("lite6_planner");
        
        std::string urdf_path = pkg_path + "/urdf/lite6.urdf";
        std::string srdf_path = planner_pkg_path + "/config/lite6.srdf";
        std::string limits_path = planner_pkg_path + "/config/kinematic_limits.yaml";

        // --- Initialize the dual-engine (Kinematics + Coal Geometry) ---
        if (!kin_engine_.init_robot_model(urdf_path, srdf_path)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize URDF/SRDF Geometry Model!");
            throw std::runtime_error("Geometry init failed");
        }

        if (!kin_engine_.load_limits(limits_path)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load kinematic limits!");
            throw std::runtime_error("Limits load failed");
        }

        // Retrieve EEF frame ID from the engine's Pinocchio model
        eef_frame_id_ = kin_engine_.get_pin_model().getFrameId("link_eef");

        current_q_ = Eigen::VectorXd::Zero(6);
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        // 2. Initialize ROS2 subscriptions and action servers
        js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                // Thread safety: Acquire unique write lock before updating state
                std::unique_lock<std::shared_mutex> lock(state_mutex_); 
                for (size_t i = 0; i < 6; ++i) {
                    auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                    if (it != msg->name.end()) {
                        current_q_[i] = msg->position[std::distance(msg->name.begin(), it)];
                    }
                }
            });

        jtc_client_ = rclcpp_action::create_client<FJT>(this, "/lite6_arm_controller/follow_joint_trajectory");

        action_server_ = rclcpp_action::create_server<IndustrialMotion>(
            this, "industrial_motion",
            std::bind(&IndustrialPlannerNode::handle_goal, this, _1, _2),
            std::bind(&IndustrialPlannerNode::handle_cancel, this, _1),
            std::bind(&IndustrialPlannerNode::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Industrial Planner Node is ONLINE & READY (Coal Collision Enabled).");
    }

private:
    pinocchio::FrameIndex eef_frame_id_;
    KinematicsEngine kin_engine_;

    Eigen::VectorXd current_q_;
    std::vector<std::string> joint_names_;
    
    // Mutex to prevent data race between ROS spin thread and Execution thread
    std::shared_mutex state_mutex_; 

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp_action::Client<FJT>::SharedPtr jtc_client_;
    rclcpp_action::Server<IndustrialMotion>::SharedPtr action_server_;

    // --- Universal Mid-air Collision Checker using Coal ---
    // Checks an entire generated trajectory for mid-air collisions (sub-sampled for performance)
    bool is_trajectory_collision_free(const trajectory_msgs::msg::JointTrajectory& traj) {
        if (traj.points.empty()) return true;
        
        // Sub-sample: Check every 10th point to save CPU (e.g., check every 0.1s if dt=0.01s)
        for (size_t i = 0; i < traj.points.size(); i += 10) {
            std::array<double, 6> q;
            for(int j=0; j<6; ++j) q[j] = traj.points[i].positions[j];
            
            if (kin_engine_.check_collision(q)) return false;
        }
        
        // Always strictly check the exact final point
        std::array<double, 6> q_end;
        for(int j=0; j<6; ++j) q_end[j] = traj.points.back().positions[j];
        if (kin_engine_.check_collision(q_end)) return false;
        
        return true;
    }


    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const IndustrialMotion::Goal> goal) {
        if (goal->velocity_scale <= 0.0 || goal->velocity_scale > 1.0) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMotion>&) {
        jtc_client_->async_cancel_all_goals();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMotion>& goal_handle) {
        std::thread{std::bind(&IndustrialPlannerNode::execute, this, _1), goal_handle}.detach();
    }

    // Core execution logic
    void execute(const std::shared_ptr<GoalHandleMotion>& goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<IndustrialMotion::Result>();
        trajectory_msgs::msg::JointTrajectory traj_msg;

        try {
            if (goal->command_type == IndustrialMotion::Goal::MOVEJ || goal->command_type == IndustrialMotion::Goal::MOVEP) {
                traj_msg = plan_ptp(goal);
            } else if (goal->command_type == IndustrialMotion::Goal::MOVEL) {
                traj_msg = plan_line(goal);
            } else if (goal->command_type == IndustrialMotion::Goal::MOVEC) {
                traj_msg = plan_circle(goal);
            } else {
                throw std::runtime_error("Unknown Command Type");
            }
        } catch (const std::exception& e) {
            std::string err_msg = e.what();
            
            // --- Strict error categorization mapping ---
            if (err_msg.find("COLLISION") != std::string::npos) {
                result->error_code = IndustrialMotion::Result::ERR_LIMIT_VIOLATION;
            } else if (err_msg.find("NO_IK") != std::string::npos) {
                result->error_code = IndustrialMotion::Result::ERR_NO_IK_SOLUTION;
            } else if (err_msg.find("Singularity") != std::string::npos) {
                result->error_code = IndustrialMotion::Result::ERR_SINGULARITY;
            } else {
                result->error_code = IndustrialMotion::Result::ERR_INVALID_INPUT;
            }
            
            result->error_string = err_msg;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Planning Aborted: %s", err_msg.c_str());
            return;
        }

        // Send the planned trajectory to the JTC action server
        send_to_jtc(traj_msg, goal_handle, result);
    }

    // --- Parameterized PTP planning with multi-solution collision avoidance ---
    trajectory_msgs::msg::JointTrajectory plan_ptp(std::shared_ptr<const IndustrialMotion::Goal> goal)
    {
        // Safely copy current joint states
        Eigen::VectorXd start_q;
        {
            std::shared_lock<std::shared_mutex> lock(state_mutex_);
            start_q = current_q_;
        }

        if (goal->command_type == IndustrialMotion::Goal::MOVEJ) {
            // MOVEJ Logic
            Eigen::VectorXd target_q_eig = Eigen::Map<const Eigen::VectorXd>(goal->joint_target.data(), 6);
            std::array<double, 6> target_q_arr;
            for(int i=0; i<6; ++i) target_q_arr[i] = target_q_eig[i];
            
            // --- Target check using Coal ---
            if (kin_engine_.check_collision(target_q_arr)) {
                throw std::runtime_error("COLLISION: Target joint configuration results in physical collision!");
            }
            
            // Generate Ruckig traj
            auto traj = generate_ruckig_traj(start_q, target_q_eig, goal->velocity_scale, goal->acceleration_scale);
            
            // --- Mid-trajectory check using Coal ---
            if (!is_trajectory_collision_free(traj)) {
                throw std::runtime_error("COLLISION: Mid-trajectory collision detected during MoveJ execution!");
            }
            return traj;
        } 
        else {
            // MOVEP Logic
            Eigen::Matrix4d T_goal = pose_to_matrix(goal->pose_target);
            
            // Local pinocchio calculation using the engine's model
            pinocchio::Data local_data(kin_engine_.get_pin_model());
            pinocchio::forwardKinematics(kin_engine_.get_pin_model(), local_data, start_q);
            pinocchio::updateFramePlacements(kin_engine_.get_pin_model(), local_data);
            
            Eigen::Matrix4d T_current = Eigen::Matrix4d::Identity();
            T_current.block<3,3>(0,0) = local_data.oMf[eef_frame_id_].rotation();
            T_current.block<3,1>(0,3) = local_data.oMf[eef_frame_id_].translation();

            double pos_err = (T_goal.block<3,1>(0,3) - T_current.block<3,1>(0,3)).norm();
            double ori_err = Eigen::AngleAxisd(T_current.block<3,3>(0,0).transpose() * T_goal.block<3,3>(0,0)).angle();

            if (pos_err < 1e-4 && std::abs(ori_err) < 1e-3) {
                // Short-circuit IK: Already at target
                return generate_ruckig_traj(start_q, start_q, goal->velocity_scale, goal->acceleration_scale);
            } 

            // Normal IK Solving - Requesting a Priority Queue of valid solutions
            std::array<double, 6> q_ref;
            Eigen::VectorXd::Map(&q_ref[0], 6) = start_q;
            
            std::vector<std::array<double, 6>> valid_ik_solutions;
            if (!kin_engine_.solve_optimal_ik(T_goal, q_ref, valid_ik_solutions, false)) {
                throw std::runtime_error("NO_IK_SOLUTION: Target is physically unreachable, outside limits, or in COLLISION.");
            }

            // --- Multi-solution iteration to avoid mid-air collisions ---
            for (const auto& q_target_candidate : valid_ik_solutions) {
                Eigen::VectorXd target_q_eig = Eigen::Map<const Eigen::VectorXd>(q_target_candidate.data(), 6);
                
                // Target is known to be IK-valid and collision-free, generate its interpolation trajectory
                auto traj = generate_ruckig_traj(start_q, target_q_eig, goal->velocity_scale, goal->acceleration_scale);
                
                // If the dynamic path doesn't hit anything, it's our golden ticket!
                if (is_trajectory_collision_free(traj)) {
                    return traj;
                }
                
                RCLCPP_WARN(this->get_logger(), "MoveP Priority IK Path implies mid-air collision. Rerouting to next sub-optimal IK solution...");
            }
            
            throw std::runtime_error("COLLISION: All possible IK routes result in a mid-air collision!");
        }
    }

    // Helper: Extracts Ruckig generation logic for re-use
    trajectory_msgs::msg::JointTrajectory generate_ruckig_traj(const Eigen::VectorXd& start_q, const Eigen::VectorXd& target_q, double v_scale, double a_scale) 
    {
        double dt = 0.01;
        ruckig::Ruckig<6> otg(dt);
        ruckig::InputParameter<6> input;
        ruckig::OutputParameter<6> output;

        for (int i = 0; i < 6; ++i) {
            input.current_position[i] = start_q[i];
            input.current_velocity[i] = 0.0;
            input.current_acceleration[i] = 0.0;
            
            input.target_position[i] = target_q[i];
            input.target_velocity[i] = 0.0;
            input.target_acceleration[i] = 0.0;

            const auto& lim = kin_engine_.get_joint_limits()[i];
            input.max_velocity[i] = lim.max_vel * v_scale;
            input.max_acceleration[i] = lim.max_acc * a_scale;
            input.max_jerk[i] = input.max_acceleration[i] * 10.0;
        }

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names_;

        ruckig::Result res;
        do {
            res = otg.update(input, output);
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            for (int i = 0; i < 6; ++i) {
                pt.positions.push_back(output.new_position[i]);
                pt.velocities.push_back(output.new_velocity[i]);
                pt.accelerations.push_back(output.new_acceleration[i]);
            }
            pt.time_from_start = rclcpp::Duration::from_seconds(output.time);
            traj.points.push_back(pt);
            output.pass_to_input(input);
        } while (res == ruckig::Result::Working);

        if (!traj.points.empty()) {
            for (int i = 0; i < 6; ++i) {
                traj.points.back().velocities[i] = 0.0;
                traj.points.back().accelerations[i] = 0.0;
            }
        }
        return traj;
    }

    // MoveL: Interpolates in Cartesian space and uses IK to find joint configurations along the path
    trajectory_msgs::msg::JointTrajectory plan_line(std::shared_ptr<const IndustrialMotion::Goal> goal)
    {
        // Safely copy current joint states
        Eigen::VectorXd start_q;
        {
            std::shared_lock<std::shared_mutex> lock(state_mutex_);
            start_q = current_q_;
        }
        
        Eigen::Matrix4d T_goal = pose_to_matrix(goal->pose_target);

        // --- Use Engine's model for Forward Kinematics ---
        pinocchio::Data local_data(kin_engine_.get_pin_model());
        pinocchio::forwardKinematics(kin_engine_.get_pin_model(), local_data, start_q);
        pinocchio::updateFramePlacements(kin_engine_.get_pin_model(), local_data);
        pinocchio::SE3 pose_start = local_data.oMf[eef_frame_id_];
        
        pinocchio::SE3 pose_end(T_goal.block<3,3>(0,0), T_goal.block<3,1>(0,3));

        double d_trans = (pose_end.translation() - pose_start.translation()).norm();
        Eigen::AngleAxisd aa(pose_start.rotation().transpose() * pose_end.rotation());
        double d_rot = std::abs(aa.angle());

        if (d_trans < 1e-4 && d_rot < 1e-4) throw std::runtime_error("Start and End pose are identical.");

        double dt = 0.01;
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names_;
        bool trajectory_valid = false;
        
        double current_v_scale = goal->velocity_scale;
        double current_a_scale = goal->acceleration_scale;
        const int MAX_RETRIES = 5;

        // Auto-Scaling Loop: iteratively decelerate if joint limits are violated
        for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
            traj.points.clear();
            
            ruckig::Ruckig<1> otg(dt);
            ruckig::InputParameter<1> input;
            ruckig::OutputParameter<1> output;
            
            input.current_position[0] = 0.0; input.current_velocity[0] = 0.0; input.current_acceleration[0] = 0.0;
            input.target_position[0] = 1.0;  input.target_velocity[0] = 0.0;  input.target_acceleration[0] = 0.0;

            const auto& clim = kin_engine_.get_cartesian_limits();
            double max_v_s = std::numeric_limits<double>::max();
            if (d_trans > 1e-4) max_v_s = std::min(max_v_s, clim.max_trans_vel / d_trans);
            if (d_rot > 1e-4) max_v_s = std::min(max_v_s, clim.max_rot_vel / d_rot);
            
            input.max_velocity[0] = max_v_s * current_v_scale;
            input.max_acceleration[0] = (clim.max_trans_acc / std::max(d_trans, 0.01)) * current_a_scale;
            input.max_jerk[0] = input.max_acceleration[0] * 10.0;

            std::array<double, 6> q_ref;
            Eigen::VectorXd::Map(&q_ref[0], 6) = start_q;
            Eigen::VectorXd prev_dq = Eigen::VectorXd::Zero(6);

            ruckig::Result res;
            try {
                do {
                    res = otg.update(input, output);
                    double s = output.new_position[0];
                    double ds = output.new_velocity[0];

                    // 1. Interpolation
                    Eigen::Vector3d p_t = pose_start.translation() + s * (pose_end.translation() - pose_start.translation());
                    
                    Eigen::Quaterniond q_s(pose_start.rotation());
                    Eigen::Quaterniond q_e(pose_end.rotation());
                    Eigen::Matrix4d T_t = Eigen::Matrix4d::Identity();
                    T_t.block<3,3>(0,0) = q_s.slerp(s, q_e).toRotationMatrix();
                    T_t.block<3,1>(0,3) = p_t;

                    // 2. Find IK solution for the interpolated pose
                    std::vector<std::array<double, 6>> valid_sols;
                    // --- Solve_optimal_ik automatically checks for continuous Collision & Singularity ---
                    if (!kin_engine_.solve_optimal_ik(T_t, q_ref, valid_sols, true)) {
                        throw std::runtime_error("COLLISION, Singularity, No IK, or Quadrant Jump detected during MOVEL.");
                    }
                    q_ref = valid_sols[0]; // Take the best continuous, collision-free solution

                    // 3. Derive joint velocities using Jacobian
                    Eigen::VectorXd q_eig = Eigen::Map<Eigen::VectorXd>(q_ref.data(), 6);
                    Eigen::Vector3d v_world = ds * (pose_end.translation() - pose_start.translation());
                    Eigen::Vector3d w_world = ds * d_rot * (pose_start.rotation() * aa.axis());
                    
                    pinocchio::Data::Matrix6x J(6, 6);
                    pinocchio::computeFrameJacobian(kin_engine_.get_pin_model(), local_data, q_eig, eef_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
                    
                    Eigen::VectorXd V_spatial(6);
                    V_spatial << v_world, w_world;
                    
                    // Damped Least Squares (DLS) Pseudoinverse for numerical stability near singularities
                    double lambda = 0.01; 
                    Eigen::MatrixXd J_dls = J.transpose() * (J * J.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6,6)).inverse();
                    Eigen::VectorXd dq = J_dls * V_spatial;

                    // 4. Discretize acceleration
                    Eigen::VectorXd ddq = (dq - prev_dq) / dt;
                    prev_dq = dq;

                    trajectory_msgs::msg::JointTrajectoryPoint pt;
                    for (int i = 0; i < 6; ++i) {
                        pt.positions.push_back(q_ref[i]);
                        pt.velocities.push_back(dq(i));
                        pt.accelerations.push_back(ddq(i));
                    }
                    pt.time_from_start = rclcpp::Duration::from_seconds(output.time);
                    traj.points.push_back(pt);
                    
                    output.pass_to_input(input);
                } while (res == ruckig::Result::Working);
            } catch (const std::exception& e) {
                // If collision or singularity breaks the generation, propagate error to abort
                throw; 
            }

            // --- Auto-Scaling Violation Check ---
            double max_violation = 1.0;
            for (const auto& pt : traj.points) {
                for (int i = 0; i < 6; ++i) {
                    double v_ratio = std::abs(pt.velocities[i]) / kin_engine_.get_joint_limits()[i].max_vel;
                    double a_ratio = std::abs(pt.accelerations[i]) / kin_engine_.get_joint_limits()[i].max_acc;
                    max_violation = std::max({max_violation, v_ratio, a_ratio});
                }
            }

            if (max_violation <= 1.01) { 
                trajectory_valid = true;
                break; // Valid trajectory found!
            } else {
                // Scale down and retry
                current_v_scale *= (0.95 / max_violation);
                current_a_scale *= (0.95 / max_violation);
                RCLCPP_WARN(this->get_logger(), "MoveL violates joint limits (Ratio: %.2f). Auto-scaling Cartesian speed down and retrying...", max_violation);
            }
        }

        if (!trajectory_valid) throw std::runtime_error("Cannot generate valid trajectory even after auto-scaling.");

        // Guarantee that the last point has zero velocity and acceleration
        if (!traj.points.empty()) {
            for (int i = 0; i < 6; ++i) {
                traj.points.back().velocities[i] = 0.0;
                traj.points.back().accelerations[i] = 0.0;
            }
        }
        return traj;
    }

    // MoveC: Plan a circular path through three points in space
    trajectory_msgs::msg::JointTrajectory plan_circle(std::shared_ptr<const IndustrialMotion::Goal> goal)
    {
        Eigen::VectorXd start_q;
        {
            std::shared_lock<std::shared_mutex> lock(state_mutex_);
            start_q = current_q_;
        }
        
        pinocchio::Data local_data(kin_engine_.get_pin_model());
        pinocchio::forwardKinematics(kin_engine_.get_pin_model(), local_data, start_q);
        pinocchio::updateFramePlacements(kin_engine_.get_pin_model(), local_data);
        
        Eigen::Vector3d p1 = local_data.oMf[eef_frame_id_].translation();
        Eigen::Vector3d p2 = pose_to_matrix(goal->pose_aux).block<3,1>(0,3);
        Eigen::Vector3d p3 = pose_to_matrix(goal->pose_target).block<3,1>(0,3);

        // 1. Derive circle center and radius from three points
        Eigen::Vector3d v1 = p2 - p1, v2 = p3 - p1;
        Eigen::Vector3d n = v1.cross(v2);
        if (n.norm() < 1e-5) throw std::runtime_error("Points are collinear, cannot form a circle.");
        n.normalize();

        double a = (p3 - p2).norm(), b = (p1 - p3).norm(), c = (p2 - p1).norm();
        double alpha = a*a * (b*b + c*c - a*a);
        double beta  = b*b * (a*a + c*c - b*b);
        double gamma = c*c * (a*a + b*b - c*c);
        Eigen::Vector3d center = (alpha*p1 + beta*p2 + gamma*p3) / (alpha + beta + gamma);
        double radius = (p1 - center).norm();

        // 2. Establish a local coordinate system on the circle plane
        Eigen::Vector3d X_dir = (p1 - center).normalized();
        Eigen::Vector3d Y_dir = n.cross(X_dir).normalized();

        double theta2 = std::atan2((p2 - center).dot(Y_dir), (p2 - center).dot(X_dir));
        double theta3 = std::atan2((p3 - center).dot(Y_dir), (p3 - center).dot(X_dir));
        
        if (theta2 < 0) theta2 += 2 * M_PI;
        if (theta3 < theta2) theta3 += 2 * M_PI; // Ensure monotonic increase from 0 -> theta2 -> theta3
        
        double total_angle = theta3;
        double d_trans = radius * total_angle;
        double dt = 0.01;

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names_;
        bool trajectory_valid = false;
        
        double current_v_scale = goal->velocity_scale;
        double current_a_scale = goal->acceleration_scale;
        const int MAX_RETRIES = 5;

        // Auto-Scaling Loop for MoveC
        for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
            traj.points.clear();

            ruckig::Ruckig<1> otg(dt);
            ruckig::InputParameter<1> input;
            ruckig::OutputParameter<1> output;
            
            input.current_position[0] = 0.0; input.current_velocity[0] = 0.0; input.current_acceleration[0] = 0.0;
            input.target_position[0] = 1.0;  input.target_velocity[0] = 0.0;  input.target_acceleration[0] = 0.0;

            const auto& clim = kin_engine_.get_cartesian_limits();
            input.max_velocity[0] = (clim.max_trans_vel / d_trans) * current_v_scale;
            input.max_acceleration[0] = (clim.max_trans_acc / d_trans) * current_a_scale;
            input.max_jerk[0] = input.max_acceleration[0] * 10.0;

            std::array<double, 6> q_ref;
            Eigen::VectorXd::Map(&q_ref[0], 6) = start_q;
            Eigen::VectorXd prev_dq = Eigen::VectorXd::Zero(6);

            Eigen::Quaterniond q_start(local_data.oMf[eef_frame_id_].rotation());
            Eigen::Quaterniond q_end(pose_to_matrix(goal->pose_target).block<3,3>(0,0));
            Eigen::AngleAxisd aa(q_start.inverse() * q_end);

            ruckig::Result res;
            try {
                do {
                    res = otg.update(input, output);
                    double s = output.new_position[0];
                    double ds = output.new_velocity[0];
                    double current_theta = s * total_angle;

                    // 1. Interpolation
                    Eigen::Vector3d p_t = center + radius * std::cos(current_theta) * X_dir + radius * std::sin(current_theta) * Y_dir;
                    
                    Eigen::Matrix4d T_t = Eigen::Matrix4d::Identity();
                    T_t.block<3,3>(0,0) = q_start.slerp(s, q_end).toRotationMatrix();
                    T_t.block<3,1>(0,3) = p_t;

                    // 2. Inverse kinematics
                    std::vector<std::array<double, 6>> valid_sols;
                    // --- Solve_optimal_ik automatically checks for continuous Collision & Singularity ---
                    if (!kin_engine_.solve_optimal_ik(T_t, q_ref, valid_sols, true)) {
                        throw std::runtime_error("COLLISION, Singularity, No IK, or Quadrant Jump detected during MOVEC.");
                    }
                    q_ref = valid_sols[0];

                    // 3. Jacobian inverse mapping for velocity
                    Eigen::VectorXd q_eig = Eigen::Map<Eigen::VectorXd>(q_ref.data(), 6);
                    double dtheta_dt = ds * total_angle;
                    Eigen::Vector3d v_world = dtheta_dt * (-radius * std::sin(current_theta) * X_dir + radius * std::cos(current_theta) * Y_dir);
                    Eigen::Vector3d w_world = ds * aa.angle() * (q_start * aa.axis());

                    pinocchio::Data::Matrix6x J(6, 6);
                    pinocchio::computeFrameJacobian(kin_engine_.get_pin_model(), local_data, q_eig, eef_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
                    
                    Eigen::VectorXd V_spatial(6); V_spatial << v_world, w_world;
                    
                    // DLS Pseudoinverse
                    double lambda = 0.01; 
                    Eigen::MatrixXd J_dls = J.transpose() * (J * J.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6,6)).inverse();
                    Eigen::VectorXd dq = J_dls * V_spatial;

                    Eigen::VectorXd ddq = (dq - prev_dq) / dt;
                    prev_dq = dq;

                    trajectory_msgs::msg::JointTrajectoryPoint pt;
                    for (int i = 0; i < 6; ++i) {
                        pt.positions.push_back(q_ref[i]);
                        pt.velocities.push_back(dq(i));
                        pt.accelerations.push_back(ddq(i));
                    }
                    pt.time_from_start = rclcpp::Duration::from_seconds(output.time);
                    traj.points.push_back(pt);
                    
                    output.pass_to_input(input);
                } while (res == ruckig::Result::Working);
            } catch (const std::exception& e) {
                throw; 
            }

            // --- Auto-Scaling Violation Check ---
            double max_violation = 1.0;
            for (const auto& pt : traj.points) {
                for (int i = 0; i < 6; ++i) {
                    double v_ratio = std::abs(pt.velocities[i]) / kin_engine_.get_joint_limits()[i].max_vel;
                    double a_ratio = std::abs(pt.accelerations[i]) / kin_engine_.get_joint_limits()[i].max_acc;
                    max_violation = std::max({max_violation, v_ratio, a_ratio});
                }
            }

            if (max_violation <= 1.01) { 
                trajectory_valid = true;
                break;
            } else {
                current_v_scale *= (0.95 / max_violation);
                current_a_scale *= (0.95 / max_violation);
                RCLCPP_WARN(this->get_logger(), "MoveC violates joint limits (Ratio: %.2f). Auto-scaling Cartesian speed down and retrying...", max_violation);
            }
        }

        if (!trajectory_valid) throw std::runtime_error("Cannot generate valid trajectory even after auto-scaling.");

        if (!traj.points.empty()) {
            for (int i = 0; i < 6; ++i) {
                traj.points.back().velocities[i] = 0.0;
                traj.points.back().accelerations[i] = 0.0;
            }
        }
        return traj;
    }

    // Helper function to convert geometry_msgs::Pose to Eigen::Matrix4d
    Eigen::Matrix4d pose_to_matrix(const geometry_msgs::msg::Pose& pose) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        T.block<3,3>(0,0) = q.toRotationMatrix();
        T.block<3,1>(0,3) << pose.position.x, pose.position.y, pose.position.z;
        return T;
    }

    // Helper function to send trajectory to JTC
    void send_to_jtc(const trajectory_msgs::msg::JointTrajectory& traj, 
                     std::shared_ptr<GoalHandleMotion> goal_handle,
                     std::shared_ptr<IndustrialMotion::Result> result)
    {
        if (!jtc_client_->wait_for_action_server(std::chrono::seconds(2))) {
            result->error_code = IndustrialMotion::Result::ERR_EXECUTION_ABORTED;
            result->error_string = "JTC Server Offline";
            goal_handle->abort(result);
            return;
        }

        auto jtc_goal = FJT::Goal();
        jtc_goal.trajectory = traj;

        auto send_goal_options = rclcpp_action::Client<FJT>::SendGoalOptions();
        send_goal_options.result_callback = 
            [this, goal_handle, result](const rclcpp_action::ClientGoalHandle<FJT>::WrappedResult& w_res) {
                if (w_res.code == rclcpp_action::ResultCode::SUCCEEDED && w_res.result->error_code == FJT::Result::SUCCESSFUL) {
                    result->error_code = IndustrialMotion::Result::SUCCESS;
                    result->error_string = "Trajectory completed successfully.";
                    goal_handle->succeed(result);
                } else {
                    result->error_code = IndustrialMotion::Result::ERR_EXECUTION_ABORTED;
                    result->error_string = "JTC aborted execution.";
                    goal_handle->abort(result);
                }
            };

        jtc_client_->async_send_goal(jtc_goal, send_goal_options);
    }
};

} // namespace lite6_planner

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lite6_planner::IndustrialPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}