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

// Ruckig & Pinocchio (Encapsulated!)
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

    IndustrialPlannerNode() 
    : Node("lite6_industrial_planner")
    {
        // Initialize Kinematics Engine (Now includes URDF, SRDF and Collision geometry)
        std::string pkg_path = ament_index_cpp::get_package_share_directory("lite6_description");
        std::string planner_pkg_path = ament_index_cpp::get_package_share_directory("lite6_planner");
        
        std::string urdf_path = pkg_path + "/urdf/lite6.urdf";
        std::string srdf_path = planner_pkg_path + "/config/lite6.srdf";
        std::string limits_path = planner_pkg_path + "/config/kinematic_limits.yaml";
        std::string ik_config_path = planner_pkg_path + "/config/ik_config.yaml";

        // --- Load externalized YAML parameters ---
        if (!kin_engine_.load_ik_config(ik_config_path)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load externalized IK Config YAML!");
            throw std::runtime_error("IK config load failed");
        }

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

        // Initialize ROS2 subscriptions and action servers
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

        RCLCPP_INFO(this->get_logger(), "Industrial Planner Node is ONLINE & READY.");
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

    // --- Joint displacement-based continuous collision sub-sampling ---
    bool is_trajectory_collision_free(const trajectory_msgs::msg::JointTrajectory& traj) {
        if (traj.points.empty()) return true;
        
        const double ACCUMULATED_JOINT_DISPLACEMENT_THRESHOLD = 0.05; // ~3 degrees
        std::array<double, 6> last_checked_q;
        for (int j = 0; j < 6; ++j) last_checked_q[j] = traj.points[0].positions[j];

        // Always verify the exact start configuration
        if (kin_engine_.check_collision(last_checked_q)) return false;

        for (size_t i = 1; i < traj.points.size() - 1; ++i) {
            std::array<double, 6> q;
            double max_diff = 0.0;
            for (int j = 0; j < 6; ++j) {
                q[j] = traj.points[i].positions[j];
                max_diff = std::max(max_diff, std::abs(q[j] - last_checked_q[j]));
            }
            
            // Execute collision check only when joint displacement exceeds limits
            if (max_diff >= ACCUMULATED_JOINT_DISPLACEMENT_THRESHOLD) {
                if (kin_engine_.check_collision(q)) return false;
                last_checked_q = q;
            }
        }
        
        // Always strictly verify the exact final configuration
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
            
            // --- Strict error categorization mapping based on prefix keywords ---
            if (err_msg.find("COLLISION") != std::string::npos) {
                result->error_code = IndustrialMotion::Result::ERR_LIMIT_VIOLATION;
            } else if (err_msg.find("NO_IK") != std::string::npos) {
                result->error_code = IndustrialMotion::Result::ERR_NO_IK_SOLUTION;
            } else if (err_msg.find("Singularity") != std::string::npos) {
                result->error_code = IndustrialMotion::Result::ERR_SINGULARITY;
            } else if (err_msg.find("QUADRANT_JUMP") != std::string::npos) {
                result->error_code = IndustrialMotion::Result::ERR_QUADRANT_JUMP;
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
            
            thread_local pinocchio::Data local_data(kin_engine_.get_pin_model());
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
            
            // Solve Inverse Kinematics for Target Pose
            std::vector<std::array<double, 6>> valid_ik_solutions;
            IKResult ik_res = kin_engine_.solve_optimal_ik(T_goal, q_ref, valid_ik_solutions, false);
            
            if (ik_res != IKResult::SUCCESS) {
                if (ik_res == IKResult::ERR_NO_MATH_SOLUTION) throw std::runtime_error("NO_IK: Target coordinate is outside the mathematical workspace.");
                if (ik_res == IKResult::ERR_LIMIT_OR_COLLISION) throw std::runtime_error("COLLISION: Target posture violates joint limits or causes physical collision.");
                if (ik_res == IKResult::ERR_SINGULARITY) throw std::runtime_error("Singularity: Target posture is in a kinematic singularity.");
                throw std::runtime_error("NO_IK: Failed to find a valid solution.");
            }

            // --- Multi-solution iteration to avoid mid-air collisions ---
            for (const auto& q_target_candidate : valid_ik_solutions) {
                Eigen::VectorXd target_q_eig = Eigen::Map<const Eigen::VectorXd>(q_target_candidate.data(), 6);
                auto traj = generate_ruckig_traj(start_q, target_q_eig, goal->velocity_scale, goal->acceleration_scale);
                
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
        ruckig::Ruckig<6> otg_ptp(0.001); 
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
            res = otg_ptp.update(input, output); 
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

    // MoveL: Interpolates in Cartesian space and uses CLIK to find joint configurations
    trajectory_msgs::msg::JointTrajectory plan_line(std::shared_ptr<const IndustrialMotion::Goal> goal)
    {
        // Safely copy current joint states
        Eigen::VectorXd start_q;
        {
            std::shared_lock<std::shared_mutex> lock(state_mutex_);
            start_q = current_q_;
        }
        
        Eigen::Matrix4d T_goal = pose_to_matrix(goal->pose_target);

        thread_local pinocchio::Data local_data(kin_engine_.get_pin_model());
        pinocchio::forwardKinematics(kin_engine_.get_pin_model(), local_data, start_q);
        pinocchio::updateFramePlacements(kin_engine_.get_pin_model(), local_data);
        pinocchio::SE3 pose_start = local_data.oMf[eef_frame_id_];
        pinocchio::SE3 pose_end(T_goal.block<3,3>(0,0), T_goal.block<3,1>(0,3));

        // Define Start and End Quaternions outside the loop for SLERP interpolation
        Eigen::Quaterniond q_start(pose_start.rotation());
        Eigen::Quaterniond q_end(pose_end.rotation());

        // Calculate Cartesian Path Distances
        double d_trans = (pose_end.translation() - pose_start.translation()).norm();
        Eigen::AngleAxisd aa(pose_start.rotation().transpose() * pose_end.rotation());
        double d_rot = std::abs(aa.angle());

        if (d_trans < 1e-4 && d_rot < 1e-4) throw std::runtime_error("Start and End pose are identical.");

        // Calculate Maximum Allowed Phase Velocity (s_dot) based on Cartesian Limits
        const auto& clim = kin_engine_.get_cartesian_limits();
        double max_s_vel = std::numeric_limits<double>::max();
        double max_s_acc = std::numeric_limits<double>::max();

        if (d_trans > 1e-4) {
            max_s_vel = std::min(max_s_vel, clim.max_trans_vel / d_trans);
            max_s_acc = std::min(max_s_acc, clim.max_trans_acc / d_trans);
        }
        if (d_rot > 1e-4) {
            max_s_vel = std::min(max_s_vel, clim.max_rot_vel / d_rot);
            max_s_acc = std::min(max_s_acc, clim.max_rot_acc / d_rot);
        }

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names_;
        
        // Initial requested limits incorporating user overrides
        double current_v_limit = max_s_vel * goal->velocity_scale;
        double current_a_limit = max_s_acc * goal->acceleration_scale;
        
        const int MAX_RETRIES = 5;
        bool trajectory_valid = false;

        const double dt_cycle = 0.001;

        for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
            traj.points.clear();

            ruckig::Ruckig<1> otg_cart(dt_cycle); 
            
            ruckig::InputParameter<1> input;
            ruckig::OutputParameter<1> output;
            
            input.current_position[0] = 0.0; input.current_velocity[0] = 0.0; input.current_acceleration[0] = 0.0;
            input.target_position[0] = 1.0;  input.target_velocity[0] = 0.0;  input.target_acceleration[0] = 0.0;

            input.max_velocity[0] = current_v_limit;
            input.max_acceleration[0] = current_a_limit;
            input.max_jerk[0] = current_a_limit * 10.0;

            Eigen::VectorXd q_current_integrated = start_q;
            const double K_cart = 50.0; 

            // Sub-sampling threshold for collision check to save planning time (check every ~2 degrees)
            const double COLLISION_CHECK_THRESHOLD = 0.035; 
            std::array<double, 6> last_checked_q;
            for(int i=0; i<6; ++i) last_checked_q[i] = start_q[i];

            ruckig::Result res;
            try {
                do {
                    res = otg_cart.update(input, output); 
                    double s = output.new_position[0];
                    double ds = output.new_velocity[0];
                    double dds = output.new_acceleration[0]; 

                    Eigen::Vector3d p_t = pose_start.translation() + s * (pose_end.translation() - pose_start.translation());
                    Eigen::Quaterniond q_target(q_start.slerp(s, q_end));
                    
                    Eigen::Vector3d v_world = ds * (pose_end.translation() - pose_start.translation());
                    Eigen::Vector3d w_world = ds * d_rot * (q_start * aa.axis());
                    Eigen::Vector3d a_world = dds * (pose_end.translation() - pose_start.translation());
                    Eigen::Vector3d alpha_world = dds * d_rot * (q_start * aa.axis());

                    pinocchio::forwardKinematics(kin_engine_.get_pin_model(), local_data, q_current_integrated);
                    pinocchio::updateFramePlacements(kin_engine_.get_pin_model(), local_data);
                    Eigen::Vector3d p_curr = local_data.oMf[eef_frame_id_].translation();
                    Eigen::Matrix3d R_curr = local_data.oMf[eef_frame_id_].rotation();

                    Eigen::Vector3d p_err = p_t - p_curr;
                    Eigen::Matrix3d R_err_mat = q_target.toRotationMatrix() * R_curr.transpose();
                    Eigen::AngleAxisd angle_axis_err(R_err_mat);
                    Eigen::Vector3d w_err = angle_axis_err.axis() * angle_axis_err.angle();

                    // Check tracking deviation
                    if (p_err.norm() > 0.005 || w_err.norm() > 0.05) {
                        throw std::runtime_error("NO_IK: Mathematical path lost. Target unreachable or requires impossible joint flip.");
                    }

                    Eigen::VectorXd V_cmd(6);
                    V_cmd << (v_world + K_cart * p_err), (w_world + K_cart * w_err);

                    pinocchio::Data::Matrix6x J(6, 6);
                    pinocchio::computeFrameJacobian(kin_engine_.get_pin_model(), local_data, q_current_integrated, eef_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
                    
                    double manipulability = std::sqrt((J * J.transpose()).determinant());
                    
                    // Check Singularity
                    std::array<double, 6> current_q_arr;
                    for(int i=0; i<6; ++i) current_q_arr[i] = q_current_integrated[i];
                    
                    if (std::abs(kin_engine_.check_singularity(current_q_arr)) < kin_engine_.get_singularity_threshold()) {
                        throw std::runtime_error("Singularity: Trajectory attempts to pass through a kinematic singularity zone.");
                    }

                    double lambda = 0.0;
                    double w_threshold = 0.04;
                    if (manipulability < w_threshold) {
                        lambda = 0.05 * std::pow(1.0 - (manipulability / w_threshold), 2);
                    }
                    Eigen::MatrixXd J_dls = J.transpose() * (J * J.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6,6)).inverse();

                    Eigen::VectorXd dq = J_dls * V_cmd;
                    
                    Eigen::VectorXd ddq_zero = Eigen::VectorXd::Zero(6);
                    pinocchio::forwardKinematics(kin_engine_.get_pin_model(), local_data, q_current_integrated, dq, ddq_zero);
                    Eigen::VectorXd J_dot_dq = pinocchio::getFrameAcceleration(kin_engine_.get_pin_model(), local_data, eef_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED).toVector();
                    
                    Eigen::VectorXd A_cmd(6);
                    A_cmd << a_world, alpha_world; 
                    Eigen::VectorXd ddq = J_dls * (A_cmd - J_dot_dq);

                    q_current_integrated = q_current_integrated + dq * dt_cycle + 0.5 * ddq * dt_cycle * dt_cycle;

                    // Check Limits and Collisions
                    double max_diff = 0.0;
                    for (int i = 0; i < 6; ++i) {
                        current_q_arr[i] = q_current_integrated[i];
                        max_diff = std::max(max_diff, std::abs(current_q_arr[i] - last_checked_q[i]));
                        
                        // Strict Joint Limit Check
                        if (current_q_arr[i] < kin_engine_.get_joint_limits()[i].min_pos || 
                            current_q_arr[i] > kin_engine_.get_joint_limits()[i].max_pos) {
                            throw std::runtime_error("COLLISION: Joint limit violation detected during MoveL.");
                        }
                    }

                    // Sparse Collision Check (Saves computation time)
                    if (max_diff >= COLLISION_CHECK_THRESHOLD || res != ruckig::Result::Working) {
                        if (kin_engine_.check_collision(current_q_arr)) {
                            throw std::runtime_error("COLLISION: Path segment causes physical collision.");
                        }
                        last_checked_q = current_q_arr;
                    }

                    // Populate Trajectory Point
                    trajectory_msgs::msg::JointTrajectoryPoint pt;
                    for (int i = 0; i < 6; ++i) {
                        pt.positions.push_back(q_current_integrated[i]);
                        pt.velocities.push_back(dq(i));
                        pt.accelerations.push_back(ddq(i));
                    }
                    pt.time_from_start = rclcpp::Duration::from_seconds(output.time);
                    traj.points.push_back(pt);
                    
                    output.pass_to_input(input);
                } while (res == ruckig::Result::Working);
            } catch (const std::exception& e) {
                throw; // Rethrow to be caught by the external exception handler in execute()
            }

            // Strict Physical Limits Violation Check for Time-Scaling
            double max_violation_ratio = 0.0;
            for (const auto& pt : traj.points) {
                for (int i = 0; i < 6; ++i) {
                    double v_ratio = std::abs(pt.velocities[i]) / kin_engine_.get_joint_limits()[i].max_vel;
                    double a_ratio = std::sqrt(std::abs(pt.accelerations[i]) / kin_engine_.get_joint_limits()[i].max_acc); 
                    
                    max_violation_ratio = std::max({max_violation_ratio, v_ratio, a_ratio});
                }
            }

            // Deterministic Scaling Logic
            if (max_violation_ratio <= 1.01) { 
                trajectory_valid = true;
                break; 
            } else {
                current_v_limit /= (max_violation_ratio * 1.02); 
                current_a_limit /= (max_violation_ratio * max_violation_ratio * 1.02); 
                
                RCLCPP_INFO(this->get_logger(), "MoveL Joint Limit Exceeded (Ratio: %.2f). Time-scaling applied, retrying...", max_violation_ratio);
            }
        }

        if (!trajectory_valid) throw std::runtime_error("Cannot generate valid trajectory. Robot posture restricts requested Cartesian motion.");

        if (!traj.points.empty()) {
            for (int i = 0; i < 6; ++i) {
                traj.points.back().velocities[i] = 0.0;
                traj.points.back().accelerations[i] = 0.0;
            }
        }
        return traj;
    }

    // MoveC: Plan a circular path through three points in space using CLIK integration
    trajectory_msgs::msg::JointTrajectory plan_circle(std::shared_ptr<const IndustrialMotion::Goal> goal)
    {
        // Safely copy current joint states
        Eigen::VectorXd start_q;
        {
            std::shared_lock<std::shared_mutex> lock(state_mutex_);
            start_q = current_q_;
        }
        
        Eigen::Matrix4d T_goal = pose_to_matrix(goal->pose_target);
    
        thread_local pinocchio::Data local_data(kin_engine_.get_pin_model());
        pinocchio::forwardKinematics(kin_engine_.get_pin_model(), local_data, start_q);
        pinocchio::updateFramePlacements(kin_engine_.get_pin_model(), local_data);
        
        Eigen::Vector3d p1 = local_data.oMf[eef_frame_id_].translation();
        Eigen::Vector3d p2 = pose_to_matrix(goal->pose_aux).block<3,1>(0,3);
        Eigen::Vector3d p3 = T_goal.block<3,1>(0,3);

        // Calculate Circle Geometry
        Eigen::Vector3d v1 = p2 - p1, v2 = p3 - p1;
        Eigen::Vector3d n = v1.cross(v2);
        if (n.norm() < 1e-5) throw std::runtime_error("ERR_INVALID_INPUT: Points are collinear, cannot form a circle.");
        n.normalize();

        double a = (p3 - p2).norm(), b = (p1 - p3).norm(), c = (p2 - p1).norm();
        double alpha = a*a * (b*b + c*c - a*a);
        double beta  = b*b * (a*a + c*c - b*b);
        double gamma = c*c * (a*a + b*b - c*c);
        Eigen::Vector3d center = (alpha*p1 + beta*p2 + gamma*p3) / (alpha + beta + gamma);
        double radius = (p1 - center).norm();

        Eigen::Vector3d X_dir = (p1 - center).normalized();
        Eigen::Vector3d Y_dir = n.cross(X_dir).normalized();

        double theta2 = std::atan2((p2 - center).dot(Y_dir), (p2 - center).dot(X_dir));
        double theta3 = std::atan2((p3 - center).dot(Y_dir), (p3 - center).dot(X_dir));
        if (theta2 < 0) theta2 += 2 * M_PI;
        if (theta3 < theta2) theta3 += 2 * M_PI; 
        
        double total_angle = theta3;
        double d_trans = radius * total_angle;
        
        Eigen::Quaterniond q_start(local_data.oMf[eef_frame_id_].rotation());
        Eigen::Quaterniond q_end(T_goal.block<3,3>(0,0));
        Eigen::AngleAxisd aa(q_start.inverse() * q_end);
        double d_rot = std::abs(aa.angle());

        // Calculate Maximum Allowed Phase Velocity (s_dot)
        const auto& clim = kin_engine_.get_cartesian_limits();
        double max_s_vel = std::numeric_limits<double>::max();
        double max_s_acc = std::numeric_limits<double>::max();

        if (d_trans > 1e-4) {
            max_s_vel = std::min(max_s_vel, clim.max_trans_vel / d_trans);
            max_s_acc = std::min(max_s_acc, clim.max_trans_acc / d_trans);
        }
        if (d_rot > 1e-4) {
            max_s_vel = std::min(max_s_vel, clim.max_rot_vel / d_rot);
            max_s_acc = std::min(max_s_acc, clim.max_rot_acc / d_rot);
        }

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names_;
        
        double current_v_limit = max_s_vel * goal->velocity_scale;
        double current_a_limit = max_s_acc * goal->acceleration_scale;
        
        const int MAX_RETRIES = 5;
        bool trajectory_valid = false;

        const double dt_cycle = 0.001;

        for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
            traj.points.clear();

            ruckig::Ruckig<1> otg_cart(dt_cycle);

            ruckig::InputParameter<1> input;
            ruckig::OutputParameter<1> output;
            
            input.current_position[0] = 0.0; input.current_velocity[0] = 0.0; input.current_acceleration[0] = 0.0;
            input.target_position[0] = 1.0;  input.target_velocity[0] = 0.0;  input.target_acceleration[0] = 0.0;

            input.max_velocity[0] = current_v_limit;
            input.max_acceleration[0] = current_a_limit;
            input.max_jerk[0] = current_a_limit * 10.0;

            Eigen::VectorXd q_current_integrated = start_q;
            const double K_cart = 50.0;

            // Sub-sampling threshold for collision check (~2 degrees)
            const double COLLISION_CHECK_THRESHOLD = 0.035; 
            std::array<double, 6> last_checked_q;
            for(int i=0; i<6; ++i) last_checked_q[i] = start_q[i];

            ruckig::Result res;
            try {
                do {
                    res = otg_cart.update(input, output); 
                    double s = output.new_position[0];
                    double ds = output.new_velocity[0];
                    double dds = output.new_acceleration[0]; 
                    
                    double current_theta = s * total_angle;
                    double dtheta_dt = ds * total_angle;
                    double ddtheta_dt = dds * total_angle;

                    // Ideal Feedforward Pose (Circle)
                    Eigen::Vector3d p_t = center + radius * std::cos(current_theta) * X_dir + radius * std::sin(current_theta) * Y_dir;
                    Eigen::Quaterniond q_target(q_start.slerp(s, q_end));

                    // Ideal Feedforward Twist & Spatial Acceleration
                    Eigen::Vector3d v_world = dtheta_dt * (-radius * std::sin(current_theta) * X_dir + radius * std::cos(current_theta) * Y_dir);
                    Eigen::Vector3d w_world = ds * d_rot * (q_start * aa.axis());

                    Eigen::Vector3d a_world = ddtheta_dt * (-radius * std::sin(current_theta) * X_dir + radius * std::cos(current_theta) * Y_dir) +
                                              (dtheta_dt * dtheta_dt) * (-radius * std::cos(current_theta) * X_dir - radius * std::sin(current_theta) * Y_dir);
                    Eigen::Vector3d alpha_world = dds * d_rot * (q_start * aa.axis());

                    // Current Integrated Pose
                    pinocchio::forwardKinematics(kin_engine_.get_pin_model(), local_data, q_current_integrated);
                    pinocchio::updateFramePlacements(kin_engine_.get_pin_model(), local_data);
                    Eigen::Vector3d p_curr = local_data.oMf[eef_frame_id_].translation();
                    Eigen::Matrix3d R_curr = local_data.oMf[eef_frame_id_].rotation();

                    // Spatial Error Computation (CLIK Feedback)
                    Eigen::Vector3d p_err = p_t - p_curr;
                    Eigen::Matrix3d R_err_mat = q_target.toRotationMatrix() * R_curr.transpose();
                    Eigen::AngleAxisd angle_axis_err(R_err_mat);
                    Eigen::Vector3d w_err = angle_axis_err.axis() * angle_axis_err.angle();

                    // Check tracking deviation
                    if (p_err.norm() > 0.005 || w_err.norm() > 0.05) {
                        throw std::runtime_error("NO_IK: Mathematical path lost. Target unreachable or requires impossible joint flip.");
                    }

                    // Command Twist
                    Eigen::VectorXd V_cmd(6);
                    V_cmd << (v_world + K_cart * p_err), (w_world + K_cart * w_err);

                    // Compute Jacobian & DLS
                    pinocchio::Data::Matrix6x J(6, 6);
                    pinocchio::computeFrameJacobian(kin_engine_.get_pin_model(), local_data, q_current_integrated, eef_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
                    
                    // Check Singularity
                    std::array<double, 6> current_q_arr;
                    for(int i=0; i<6; ++i) current_q_arr[i] = q_current_integrated[i];
                    if (std::abs(kin_engine_.check_singularity(current_q_arr)) < kin_engine_.get_singularity_threshold()) {
                        throw std::runtime_error("Singularity: Trajectory attempts to pass through a kinematic singularity zone.");
                    }

                    double manipulability = std::sqrt((J * J.transpose()).determinant());
                    double lambda = 0.0;
                    double w_threshold = 0.04;
                    if (manipulability < w_threshold) {
                        lambda = 0.05 * std::pow(1.0 - (manipulability / w_threshold), 2);
                    }
                    Eigen::MatrixXd J_dls = J.transpose() * (J * J.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6,6)).inverse();

                    // Differential Kinematics
                    Eigen::VectorXd dq = J_dls * V_cmd;
                    
                    Eigen::VectorXd ddq_zero = Eigen::VectorXd::Zero(6);
                    pinocchio::forwardKinematics(kin_engine_.get_pin_model(), local_data, q_current_integrated, dq, ddq_zero);
                    Eigen::VectorXd J_dot_dq = pinocchio::getFrameAcceleration(kin_engine_.get_pin_model(), local_data, eef_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED).toVector();
                    
                    Eigen::VectorXd A_cmd(6);
                    A_cmd << a_world, alpha_world;
                    Eigen::VectorXd ddq = J_dls * (A_cmd - J_dot_dq);

                    // Absolute numerical integration (Euler)
                    q_current_integrated = q_current_integrated + dq * dt_cycle + 0.5 * ddq * dt_cycle * dt_cycle;

                    // Check Limits and Collisions
                    double max_diff = 0.0;
                    for (int i = 0; i < 6; ++i) {
                        current_q_arr[i] = q_current_integrated[i];
                        max_diff = std::max(max_diff, std::abs(current_q_arr[i] - last_checked_q[i]));
                        
                        if (current_q_arr[i] < kin_engine_.get_joint_limits()[i].min_pos || 
                            current_q_arr[i] > kin_engine_.get_joint_limits()[i].max_pos) {
                            throw std::runtime_error("COLLISION: Joint limit violation detected during MoveC.");
                        }
                    }

                    if (max_diff >= COLLISION_CHECK_THRESHOLD || res != ruckig::Result::Working) {
                        if (kin_engine_.check_collision(current_q_arr)) {
                            throw std::runtime_error("COLLISION: Path segment causes physical collision.");
                        }
                        last_checked_q = current_q_arr;
                    }

                    // Populate Trajectory Point
                    trajectory_msgs::msg::JointTrajectoryPoint pt;
                    for (int i = 0; i < 6; ++i) {
                        pt.positions.push_back(q_current_integrated[i]);
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

            // Strict Physical Limits Violation Check for Time-Scaling
            double max_violation_ratio = 0.0;
            for (const auto& pt : traj.points) {
                for (int i = 0; i < 6; ++i) {
                    double v_ratio = std::abs(pt.velocities[i]) / kin_engine_.get_joint_limits()[i].max_vel;
                    double a_ratio = std::sqrt(std::abs(pt.accelerations[i]) / kin_engine_.get_joint_limits()[i].max_acc); 
                    max_violation_ratio = std::max({max_violation_ratio, v_ratio, a_ratio});
                }
            }

            // Deterministic Scaling Logic
            if (max_violation_ratio <= 1.01) { 
                trajectory_valid = true;
                break;
            } else {
                current_v_limit /= (max_violation_ratio * 1.02);
                current_a_limit /= (max_violation_ratio * max_violation_ratio * 1.02);
                RCLCPP_INFO(this->get_logger(), "MoveC Joint Limit Exceeded (Ratio: %.2f). Time-scaling applied, retrying...", max_violation_ratio);
            }
        }

        if (!trajectory_valid) throw std::runtime_error("Cannot generate valid trajectory. Robot posture restricts requested Cartesian motion.");

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