#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Include the action definition and kinematics engine header
#include "lite6_interfaces/action/industrial_motion.hpp"
#include "lite6_planner/kinematics_engine.hpp"

// Ruckig & Pinocchio
#include <ruckig/ruckig.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
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
        // 1. Initialize Pinocchio and kinematics engine
        std::string pkg_path = ament_index_cpp::get_package_share_directory("lite6_description");
        std::string urdf_path = pkg_path + "/urdf/lite6.urdf";
        std::string limits_path = ament_index_cpp::get_package_share_directory("lite6_planner") + "/config/kinematic_limits.yaml";

        pinocchio::urdf::buildModel(urdf_path, pin_model_);
        pin_data_ = pinocchio::Data(pin_model_);
        eef_frame_id_ = pin_model_.getFrameId("link_eef");

        if (!kin_engine_.load_limits(limits_path)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load kinematic limits!");
            throw std::runtime_error("Limits load failed");
        }

        current_q_ = Eigen::VectorXd::Zero(6);
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        // 2. Initialize ROS2 subscriptions and action servers
        js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
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
    pinocchio::Model pin_model_;
    pinocchio::Data pin_data_;
    pinocchio::FrameIndex eef_frame_id_;
    KinematicsEngine kin_engine_;

    Eigen::VectorXd current_q_;
    std::vector<std::string> joint_names_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp_action::Client<FJT>::SharedPtr jtc_client_;
    rclcpp_action::Server<IndustrialMotion>::SharedPtr action_server_;

    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const IndustrialMotion::Goal> goal) {
        if (goal->velocity_scale <= 0.0 || goal->velocity_scale > 1.0) return rclcpp_action::GoalResponse::REJECT;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMotion>&) {
        // Cancel all goals if a cancel request is received
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
            result->error_code = IndustrialMotion::Result::ERR_NO_IK_SOLUTION;
            result->error_string = e.what();
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Planning Aborted: %s", e.what());
            return;
        }

        // Send the planned trajectory to the JTC action server
        send_to_jtc(traj_msg, goal_handle, result);
    }

    // Parameterized PTP planning using Ruckig
    trajectory_msgs::msg::JointTrajectory plan_ptp(std::shared_ptr<const IndustrialMotion::Goal> goal)
    {
        std::array<double, 6> q_target_arr;
        
        if (goal->command_type == IndustrialMotion::Goal::MOVEJ) {
            for(int i=0; i<6; ++i) q_target_arr[i] = goal->joint_target[i];
        } else {
            // MoveP: Turns the target pose into joint angles using optimal IK
            Eigen::Matrix4d T_goal = pose_to_matrix(goal->pose_target);
            std::array<double, 6> q_ref;
            Eigen::VectorXd::Map(&q_ref[0], 6) = current_q_;
            
            if (!kin_engine_.solve_optimal_ik(T_goal, q_ref, q_target_arr, false)) {
                throw std::runtime_error("IK Failed or Limits Violated for Target Pose.");
            }
        }

        // Instantiate Ruckig for 6 DoF with a time step of 10ms
        double dt = 0.01;
        ruckig::Ruckig<6> otg(dt);
        ruckig::InputParameter<6> input;
        ruckig::OutputParameter<6> output;

        for (int i = 0; i < 6; ++i) {
            input.current_position[i] = current_q_[i];
            input.current_velocity[i] = 0.0;
            input.current_acceleration[i] = 0.0;
            
            input.target_position[i] = q_target_arr[i];
            input.target_velocity[i] = 0.0;
            input.target_acceleration[i] = 0.0;

            const auto& lim = kin_engine_.get_joint_limits()[i];
            input.max_velocity[i] = lim.max_vel * goal->velocity_scale;
            input.max_acceleration[i] = lim.max_acc * goal->acceleration_scale;
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

        // Guarantee that the last point has zero velocity and acceleration
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
        pinocchio::forwardKinematics(pin_model_, pin_data_, current_q_);
        pinocchio::updateFramePlacements(pin_model_, pin_data_);
        pinocchio::SE3 pose_start = pin_data_.oMf[eef_frame_id_];
        
        Eigen::Matrix4d T_goal = pose_to_matrix(goal->pose_target);
        pinocchio::SE3 pose_end(T_goal.block<3,3>(0,0), T_goal.block<3,1>(0,3));

        double d_trans = (pose_end.translation() - pose_start.translation()).norm();
        Eigen::AngleAxisd aa(pose_start.rotation().transpose() * pose_end.rotation());
        double d_rot = std::abs(aa.angle());

        if (d_trans < 1e-4 && d_rot < 1e-4) throw std::runtime_error("Start and End pose are identical.");

        // 1D Ruckig path planning parameter s \in [0, 1]
        double dt = 0.01;
        ruckig::Ruckig<1> otg(dt);
        ruckig::InputParameter<1> input;
        ruckig::OutputParameter<1> output;
        
        input.current_position[0] = 0.0; input.current_velocity[0] = 0.0; input.current_acceleration[0] = 0.0;
        input.target_position[0] = 1.0;  input.target_velocity[0] = 0.0;  input.target_acceleration[0] = 0.0;

        const auto& clim = kin_engine_.get_cartesian_limits();
        double max_v_s = std::numeric_limits<double>::max();
        if (d_trans > 1e-4) max_v_s = std::min(max_v_s, clim.max_trans_vel / d_trans);
        if (d_rot > 1e-4) max_v_s = std::min(max_v_s, clim.max_rot_vel / d_rot);
        
        input.max_velocity[0] = max_v_s * goal->velocity_scale;
        input.max_acceleration[0] = (clim.max_trans_acc / std::max(d_trans, 0.01)) * goal->acceleration_scale;
        input.max_jerk[0] = input.max_acceleration[0] * 10.0;

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names_;
        
        std::array<double, 6> q_ref;
        Eigen::VectorXd::Map(&q_ref[0], 6) = current_q_;
        Eigen::VectorXd prev_dq = Eigen::VectorXd::Zero(6);

        ruckig::Result res;
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
            std::array<double, 6> q_ik;
            if (!kin_engine_.solve_optimal_ik(T_t, q_ref, q_ik, true)) {
                throw std::runtime_error("Singularity or Quadrant Jump detected during MOVEL.");
            }
            q_ref = q_ik;

            // 3. Derive joint velocities using Jacobian
            Eigen::VectorXd q_eig = Eigen::Map<Eigen::VectorXd>(q_ik.data(), 6);
            Eigen::Vector3d v_world = ds * (pose_end.translation() - pose_start.translation());
            Eigen::Vector3d w_world = ds * d_rot * (pose_start.rotation() * aa.axis());
            
            pinocchio::Data::Matrix6x J(6, 6);
            pinocchio::computeFrameJacobian(pin_model_, pin_data_, q_eig, eef_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
            
            Eigen::VectorXd V_spatial(6);
            V_spatial << v_world, w_world;
            Eigen::VectorXd dq = J.colPivHouseholderQr().solve(V_spatial); 

            // 4. Discretize acceleration
            Eigen::VectorXd ddq = (dq - prev_dq) / dt;
            prev_dq = dq;

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            for (int i = 0; i < 6; ++i) {
                pt.positions.push_back(q_ik[i]);
                pt.velocities.push_back(dq(i));
                pt.accelerations.push_back(ddq(i));
            }
            pt.time_from_start = rclcpp::Duration::from_seconds(output.time);
            traj.points.push_back(pt);
            
            output.pass_to_input(input);
        } while (res == ruckig::Result::Working);

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
        pinocchio::forwardKinematics(pin_model_, pin_data_, current_q_);
        pinocchio::updateFramePlacements(pin_model_, pin_data_);
        
        Eigen::Vector3d p1 = pin_data_.oMf[eef_frame_id_].translation();
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

        // 3. 1D Ruckig
        double dt = 0.01;
        ruckig::Ruckig<1> otg(dt);
        ruckig::InputParameter<1> input;
        ruckig::OutputParameter<1> output;
        
        input.current_position[0] = 0.0; input.current_velocity[0] = 0.0; input.current_acceleration[0] = 0.0;
        input.target_position[0] = 1.0;  input.target_velocity[0] = 0.0;  input.target_acceleration[0] = 0.0;

        const auto& clim = kin_engine_.get_cartesian_limits();
        input.max_velocity[0] = (clim.max_trans_vel / d_trans) * goal->velocity_scale;
        input.max_acceleration[0] = (clim.max_trans_acc / d_trans) * goal->acceleration_scale;
        input.max_jerk[0] = input.max_acceleration[0] * 10.0;

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = joint_names_;
        
        std::array<double, 6> q_ref;
        Eigen::VectorXd::Map(&q_ref[0], 6) = current_q_;
        Eigen::VectorXd prev_dq = Eigen::VectorXd::Zero(6);

        Eigen::Quaterniond q_start(pin_data_.oMf[eef_frame_id_].rotation());
        Eigen::Quaterniond q_end(pose_to_matrix(goal->pose_target).block<3,3>(0,0));
        Eigen::AngleAxisd aa(q_start.inverse() * q_end);

        ruckig::Result res;
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
            std::array<double, 6> q_ik;
            if (!kin_engine_.solve_optimal_ik(T_t, q_ref, q_ik, true)) {
                throw std::runtime_error("Singularity or Quadrant Jump detected during MOVEC.");
            }
            q_ref = q_ik;

            // 3. Jacobian inverse mapping for velocity
            Eigen::VectorXd q_eig = Eigen::Map<Eigen::VectorXd>(q_ik.data(), 6);
            double dtheta_dt = ds * total_angle;
            Eigen::Vector3d v_world = dtheta_dt * (-radius * std::sin(current_theta) * X_dir + radius * std::cos(current_theta) * Y_dir);
            Eigen::Vector3d w_world = ds * aa.angle() * (q_start * aa.axis());

            pinocchio::Data::Matrix6x J(6, 6);
            pinocchio::computeFrameJacobian(pin_model_, pin_data_, q_eig, eef_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);
            
            Eigen::VectorXd V_spatial(6); V_spatial << v_world, w_world;
            Eigen::VectorXd dq = J.colPivHouseholderQr().solve(V_spatial);

            Eigen::VectorXd ddq = (dq - prev_dq) / dt;
            prev_dq = dq;

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            for (int i = 0; i < 6; ++i) {
                pt.positions.push_back(q_ik[i]);
                pt.velocities.push_back(dq(i));
                pt.accelerations.push_back(ddq(i));
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