#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include "utils/graph_math.hpp"

#include <chrono>
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	enum State {
			IDLE,
			TAKEOFF,
			MISSION,
			LAND
		};
	OffboardControl() : Node("offboard_control")
	{
        swarm_graph_ = std::make_unique<SwarmGraph>();
        this->declare_parameter<int>("drone_id", 0);
        int drone_id = this->get_parameter("drone_id").as_int();
        drone_id_ = drone_id;
        std::string node_name = "offboard_control_" + std::to_string(drone_id);
        RCLCPP_INFO(this->get_logger(), "Initializing %s with drone_id: %d", node_name.c_str(), drone_id);
		std::string prefix; // Define outside if-else
		if (drone_id == 0) {
			prefix = "";
		}
		else {
			prefix = "/px4_" + std::to_string(drone_id);
		}	

        /**
         * @brief Publishers and Subscribers   
         */

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(prefix + "/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(prefix + "/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(prefix + "/fmu/in/vehicle_command", 10);
        

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        drone_odom_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(prefix + "/fmu/out/vehicle_odometry", qos,
            [this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg){
                // Handle incoming odometry data here
                odometry_received_ = true;
                this->timestamp_ = msg->timestamp;
                current_position_ = {msg->position[0], msg->position[1], msg->position[2]};
				if (!spawned_)
				{
					spawn_position_ = current_position_;
					target_position_[0] = spawn_position_[0];
					target_position_[1] = spawn_position_[1];
					target_position_[2] = -10.0;
					spawned_ = true;
				}
                
                if (drone_id_ >= 0 && drone_id_ < swarm_size) {
                    all_positions_[drone_id_] = current_position_;
                }
        });

        swarm_size = 4; // Define the swarm size
        all_positions_.resize(swarm_size);

        for (int i = 0; i < swarm_size; i++)
        {
            if (i == drone_id_)
                continue; // Skip subscribing to own odometry
            
            std::string target_topic;
            if (i == 0) {
                target_topic = "/fmu/out/vehicle_odometry";  // for drone_id=0
            } else {
                target_topic = "/px4_" + std::to_string(i) + "/fmu/out/vehicle_odometry";
            }
            
            int position_index = i;  // drone 0 → index 0, drone 1 → index 1, drone 2 → index 2
            
            auto sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(target_topic, qos,
                [this, position_index](const px4_msgs::msg::VehicleOdometry::UniquePtr msg){
                    all_positions_[position_index] = {msg->position[0], msg->position[1], msg->position[2]};
            });
            swarm_subs_.push_back(sub);
        }
        
		current_state_ = State::IDLE;
		offboard_setpoint_counter_=0;

		auto timer_callback = [this]() -> void {
					
		switch (current_state_)
		{
		case State::IDLE:
				// Wait for odometry
				if (!odometry_received_) {
					RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
						"Waiting for odometry...");
					return;  
				}
				
				// Wait until spawn position is recorded
				if (!spawned_) {
					RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
						"Waiting for spawn position...");
					return;
				}
				
				// Send the first 40 setpoints (2 seconds @ 50ms = 40 messages)
				if (offboard_setpoint_counter_ < 40) {
					// Only send setpoints, do not ARM/change mode yet
					break;
				}
				
				// Switch to OFFBOARD mode
				if (offboard_setpoint_counter_ == 40) {
					RCLCPP_INFO(this->get_logger(), "Switching to OFFBOARD mode...");
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				}
				
				// Send ARM commands (try several times)
				if (offboard_setpoint_counter_ >= 45 && offboard_setpoint_counter_ <= 60) {
					this->arm();
				}
				
				// If sufficient time has passed, switch to TAKEOFF
				if (offboard_setpoint_counter_ > 70) {
					RCLCPP_WARN(this->get_logger(), "Switching to TAKEOFF");
					current_state_ = State::TAKEOFF;
				}
				break;
		case State::TAKEOFF:
            // After takeoff and some time, command to go to a position
            if (current_position_[2] <= -9.9){
				RCLCPP_INFO(this->get_logger(), "Takeoff successful, current altitude: %f", current_position_[2]);
				
				// Calculate spawn centroid from all drones' current positions
				Eigen::Vector3d spawn_centroid = Eigen::Vector3d::Zero();
				for (const auto& pos : all_positions_) {
					spawn_centroid += Eigen::Vector3d(pos[0], pos[1], pos[2]);
				}
				spawn_centroid /= static_cast<double>(swarm_size);
				
				RCLCPP_INFO(this->get_logger(), "Spawn centroid: (%.2f, %.2f, %.2f)", 
					spawn_centroid.x(), spawn_centroid.y(), spawn_centroid.z());
				
				// Define desired formation relative to spawn centroid
				std::vector<Eigen::Vector3d> desired_formation;
				double alt = -10.0;
				
				desired_formation.push_back(spawn_centroid + Eigen::Vector3d(0.0, 0.0, alt - spawn_centroid.z()));           // 0
				desired_formation.push_back(spawn_centroid + Eigen::Vector3d(10.0, 0.0, alt - spawn_centroid.z()));      // 1
				desired_formation.push_back(spawn_centroid + Eigen::Vector3d(20.0, 0.0, alt - spawn_centroid.z())); // 2
				desired_formation.push_back(spawn_centroid + Eigen::Vector3d(30.0, 0.0, alt - spawn_centroid.z()));      // 3

				swarm_graph_->setDesiredForm(desired_formation);

				target_position_[0] = static_cast<float>(desired_formation[drone_id_ ].x());
				target_position_[1] = static_cast<float>(desired_formation[drone_id_ ].y());
				target_position_[2] = static_cast<float>(desired_formation[drone_id_ ].z());
				
				RCLCPP_INFO(this->get_logger(), "Drone %d target: (%.2f, %.2f, %.2f)", 
					drone_id_.load(), target_position_[0], target_position_[1], target_position_[2]);
				RCLCPP_INFO(this->get_logger(), "Graph Initialized! Switching to MISSION.");
				current_state_ = State::MISSION;
			
            }
			break;
		case State::MISSION:
			{
					RCLCPP_WARN(this->get_logger(), 
					"[Drone %d] all_positions: [0]=(%.1f,%.1f) [1]=(%.1f,%.1f) [2]=(%.1f,%.1f) [3]=(%.1f,%.1f)",
					drone_id_.load(),
					all_positions_[0][0], all_positions_[0][1],
					all_positions_[1][0], all_positions_[1][1],
					all_positions_[2][0], all_positions_[2][1],
					all_positions_[3][0], all_positions_[3][1]);				
					std::vector<Eigen::Vector3d> swarm_positions_eigen;
					for (const auto& pos_arr : all_positions_) {
						swarm_positions_eigen.push_back(arrayToEigen(pos_arr));
					}
					Eigen::Vector3d vel_cmd = swarm_graph_->computeVelocity(drone_id_ , swarm_positions_eigen);

					TrajectorySetpoint msg{};
					msg.position = {NAN, NAN, NAN}; 
					msg.velocity = {
						(float)vel_cmd.x(), 
						(float)vel_cmd.y(), 
						(float)vel_cmd.z()
					};
					msg.yaw = NAN; 
					trajectory_setpoint_publisher_->publish(msg);
				}
            
			break;
		case State::LAND:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 6);
			break;
		default:
			break;
		}	
			publish_offboard_control_mode();
			if (current_state_ != State::MISSION) {
				publish_trajectory_setpoint();
			}

			if (offboard_setpoint_coun