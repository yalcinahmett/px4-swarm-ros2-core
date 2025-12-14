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
					target_position_[2] = -5.0;
					spawned_ = true;
				}
                
                // Add own position to swarm positions
                if (drone_id_ >= 0 && drone_id_ < swarm_size) {
                    all_positions_[drone_id_] = current_position_;
                }
        });

        swarm_size = 4; // Define the swarm size
        all_positions_.resize(swarm_size);

        for (int i = 0; i < swarm_size; i++)
        {
            if (i == drone_id_)
                continue; // Already received own odometry above
            
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
				
				// Wait for spawn position
				if (!spawned_) {
					RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
						"Waiting for spawn position...");
					return;
				}
				
				// Send first 40 setpoints (2 seconds @ 50ms = 40 messages)
				if (offboard_setpoint_counter_ < 40) {
					// Only send setpoints, do not change ARM/mode yet
					break;
				}
				
				// Switch to Offboard mode
				if (offboard_setpoint_counter_ == 40) {
					RCLCPP_INFO(this->get_logger(), "Switching to OFFBOARD mode...");
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				}
				
				// Send ARM commands (try several times)
				if (offboard_setpoint_counter_ >= 45 && offboard_setpoint_counter_ <= 60) {
					this->arm();
				}
				
				// Switch to TAKEOFF if enough time has passed
				if (offboard_setpoint_counter_ > 70) {
					RCLCPP_WARN(this->get_logger(), "Switching to TAKEOFF");
					current_state_ = State::TAKEOFF;
				}
				break;
		case State::TAKEOFF:
            // After takeoff and some time, command to go to a position
            if (current_position_[2] <= -4.9){
				RCLCPP_INFO(this->get_logger(), "Takeoff successful, current altitude: %f", current_position_[2]);
				
				// Calculate spawn centroid from all drones' current positions
				Eigen::Vector3d spawn_centroid = Eigen::Vector3d::Zero();
				for (const auto& pos : all_positions_) {
					spawn_centroid += Eigen::Vector3d(pos[0], pos[1], pos[2]);
				}
				spawn_centroid /= static_cast<double>(swarm_size);
				
				RCLCPP_INFO(this->get_logger(), "Spawn centroid: (%.2f, %.2f, %.2f)", 
					spawn_centroid.x(), spawn_centroid.y(), spawn_centroid.z());
				
				// Create MUCH WIDER formation - 8 meters instead of 5
				std::vector<Eigen::Vector3d> desired_formation;
				double alt = -5.0;
				double spacing = 8.0;  // Increased spacing
				
				// Square formation with wider spacing
				desired_formation.push_back(spawn_centroid + Eigen::Vector3d(0.0, 0.0, alt - spawn_centroid.z()));           // 0
				desired_formation.push_back(spawn_centroid + Eigen::Vector3d(0.0, -spacing, alt - spawn_centroid.z()));      // 1
				desired_formation.push_back(spawn_centroid + Eigen::Vector3d(-spacing, -spacing, alt - spawn_centroid.z())); // 2
				desired_formation.push_back(spawn_centroid + Eigen::Vector3d(-spacing, 0.0, alt - spawn_centroid.z()));      // 3

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

			if (offboard_setpoint_counter_ < 100) {
				offboard_setpoint_counter_++;
			}
					
		};
		timer_ = this->create_wall_timer(50ms, timer_callback);
	}

	void arm();
	void disarm();
    void land();
    void goto_position(float x, float y, float z, float yaw_deg);
	bool OdometryReceived();
	bool Takeoff_Success();

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr drone_odom_subscription_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    int swarm_size;
	std::atomic<bool> odometry_received_{false};
	std::array<float, 3> current_position_ = {0.0, 0.0, -5.0}; // x, y, z
	std::array<float, 3> spawn_position_ = {0.0, 0.0, 0.0}; // x, y, z
    std::vector<std::array<float, 3>> all_positions_;
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr> swarm_subs_;
	std::atomic<size_t> current_waypoint_index_;
	std::atomic<bool> is_mission_started_;
	State current_state_;
    std::atomic<int> drone_id_;
    std::unique_ptr<SwarmGraph> swarm_graph_;
    bool graph_initialized_ = false;

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    // Target position and yaw for offboard control
	bool spawned_ = false;
    std::array<float, 3> target_position_ = {0.0, 0.0, -5.0}; // x, y, z
    float target_yaw_ = 0.0;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);
    Eigen::Vector3d arrayToEigen(const std::array<float, 3> &arr){
        Eigen::Vector3d output_vector;
        output_vector.x()=static_cast<double>(arr[0]);
        output_vector.y()=static_cast<double>(arr[1]);
        output_vector.z()=static_cast<double>(arr[2]);
        return output_vector;   
    }
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Send a command to takeoff to a specified altitude
 * @param altitude  Target altitude in meters
 */

void OffboardControl::goto_position(float x, float y, float z, float yaw_deg)
{
    target_position_ = {x, y, z};
    target_yaw_ = yaw_deg * (M_PI / 180.0); // Convert degrees to radians
    RCLCPP_INFO(this->get_logger(), "Go to position command send");
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = target_position_; // x, y, z in meters
	msg.yaw = target_yaw_; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

bool OffboardControl::OdometryReceived()
{
	// Placeholder function to check if odometry has been received
	return odometry_received_;
}
/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 * @param param3    Command parameter 3
 * @param param4    Command parameter 4
 * @param param5    Command parameter 5
 * @param param6    Command parameter 6
 * @param param7    Command parameter 7
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.command = command;
    msg.target_system = drone_id_ + 1; // Assuming drone IDs start from 1 in PX4
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting waypoint navigation..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    
    // Create OffboardControl node
    rclcpp::spin(std::make_shared<OffboardControl>());
 
    rclcpp::shutdown();
    return 0;
}