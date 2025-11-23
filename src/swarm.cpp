#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <vector>
#include <cmath>

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
        this->declare_parameter<int>("drone_id", 1);
        int drone_id = this->get_parameter("drone_id").as_int();
        drone_id_ = drone_id;
        std::string node_name = "offboard_control_" + std::to_string(drone_id);
        RCLCPP_INFO(this->get_logger(), "Initializing %s with drone_id: %d", node_name.c_str(), drone_id);
		std::string prefix = "/px4_" + std::to_string(drone_id);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(prefix + "/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(prefix + "/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(prefix + "/fmu/in/vehicle_command", 10);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(prefix + "/fmu/out/vehicle_odometry", qos,
		[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg){
			// Handle incoming odometry data here
			odometry_received_ = true;
			this->timestamp_ = msg->timestamp;
			current_position_ = {msg->position[0], msg->position[1], msg->position[2]};
		});

		current_state_ = State::IDLE;
		offboard_setpoint_counter_=0;
		current_waypoint_index_ = 0;

		auto timer_callback = [this]() -> void {
					
		switch (current_state_)
		{
		case State::IDLE:
			if (offboard_setpoint_counter_ >= 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
				current_state_ = State::TAKEOFF;
				
				RCLCPP_INFO(this->get_logger(), "Switched to Offboard mode and armed the vehicle");
			}
			break;
		case State::TAKEOFF:
			if (Takeoff_Success()) {
                // After takeoff and some time, command to go to a position
				RCLCPP_WARN(this->get_logger(), "MISSION has started");
                this->goto_position(waypoints_[0][0], waypoints_[0][1], waypoints_[0][2], NAN); // x=10m, y=0m, z=5m, yaw=90 degrees
				RCLCPP_INFO(this->get_logger(), "Going to position (%.2f, %.2f, %.2f)", waypoints_[0][0], waypoints_[0][1], waypoints_[0][2]);
				current_state_ = State::MISSION;
				
            }
			break;
		case State::MISSION:
			WaypointHandler();
			break;
		case State::LAND:
			this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 2, 6);
			break;
		default:
			break;
		}	
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 100
			if (offboard_setpoint_counter_ < 100) {
				offboard_setpoint_counter_++;
			}
			
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();
    void land();
    void goto_position(float x, float y, float z, float yaw_deg);
	bool OdometryReceived();
	bool WaypointReached(const std::array<float, 3>& current, const std::array<float, 3>& target, float threshold);
	void WaypointHandler();
	bool Takeoff_Success();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
	std::atomic<bool> odometry_received_{false};
	std::array<float, 3> current_position_ = {0.0, 0.0, -5.0}; // x, y, z
	std::atomic<size_t> current_waypoint_index_;
	std::atomic<bool> is_mission_started_;
	std::vector<std::array<float, 3>> waypoints_ = {
		{0.0, 0.0, -5.0},
		{10.0, 0.0, -5.0},
		{10.0, 10.0, -5.0},
		{0.0, 10.0, -5.0},
		{0.0, 0.0, -5.0}
	};
	State current_state_;
    std::atomic<int> drone_id_;

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    // Target position and yaw for offboard control
    std::array<float, 3> target_position_ = {0.0, 0.0, -5.0}; // x, y, z
    float target_yaw_ = 0.0;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);

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
	msg.position = true;
	msg.velocity = false;
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

void OffboardControl::WaypointHandler()
{
	if (current_state_ != State::MISSION || !OdometryReceived()) {
		return;
	}

	if (WaypointReached(current_position_, waypoints_[current_waypoint_index_], 0.5)) {
		RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_waypoint_index_ + 1);
		current_waypoint_index_++;
		if (current_waypoint_index_ < waypoints_.size()) {
			goto_position(waypoints_[current_waypoint_index_][0], waypoints_[current_waypoint_index_][1], waypoints_[current_waypoint_index_][2], NAN);
			RCLCPP_INFO(this->get_logger(), "Going to position (%.2f, %.2f, %.2f)", waypoints_[current_waypoint_index_][0], waypoints_[current_waypoint_index_][1], waypoints_[current_waypoint_index_][2]);
		} else {
			RCLCPP_WARN(this->get_logger(), "Mission completed. Landing...");
			current_state_ = State::LAND;
		}
	}
}

bool OffboardControl::Takeoff_Success()
{
	if (!WaypointReached(current_position_, {0.0, 0.0, -5.0}, 0.5)) {
		return false;
	}
	else
	{
		return true;
	}
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

bool OffboardControl::WaypointReached(const std::array<float, 3>& current, const std::array<float, 3>& target, float threshold = 0.5)
{
	float dx = current[0] - target[0];
	float dy = current[1] - target[1];
	float dz = current[2] - target[2];
	float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
	return distance < threshold;
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
    msg.target_system = drone_id_ + 1;
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
    
    // Create OffboardControl and OdometryListener nodes
    rclcpp::spin(std::make_shared<OffboardControl>());
 
    rclcpp::shutdown();
    return 0;
}