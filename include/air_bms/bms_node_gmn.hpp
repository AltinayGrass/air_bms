#ifndef BMS_NODE_
#define BMS_NODE_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "air_bms/bms_uart_gmn.hpp" // Include the updated BMS UART interface header
#include <chrono>             // For time durations
#include <string>             // For std::string

/**
 * @class BatteryStatus
 * @brief A ROS 2 node that interfaces with the BMS via BMS_UART, retrieves battery data,
 *        and publishes it as a sensor_msgs/BatteryState message. It also logs a summary periodically.
 */
class BatteryStatus : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Battery Status Node object.
     * @details Initializes the node, the BMS_UART interface, the publisher, and the timer.
     */
    BatteryStatus();

private:
    /**
     * @brief Callback function triggered periodically by the timer.
     * @details Attempts to update data from the BMS using bms_.update(). If successful,
     *          it populates and publishes a BatteryState message. Handles communication errors
     *          and performs periodic logging.
     */
    void BatteryStatusCallBack();

    /**
     * @brief Helper function to convert ROS power supply status enum to a readable string.
     * @param status The status code from sensor_msgs::msg::BatteryState::power_supply_status.
     * @return std::string A human-readable string representing the status (e.g., "Charging").
     */
    std::string getStatusString(uint8_t status);

    std::string getHealthString(uint8_t health);

    // --- Member Variables ---

    BMS_UART bms_; // Instance of the BMS UART communication class

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_; // ROS 2 Publisher for BatteryState messages
    rclcpp::TimerBase::SharedPtr timer_; // ROS 2 Timer to trigger periodic updates

    // --- Logging Control ---
    rclcpp::Time last_log_time_; // Stores the timestamp of the last summary log message
    // Define the logging interval (e.g., 10 seconds) using chrono literals
    const rclcpp::Duration LOG_INTERVAL = rclcpp::Duration(std::chrono::seconds(10));

}; // End class BatteryStatus

#endif // AIR_BMS__BMS_NODE_HPP_