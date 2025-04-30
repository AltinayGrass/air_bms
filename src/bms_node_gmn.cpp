#include "air_bms/bms_node_gmn.hpp" // Include the corresponding header file

#include <chrono>             // For std::chrono::seconds
#include <sstream>            // For std::ostringstream (used for logging)
#include <rclcpp/rclcpp.hpp>  // ROS 2 C++ client library
#include <sensor_msgs/msg/battery_state.hpp> // Standard ROS message for battery status
#include <cmath>              // For std::nanf, std::abs
#include <limits>             // For std::numeric_limits

// Use chrono literals for defining time durations (e.g., 1s)
using namespace std::chrono_literals;

/**
 * @brief Constructor for the BatteryStatus ROS 2 Node.
 */
BatteryStatus::BatteryStatus() :
    Node{"bms_status"},                           // Initialize Node with name "bms_status"
    bms_{"/dev/ttyUSB0"}                          // Initialize BMS_UART object with the serial port device path
{
    RCLCPP_INFO(this->get_logger(), "Initializing BMS Status Node...");

    // Attempt to initialize the BMS serial communication settings
    if (!bms_.Init())
    {
        // Log a fatal error and request shutdown if BMS initialization fails
        RCLCPP_FATAL(this->get_logger(), "BMS hardware initialization failed! Check port, permissions, and BMS power. Shutting down.");
        // Signal ROS 2 to shutdown. Using rclcpp::shutdown() is generally preferred within a node's scope
        // if initialization fails critically, rather than calling exit().
        // Need a slight delay or separate thread to allow the shutdown to propagate if called directly here.
        // A common pattern is to let the constructor return and check rclcpp::ok() in main.
        // For simplicity here, we log FATAL and proceed, main() should handle the shutdown if ok() is false.
         if (rclcpp::ok()) {
             rclcpp::shutdown();
         }
        return; // Exit constructor early
    }

    RCLCPP_INFO(this->get_logger(), "BMS hardware initialized successfully.");

    // --- ROS 2 Communication Setup ---

    // Create a publisher for the BatteryState message on the "bms_status" topic
    // QoS setting: Use reliable communication, keep last 10 messages. Adjust if needed.
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("bms_status", qos_profile);

    // Create a wall timer that triggers the BatteryStatusCallBack function every 1 second
    timer_ = this->create_wall_timer(
        1s, // Use chrono literal for 1 second
        std::bind(&BatteryStatus::BatteryStatusCallBack, this));

    // --- Logging Setup ---

    // Initialize the time point for the last log message using the node's clock
    // Set it initially so that the first check in the callback triggers a log immediately.
    last_log_time_ = this->get_clock()->now() - LOG_INTERVAL;

    RCLCPP_INFO(this->get_logger(), "BMS Status node initialization complete. Publishing data every 1s, logging summary every %ld s.",
                 std::chrono::duration_cast<std::chrono::seconds>(LOG_INTERVAL.to_chrono<std::chrono::nanoseconds>()).count());
}

/**
 * @brief Callback function triggered by the 1-second wall timer.
 * @details Fetches data from BMS, handles errors, publishes BatteryState, and logs summary.
 */
void BatteryStatus::BatteryStatusCallBack()
{
    // --- Data Acquisition ---
    // Call the update function of the BMS_UART class and get the communication status.
    BMS_UART::CommStatus status = bms_.update();

    // Get the current time once for timestamping and logging interval check
    rclcpp::Time now = this->get_clock()->now();

    // --- Handle Communication Status ---
    if (status != BMS_UART::CommStatus::SUCCESS) {
        // Log a warning indicating the failure and the specific status code
        RCLCPP_WARN(this->get_logger(), "Failed to update BMS data! Status: %d", static_cast<int>(status));

        // Publish a BatteryState message indicating the error/unknown state
        sensor_msgs::msg::BatteryState error_msg;
        error_msg.header.stamp = now;
        error_msg.header.frame_id = "bms_link"; // Use a consistent frame_id
        error_msg.present = false; // Indicate battery data is not reliable or battery not present
        error_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        error_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        // Fill potentially invalid fields with NaN (Not a Number)
        error_msg.voltage = std::numeric_limits<float>::quiet_NaN();
        error_msg.current = std::numeric_limits<float>::quiet_NaN();
        error_msg.percentage = std::numeric_limits<float>::quiet_NaN();
        error_msg.charge = std::numeric_limits<float>::quiet_NaN(); // Remaining capacity
        error_msg.capacity = std::numeric_limits<float>::quiet_NaN(); // Full capacity
        error_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
        error_msg.temperature = std::numeric_limits<float>::quiet_NaN();
        error_msg.cell_voltage.clear(); // No valid cell data

        publisher_->publish(error_msg);

        // If the error seems persistent (timeout, port closed, no response), log more severely
        if (status == BMS_UART::CommStatus::ERROR_NO_RESPONSE ||
            status == BMS_UART::CommStatus::ERROR_PORT_CLOSED ||
            status == BMS_UART::CommStatus::ERROR_READ_TIMEOUT ||
            status == BMS_UART::CommStatus::ERROR_WRITE_FAILED)
        {
             RCLCPP_ERROR(this->get_logger(), "Persistent BMS communication failure detected. Status: %d. Check connection/BMS power.", static_cast<int>(status));
             // Consider further actions here: e.g., attempt re-initialization, stop the timer after N errors, or rely on external process monitoring.
        }

        // Do not proceed with populating message from potentially stale data or logging summary
        return;
    }

    // --- Message Preparation (if status is SUCCESS) ---

    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now;             // Set message timestamp
    msg.header.frame_id = "bms_link";   // Set frame ID (make this configurable if needed)

    // Populate the message fields using data directly from the bms_.get struct
    msg.voltage = bms_.get.packVoltage;           // Total battery pack voltage (Volts)
    msg.current = bms_.get.packCurrent;           // Current (Amperes). Positive=charging, Negative=discharging.
    // Convert remaining capacity from mAh to Ah
    msg.charge = static_cast<float>(bms_.get.resCapacitymAh) / 1000.0f;
    msg.capacity = 46.0f;                         // Full charge capacity (Ah) - TODO: Get from BMS if possible, or make parameter
    msg.design_capacity = 46.0f;                  // Design capacity (Ah) - TODO: Get from BMS if possible, or make parameter
    // Convert SOC from % (0-100) to ratio (0.0-1.0)
    msg.percentage = bms_.get.packSOC / 100.0f;
    msg.temperature = bms_.get.tempAverage;       // Average battery temperature (Celsius). Verify unit from BMS docs.
    msg.present = true;                           // Indicate that the battery is present and data is valid
    msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION; // Set battery chemistry

    // Map the BMS status code (bms_.get.batteryStatus) to ROS standard power supply status
    switch (bms_.get.batteryStatus) {
        case 1: // BMS indicates Charging
            msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
            break;
        case 2: // BMS indicates Discharging
            msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
            break;
        case 0: // BMS indicates Stationary
             // Infer based on current and SOC if possible
             if (std::abs(msg.current) < 0.1f && msg.percentage >= 0.98f) { // Near zero current and high SOC -> Full
                  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
             } else if (std::abs(msg.current) < 0.1f) { // Near zero current, not full -> Not Charging
                  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
             } else { // Current is flowing but status is Stationary? Ambiguous case.
                 msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
             }
            break;
        default: // Unknown status code from BMS
            msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
            break;
    }

    // Populate individual cell voltages safely
    if (bms_.get.numberOfCells >= BMS_UART::MIN_NUMBER_CELLS && bms_.get.numberOfCells <= BMS_UART::MAX_NUMBER_CELLS) {
        msg.cell_voltage.resize(bms_.get.numberOfCells); // Resize vector to hold voltage for each cell
        for (int i = 0; i < bms_.get.numberOfCells; ++i) {
            // Convert millivolts (mV) from BMS to volts (V) for the message
            msg.cell_voltage[i] = bms_.get.cellVmV[i] / 1000.0f;
        }
    } else {
         // Handle invalid number of cells reported, even if update() succeeded overall
         if (bms_.get.numberOfCells != 0) { // Avoid logging if 0 cells reported (might be initial state)
            RCLCPP_WARN(this->get_logger(), "Invalid number of cells (%d) reported by BMS, clearing cell_voltage field.", bms_.get.numberOfCells);
         }
         msg.cell_voltage.clear(); // Ensure the vector is empty if cell count is invalid
    }

    // Determine battery health based on alarm flags from bms_.alarm struct
    // Check most critical alarms first
    if (bms_.alarm.levelTwoCellVoltageTooHigh || bms_.alarm.levelTwoPackVoltageTooHigh ||
        bms_.alarm.levelOneCellVoltageTooHigh || bms_.alarm.levelOnePackVoltageTooHigh) {
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    } else if (bms_.alarm.levelTwoCellVoltageTooLow || bms_.alarm.levelTwoPackVoltageTooLow ||
               bms_.alarm.levelOneCellVoltageTooLow || bms_.alarm.levelOnePackVoltageTooLow ||
               bms_.alarm.failureOfLowVoltageNoCharging) {
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD; // Or HEALTH_UNKNOWN
    } else if (bms_.alarm.levelTwoDischargeTempTooHigh || bms_.alarm.levelTwoChargeTempTooHigh ||
               bms_.alarm.levelOneDischargeTempTooHigh || bms_.alarm.levelOneChargeTempTooHigh ||
               bms_.alarm.chargeFETTemperatureTooHigh || bms_.alarm.dischargeFETTemperatureTooHigh) {
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
    } else if (bms_.alarm.levelTwoDischargeTempTooLow || bms_.alarm.levelTwoChargeTempTooLow ||
               bms_.alarm.levelOneDischargeTempTooLow || bms_.alarm.levelOneChargeTempTooLow) {
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_COLD;
    } else if (bms_.alarm.levelTwoDischargeCurrentTooHigh || bms_.alarm.levelTwoChargeCurrentTooHigh ||
               bms_.alarm.levelOneDischargeCurrentTooHigh || bms_.alarm.levelOneChargeCurrentTooHigh) {
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE; // No specific OVERCURRENT health state, use OVERVOLTAGE as proxy? or UNKNOWN?
    } else if (bms_.alarm.failureOfAFEAcquisitionModule || bms_.alarm.failureOfVoltageSensorModule ||
               bms_.alarm.failureOfTemperatureSensorModule || bms_.alarm.failureOfEEPROMStorageModule ||
               bms_.alarm.failureOfCurrentSensorModule || bms_.alarm.failureOfMainVoltageSensorModule) {
         msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
    } else {
        // If no specific critical alarms are active, assume health is good
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }

    // --- Publishing ---
    publisher_->publish(msg);

    // --- Conditional Logging (every LOG_INTERVAL seconds) ---
    if (now - last_log_time_ >= LOG_INTERVAL)
    {
        // Prepare the log string using an output string stream for better formatting
        std::ostringstream log_bms;
        log_bms << "\n--- BMS Status Summary (SUCCESS) ---"
                // Include raw status string from BMS and mapped ROS status string
                << "\n[ Charge Status ] : " << bms_.get.chargeDischargeStatus << " (" << getStatusString(msg.power_supply_status) << ")"
                << "\n[ Voltage ]       : " << msg.voltage << " V"
                << "\n[ Current ]       : " << msg.current << " A"
                << "\n[ State of Charge ] : " << msg.percentage * 100.0 << " %"
                << "\n[ Temperature (Avg) ] : " << msg.temperature << " C" // Ensure unit is Celsius
                << "\n[ Remaining Capacity ]: " << msg.charge << " Ah"
                << "\n[ BMS Heartbeat ] : " << bms_.get.bmsHeartBeat // Assuming this is a counter or status indicator
                << "\n[ Cycle Count ]   : " << bms_.get.bmsCycles
                << "\n[ Health Status ] : " << static_cast<int>(msg.power_supply_health); // Log health code

        // Log the formatted string using RCLCPP_INFO level
        RCLCPP_INFO(this->get_logger(), "%s", log_bms.str().c_str());

        // Update the time of the last log message
        last_log_time_ = now;
    }
}

/**
 * @brief Helper function to convert power supply status enum to string for logging.
 */
std::string BatteryStatus::getStatusString(uint8_t status) {
    switch (status) {
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING:     return "Charging";
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:  return "Discharging";
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING: return "Not Charging";
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL:         return "Full";
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN:
        default:                                                               return "Unknown";
    }
}


// --- Main Function ---
// Entry point of the program
int main(int argc, char **argv)
{
    // Initialize the ROS 2 C++ client library
    rclcpp::init(argc, argv);

    std::shared_ptr<BatteryStatus> node = nullptr;
    try {
        // Create a shared pointer to an instance of the BatteryStatus node
        node = std::make_shared<BatteryStatus>();

        // Check if ROS 2 is still okay (initialization might have failed and requested shutdown)
        if (rclcpp::ok()) {
           RCLCPP_INFO(node->get_logger(), "Starting BMS Status node spin loop.");
           // Spin the node, making it process callbacks (timers, subscriptions, etc.)
           // This keeps the node alive and responsive until shutdown is called externally or internally.
           rclcpp::spin(node);
        } else {
            // Log if initialization failed leading to immediate shutdown request
             std::cerr << "ROS 2 shutdown requested during node initialization." << std::endl;
        }

    } catch (const std::exception & e) {
        // Catch potential exceptions during node creation or spinning
        // Using std::cerr here as logger might not be available if node creation failed badly
        std::cerr << "Unhandled exception in BMS node: " << e.what() << std::endl;
        // Ensure shutdown is called even if spin didn't start/finish cleanly
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1; // Indicate error exit
    } catch (...) {
         std::cerr << "Unknown exception in BMS node." << std::endl;
          if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }


    // Shutdown the ROS 2 C++ client library cleanly (if not already done)
    // Note: rclcpp::shutdown() can be called multiple times safely.
    rclcpp::shutdown();

    std::cout << "BMS Status node finished." << std::endl;
    // Return 0 indicating successful execution (or controlled shutdown)
    return 0;
}