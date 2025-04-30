#include "air_bms/bms_node_gmn.hpp" // Include the corresponding header file

#include <chrono>             // For std::chrono::duration, std::chrono::nanoseconds
#include <sstream>            // For std::ostringstream (used for logging)
#include <rclcpp/rclcpp.hpp>  // ROS 2 C++ client library
#include <sensor_msgs/msg/battery_state.hpp> // Standard ROS message for battery status
#include <cmath>              // For std::nanf, std::abs
#include <limits>             // For std::numeric_limits
#include <stdexcept>          // For std::runtime_error in parameter validation

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

    // --- Parameter Declaration ---
    // Declare a parameter for the update/log interval in seconds. Default to 1.0 second.
    this->declare_parameter<double>("update_interval_seconds", 1.0);

    // --- Get Parameter Value ---
    double update_interval_sec = this->get_parameter("update_interval_seconds").as_double();

    // --- Parameter Validation ---
    if (update_interval_sec <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
                    "Invalid 'update_interval_seconds' parameter (%.2f). Must be positive. Using default 1.0s.",
                    update_interval_sec);
        update_interval_sec = 1.0; // Reset to a safe default
    }
    // Convert the interval from seconds (double) to std::chrono::duration
    auto update_interval_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(update_interval_sec)
    );


    // --- BMS Hardware Initialization ---
    if (!bms_.Init())
    {
        // Log a fatal error and request shutdown if BMS initialization fails
        RCLCPP_FATAL(this->get_logger(), "BMS hardware initialization failed! Check port, permissions, and BMS power. Shutting down.");
        // Signal ROS 2 to shutdown.
        if (rclcpp::ok()) {
            rclcpp::shutdown(nullptr, "BMS Initialization Failed"); // Add context to shutdown reason
        }
        // Throw an exception to prevent further initialization if shutdown doesn't happen immediately
        throw std::runtime_error("BMS Hardware Initialization Failed");
        // Note: Returning early might still allow main() to spin if not checked properly.
        // Throwing or immediate shutdown is safer for critical init failures.
    }

    RCLCPP_INFO(this->get_logger(), "BMS hardware initialized successfully.");

    // --- ROS 2 Communication Setup ---

    // Create a publisher for the BatteryState message on the "bms_status" topic
    // QoS setting: Use reliable communication, keep last 10 messages.
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("bms_status", qos_profile);

    // Create a wall timer that triggers the BatteryStatusCallBack function
    // at the specified interval derived from the parameter.
    timer_ = this->create_wall_timer(
        update_interval_duration,
        std::bind(&BatteryStatus::BatteryStatusCallBack, this));

    // Log the actual interval being used.
    RCLCPP_INFO(this->get_logger(), "BMS Status node initialization complete. Publishing and logging data every %.2f s.",
                 update_interval_sec);

    // Logging setup: No need for last_log_time_ anymore, as logging happens every timer callback.
}

/**
 * @brief Callback function triggered by the wall timer at the configured interval.
 * @details Fetches data from BMS, handles errors, publishes BatteryState, and logs summary.
 */
void BatteryStatus::BatteryStatusCallBack()
{
    // --- Data Acquisition ---
    // Call the update function of the BMS_UART class and get the communication status.
    BMS_UART::CommStatus status = bms_.update();

    // Get the current time once for timestamping
    rclcpp::Time now = this->get_clock()->now();

    // --- Handle Communication Status ---
    if (status != BMS_UART::CommStatus::SUCCESS) {
        // Log a warning indicating the failure and the specific status code
        // Rate limit this warning? Maybe not necessary if timer interval is reasonably large.
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
    msg.charge = static_cast<float>(bms_.get.resCapacitymAh) / 1000.0f; // Convert mAh to Ah
    msg.capacity = 46.0f;                         // Full charge capacity (Ah) - TODO: Get from BMS if possible, or make parameter
    msg.design_capacity = 46.0f;                  // Design capacity (Ah) - TODO: Get from BMS if possible, or make parameter
    msg.percentage = bms_.get.packSOC / 100.0f; // Convert SOC from % (0-100) to ratio (0.0-1.0)
    msg.temperature = bms_.get.tempAverage;       // Average battery temperature (Celsius). Verify unit.
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
             if (std::abs(msg.current) < 0.1f && msg.percentage >= 0.98f) { // Near zero current and high SOC -> Full
                  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
             } else if (std::abs(msg.current) < 0.1f) { // Near zero current, not full -> Not Charging
                  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
             } else { // Current is flowing but status is Stationary? Ambiguous case. Default to unknown or keep not_charging?
                 msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING; // Assume not charging if stationary but current != 0
                 RCLCPP_DEBUG(this->get_logger(), "Ambiguous BMS status (Stationary with non-zero current: %.2f A). Reporting as NOT_CHARGING.", msg.current);
             }
            break;
        default: // Unknown status code from BMS
            msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
            break;
    }

    // Populate individual cell voltages safely
    if (bms_.get.numberOfCells >= BMS_UART::MIN_NUMBER_CELLS && bms_.get.numberOfCells <= BMS_UART::MAX_NUMBER_CELLS) {
        msg.cell_voltage.resize(bms_.get.numberOfCells); // Resize vector
        for (int i = 0; i < bms_.get.numberOfCells; ++i) {
            msg.cell_voltage[i] = bms_.get.cellVmV[i] / 1000.0f; // Convert mV to V
        }
    } else {
         if (bms_.get.numberOfCells != 0) { // Avoid logging if 0 cells reported (might be initial state)
            RCLCPP_WARN(this->get_logger(), "Invalid number of cells (%d) reported by BMS, clearing cell_voltage field.", bms_.get.numberOfCells);
         }
         msg.cell_voltage.clear();
    }

    // Determine battery health based on alarm flags from bms_.alarm struct
    // (Keeping the existing detailed health logic)
    if (bms_.alarm.levelTwoCellVoltageTooHigh || bms_.alarm.levelTwoPackVoltageTooHigh ||
        bms_.alarm.levelOneCellVoltageTooHigh || bms_.alarm.levelOnePackVoltageTooHigh) {
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    } else if (bms_.alarm.levelTwoCellVoltageTooLow || bms_.alarm.levelTwoPackVoltageTooLow ||
               bms_.alarm.levelOneCellVoltageTooLow || bms_.alarm.levelOnePackVoltageTooLow ||
               bms_.alarm.failureOfLowVoltageNoCharging) {
        // Consider if DEAD is too strong. Maybe UNSPEC_FAILURE or UNKNOWN if recoverable?
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    } else if (bms_.alarm.levelTwoDischargeTempTooHigh || bms_.alarm.levelTwoChargeTempTooHigh ||
               bms_.alarm.levelOneDischargeTempTooHigh || bms_.alarm.levelOneChargeTempTooHigh ||
               bms_.alarm.chargeFETTemperatureTooHigh || bms_.alarm.dischargeFETTemperatureTooHigh) {
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
    } else if (bms_.alarm.levelTwoDischargeTempTooLow || bms_.alarm.levelTwoChargeTempTooLow ||
               bms_.alarm.levelOneDischargeTempTooLow || bms_.alarm.levelOneChargeTempTooLow) {
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_COLD;
    } else if (bms_.alarm.levelTwoDischargeCurrentTooHigh || bms_.alarm.levelTwoChargeCurrentTooHigh ||
               bms_.alarm.levelOneDischargeCurrentTooHigh || bms_.alarm.levelOneChargeCurrentTooHigh) {
        // No specific OVERCURRENT health state. Using UNSPEC_FAILURE might be better than OVERVOLTAGE.
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
         RCLCPP_WARN_ONCE(this->get_logger(), "BMS reports overcurrent condition, mapping to UNSPEC_FAILURE health state.");
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

    // --- Logging (runs every time the callback is executed successfully) ---
    // Prepare the log string using an output string stream
    std::ostringstream log_bms;
    log_bms << "\n--- BMS Status Summary (SUCCESS) ---"
            << "\n[ Timestamp ]     : " << now.seconds() // Log timestamp for context
            << "\n[ Charge Status ] : " << bms_.get.chargeDischargeStatus << " (" << getStatusString(msg.power_supply_status) << ")"
            << "\n[ Voltage ]       : " << msg.voltage << " V"
            << "\n[ Current ]       : " << msg.current << " A"
            << "\n[ State of Charge ] : " << msg.percentage * 100.0 << " %"
            << "\n[ Temperature (Avg) ] : " << msg.temperature << " C"
            << "\n[ Remaining Capacity ]: " << msg.charge << " Ah"
            << "\n[ BMS Heartbeat ] : " << bms_.get.bmsHeartBeat
            << "\n[ Cycle Count ]   : " << bms_.get.bmsCycles
            << "\n[ Health Status ] : " << getHealthString(msg.power_supply_health) << " (" << static_cast<int>(msg.power_supply_health) << ")"; // Log health string and code

    // Log the formatted string using RCLCPP_INFO level
    RCLCPP_INFO(this->get_logger(), "%s", log_bms.str().c_str());

    // No need to update last_log_time_ anymore
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

/**
 * @brief Helper function to convert power supply health enum to string for logging.
 */
std::string BatteryStatus::getHealthString(uint8_t health) {
     switch (health) {
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD:            return "Good";
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT:        return "Overheat";
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD:            return "Dead/Deep Discharge";
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE:     return "Overvoltage";
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE:  return "Unspecified Failure/Overcurrent";
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_COLD:            return "Cold";
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: return "Watchdog Expired"; // If applicable
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: return "Safety Timer Expired"; // If applicable
        case sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN:
        default:                                                                  return "Unknown";
    }
}


// --- Main Function ---
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<BatteryStatus> node = nullptr;
    try {
        node = std::make_shared<BatteryStatus>();

        // Initialization check: If node constructor requested shutdown or threw, rclcpp::ok() might be false
        if (rclcpp::ok()) {
           RCLCPP_INFO(node->get_logger(), "Starting BMS Status node spin loop.");
           rclcpp::spin(node);
        } else {
           // Node likely failed initialization, logging handled inside constructor/exception
           std::cerr << "ROS 2 shutdown requested or node initialization failed before spin." << std::endl;
        }

    } // --- CATCH BLOCK ORDERING FIX ---
    // Catch the more specific exception type FIRST
    catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
        std::cerr << "Invalid parameter value exception: " << e.what() << std::endl;
        if (rclcpp::ok()) {
            rclcpp::shutdown(nullptr, "Invalid Parameter");
        }
        return 1;
    }
    // Catch the base exception type AFTER its derived types
    catch (const std::runtime_error & e) {
        // Catch initialization error specifically
        std::cerr << "Runtime error during BMS node execution: " << e.what() << std::endl; // Message changed slightly for clarity
        // Ensure shutdown is called
        if (rclcpp::ok()) {
            rclcpp::shutdown(nullptr, "Runtime Error"); // Updated reason
        }
        return 1; // Indicate error exit
    }
    // Catch other standard exceptions
    catch (const std::exception & e) {
        std::cerr << "Unhandled standard exception in BMS node: " << e.what() << std::endl; // Message changed slightly
        if (rclcpp::ok()) {
            rclcpp::shutdown(nullptr, "Unhandled Standard Exception"); // Updated reason
        }
        return 1;
    }
    // Catch any other unknown exceptions (should be last)
    catch (...) {
        std::cerr << "Unknown exception in BMS node." << std::endl;
        if (rclcpp::ok()) {
        rclcpp::shutdown(nullptr, "Unknown Exception");
    }
    return 1;
    }

    // Shutdown is likely already called on error or external signal, but call again ensures cleanup.
    rclcpp::shutdown();

    std::cout << "BMS Status node finished." << std::endl;
    return 0;
}