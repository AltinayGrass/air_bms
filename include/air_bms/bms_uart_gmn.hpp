#ifndef BMS_UART_HPP
#define BMS_UART_HPP

#include <string>
#include <stdint.h> // For uint8_t, uint16_t, uint32_t
#include <chrono>   // For std::chrono time points and durations
#include <unistd.h>
#include <fcntl.h>
#include <termio.h>
#include <sys/select.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>
#include <vector> 
#include <iostream>

// Forward declaration for termios to avoid including the full header here
// struct termios; // Alternatively, include <termios.h> if needed directly in header (unlikely)

/**
 * @class BMS_UART
 * @brief Handles serial communication (UART) with a specific type of Battery Management System (BMS).
 *
 * This class encapsulates the logic for sending commands, receiving responses (including multi-frame messages),
 * parsing data, validating checksums, and managing communication status and timeouts.
 */
class BMS_UART
{
public:
    // --- Constants ---
    static constexpr int XFER_BUFFER_LENGTH = 13;      // Standard length of a single command/response frame
    static constexpr int MIN_NUMBER_CELLS = 1;         // Minimum expected cell count
    static constexpr int MAX_NUMBER_CELLS = 48;        // Maximum supported cell count (adjust based on BMS/buffer)
    static constexpr int MIN_NUMBER_TEMP_SENSORS = 1;  // Minimum expected temperature sensor count
    static constexpr int MAX_NUMBER_TEMP_SENSORS = 16; // Maximum supported temperature sensor count
    // RX Buffer size needs to accommodate the largest possible multi-frame response
    // Max cells = 48, 3 cells/frame -> ceil(48/3) = 16 frames. 16 * 13 = 208 bytes.
    // Max temps = 16, 7 temps/frame -> ceil(16/7) = 3 frames. 3 * 13 = 39 bytes.
    // Use a buffer size slightly larger than the maximum needed (208 bytes).
    static constexpr int RX_BUFFER_MAX_SIZE = 256;

    /**
     * @enum CommStatus
     * @brief Defines the possible outcomes of communication operations.
     */
    enum class CommStatus
    {
        SUCCESS,                    // Operation completed successfully
        ERROR_PORT_CLOSED,          // Serial port is not open or failed during operation
        ERROR_READ_TIMEOUT,         // Timeout occurred while waiting for data from BMS
        ERROR_READ_MISMATCH,        // Received fewer or more bytes than expected (generic read error)
        ERROR_CHECKSUM,             // Received data failed checksum validation
        ERROR_WRITE_FAILED,         // Failed to write command data to the serial port
        ERROR_NO_RESPONSE,          // No successful communication for the overall timeout period
        ERROR_PRECONDITION_FAIL,    // A necessary condition (e.g., valid cell count) was not met for the operation
        ERROR_INTERNAL_BUFFER_SMALL,// The internal RX buffer is too small for the expected response
        ERROR_FRAMING,              // Invalid start byte detected in the response frame
        ERROR_CMD_MISMATCH,         // Command byte in the response frame did not match the sent command
        ERROR_FRAME_SEQUENCE,       // Multi-frame response received frames out of order
        ERROR_PARSE_FAIL            // Failed to parse the received data payload meaningfully
    };

    /**
     * @enum COMMAND
     * @brief Defines the command codes to interact with the BMS.
     */
    enum COMMAND : uint8_t // Explicitly type as uint8_t
    {
        VOUT_IOUT_SOC = 0x90,              // Read Pack Voltage, Current, SOC
        MIN_MAX_CELL_VOLTAGE = 0x91,       // Read Min/Max Cell Voltage and Cell Numbers
        MIN_MAX_TEMPERATURE = 0x92,        // Read Min/Max Temperature Sensor Readings
        DISCHARGE_CHARGE_MOS_STATUS = 0x93,// Read FET Status, Battery Status, Heartbeat, Remaining Capacity
        STATUS_INFO = 0x94,                // Read Cell Count, Temp Sensor Count, Charger/Load Status, Cycles
        CELL_VOLTAGES = 0x95,              // Read Individual Cell Voltages (Multi-frame possible)
        CELL_TEMPERATURE = 0x96,           // Read Individual Temperatures (Multi-frame possible)
        CELL_BALANCE_STATE = 0x97,         // Read Cell Balancing Status
        FAILURE_CODES = 0x98,              // Read Alarm/Failure Flags
        DISCHRG_FET = 0xD9,                // Control Discharge FET (Requires payload: 0x00=OFF, 0x01=ON)
        CHRG_FET = 0xDA,                   // Control Charge FET (Requires payload: 0x00=OFF, 0x01=ON)
        BMS_RESET = 0x00                   // Reset the BMS
    };

    /**
     * @struct DataGetters
     * @brief Holds all the data read from the BMS, populated by the update() method.
     *        (Renamed from 'get' to avoid potential keyword conflicts/confusion).
     */
    struct DataGetters // Renamed from 'get'
    {
        // Data from 0x90
        float packVoltage = 0.0f;    // Pack Voltage (V)
        float packCurrent = 0.0f;    // Pack Current (A, positive=charging, negative=discharging)
        float packSOC = 0.0f;        // State Of Charge (%)

        // Data from 0x91
        float maxCellmV = 0.0f;      // Maximum cell voltage (mV)
        int maxCellVNum = 0;         // Cell number with maximum voltage
        float minCellmV = 0.0f;      // Minimum cell voltage (mV)
        int minCellVNum = 0;         // Cell number with minimum voltage
        float cellDiff = 0.0f;       // Difference between max and min cell voltage (mV)

        // Data from 0x92
        int tempMax = -99;           // Maximum temperature reading (°C)
        int tempMin = -99;           // Minimum temperature reading (°C)
        float tempAverage = -99.0f;  // Average temperature (°C)

        // Data from 0x93
        std::string chargeDischargeStatus = "Unknown"; // Charge/Discharge status string ("Stationary", "Charge", "Discharge")
        uint8_t batteryStatus = 0;   // Raw battery status code (0=Stationary, 1=Charge, 2=Discharge)
        bool chargeFetState = false; // Charging FET state (true=On, false=Off)
        bool disChargeFetState = false;// Discharging FET state (true=On, false=Off)
        int bmsHeartBeat = 0;        // BMS life counter/heartbeat (0-255)
        int resCapacitymAh = 0;      // Remaining capacity (mAh)

        // Data from 0x94
        int numberOfCells = 0;       // Number of battery cells detected
        int numOfTempSensors = 0;    // Number of temperature sensors detected
        bool chargeState = false;    // Charger connection status (true=connected, false=disconnected)
        bool loadState = false;      // Load connection status (true=connected, false=disconnected)
        bool dIO[8] = {false};       // State of 8 Digital Inputs/Outputs (interpret based on BMS docs)
        int bmsCycles = 0;           // Charge/Discharge cycle count

        // Data from 0x95 (Multi-frame)
        float cellVmV[MAX_NUMBER_CELLS] = {0.0f}; // Array to store individual cell voltages (mV)

        // Data from 0x96 (Multi-frame)
        int cellTemperature[MAX_NUMBER_TEMP_SENSORS] = {-99}; // Array to store individual temperatures (°C)

        // Data from 0x97
        bool cellBalanceState[MAX_NUMBER_CELLS] = {false}; // Array of cell balancing states (true=balancing)
        bool cellBalanceActive = false; // Overall flag indicating if any cell is currently balancing

        // debug data string (kept from original, usage unclear)
        // std::string aDebug; // Consider removing if unused
    } get; // Instance of the data struct

    /**
     * @struct AlarmFlags
     * @brief Holds boolean flags corresponding to BMS alarms/failure codes.
     *        Populated by the update() method when reading FAILURE_CODES (0x98).
     *        (Renamed from 'alarm' for clarity).
     */
    struct AlarmFlags // Renamed from 'alarm'
    {
        // Data from 0x98 (Byte indices based on BMS protocol for this command)
        /* Byte 4: Voltage Alarms */
        bool levelOneCellVoltageTooHigh = false;
        bool levelTwoCellVoltageTooHigh = false;
        bool levelOneCellVoltageTooLow = false;
        bool levelTwoCellVoltageTooLow = false;
        bool levelOnePackVoltageTooHigh = false;
        bool levelTwoPackVoltageTooHigh = false;
        bool levelOnePackVoltageTooLow = false;
        bool levelTwoPackVoltageTooLow = false;

        /* Byte 5: Temperature Alarms */
        bool levelOneChargeTempTooHigh = false;
        bool levelTwoChargeTempTooHigh = false;
        bool levelOneChargeTempTooLow = false;
        bool levelTwoChargeTempTooLow = false;
        bool levelOneDischargeTempTooHigh = false;
        bool levelTwoDischargeTempTooHigh = false;
        bool levelOneDischargeTempTooLow = false;
        bool levelTwoDischargeTempTooLow = false;

        /* Byte 6: Current / SOC Alarms */
        bool levelOneChargeCurrentTooHigh = false;
        bool levelTwoChargeCurrentTooHigh = false;
        bool levelOneDischargeCurrentTooHigh = false;
        bool levelTwoDischargeCurrentTooHigh = false;
        bool levelOneStateOfChargeTooHigh = false;
        bool levelTwoStateOfChargeTooHigh = false;
        bool levelOneStateOfChargeTooLow = false;
        bool levelTwoStateOfChargeTooLow = false;

        /* Byte 7: Difference Alarms */
        bool levelOneCellVoltageDifferenceTooHigh = false;
        bool levelTwoCellVoltageDifferenceTooHigh = false;
        bool levelOneTempSensorDifferenceTooHigh = false; // Corrected name
        bool levelTwoTempSensorDifferenceTooHigh = false; // Corrected name

        /* Byte 8: FET / Sensor Failures */
        bool chargeFETTemperatureTooHigh = false;
        bool dischargeFETTemperatureTooHigh = false;
        bool failureOfChargeFETTemperatureSensor = false;
        bool failureOfDischargeFETTemperatureSensor = false;
        bool failureOfChargeFETAdhesion = false;
        bool failureOfDischargeFETAdhesion = false;
        bool failureOfChargeFETTBreaker = false; // Corrected name
        bool failureOfDischargeFETBreaker = false; // Corrected name

        /* Byte 9: Internal Module Failures */
        bool failureOfAFEAcquisitionModule = false;
        bool failureOfVoltageSensorModule = false;
        bool failureOfTemperatureSensorModule = false;
        bool failureOfEEPROMStorageModule = false;
        bool failureOfRealtimeClockModule = false;
        bool failureOfPrechargeModule = false;
        bool failureOfVehicleCommunicationModule = false;
        bool failureOfIntranetCommunicationModule = false;

        /* Byte 10: Other Failures */
        bool failureOfCurrentSensorModule = false;
        bool failureOfMainVoltageSensorModule = false;
        bool failureOfShortCircuitProtection = false;
        bool failureOfLowVoltageNoCharging = false;
    } alarm; // Instance of the alarm struct


    // --- Constructor & Destructor ---

    /**
     * @brief Construct a new BMS_UART object and attempt to open the serial port.
     * @param serialDev The path to the serial device (e.g., "/dev/ttyUSB0").
     */
    explicit BMS_UART(const std::string& serialDev); // Mark explicit

    /**
     * @brief Destroy the BMS_UART object and ensure the serial port is closed.
     */
    ~BMS_UART();

    // --- Core Methods ---

    /**
     * @brief Initializes the serial port communication settings (baud, parity, etc.).
     * @details Must be called after construction and before any other communication attempts.
     * @return True if initialization is successful, false otherwise.
     */
    bool Init();

    /**
     * @brief Updates all BMS data by querying the device for each relevant piece of information.
     * @details Populates the 'get' and 'alarm' structs. Handles multi-frame responses internally.
     * @return CommStatus indicating the result of the update sequence (SUCCESS or the first error encountered).
     */
    CommStatus update();

    /**
     * @brief Gets the current overall communication status.
     * @details Checks port status and overall communication timeout.
     * @return CommStatus The current assessed status.
     */
    CommStatus getCommunicationStatus() const;

    // --- Control Methods ---

    /**
     * @brief Set the state of the Discharge MOSFET.
     * @param sw True to turn ON, False to turn OFF.
     * @return True on successful command transmission and acknowledgment, false otherwise.
     */
    bool setDischargeMOS(bool sw);

    /**
     * @brief Set the state of the Charge MOSFET.
     * @param sw True to turn ON, False to turn OFF.
     * @return True on successful command transmission and acknowledgment, false otherwise.
     */
    bool setChargeMOS(bool sw);

    /**
     * @brief Send the BMS reset command.
     * @details BMS may become unresponsive temporarily after reset.
     * @return True on successful command transmission and acknowledgment, false otherwise.
     */
    bool setBmsReset();

    // --- Deleted Methods ---
    // Prevent copying and assignment
    BMS_UART(const BMS_UART&) = delete;
    BMS_UART& operator=(const BMS_UART&) = delete;

private:
    // --- Private Helper Methods ---

    /**
     * @brief Sends a command, receives the full response (handling multi-frame), validates, and parses it.
     * @param cmdID The command to send.
     * @return CommStatus Indicating success or the specific error encountered.
     */
    CommStatus sendCommandAndReceive(COMMAND cmdID);

    /**
     * @brief Prepares and sends a command frame to the BMS via the serial port.
     * @param cmdID The command to send.
     * @return True if the write operation was successful, false otherwise. Updates internal status on failure.
     */
    bool sendCommand(COMMAND cmdID);

    /**
     * @brief Receives the expected number of bytes for a given command response.
     * @details Handles timeouts, multi-frame assembly, and basic frame validation (start byte, cmd echo, sequence).
     *          Calls checksum validation for each frame upon completion.
     * @param cmdID The command for which a response is expected.
     * @return CommStatus Indicating success or the specific error encountered during reception.
     */
    CommStatus receiveBytes(COMMAND cmdID);

    /**
     * @brief Validates the checksum of a single received frame.
     * @param frame_buffer Pointer to the start of the frame data.
     * @param length The length of the frame (should be XFER_BUFFER_LENGTH).
     * @return true if the checksum is correct, false otherwise.
     */
    bool validateChecksumFrame(const uint8_t* frame_buffer, size_t length);

    /**
     * @brief Prints the contents of the RX buffer to std::cout for debugging purposes.
     * @param length The number of bytes currently in the buffer to print.
     */
    void barfRXBuffer(size_t length);

    /**
     * @brief Checks if the serial port file descriptor is valid (port is considered open).
     * @return true if the port is open, false otherwise.
     */
    bool isPortOpen() const;

    /**
     * @brief Updates the timestamp of the last successful communication and resets status if needed.
     */
    void updateLastSuccessTime();


    // --- Member Variables ---

    int my_serialIntf; // File descriptor for the serial port connection (-1 if not open)

    // Transmit buffer (holds the command packet to be sent)
    uint8_t my_txBuffer[XFER_BUFFER_LENGTH];

    // Receive buffer (needs to be large enough for multi-frame responses)
    uint8_t my_rxBuffer[RX_BUFFER_MAX_SIZE];

    // --- Timeout and Status Tracking ---
    std::chrono::steady_clock::time_point last_success_time_; // Time of last successful read/checksum validation
    CommStatus current_status_; // Stores the current known communication status
    bool port_ever_opened_;     // Flag to track if Init() was ever called successfully

    // Configuration for timeouts (can be adjusted)
    static constexpr std::chrono::milliseconds READ_TIMEOUT_MS{2000L}; // Base timeout for a single receive operation (might be extended for multi-frame)
    static constexpr std::chrono::seconds OVERALL_TIMEOUT_S{10L};       // Overall timeout if no successful communication occurs
};

#endif //BMS_UART_HPP