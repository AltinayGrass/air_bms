/**
 * @file bms_uart.cpp
 * @brief Implementation file for the BMS_UART class, handling serial communication
 *        with the Battery Management System (BMS). Includes multi-frame message handling.
 */

 #include "air_bms/bms_uart_gmn.hpp" // Corresponding header file

 #include <thread>    // For sleep_for
 #include <chrono>
 
 // Use chrono literals (e.g., 20ms, 2s)
 using namespace std::chrono_literals;
 
 //----------------------------------------------------------------------
 // Helper Functions (Internal)
 //----------------------------------------------------------------------
 
 /**
  * @brief Reads the value of a specific bit within an integer.
  * @param x The integer containing the bits.
  * @param bit The bit position to read (0-indexed).
  * @return uint8_t The value of the bit (0 or 1).
  */
 inline uint8_t bitRead(uint32_t x, uint32_t bit) {
     return (x >> bit) & 1;
 }
 
 //----------------------------------------------------------------------
 // Constructor & Destructor
 //----------------------------------------------------------------------
 
 /**
  * @brief Construct a new BMS_UART object and attempt to open the serial port.
  * @param serialDev The path to the serial device (e.g., "/dev/ttyUSB0").
  */
 BMS_UART::BMS_UART(const std::string& serialDev) :
     my_serialIntf(-1), // Initialize file descriptor to invalid state
     current_status_(CommStatus::ERROR_PORT_CLOSED),
     port_ever_opened_(false)
 {
     // Attempt to open the serial port in read/write mode, without becoming controlling terminal,
     // and initially non-blocking (O_NDELAY) which we'll clear after opening.
     this->my_serialIntf = open( serialDev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
 
     if (this->my_serialIntf < 0) {
         // Log error if opening failed
         std::cerr << "BMS_UART Error " << errno << " opening " << serialDev << ": " << strerror(errno) << std::endl;
         // Status remains ERROR_PORT_CLOSED
     } else {
         // Clear the non-blocking flag (O_NDELAY) so subsequent reads can block with timeouts
         if (fcntl(this->my_serialIntf, F_SETFL, 0) < 0) {
              std::cerr << "BMS_UART Error clearing O_NDELAY for " << serialDev << ": " << strerror(errno) << std::endl;
              // Close the port if we can't set flags properly
              close(my_serialIntf);
              my_serialIntf = -1;
              // Status remains ERROR_PORT_CLOSED
         } else {
             std::cout << "BMS_UART: Successfully opened " << serialDev << std::endl;
             // Port is open, but not yet configured. Status only becomes SUCCESS after successful Init().
         }
     }
 
     // Initialize last success time reasonably far in the past so first check doesn't immediately timeout
     last_success_time_ = std::chrono::steady_clock::now() - OVERALL_TIMEOUT_S * 2;
 }
 
 /**
  * @brief Destroy the BMS_UART object and ensure the serial port is closed.
  */
 BMS_UART::~BMS_UART()
 {
     if (isPortOpen()) {
         std::cout << "BMS_UART: Closing serial port." << std::endl;
         close(my_serialIntf);
         my_serialIntf = -1;
     }
 }
 
 //----------------------------------------------------------------------
 // Public Member Functions
 //----------------------------------------------------------------------
 
 /**
  * @brief Initializes the serial port communication settings.
  * @details Configures baud rate, data bits, parity, stop bits, and flow control.
  *          Pre-populates the transmit buffer with command-independent bytes.
  * @return True if initialization is successful, false otherwise.
  */
 bool BMS_UART::Init() {
     if (!isPortOpen()) {
         current_status_ = CommStatus::ERROR_PORT_CLOSED;
         port_ever_opened_ = false;
         std::cerr << "BMS_UART Error: Init() called but port is not open." << std::endl;
         return false;
     }
 
     // Structure to hold serial port settings
     struct termios tty;
     memset(&tty, 0, sizeof tty); // Clear the struct for safety
 
     // Get current attributes of the serial port
     if (tcgetattr(this->my_serialIntf, &tty) != 0) {
         std::cerr << "BMS_UART Error: tcgetattr failed: " << strerror(errno) << std::endl;
         current_status_ = CommStatus::ERROR_PORT_CLOSED; // Mark as config failed
         port_ever_opened_ = false; // Can't consider it properly opened if config fails
         return false;
     }
 
     // --- Configure Serial Port Settings ---
     // Set Baud Rate (Input and Output) to 9600

     // Control Flags (c_cflag)
     tty.c_cflag &= ~CSIZE;       // Clear current size bits
     tty.c_cflag |= CS8;          // 8 data bits
     tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
     tty.c_cflag &= ~(PARENB | PARODD); // Disable parity
     tty.c_cflag &= ~CSTOPB;      // 1 stop bit
     tty.c_cflag &= ~CRTSCTS;     // Disable hardware flow control
 
     // Input Flags (c_iflag) - Aim for raw input
     tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
     tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable special handling of received bytes
 
     // Local Flags (c_lflag) - Non-canonical mode, no echo, no signals
     tty.c_lflag = 0;
 
     // Output Flags (c_oflag) - Raw output
     tty.c_oflag = 0;
 
     // Control Characters (c_cc) - Timeout settings for read()
     // VMIN = 0, VTIME > 0: Timeout read. read() waits for VTIME deciseconds (0.1s units).
     // It returns either when at least one byte is received, or when the timer expires.
     tty.c_cc[VMIN] = 0;
     tty.c_cc[VTIME] = 5; // Wait up to 0.5 seconds for *any* data in a single read() call. Adjust if needed.
     
     cfsetispeed(&tty, B9600);
     cfsetospeed(&tty, B9600);
     cfmakeraw(&tty);

     // Apply the new settings
     if (tcsetattr(this->my_serialIntf, TCSANOW, &tty) != 0) {
         std::cerr << "BMS_UART Error: tcsetattr failed: " << strerror(errno) << std::endl;
         current_status_ = CommStatus::ERROR_PORT_CLOSED;
         port_ever_opened_ = false;
         return false;
     }
 
     // Flush any pending data in the input/output buffers
     if (tcflush(my_serialIntf, TCIOFLUSH) != 0) {
          std::cerr << "BMS_UART Warning: tcflush failed: " << strerror(errno) << std::endl;
          // Non-fatal, but might indicate an issue.
     }
 
     // --- Pre-populate Transmit Buffer ---
     this->my_txBuffer[0] = 0xA5; // Start byte
     this->my_txBuffer[1] = 0x40; // Host address (assuming fixed)
     this->my_txBuffer[3] = 0x08; // Default payload length for most read commands
     // Clear default payload section (bytes 4-11)
     for (uint8_t i = 4; i < 12; i++) {
         this->my_txBuffer[i] = 0x00;
     }
 
     std::cout << "BMS_UART: Serial port initialized successfully." << std::endl;
     current_status_ = CommStatus::SUCCESS; // Mark status as good after successful Init
     port_ever_opened_ = true;
     updateLastSuccessTime(); // Mark communication as potentially possible now
     return true;
 }
 
 /**
  * @brief Updates all BMS data by querying the device for each relevant piece of information.
  * @details Iterates through a sequence of read commands. Stops and returns the error status
  *          if any command fails.
  * @return CommStatus indicating the result of the update sequence (SUCCESS or the first error encountered).
  */
 BMS_UART::CommStatus BMS_UART::update()
 {
     // Check overall communication health first (port open, no overall timeout)
     CommStatus overall_status = getCommunicationStatus();
     if (overall_status != CommStatus::SUCCESS) {
         // If already in a persistent error state (timeout or port closed), return it immediately
         if (overall_status == CommStatus::ERROR_NO_RESPONSE || overall_status == CommStatus::ERROR_PORT_CLOSED) {
              return overall_status;
         }
         // For other transient errors (like checksum failure from last cycle), log but try updating again.
         std::cerr << "BMS_UART Warning: Starting update despite previous transient error: "
                   << static_cast<int>(overall_status) << std::endl;
     }
 
     // Ensure port is still considered open before proceeding
     if (!isPortOpen()) {
          current_status_ = CommStatus::ERROR_PORT_CLOSED;
          return current_status_;
     }
 
     CommStatus result;
 
     // Define the sequence of commands needed to update all 'get' struct members
     // Note: Order might matter if one command provides data needed for another (e.g., cell count)
     const std::vector<COMMAND> update_sequence = {
         COMMAND::VOUT_IOUT_SOC,              // 0x90
         COMMAND::MIN_MAX_CELL_VOLTAGE,       // 0x91
         COMMAND::MIN_MAX_TEMPERATURE,        // 0x92
         COMMAND::DISCHARGE_CHARGE_MOS_STATUS,// 0x93
         COMMAND::STATUS_INFO,                // 0x94 (provides numberOfCells, numOfTempSensors needed for others)
         COMMAND::CELL_VOLTAGES,              // 0x95 (Multi-frame possible)
         COMMAND::CELL_TEMPERATURE,           // 0x96 (Multi-frame possible)
         COMMAND::CELL_BALANCE_STATE,         // 0x97
         COMMAND::FAILURE_CODES               // 0x98
     };
 
     // Execute each command in the sequence
     for (const auto& cmd : update_sequence) {
         result = sendCommandAndReceive(cmd); // Use the helper function
         if (result != CommStatus::SUCCESS) {
             // If any command fails, update the internal status, log the error, and stop the update sequence
             current_status_ = result;
             std::cerr << "BMS_UART: Update failed on command 0x" << std::hex << static_cast<int>(cmd)
                       << ". Error: " << static_cast<int>(result) << std::dec << std::endl;
             return current_status_;
         }
         // Small delay between commands can sometimes help BMS devices process
         std::this_thread::sleep_for(20ms); // Adjust delay if needed (e.g., 10ms, 50ms) or remove if unnecessary
     }
 
     // If all commands in the sequence succeeded
     current_status_ = CommStatus::SUCCESS; // Ensure status is explicitly SUCCESS
     // last_success_time_ is updated within receiveBytes on successful read/checksum
     return CommStatus::SUCCESS;
 }
 
 
 /**
  * @brief Gets the current overall communication status.
  * @details Checks if the port was ever opened, if it's currently closed, or if the
  *          overall communication timeout has been exceeded since the last successful read.
  * @return CommStatus The current status.
  */
 BMS_UART::CommStatus BMS_UART::getCommunicationStatus() const {
     // If the port was never successfully initialized, it's effectively closed/unusable.
     if (!port_ever_opened_) {
         return CommStatus::ERROR_PORT_CLOSED;
     }
 
     // If the status is already reflecting a closed port, return that.
     if (current_status_ == CommStatus::ERROR_PORT_CLOSED) {
         return CommStatus::ERROR_PORT_CLOSED;
     }
 
     // Check for overall communication timeout only if the port seems okay otherwise.
     auto now = std::chrono::steady_clock::now();
     if ((now - last_success_time_) > OVERALL_TIMEOUT_S) {
         // If the timeout is exceeded *and* the current status was previously SUCCESS,
         // report the timeout error. Don't overwrite other existing error states.
         if (current_status_ == CommStatus::SUCCESS) {
             std::cerr << "BMS_UART: Overall communication timeout (> " << OVERALL_TIMEOUT_S.count() << "s)." << std::endl;
             // We return the timeout status but don't permanently change 'current_status_' here.
             // Let the next actual read attempt confirm if communication is truly lost.
             return CommStatus::ERROR_NO_RESPONSE;
         } else {
              // If already in an error state (e.g., checksum fail), keep that state but timeout has also occurred.
              // Return the existing error status for consistency.
               return current_status_;
         }
     }
 
     // If no timeout and port seems open, return the last known status from read/write operations.
     return current_status_;
 }
 
 
 /**
  * @brief Set the state of the Discharge MOSFET.
  * @param sw True to turn ON, False to turn OFF.
  * @return True on successful command transmission and acknowledgment, false otherwise.
  */
 bool BMS_UART::setDischargeMOS(bool sw) // Command 0xD9
 {
     std::cout << "BMS_UART: Attempting to switch discharge MOSFET " << (sw ? "ON" : "OFF") << std::endl;
     // Set payload byte 4 based on desired state *before* calling send command
     this->my_txBuffer[4] = sw ? 0x01 : 0x00;
 
     // Use the combined function to send command and receive+validate response
     CommStatus result = sendCommandAndReceive(COMMAND::DISCHRG_FET);
 
     // Clear the payload byte in the buffer after use (good practice)
     this->my_txBuffer[4] = 0x00;
 
     if (result != CommStatus::SUCCESS) {
          std::cerr << "BMS_UART Error: Failed to set Discharge MOS. Status: " << static_cast<int>(result) << std::endl;
          return false;
     }
 
     // Optional: Verify the response payload byte (e.g., my_rxBuffer[4]) confirms the state if BMS protocol supports it.
 
     std::cout << "BMS_UART: Set Discharge MOS command " << (sw ? "ON" : "OFF") << " acknowledged successfully." << std::endl;
     return true;
 }
 
 /**
  * @brief Set the state of the Charge MOSFET.
  * @param sw True to turn ON, False to turn OFF.
  * @return True on successful command transmission and acknowledgment, false otherwise.
  */
 bool BMS_UART::setChargeMOS(bool sw) // Command 0xDA
 {
     std::cout << "BMS_UART: Attempting to switch charge MOSFET " << (sw ? "ON" : "OFF") << std::endl;
     this->my_txBuffer[4] = sw ? 0x01 : 0x00; // Set payload byte 4
 
     CommStatus result = sendCommandAndReceive(COMMAND::CHRG_FET);
 
     this->my_txBuffer[4] = 0x00; // Clear payload byte
 
     if (result != CommStatus::SUCCESS) {
          std::cerr << "BMS_UART Error: Failed to set Charge MOS. Status: " << static_cast<int>(result) << std::endl;
          return false;
     }
     std::cout << "BMS_UART: Set Charge MOS command " << (sw ? "ON" : "OFF") << " acknowledged successfully." << std::endl;
     return true;
 }
 
 /**
  * @brief Send the BMS reset command.
  * @return True on successful command transmission and acknowledgment, false otherwise.
  */
 bool BMS_UART::setBmsReset() // Command 0x00
 {
     std::cout << "BMS_UART: Attempting to reset the BMS." << std::endl;
     // Reset command might not need specific payload; sendCommandAndReceive uses default 0 payload.
     CommStatus result = sendCommandAndReceive(COMMAND::BMS_RESET);
 
     if (result != CommStatus::SUCCESS) {
          std::cerr << "BMS_UART Error: Failed to send BMS Reset. Status: " << static_cast<int>(result) << std::endl;
          return false;
     }
     std::cout << "BMS_UART: BMS Reset command acknowledged successfully." << std::endl;
     // Note: BMS will likely be unresponsive for a short period after reset.
     // The calling code might need to handle this (e.g., add a delay, re-check status).
     return true;
 }
 
 
 //----------------------------------------------------------------------
 // Private Member Functions
 //----------------------------------------------------------------------
 
 /**
  * @brief Checks if the serial port file descriptor is valid (>= 0).
  * @return true if the port appears open, false otherwise.
  */
 bool BMS_UART::isPortOpen() const {
     return my_serialIntf >= 0;
 }
 
 /**
  * @brief Updates the timestamp of the last successful communication.
  * @details Also resets the `current_status_` to SUCCESS if it was previously an error,
  *          indicating that communication has been re-established.
  */
 void BMS_UART::updateLastSuccessTime() {
     last_success_time_ = std::chrono::steady_clock::now();
     // If status was previously an error, log that communication is back
     if (current_status_ != CommStatus::SUCCESS) {
          std::cout << "BMS_UART: Communication re-established." << std::endl;
          current_status_ = CommStatus::SUCCESS; // Reset status on success
     }
 }
 
 /**
  * @brief Helper function to send a command, receive the response, validate it, and parse the data.
  * @param cmdID The COMMAND enum value to send.
  * @return CommStatus Indicating success or the specific error encountered during send, receive, or parsing.
  */
 BMS_UART::CommStatus BMS_UART::sendCommandAndReceive(COMMAND cmdID) {
     // 1. Send the command
     if (!sendCommand(cmdID)) {
         // sendCommand updates current_status_ on failure
         return current_status_;
     }
 
     // 2. Receive the response (handles multi-frame, timeouts, checksums)
     CommStatus rx_status = receiveBytes(cmdID);
     if (rx_status != CommStatus::SUCCESS) {
         current_status_ = rx_status; // Update status with the receive error
         return current_status_;
     }
 
     // 3. Parse the received data (only if receive was successful)
     bool parse_success = true;
     switch (cmdID) {
         case COMMAND::VOUT_IOUT_SOC: // 0x90
             // Bytes 4,5: Voltage (0.1V)
             // Bytes 8,9: Current (0.1A, 30000 offset)
             // Bytes 10,11: SOC (0.1%)
             get.packVoltage = static_cast<float>((this->my_rxBuffer[4] << 8) | this->my_rxBuffer[5]) / 10.0f;
             get.packCurrent = static_cast<float>((int16_t)((this->my_rxBuffer[8] << 8) | this->my_rxBuffer[9]) - 30000) / 10.0f; // Use signed int16
             get.packSOC = static_cast<float>((this->my_rxBuffer[10] << 8) | this->my_rxBuffer[11]) / 10.0f;
             break;
 
         case COMMAND::MIN_MAX_CELL_VOLTAGE: // 0x91
             // Bytes 4,5: Max Cell Voltage (mV)
             // Byte 6: Max Cell Number
             // Bytes 7,8: Min Cell Voltage (mV)
             // Byte 9: Min Cell Number
             get.maxCellmV = static_cast<float>((this->my_rxBuffer[4] << 8) | this->my_rxBuffer[5]);
             get.maxCellVNum = this->my_rxBuffer[6];
             get.minCellmV = static_cast<float>((this->my_rxBuffer[7] << 8) | this->my_rxBuffer[8]);
             get.minCellVNum = this->my_rxBuffer[9];
             get.cellDiff = (get.maxCellmV - get.minCellmV);
             break;
 
         case COMMAND::MIN_MAX_TEMPERATURE: // 0x92
             // Byte 4: Max Temperature (Offset 40 C)
             // Byte 6: Min Temperature (Offset 40 C)
             get.tempMax = static_cast<int>(this->my_rxBuffer[4]) - 40;
             get.tempMin = static_cast<int>(this->my_rxBuffer[6]) - 40;
             // Calculate average (use float division)
             get.tempAverage = (static_cast<float>(get.tempMax) + static_cast<float>(get.tempMin)) / 2.0f;
             break;
 
         case COMMAND::DISCHARGE_CHARGE_MOS_STATUS: // 0x93
             // Byte 4: Charge/Discharge Status (0=Stationary, 1=Charge, 2=Discharge)
             // Byte 5: Charge FET state (0=Off, 1=On)
             // Byte 6: Discharge FET state (0=Off, 1=On)
             // Byte 7: BMS Heartbeat/Life (0-255)
             // Bytes 8-11: Remaining Capacity (mAh, 32-bit unsigned)
             get.batteryStatus = this->my_rxBuffer[4]; // Store the raw status code
             switch (get.batteryStatus) {
                 case 0: get.chargeDischargeStatus = "Stationary"; break;
                 case 1: get.chargeDischargeStatus = "Charge"; break;
                 case 2: get.chargeDischargeStatus = "Discharge"; break;
                 default: get.chargeDischargeStatus = "Unknown (" + std::to_string(get.batteryStatus) + ")"; break;
             }
             get.chargeFetState = (this->my_rxBuffer[5] == 1);    // Convert to bool
             get.disChargeFetState = (this->my_rxBuffer[6] == 1); // Convert to bool
             get.bmsHeartBeat = this->my_rxBuffer[7];
             get.resCapacitymAh = ((uint32_t)my_rxBuffer[8] << 24) | ((uint32_t)my_rxBuffer[9] << 16) |
                                  ((uint32_t)my_rxBuffer[10] << 8) | (uint32_t)my_rxBuffer[11];
             break;
 
         case COMMAND::STATUS_INFO: // 0x94
             // Byte 4: Number of cells
             // Byte 5: Number of temp sensors
             // Byte 6: Charger status (0=disconnected, 1=connected)
             // Byte 7: Load status (0=disconnected, 1=connected)
             // Byte 8: DI state (8 bits)
             // Bytes 9,10: Charge/Discharge Cycles (16-bit unsigned)
             get.numberOfCells = this->my_rxBuffer[4];
             get.numOfTempSensors = this->my_rxBuffer[5];
             // Basic validation of counts
             if (get.numberOfCells > MAX_NUMBER_CELLS || get.numberOfCells < MIN_NUMBER_CELLS ||
                 get.numOfTempSensors > MAX_NUMBER_TEMP_SENSORS || get.numOfTempSensors < MIN_NUMBER_TEMP_SENSORS)
             {
                  std::cerr << "BMS_UART Warning: Received potentially invalid cell/sensor count ("
                            << get.numberOfCells << "/" << get.numOfTempSensors << ")" << std::endl;
                  // Proceed cautiously, subsequent reads might fail if counts are wrong.
                  // Consider setting parse_success = false here if counts are critical.
             }
             get.chargeState = (this->my_rxBuffer[6] == 1); // Convert to bool
             get.loadState = (this->my_rxBuffer[7] == 1);   // Convert to bool
             for (size_t i = 0; i < 8; i++) {
                  get.dIO[i] = bitRead(this->my_rxBuffer[8], i);
             }
             get.bmsCycles = ((uint16_t)this->my_rxBuffer[9] << 8) | (uint16_t)this->my_rxBuffer[10];
             break;
 
         case COMMAND::CELL_VOLTAGES: // 0x95 (Multi-frame)
             { // Scope for local variables
                 int cellNo = 0; // Index for storing cell voltages
                 // Check if numberOfCells is valid before proceeding
                 if (get.numberOfCells < MIN_NUMBER_CELLS || get.numberOfCells > MAX_NUMBER_CELLS) {
                     std::cerr << "BMS_UART Error: Cannot parse cell voltages, invalid cell count: " << get.numberOfCells << std::endl;
                     parse_success = false;
                     break;
                 }
                 // Calculate expected number of frames based on 3 cells per frame
                 size_t total_expected_frames = static_cast<size_t>(ceil(static_cast<double>(get.numberOfCells) / 3.0));
 
                 // Iterate through each frame received in the my_rxBuffer
                 for (size_t frame_idx = 0; frame_idx < total_expected_frames; ++frame_idx) {
                     // Pointer to the start of the current frame's data in the buffer
                     uint8_t* current_frame_ptr = my_rxBuffer + frame_idx * XFER_BUFFER_LENGTH;
 
                     // Optional: Verify frame number (byte 4) if needed: if (current_frame_ptr[4] != frame_idx) { parse_success = false; break; }
 
                     // Determine how many cells are in *this* specific frame (usually 3, maybe fewer in the last frame)
                     int cells_in_this_frame = 3;
                     if (frame_idx == total_expected_frames - 1) { // Last frame
                          cells_in_this_frame = get.numberOfCells - cellNo;
                     }
                     cells_in_this_frame = std::min(cells_in_this_frame, 3); // Ensure not more than 3
 
                     // Parse cell voltages from this frame (data starts at byte 5, 2 bytes/cell)
                     for (int i = 0; i < cells_in_this_frame; ++i) {
                         if (cellNo < get.numberOfCells && cellNo < MAX_NUMBER_CELLS) { // Bounds check
                             // Combine High Byte (byte 5+i*2) and Low Byte (byte 6+i*2) for mV value
                             get.cellVmV[cellNo] = static_cast<float>((current_frame_ptr[5 + i * 2] << 8) | current_frame_ptr[6 + i * 2]);
                             cellNo++;
                         } else {
                             std::cerr << "BMS_UART Error: Cell index out of bounds during parsing frame " << frame_idx << std::endl;
                             parse_success = false;
                             break; // Exit inner loop
                         }
                     }
                     if (!parse_success) break; // Exit outer loop if inner failed
                 } // End frame loop
 
                 // Final check: Did we parse the expected number of cells?
                 if (parse_success && cellNo != get.numberOfCells) {
                     std::cerr << "BMS_UART Warning: Parsed " << cellNo << " cell voltages, but expected " << get.numberOfCells << std::endl;
                     // Consider this a parsing failure if strict matching is required
                     // parse_success = false;
                 }
             } // End scope
             break;
 
         case COMMAND::CELL_TEMPERATURE: // 0x96 (Multi-frame)
             { // Scope for local variables
                 int sensorNo = 0; // Index for storing temperatures
                 // Check if numOfTempSensors is valid
                 if (get.numOfTempSensors < MIN_NUMBER_TEMP_SENSORS || get.numOfTempSensors > MAX_NUMBER_TEMP_SENSORS) {
                     std::cerr << "BMS_UART Error: Cannot parse cell temperatures, invalid sensor count: " << get.numOfTempSensors << std::endl;
                     parse_success = false;
                     break;
                 }
                 // Calculate expected frames based on 7 sensors per frame
                 size_t total_expected_frames = static_cast<size_t>(ceil(static_cast<double>(get.numOfTempSensors) / 7.0));
 
                 // Iterate through each frame received
                 for (size_t frame_idx = 0; frame_idx < total_expected_frames; ++frame_idx) {
                     uint8_t* current_frame_ptr = my_rxBuffer + frame_idx * XFER_BUFFER_LENGTH;
                     // Optional: Verify frame number (byte 4)
 
                     // Determine sensors in this frame (usually 7, maybe fewer in last)
                     int sensors_in_this_frame = 7;
                     if (frame_idx == total_expected_frames - 1) { // Last frame
                          sensors_in_this_frame = get.numOfTempSensors - sensorNo;
                     }
                     sensors_in_this_frame = std::min(sensors_in_this_frame, 7); // Ensure not more than 7
 
                     // Parse temperatures (data starts at byte 5, 1 byte/sensor, offset 40)
                     for (int i = 0; i < sensors_in_this_frame; ++i) {
                         if (sensorNo < get.numOfTempSensors && sensorNo < MAX_NUMBER_TEMP_SENSORS) { // Bounds check
                             get.cellTemperature[sensorNo] = static_cast<int>(current_frame_ptr[5 + i]) - 40;
                             sensorNo++;
                         } else {
                              std::cerr << "BMS_UART Error: Sensor index out of bounds during parsing frame " << frame_idx << std::endl;
                              parse_success = false;
                              break; // Exit inner loop
                         }
                     }
                      if (!parse_success) break; // Exit outer loop
                 } // End frame loop
 
                 // Final check
                 if (parse_success && sensorNo != get.numOfTempSensors) {
                      std::cerr << "BMS_UART Warning: Parsed " << sensorNo << " temperatures, but expected " << get.numOfTempSensors << std::endl;
                      // parse_success = false;
                 }
             } // End scope
             break;
 
         case COMMAND::CELL_BALANCE_STATE: // 0x97 (Single frame expected for up to 48 cells based on description)
             { // Scope for local variables
                 int cellBit = 0; // Bit index (corresponds to cell number 0-47)
                 get.cellBalanceActive = false; // Reset flag
                 // Check cell count validity
                 if (get.numberOfCells < MIN_NUMBER_CELLS || get.numberOfCells > MAX_NUMBER_CELLS) {
                      std::cerr << "BMS_UART Error: Cannot parse balance state, invalid cell count: " << get.numberOfCells << std::endl;
                      parse_success = false;
                      break;
                 }
                 // Balance states are in bits of data bytes 4 through 9 (6 bytes = 48 bits)
                 for (size_t byte_idx = 0; byte_idx < 6; byte_idx++) {
                     uint8_t current_byte = this->my_rxBuffer[4 + byte_idx];
                     // Iterate through bits within the byte
                     for (size_t bit_idx = 0; bit_idx < 8; bit_idx++) {
                         if (cellBit < get.numberOfCells && cellBit < MAX_NUMBER_CELLS) { // Check bounds
                              bool state = bitRead(current_byte, bit_idx);
                              get.cellBalanceState[cellBit] = state;
                              if (state) get.cellBalanceActive = true; // Set flag if any cell is balancing
                         } else {
                              // Stop processing bits if we've covered all known cells
                              goto end_balance_parse; // Use goto for clean exit from nested loops
                         }
                         cellBit++;
                     }
                 }
             end_balance_parse:; // Label for goto target
             } // End scope
             break;
 
         case COMMAND::FAILURE_CODES: // 0x98 (Alarm flags)
             // Parse alarm bits from bytes 4 through 11 (as per header definition)
             // Assuming bitRead works correctly and alarm struct members are ordered correctly
             /* Byte 4: 0x00 */
             alarm.levelOneCellVoltageTooHigh = bitRead(this->my_rxBuffer[4], 0);
             alarm.levelTwoCellVoltageTooHigh = bitRead(this->my_rxBuffer[4], 1);
             alarm.levelOneCellVoltageTooLow = bitRead(this->my_rxBuffer[4], 2);
             alarm.levelTwoCellVoltageTooLow = bitRead(this->my_rxBuffer[4], 3);
             alarm.levelOnePackVoltageTooHigh = bitRead(this->my_rxBuffer[4], 4);
             alarm.levelTwoPackVoltageTooHigh = bitRead(this->my_rxBuffer[4], 5);
             alarm.levelOnePackVoltageTooLow = bitRead(this->my_rxBuffer[4], 6);
             alarm.levelTwoPackVoltageTooLow = bitRead(this->my_rxBuffer[4], 7);
             /* Byte 5: 0x01 */
             alarm.levelOneChargeTempTooHigh = bitRead(this->my_rxBuffer[5], 0); // Corrected index from original
             alarm.levelTwoChargeTempTooHigh = bitRead(this->my_rxBuffer[5], 1);
             alarm.levelOneChargeTempTooLow = bitRead(this->my_rxBuffer[5], 2);
             alarm.levelTwoChargeTempTooLow = bitRead(this->my_rxBuffer[5], 3);
             alarm.levelOneDischargeTempTooHigh = bitRead(this->my_rxBuffer[5], 4);
             alarm.levelTwoDischargeTempTooHigh = bitRead(this->my_rxBuffer[5], 5);
             alarm.levelOneDischargeTempTooLow = bitRead(this->my_rxBuffer[5], 6);
             alarm.levelTwoDischargeTempTooLow = bitRead(this->my_rxBuffer[5], 7);
             /* Byte 6: 0x02 */
             alarm.levelOneChargeCurrentTooHigh = bitRead(this->my_rxBuffer[6], 0);
             alarm.levelTwoChargeCurrentTooHigh = bitRead(this->my_rxBuffer[6], 1);
             alarm.levelOneDischargeCurrentTooHigh = bitRead(this->my_rxBuffer[6], 2);
             alarm.levelTwoDischargeCurrentTooHigh = bitRead(this->my_rxBuffer[6], 3);
             alarm.levelOneStateOfChargeTooHigh = bitRead(this->my_rxBuffer[6], 4);
             alarm.levelTwoStateOfChargeTooHigh = bitRead(this->my_rxBuffer[6], 5);
             alarm.levelOneStateOfChargeTooLow = bitRead(this->my_rxBuffer[6], 6);
             alarm.levelTwoStateOfChargeTooLow = bitRead(this->my_rxBuffer[6], 7);
             /* Byte 7: 0x03 */
             alarm.levelOneCellVoltageDifferenceTooHigh = bitRead(this->my_rxBuffer[7], 0);
             alarm.levelTwoCellVoltageDifferenceTooHigh = bitRead(this->my_rxBuffer[7], 1);
             alarm.levelOneTempSensorDifferenceTooHigh = bitRead(this->my_rxBuffer[7], 2);
             alarm.levelTwoTempSensorDifferenceTooHigh = bitRead(this->my_rxBuffer[7], 3);
             /* Byte 8: 0x04 */
             alarm.chargeFETTemperatureTooHigh = bitRead(this->my_rxBuffer[8], 0);
             alarm.dischargeFETTemperatureTooHigh = bitRead(this->my_rxBuffer[8], 1);
             alarm.failureOfChargeFETTemperatureSensor = bitRead(this->my_rxBuffer[8], 2);
             alarm.failureOfDischargeFETTemperatureSensor = bitRead(this->my_rxBuffer[8], 3);
             alarm.failureOfChargeFETAdhesion = bitRead(this->my_rxBuffer[8], 4);
             alarm.failureOfDischargeFETAdhesion = bitRead(this->my_rxBuffer[8], 5);
             alarm.failureOfChargeFETTBreaker = bitRead(this->my_rxBuffer[8], 6);
             alarm.failureOfDischargeFETBreaker = bitRead(this->my_rxBuffer[8], 7);
             /* Byte 9: 0x05 */
             alarm.failureOfAFEAcquisitionModule = bitRead(this->my_rxBuffer[9], 0);
             alarm.failureOfVoltageSensorModule = bitRead(this->my_rxBuffer[9], 1);
             alarm.failureOfTemperatureSensorModule = bitRead(this->my_rxBuffer[9], 2);
             alarm.failureOfEEPROMStorageModule = bitRead(this->my_rxBuffer[9], 3);
             alarm.failureOfRealtimeClockModule = bitRead(this->my_rxBuffer[9], 4);
             alarm.failureOfPrechargeModule = bitRead(this->my_rxBuffer[9], 5);
             alarm.failureOfVehicleCommunicationModule = bitRead(this->my_rxBuffer[9], 6);
             alarm.failureOfIntranetCommunicationModule = bitRead(this->my_rxBuffer[9], 7);
             /* Byte 10: 0x06 */
             alarm.failureOfCurrentSensorModule = bitRead(this->my_rxBuffer[10], 0);
             alarm.failureOfMainVoltageSensorModule = bitRead(this->my_rxBuffer[10], 1);
             alarm.failureOfShortCircuitProtection = bitRead(this->my_rxBuffer[10], 2);
             alarm.failureOfLowVoltageNoCharging = bitRead(this->my_rxBuffer[10], 3);
             break;
 
         // SET commands (CHRG_FET, DISCHRG_FET, BMS_RESET) usually don't require parsing the response payload,
         // just checking that a valid response (correct command echo, checksum) was received, which receiveBytes() handles.
         // Add parsing here if the BMS response for SET commands contains meaningful status data.
         case COMMAND::CHRG_FET:
         case COMMAND::DISCHRG_FET:
         case COMMAND::BMS_RESET:
             // No specific parsing needed for acknowledgment typically
             break;
 
         default:
             // Unknown command or command not requiring parsing here
             std::cerr << "BMS_UART Warning: No specific parsing implemented for received command 0x"
                       << std::hex << static_cast<int>(cmdID) << std::dec << std::endl;
             break;
     } // End switch (cmdID) for parsing
 
     if (!parse_success) {
         std::cerr << "BMS_UART Error: Failed to parse data for command 0x" << std::hex << static_cast<int>(cmdID) << std::dec << std::endl;
         current_status_ = CommStatus::ERROR_PARSE_FAIL; // Use a sp/ Gerekli
     return current_status_;
    }
    return CommStatus::SUCCESS;
}
 
 /**
  * @brief Sends a command packet to the BMS.
  * @details Calculates the checksum and writes the complete packet to the serial port.
  * @param cmdID The COMMAND enum value to send.
  * @return True on successful write, false on failure. Updates internal status on failure.
  */
 bool BMS_UART::sendCommand(COMMAND cmdID)
 {
     if (!isPortOpen()) {
         current_status_ = CommStatus::ERROR_PORT_CLOSED;
         return false;
     }
 
     // Flush input buffer before sending to discard any stale data from BMS
     // Note: Flushing output buffer (TCOFLUSH) might be needed if previous writes could be pending.
     // TCIFLUSH only clears received data not yet read.
     if (tcflush(my_serialIntf, TCIFLUSH) != 0) {
          std::cerr << "BMS_UART Warning: tcflush (input) failed before send: " << strerror(errno) << std::endl;
     }
 
     // Optional small delay before sending, sometimes helps reliability
     // std::this_thread::sleep_for(10ms);
 
     // --- Prepare the Transmit Buffer ---
     // Bytes 0 (Start=0xA5), 1 (Addr=0x40) are set in Init() or are fixed.
     my_txBuffer[2] = static_cast<uint8_t>(cmdID); // Set Command ID
     my_txBuffer[3] = 0x08; // Default payload length (Data Length = 8)
 
     // Clear payload section (bytes 4-11) unless it was pre-filled for a SET command
     // Note: SET commands (setChargeMOS etc.) fill byte 4 *before* calling this function.
     if (cmdID != COMMAND::CHRG_FET && cmdID != COMMAND::DISCHRG_FET) {
          for (uint8_t i = 4; i < 12; i++) {
              my_txBuffer[i] = 0x00;
          }
     }
     // If other commands require specific non-zero payloads, handle them here.
 
     // Calculate Checksum: Sum of bytes from Address (index 1) to end of payload (index 11)
     // Verify checksum range with BMS protocol document!
     uint8_t checksum = 0;
     for (uint8_t i = 0; i <= 11; i++) {
         checksum += this->my_txBuffer[i];
     }
     my_txBuffer[12] = checksum; // Store checksum in the last byte
 
     // --- Write the command to the serial port ---
     ssize_t bytes_written = write(this->my_serialIntf, this->my_txBuffer, XFER_BUFFER_LENGTH);
 
     // Check for write errors
     if (bytes_written < 0) {
         std::cerr << "BMS_UART Error: write failed: " << strerror(errno) << std::endl;
         current_status_ = CommStatus::ERROR_WRITE_FAILED;
         // Consider closing port if write fails persistently? Depends on error type.
         // if (errno == EIO || errno == ENXIO || errno == EBADF) { close(my_serialIntf); my_serialIntf = -1; }
         return false;
     }
 
     // Check if the correct number of bytes were written
     if (bytes_written != XFER_BUFFER_LENGTH) {
         std::cerr << "BMS_UART Warning: write incomplete. Wrote " << bytes_written << "/" << XFER_BUFFER_LENGTH << " bytes." << std::endl;
         current_status_ = CommStatus::ERROR_WRITE_FAILED; // Treat incomplete write as failure
         return false;
     }
 
     // Optional: Ensure data is transmitted before proceeding (useful if reads follow immediately)
     // tcdrain(my_serialIntf); // Waits until all output written has been transmitted
 
     return true; // Write successful
 }
 
 
 /**
  * @brief Receives bytes from the serial port, handling timeouts and multi-frame responses.
  * @details Reads data iteratively until the expected total number of bytes (calculated based
  *          on the command) is received or a timeout occurs. Validates checksum for each frame.
  * @param cmdID The COMMAND for which a response is expected (used to determine expected length).
  * @return CommStatus Indicating SUCCESS or the specific error (TIMEOUT, READ_MISMATCH, CHECKSUM, PORT_CLOSED).
  */
 BMS_UART::CommStatus BMS_UART::receiveBytes(COMMAND cmdID)
 {
     if (!isPortOpen()) {
         current_status_ = CommStatus::ERROR_PORT_CLOSED;
         return current_status_;
     }

     // --- Determine Expected Total Response Length (unchanged) ---
     size_t single_frame_len = XFER_BUFFER_LENGTH;
     size_t total_expected_length = single_frame_len;
     bool is_multi_frame_command = false;
     // ... (switch statement to calculate total_expected_length and set is_multi_frame_command) ...
      switch (cmdID) {
         case COMMAND::CELL_VOLTAGES:
             if (get.numberOfCells < MIN_NUMBER_CELLS || get.numberOfCells > MAX_NUMBER_CELLS) {
                  std::cerr << "BMS_UART Error: Cannot receive cell voltages, invalid cell count: " << get.numberOfCells << std::endl;
                  return CommStatus::ERROR_PRECONDITION_FAIL;
             }
             is_multi_frame_command = true;
             total_expected_length = single_frame_len * static_cast<size_t>(ceil(static_cast<double>(get.numberOfCells) / 3.0));
             break;
         case COMMAND::CELL_TEMPERATURE:
              if (get.numOfTempSensors < MIN_NUMBER_TEMP_SENSORS || get.numOfTempSensors > MAX_NUMBER_TEMP_SENSORS) {
                  std::cerr << "BMS_UART Error: Cannot receive cell temps, invalid sensor count: " << get.numOfTempSensors << std::endl;
                  return CommStatus::ERROR_PRECONDITION_FAIL;
              }
              is_multi_frame_command = true;
              total_expected_length = single_frame_len * static_cast<size_t>(ceil(static_cast<double>(get.numOfTempSensors) / 7.0));
              break;
         default:
             is_multi_frame_command = false;
             total_expected_length = single_frame_len;
             break;
     }


     // --- Buffer Size Check (unchanged) ---
     if (total_expected_length > sizeof(my_rxBuffer)) { /* ... ERROR HANDLING ... */ }

     // --- Clear Buffer and Initialize Variables (unchanged) ---
     memset(this->my_rxBuffer, 0, sizeof(this->my_rxBuffer));
     size_t bytes_received_total = 0;
     auto start_time = std::chrono::steady_clock::now();
     auto read_timeout_duration = READ_TIMEOUT_MS + (total_expected_length / single_frame_len) * 500ms;

     // --- SIMPLIFIED Read Loop (No intermediate validation) ---
     while (bytes_received_total < total_expected_length) {
         // Check for overall timeout
         auto now = std::chrono::steady_clock::now();
         if (now - start_time > read_timeout_duration) {
             std::cerr << "BMS_UART Error: Read timeout waiting for command 0x" << std::hex << static_cast<int>(cmdID) << std::dec
                       << ". Received " << bytes_received_total << "/" << total_expected_length << " bytes." << std::endl;
             current_status_ = CommStatus::ERROR_READ_TIMEOUT;
             barfRXBuffer(bytes_received_total);
             return current_status_;
         }

         // Use select() to wait for data
         fd_set readfd; // Define fd_set inside the loop or outside? Outside is fine.
         FD_ZERO(&readfd);
         FD_SET(this->my_serialIntf, &readfd);
         timeval select_timeout = {0, 200 * 1000}; // 200ms

         int select_result = select(this->my_serialIntf + 1, &readfd, NULL, NULL, &select_timeout);

         if (select_result < 0) { /* ... handle select error ... */ }

         if (select_result > 0 && FD_ISSET(this->my_serialIntf, &readfd)) {
             // Read available data
             ssize_t bytes_read_now = read(this->my_serialIntf,
                                           this->my_rxBuffer + bytes_received_total,
                                           total_expected_length - bytes_received_total);

             if (bytes_read_now < 0) { /* ... handle read error ... */ }
             if (bytes_read_now == 0) { continue; } // No data read, loop again

             bytes_received_total += bytes_read_now;
             // NO validation inside this loop anymore
         } else {
             // select() timed out, loop continues
             continue;
         }
     } // End while (bytes_received_total < total_expected_length)

     // --- Post-Read Validation (Do all checks here) ---

     // 1. Check if we received exactly the expected number of bytes
     if (bytes_received_total != total_expected_length) {
         std::cerr << "BMS_UART Error: Final byte count mismatch. Expected " << total_expected_length
                   << ", Got " << bytes_received_total << "." << std::endl;
         barfRXBuffer(bytes_received_total);
         current_status_ = CommStatus::ERROR_READ_MISMATCH;
         return current_status_;
     }

     // 2. Iterate through each frame in the buffer and validate
     int expected_frame_num_for_validation = 1; // Start expecting frame 1
     for (size_t frame_offset = 0; frame_offset < total_expected_length; frame_offset += single_frame_len) {
         uint8_t* current_frame = my_rxBuffer + frame_offset;

         // Check Start Byte
         if (current_frame[0] != 0xA5) {
             std::cerr << "BMS_UART Error: Invalid start byte (0x" << std::hex << static_cast<int>(current_frame[0]) << ") in frame at offset " << std::dec << frame_offset << std::endl;
             barfRXBuffer(total_expected_length); current_status_ = CommStatus::ERROR_FRAMING; return current_status_;
         }
         // Check Command Echo
         if (current_frame[2] != static_cast<uint8_t>(cmdID)) {
             std::cerr << "BMS_UART Error: Command mismatch (Expected 0x" << std::hex << static_cast<int>(cmdID) << ", Got 0x" << static_cast<int>(current_frame[2]) << ") in frame at offset " << std::dec << frame_offset << std::endl;
             barfRXBuffer(total_expected_length); current_status_ = CommStatus::ERROR_CMD_MISMATCH; return current_status_;
         }
         // Check Frame Sequence (if multi-frame)
         if (is_multi_frame_command) {
             int received_num = current_frame[4];
             if (received_num != expected_frame_num_for_validation) {
                 std::cerr << "BMS_UART Error: Out-of-order frame validation! Expected frame " << expected_frame_num_for_validation
                           << ", got frame " << received_num << " at offset " << frame_offset << std::endl;
                 barfRXBuffer(total_expected_length);
                 current_status_ = CommStatus::ERROR_FRAME_SEQUENCE; return current_status_;
             }
             expected_frame_num_for_validation++; // Increment expected number for the next frame check
         }
         // Checksum
         if (!validateChecksumFrame(current_frame, single_frame_len)) {
             std::cerr << "BMS_UART Error: Checksum failed for frame at offset " << frame_offset << std::endl;
             barfRXBuffer(total_expected_length); current_status_ = CommStatus::ERROR_CHECKSUM; return current_status_;
         }
     } // End validation loop

     // --- Success ---
     updateLastSuccessTime();
     current_status_ = CommStatus::SUCCESS;
     return CommStatus::SUCCESS;
 }
 
 
 /**
  * @brief Validates the checksum of a single frame buffer.
  * @param frame_buffer Pointer to the start of the 13-byte frame data.
  * @param length The length of the frame (should be XFER_BUFFER_LENGTH).
  * @return true if the checksum matches, false otherwise.
  */
 bool BMS_UART::validateChecksumFrame(const uint8_t* frame_buffer, size_t length) {
     if (length == 0) return false; // Cannot validate empty frame
 
     uint8_t calculated_checksum = 0x00;
     // Checksum usually covers Address, Command, Length, Data payload.
     // Verify range with BMS protocol! Assuming indices 1 through length-2 (e.g., 1-11 for 13-byte frame).
     size_t checksum_end_index = length - 2; // Index of the last byte included in checksum calculation
     for (size_t i = 0; i <= checksum_end_index; i++) {
         calculated_checksum += frame_buffer[i];
     }
 
     // The last byte of the frame is the received checksum
     uint8_t received_checksum = frame_buffer[length - 1];
 
     return (calculated_checksum == received_checksum);
 }
 
 /**
  * @brief Prints the contents of the RX buffer to std::cout for debugging.
  * @param length The number of bytes in the buffer to print.
  */
 void BMS_UART::barfRXBuffer(size_t length)
 {
     std::cout << "<BMS DEBUG> RX Buffer (" << length << "/" << sizeof(my_rxBuffer) << " bytes): [";
     for (size_t i = 0; i < length && i < sizeof(my_rxBuffer); i++) // Prevent overflow
     {
         // Print hex value formatted as "0xXX"
         char hex_byte[5];
         snprintf(hex_byte, sizeof(hex_byte), "0x%02X", this->my_rxBuffer[i]);
         std::cout << hex_byte << (i == length - 1 ? "" : ", ");
     }
     std::cout << "]" << std::endl;
 }