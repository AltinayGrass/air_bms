#include "air_bms/bms_uart.hpp"

//----------------------------------------------------------------------
// Public Functions
//----------------------------------------------------------------------
#include <thread>
#include <chrono>
using namespace std::chrono_literals;


inline uint8_t bitRead(uint32_t x, uint32_t bit){
    return (x >> bit) & 1;
}

BMS_UART::BMS_UART(const std::string& serialDev)
{
    this->my_serialIntf = open( serialDev.c_str(), O_RDWR );
    if(this->my_serialIntf < 0) {
        std::cout<<" Error "<<errno<<" from open: "<<serialDev.c_str();
    }
  //  fcntl(this->my_serialIntf, F_SETFL, FASYNC);
}

bool BMS_UART::readyRead(bool delayed)
{
    FD_ZERO(&readfd);
    FD_SET(this->my_serialIntf, &readfd);
    timeval tv;
    tv.tv_sec = delayed ? 1: 0;;
    tv.tv_usec = 0;
    select(this->my_serialIntf+ 1, &readfd, NULL, NULL, &tv);
    return FD_ISSET(this->my_serialIntf, &readfd);     

}

bool BMS_UART::Init(){

    if(this->my_serialIntf < 0 )
    {
        return false;
    }

    // Initialize the serial link to 9600 baud with 8 data bits and no parity bits, per the BMS spec
    struct termios tty;
    //tcgetattr(this->my_serialIntf, &tty);
    memset(&tty, 0, sizeof tty);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    // tty.c_lflag &= ~ICANON;
    // tty.c_lflag &= ~ECHO; // Disable echo
    // tty.c_lflag &= ~ECHOE; // Disable erasure
    // tty.c_lflag &= ~ECHONL; // Disable new-line echo
    // tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and 
    // tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    // tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    // tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    // tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
    // tty.c_cc[VEOT] = 0x05;
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    cfmakeraw(&tty);
    tcsetattr(this->my_serialIntf, TCSANOW, &tty);
 

    // Set up the output buffer with some values that won't be changing

    this->my_txBuffer[0] = 0xA5; // Start byte
    this->my_txBuffer[1] = 0x40; // Host address
    this->my_txBuffer[3] = 0x08; // Length

    // Fill bytes 5-11 with 0s
    for(uint8_t i = 4; i < 12; i++)
    {
        this->my_rxBuffer[i] = 0x00;
    }
    return true;
}

bool  BMS_UART::update()
{
    // Call all get___() functions to populate all members of the "get" struct
    if (!getPackMeasurements())
        return false; // 0x90
    if (!getMinMaxCellVoltage())
        return false; // 0x91
    if (!getPackTemp())
        return false; // 0x92
    if (!getDischargeChargeMosStatus())
        return false; // 0x93
    if (!getStatusInfo())
        return false; // 0x94
    if (!getCellVoltages())
        return false; // 0x95
    if (!getCellTemperature())
        return false; // 0x96
    if (!getCellBalanceState())
        return false; // 0x97
    if (!getFailureCodes())
        return false; // 0x98
    return true;
}
bool BMS_UART::getPackMeasurements() //0x90
{
    this->sendCommand(COMMAND::VOUT_IOUT_SOC);
    if (!this->receiveBytes(COMMAND::VOUT_IOUT_SOC))
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<BMS DEBUG> Receive failed, V, I, & SOC values won't be modified!\n");
#endif
        return false;
    }

    // Pull the relevent values out of the buffer
    get.packVoltage = ((float)((this->my_rxBuffer[4] << 8) | this->my_rxBuffer[5]) / 10.0f);
    // The current measurement is given with a 30000 unit offset (see /docs/)
    get.packCurrent = ((float)(((this->my_rxBuffer[8] << 8) | this->my_rxBuffer[9]) - 30000) / 10.0f);
    get.packSOC = ((float)((this->my_rxBuffer[10] << 8) | this->my_rxBuffer[11]) / 10.0f);
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("<BMS DEBUG> " + (String)get.packVoltage + "V, " + (String)get.packCurrent + "A, " + (String)get.packSOC + "SOC");
#endif

    return true;
}
bool BMS_UART::getMinMaxCellVoltage() // 0X91
{
    this->sendCommand(COMMAND::MIN_MAX_CELL_VOLTAGE);
    if(!this->receiveBytes(COMMAND::MIN_MAX_CELL_VOLTAGE))
    {
    
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("<BMS DEBUG> Receive failed, min/max cell values won't be modified!\n");
#endif
        return false;
    }
    
    get.maxCellmV = (float)((this->my_rxBuffer[4] << 8) | this->my_rxBuffer[5]);
    get.maxCellVNum = this->my_rxBuffer[6];
    get.minCellmV = (float)((this->my_rxBuffer[7] << 8) | this->my_rxBuffer[8]);
    get.minCellVNum = this->my_rxBuffer[9];
    get.cellDiff = (get.maxCellmV - get.minCellmV);

    return true;
}
bool BMS_UART::getPackTemp() // 0x92
{
    this->sendCommand(COMMAND::MIN_MAX_TEMPERATURE);
    if(!this->receiveBytes(COMMAND::MIN_MAX_TEMPERATURE))
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<BMS DEBUG> Receive failed, Temp values won't be modified!\n");
    
#endif
    return false;
    }
    // An offset of 40 is added by the BMS to avoid having to deal with negative numbers, see protocol in /docs/
    get.tempMax = (this->my_rxBuffer[4] - 40);
    get.tempMin = (this->my_rxBuffer[6] - 40);
    get.tempAverage = (get.tempMax + get.tempMin) / 2;

    return true;
}

bool BMS_UART::getDischargeChargeMosStatus() // 0x93
{
    this->sendCommand(COMMAND::DISCHARGE_CHARGE_MOS_STATUS);

    if (!this->receiveBytes(COMMAND::DISCHARGE_CHARGE_MOS_STATUS))
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<BMS DEBUG> Receive failed, Charge / discharge mos Status won't be modified!\n");
#endif
        return false;
    }

    switch (this->my_rxBuffer[4])
    {
    case 0:
        get.chargeDischargeStatus = "Stationary";
        break;
    case 1:
        get.chargeDischargeStatus = "Charge";
        break;
    case 2:
        get.chargeDischargeStatus = "Discharge";
        break;
    }
    get.batteryStatus = this->my_rxBuffer[4];
    get.chargeFetState = this->my_rxBuffer[5];
    get.disChargeFetState = this->my_rxBuffer[6];
    get.bmsHeartBeat = this->my_rxBuffer[7];
    get.resCapacitymAh = ((uint32_t)my_rxBuffer[8] << 0x18) | ((uint32_t)my_rxBuffer[9] << 0x10) | ((uint32_t)my_rxBuffer[10] << 0x08) | (uint32_t)my_rxBuffer[11];

    return true;
}

bool BMS_UART::getStatusInfo() // 0x94
{
    this->sendCommand(COMMAND::STATUS_INFO);

    if (!this->receiveBytes(COMMAND::STATUS_INFO))
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<BMS DEBUG> Receive failed, Status info won't be modified!\n");
#endif
        return false;
    }

    get.numberOfCells = this->my_rxBuffer[4];
    get.numOfTempSensors = this->my_rxBuffer[5];
    get.chargeState = this->my_rxBuffer[6];
    get.loadState = this->my_rxBuffer[7];

    // Parse the 8 bits into 8 booleans that represent the states of the Digital IO
    for (size_t i = 0; i < 8; i++)
    {
        get.dIO[i] = bitRead(this->my_rxBuffer[8], i);
    }

    get.bmsCycles = ((uint16_t)this->my_rxBuffer[9] << 0x08) | (uint16_t)this->my_rxBuffer[10];

    return true;
}

bool BMS_UART::getCellVoltages() // 0x95
{
    int cellNo = 0;
        // Check to make sure we have a valid number of cells
    if (get.numberOfCells < MIN_NUMBER_CELLS && get.numberOfCells >= MAX_NUMBER_CELLS)
    {
        return false;
    }

    this->sendCommand(COMMAND::CELL_VOLTAGES);
    

    for (size_t j = 0; j < ceil(get.numberOfCells / 3); j++)
    {
        
    if (!this->receiveBytes(COMMAND::CELL_VOLTAGES))
        {
#ifdef DEBUG_SERIAL
            DEBUG_SERIAL.print("<BMS DEBUG> Receive failed, Cell Voltages won't be modified!\n");
#endif
            return false;
        }
        for (size_t i = 0; i < 3; i++)
        {

#ifdef DEBUG_SERIAL
            DEBUG_SERIAL.print("<BMS DEBUG> Frame No.: " + (String)this->my_rxBuffer[4]);
            DEBUG_SERIAL.println(" Cell No: " + (String)(cellNo + 1) + ". " + (String)((this->my_rxBuffer[5 + i + i] << 8) | this->my_rxBuffer[6 + i + i]) + "mV");
#endif

            get.cellVmV[cellNo] = (this->my_rxBuffer[5 + i + i] << 8) | this->my_rxBuffer[6 + i + i];
            cellNo++;
            if (cellNo >= get.numberOfCells)
                break;        
        }
    }
    return true;
}

bool BMS_UART::getCellTemperature() // 0x96
{
    int sensorNo = 0;
    // Check to make sure we have a valid number of temp sensors
    if ((get.numOfTempSensors < MIN_NUMBER_TEMP_SENSORS) && (get.numOfTempSensors >= MAX_NUMBER_TEMP_SENSORS))
    {
        return false;
    }

    this->sendCommand(COMMAND::CELL_TEMPERATURE);

    for (size_t i = 0; i <= ceil(get.numOfTempSensors / 7); i++)
    {

        if (!this->receiveBytes(COMMAND::CELL_TEMPERATURE))
        {
#ifdef DEBUG_SERIAL
            DEBUG_SERIAL.print("<BMS DEBUG> Receive failed, Cell Temperatures won't be modified!\n");
#endif
            return false;
        }

        for (size_t i = 0; i < 7; i++)
        {

#ifdef DEBUG_SERIAL
            DEBUG_SERIAL.print("<BMS DEBUG> Frame No.: " + (String)this->my_rxBuffer[4]);
            DEBUG_SERIAL.println(" Sensor No: " + (String)(sensorNo + 1) + ". " + String(this->my_rxBuffer[5 + i] - 40) + "°C");
#endif

            get.cellTemperature[sensorNo] = (this->my_rxBuffer[5 + i] - 40);
            sensorNo++;
            if (sensorNo + 1 >= get.numOfTempSensors)
                break;
        }
    }
    return true;
}

bool BMS_UART::getCellBalanceState() // 0x97
{
    int cellBalance = 0;
    int cellBit = 0;

    // Check to make sure we have a valid number of cells
    if (get.numberOfCells < MIN_NUMBER_CELLS && get.numberOfCells >= MAX_NUMBER_CELLS)
    {
        return false;
    }

    this->sendCommand(COMMAND::CELL_BALANCE_STATE);

    if (!this->receiveBytes(COMMAND::CELL_BALANCE_STATE))
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("<BMS DEBUG> Receive failed, Cell Balance State won't be modified!\n");
#endif
        return false;
    }

    // We expect 6 bytes response for this command
    for (size_t i = 0; i < 6; i++)
    {
        // For each bit in the byte, pull out the cell balance state boolean
        for (size_t j = 0; j < 8; j++)
        {
            get.cellBalanceState[cellBit] = bitRead(this->my_rxBuffer[i + 4], j);
            cellBit++;
            if (bitRead(this->my_rxBuffer[i + 4], j))
            {
                cellBalance++;
            }
            if (cellBit >= 47)
            {
                break;
            }
        }
    }

#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("<BMS DEBUG> Cell Balance State: ");
    for (int i = 0; i < get.numberOfCells; i++)
    {
        DEBUG_SERIAL.print(get.cellBalanceState[i]);
    }
    DEBUG_SERIAL.println();
#endif

    if (cellBalance > 0)
    {
        get.cellBalanceActive = true;
    }
    else
    {
        get.cellBalanceActive = false;
    }

    return true;
}

bool BMS_UART::getFailureCodes() // 0x98
{
    this->sendCommand(COMMAND::FAILURE_CODES);

    if (!this->receiveBytes(COMMAND::FAILURE_CODES))
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<BMS DEBUG> Receive failed, Failure Flags won't be modified!\n");
#endif
        return false;
    }

    /* 0x00 */
    alarm.levelOneCellVoltageTooHigh = bitRead(this->my_rxBuffer[4], 0);
    alarm.levelTwoCellVoltageTooHigh = bitRead(this->my_rxBuffer[4], 1);
    alarm.levelOneCellVoltageTooLow = bitRead(this->my_rxBuffer[4], 2);
    alarm.levelTwoCellVoltageTooLow = bitRead(this->my_rxBuffer[4], 3);
    alarm.levelOnePackVoltageTooHigh = bitRead(this->my_rxBuffer[4], 4);
    alarm.levelTwoPackVoltageTooHigh = bitRead(this->my_rxBuffer[4], 5);
    alarm.levelOnePackVoltageTooLow = bitRead(this->my_rxBuffer[4], 6);
    alarm.levelTwoPackVoltageTooLow = bitRead(this->my_rxBuffer[4], 7);

    /* 0x01 */
    alarm.levelOneChargeTempTooHigh = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelTwoChargeTempTooHigh = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelOneChargeTempTooLow = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelTwoChargeTempTooLow = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelOneDischargeTempTooHigh = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelTwoDischargeTempTooHigh = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelOneDischargeTempTooLow = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelTwoDischargeTempTooLow = bitRead(this->my_rxBuffer[5], 1);

    /* 0x02 */
    alarm.levelOneChargeCurrentTooHigh = bitRead(this->my_rxBuffer[6], 0);
    alarm.levelTwoChargeCurrentTooHigh = bitRead(this->my_rxBuffer[6], 1);
    alarm.levelOneDischargeCurrentTooHigh = bitRead(this->my_rxBuffer[6], 2);
    alarm.levelTwoDischargeCurrentTooHigh = bitRead(this->my_rxBuffer[6], 3);
    alarm.levelOneStateOfChargeTooHigh = bitRead(this->my_rxBuffer[6], 4);
    alarm.levelTwoStateOfChargeTooHigh = bitRead(this->my_rxBuffer[6], 5);
    alarm.levelOneStateOfChargeTooLow = bitRead(this->my_rxBuffer[6], 6);
    alarm.levelTwoStateOfChargeTooLow = bitRead(this->my_rxBuffer[6], 7);

    /* 0x03 */
    alarm.levelOneCellVoltageDifferenceTooHigh = bitRead(this->my_rxBuffer[7], 0);
    alarm.levelTwoCellVoltageDifferenceTooHigh = bitRead(this->my_rxBuffer[7], 1);
    alarm.levelOneTempSensorDifferenceTooHigh = bitRead(this->my_rxBuffer[7], 2);
    alarm.levelTwoTempSensorDifferenceTooHigh = bitRead(this->my_rxBuffer[7], 3);

    /* 0x04 */
    alarm.chargeFETTemperatureTooHigh = bitRead(this->my_rxBuffer[8], 0);
    alarm.dischargeFETTemperatureTooHigh = bitRead(this->my_rxBuffer[8], 1);
    alarm.failureOfChargeFETTemperatureSensor = bitRead(this->my_rxBuffer[8], 2);
    alarm.failureOfDischargeFETTemperatureSensor = bitRead(this->my_rxBuffer[8], 3);
    alarm.failureOfChargeFETAdhesion = bitRead(this->my_rxBuffer[8], 4);
    alarm.failureOfDischargeFETAdhesion = bitRead(this->my_rxBuffer[8], 5);
    alarm.failureOfChargeFETTBreaker = bitRead(this->my_rxBuffer[8], 6);
    alarm.failureOfDischargeFETBreaker = bitRead(this->my_rxBuffer[8], 7);

    /* 0x05 */
    alarm.failureOfAFEAcquisitionModule = bitRead(this->my_rxBuffer[9], 0);
    alarm.failureOfVoltageSensorModule = bitRead(this->my_rxBuffer[9], 1);
    alarm.failureOfTemperatureSensorModule = bitRead(this->my_rxBuffer[9], 2);
    alarm.failureOfEEPROMStorageModule = bitRead(this->my_rxBuffer[9], 3);
    alarm.failureOfRealtimeClockModule = bitRead(this->my_rxBuffer[9], 4);
    alarm.failureOfPrechargeModule = bitRead(this->my_rxBuffer[9], 5);
    alarm.failureOfVehicleCommunicationModule = bitRead(this->my_rxBuffer[9], 6);
    alarm.failureOfIntranetCommunicationModule = bitRead(this->my_rxBuffer[9], 7);

    /* 0x06 */
    alarm.failureOfCurrentSensorModule = bitRead(this->my_rxBuffer[10], 0);
    alarm.failureOfMainVoltageSensorModule = bitRead(this->my_rxBuffer[10], 1);
    alarm.failureOfShortCircuitProtection = bitRead(this->my_rxBuffer[10], 2);
    alarm.failureOfLowVoltageNoCharging = bitRead(this->my_rxBuffer[10], 3);

    return true;
}

bool BMS_UART::setDischargeMOS(bool sw) // 0xD9 0x80 First Byte 0x01=ON 0x00=OFF
{
    if (sw)
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("Attempting to switch discharge MOSFETs on");
#endif
        // Set the first byte of the data payload to 1, indicating that we want to switch on the MOSFET
        this->my_txBuffer[4] = 0x01;
        this->sendCommand(COMMAND::DISCHRG_FET);
        // Clear the buffer for further use
        this->my_txBuffer[4] = 0x00;
    }
    else
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("Attempting to switch discharge MOSFETs off");
#endif
        this->sendCommand(COMMAND::DISCHRG_FET);
    }
    if (!this->receiveBytes(COMMAND::DISCHRG_FET))
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<BMS DEBUG> No response from BMS! Can't verify MOSFETs switched.\n");
#endif
        return false;
    }

    return true;
}

bool BMS_UART::setChargeMOS(bool sw) // 0xDA 0x80 First Byte 0x01=ON 0x00=OFF
{
    if (sw == true)
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("Attempting to switch charge MOSFETs on");
#endif
        // Set the first byte of the data payload to 1, indicating that we want to switch on the MOSFET
        this->my_txBuffer[4] = 0x01;
        this->sendCommand(COMMAND::CHRG_FET);
        // Clear the buffer for further use
        this->my_txBuffer[4] = 0x00;
    }
    else
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("Attempting to switch charge MOSFETs off");
#endif
        this->sendCommand(COMMAND::CHRG_FET);
    }

    if (!this->receiveBytes(COMMAND::CHRG_FET))
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<BMS DEBUG> No response from BMS! Can't verify MOSFETs switched.\n");
#endif
        return false;
    }

    return true;
}

bool BMS_UART::setBmsReset() // 0x00 Reset the BMS
{
    this->sendCommand(COMMAND::BMS_RESET);

    if (!this->receiveBytes(COMMAND::BMS_RESET))
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<BMS DEBUG> Send failed, can't verify BMS was reset!\n");
#endif
        return false;
    }

    return true;
}

//----------------------------------------------------------------------
// Private Functions
//----------------------------------------------------------------------

void BMS_UART::sendCommand(COMMAND cmdID)
{
    while ( readyRead() )
    {
        char discard;
        read(this->my_serialIntf, &discard, 1);
    };

    uint8_t checksum = 0;
    this->my_txBuffer[2] = cmdID;
    // Calculate the checksum
    for (uint8_t i = 0; i <= 11; i++)
    {
        checksum += this->my_txBuffer[i];
    }
    // put it on the frame
    this->my_txBuffer[12] = checksum;

#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("\n<BMS DEBUG> Send command: 0x");
    DEBUG_SERIAL.print(cmdID, HEX);
    DEBUG_SERIAL.print(" Checksum = 0x");
    DEBUG_SERIAL.println(checksum, HEX);
#endif
    write(this->my_serialIntf, this->my_txBuffer, XFER_BUFFER_LENGTH);
}

bool BMS_UART::receiveBytes(COMMAND cmdID)
{
    uint8_t xfer_length=XFER_BUFFER_LENGTH;
    uint8_t rxByteNum=0;
    uint8_t rxByteNum2=0;

    switch (cmdID)
    {
    case CELL_VOLTAGES:
        // xfer_length=xfer_length*3;
        memset(this->my_rxBuffer, 0, xfer_length);
        /* code */
        break;
    
    default:
        // Clear out the input buffer
        memset(this->my_rxBuffer, 0, xfer_length);
        break;
    }

    
    while ( !readyRead(true) )
    {
    }
    
    switch (cmdID)
    {
    case CELL_VOLTAGES:
        // Read bytes from the specified serial interface 
        // my_buffer = my_buffer[0];
        rxByteNum = read(this->my_serialIntf,  this->my_rxBuffer, xfer_length);
        if (rxByteNum < xfer_length)
        {    
            while ( !readyRead() )
            {
            }
            rxByteNum2= read(this->my_serialIntf,  this->my_rxBuffer+rxByteNum, xfer_length-rxByteNum); 
        }
        rxByteNum += rxByteNum2;
        break;
    
    default:
        // Read bytes from the specified serial interface
        rxByteNum = read(this->my_serialIntf,  this->my_rxBuffer, xfer_length);
        break;
    }

    // Make sure we got the correct number of bytes
    if (rxByteNum != xfer_length)
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<BMS DEBUG> Error: Received the wrong number of bytes! Expected 13, got ");
        DEBUG_SERIAL.println(rxByteNum, DEC);
        this->barfRXBuffer();
#endif
        // std::cout << "<BMS DEBUG> Error: Received the wrong number of bytes! Expected " << +xfer_length << " , got " << +rxByteNum << std::endl;
        // for (size_t i = 0; i < rxByteNum; i++)
        //     {
        //     std::cout <<"0x" << std::hex << +this->my_rxBuffer[i] << " ";
        //     }
        // std::cout << std::dec << std::endl;

        // if (cmdID != CELL_VOLTAGES) return false;
    }
    //     else
    // {
    //     // if (cmdID == CELL_VOLTAGES) {
        
    //     for (size_t i = 0; i < rxByteNum; i++)
    //         {
    //         std::cout <<"0x" << std::hex << +this->my_rxBuffer[i] << " ";
    //         }
    //     std::cout << std::dec << std::endl;
    //     // }
    // }

    if (!validateChecksum(xfer_length))
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("<BMS DEBUG> Error: Checksum failed!");
        this->barfRXBuffer();
#endif
        std::cout << "<BMS DEBUG> Error: Checksum failed!" << std::endl;
        if (cmdID != CELL_VOLTAGES) return false;
    }
    return true;
}

bool BMS_UART::validateChecksum(int lenght)
{
    uint8_t checksum = 0x00;

    for (int i = 0; i < lenght - 1; i++)
    {
        checksum += this->my_rxBuffer[i];
    }

#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("<BMS DEBUG> Calculated checksum: " + (String)checksum + ", Received: " + (String)this->my_rxBuffer[XFER_BUFFER_LENGTH - 1] + "\n");
#endif

    // Compare the calculated checksum to the real checksum (the last received byte)
    return (checksum == this->my_rxBuffer[lenght - 1]);
}

void BMS_UART::barfRXBuffer(void)
{
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("<BMS DEBUG> RX Buffer: [");
    for (int i = 0; i < XFER_BUFFER_LENGTH; i++)
    {
        DEBUG_SERIAL.print(",0x" + (String)this->my_rxBuffer[i]);
    }
    DEBUG_SERIAL.print("]\n");
#endif
}