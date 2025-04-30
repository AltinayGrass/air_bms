#ifndef BMS_NODE_
#define BMS_NODE_

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "air_bms/bms_uart.hpp"

class BatteryStatus : public rclcpp::Node{


    public:
    BatteryStatus();


    private:
    void BatteryStatusCallBack();
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    BMS_UART bms_; 
    
    // const int MAX_TEMP_THRESHOLD_;
    // std::string STATUS_;  

    // enum class BatteryChargeState{
    //     STATIONARY, 
    //     CHARGING,
    //     DECHARGING 

    // };
};









#endif //BMS_NODE_