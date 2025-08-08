#include <ros/ros.h>
#include <rectrial/pub_data.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PointStamped.h> 
#include "Definitions.h" // Your Maxon motor library header

#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <fstream>
#include <csignal>
#include <sstream> // <<< FIX: Added for std::stringstream to parse strings

typedef int BOOL;

// Use named constants instead of magic numbers
const double FISH_DIRECTION_SCALAR = 1.0;
const double POSITION_SCALING_FACTOR = 29.0;
const double REAFFERENT_SCALING_FACTOR = 29.55;
const int NUM_SINE_FREQUENCIES = 13;

class MaxonMotorController
{
public:
    MaxonMotorController() : nh_("~")
    {
        {
            loadParams();
            // This is now the ONLY subscriber in this node.
            sub_command_ = root_nh_.subscribe("/imager", 100, &MaxonMotorController::commandCallback, this);
            
            refuge_state_pub_ = root_nh_.advertise<geometry_msgs::PointStamped>("/refuge_state", 10);
    
            if (!openDevice() || !prepareMotor() || !initializeMotor()) {
                ros::shutdown();
                return;
            }
        }

        ROS_INFO("Maxon motor node initialized successfully.");
    }

    ~MaxonMotorController()
    {
        if (key_handle_) {
            unsigned int error_code = 0;
            ROS_INFO("Closing device...");
            VCS_SetDisableState(key_handle_, node_id_, &error_code);
            VCS_CloseDevice(key_handle_, &error_code);
        }
    }

private:
    ros::NodeHandle root_nh_;
    ros::Publisher refuge_state_pub_;
    ros::NodeHandle nh_; 
    ros::Subscriber sub_motor_freq_;
    ros::Subscriber sub_experiment_trigger_;
    ros::Subscriber sub_state_;
    ros::Subscriber sub_command_;

    
    bool is_open_loop_ = false;
    bool is_closed_loop_ = false;
    bool is_gain_mode_ = false;
    bool is_sum_of_sines_ = false;

    double reafferent_gain_ = 0.0;
    double gain_limit_ = 10.0;
    double frequency_ = 2.05;
    double amp_mm_ = 10.0;
    
    std::chrono::steady_clock::time_point loop_start_time_;
    int frame_counter_ = 0;
    int overshoot_counter_ = 0;

    void* key_handle_ = nullptr;
    unsigned short node_id_ = 1;
    std::string device_name_;
    std::string protocol_stack_name_;
    std::string interface_name_;
    std::string port_name_;
    int baudrate_ = 0;
    double a_pos_ = 0.0; 

    double last_refuge_position_ = 0.0;

    void commandCallback(const rectrial::pub_data::ConstPtr& msg)
    {
        if (msg->finish_c == "end") {
            ROS_INFO("END command received. Homing motor for next trial.");
            initializeMotor();
            return;
        }

        // <<< FIX: Parse the new data string format
        std::string data_str = msg->data_e;
        std::stringstream ss_main(data_str);
        std::string final_pos_part;

        // 1. Get only the part before the first semicolon (e.g., "x,y")
        std::getline(ss_main, final_pos_part, ';');

        // 2. Parse that first part to get the x-coordinate
        double target_position_float = 0.0;
        std::stringstream ss_pos(final_pos_part);
        std::string x_str;
        if (std::getline(ss_pos, x_str, ',')) {
             try {
                target_position_float = std::stod(x_str);
             } catch (const std::exception& e) {
                ROS_ERROR("Failed to parse x-position from data_e: %s", msg->data_e.c_str());
             }
        }

        // 3. Command the motor
        unsigned int error_code = 0;
        long target_position = static_cast<long>(target_position_float);
        if (!VCS_MoveToPosition(key_handle_, node_id_, target_position, 1, 1, &error_code)) {
            ROS_ERROR("VCS_MoveToPosition failed. Error: 0x%X", error_code);
        }

        // 4. Publish the commanded position for the logger
        geometry_msgs::PointStamped refuge_msg;
        refuge_msg.header.stamp = msg->image_e.header.stamp;
        refuge_msg.point.x = target_position_float;
        refuge_state_pub_.publish(refuge_msg);
    }


    void loadParams() {
        int temp_node_id = 1;
        nh_.param<int>("node_id", temp_node_id, 1);
        node_id_ = static_cast<unsigned short>(temp_node_id);

        nh_.param<std::string>("device_name", device_name_, "EPOS4");
        nh_.param<std::string>("protocol_stack_name", protocol_stack_name_, "MAXON SERIAL V2");
        nh_.param<std::string>("interface_name", interface_name_, "USB");
        nh_.param<std::string>("port_name", port_name_, "USB0");
        nh_.param<int>("baudrate", baudrate_, 1000000);
        
        nh_.param<double>("amplitude_mm", amp_mm_, 10.0);
        nh_.param<double>("gain_limit", gain_limit_, 10.0);
        
        double mm_per_rev = 10.0;
        double encoder_count = 512.0;
        double gear_ratio = (624.0 / 35.0) * 4.0;
        
        double count_per_rev = encoder_count * gear_ratio;
        double count_per_mm = count_per_rev / mm_per_rev;
        a_pos_ = count_per_mm * amp_mm_;

        ROS_INFO("--- Motor Settings ---");
        ROS_INFO("Node ID: %d", node_id_);
        ROS_INFO("Device: %s on Port: %s", device_name_.c_str(), port_name_.c_str());
        ROS_INFO("Amplitude (counts): %.2f", a_pos_);
        ROS_INFO("----------------------");
    }

    bool openDevice() {
        unsigned int error_code = 0;
        ROS_INFO("Opening device: %s...", device_name_.c_str());
        key_handle_ = VCS_OpenDevice((char*)device_name_.c_str(), (char*)protocol_stack_name_.c_str(), (char*)interface_name_.c_str(), (char*)port_name_.c_str(), &error_code);

        if (key_handle_ != nullptr && error_code == 0) {
            unsigned int current_baudrate = 0, timeout = 0;
            if (VCS_GetProtocolStackSettings(key_handle_, &current_baudrate, &timeout, &error_code) &&
                VCS_SetProtocolStackSettings(key_handle_, baudrate_, timeout, &error_code)) {
                ROS_INFO("Device opened successfully.");
                return true;
            }
        }
        ROS_ERROR("Failed to open device. Error: 0x%X", error_code);
        key_handle_ = nullptr;
        return false;
    }

    bool prepareMotor() {
        unsigned int error_code = 0;
        BOOL is_fault = 0;
        if (!VCS_GetFaultState(key_handle_, node_id_, &is_fault, &error_code)) {
            ROS_ERROR("VCS_GetFaultState failed. Error: 0x%X", error_code);
            return false;
        }

        if (is_fault) {
            ROS_INFO("Motor is in fault state, clearing fault...");
            if (!VCS_ClearFault(key_handle_, node_id_, &error_code)) {
                ROS_ERROR("VCS_ClearFault failed. Error: 0x%X", error_code);
                return false;
            }
        }

        BOOL is_enabled = 0;
        if (!VCS_GetEnableState(key_handle_, node_id_, &is_enabled, &error_code)) {
            ROS_ERROR("VCS_GetEnableState failed. Error: 0x%X", error_code);
            return false;
        }

        if (!is_enabled) {
            ROS_INFO("Enabling motor...");
            if (!VCS_SetEnableState(key_handle_, node_id_, &error_code)) {
                ROS_ERROR("VCS_SetEnableState failed. Error: 0x%X", error_code);
                return false;
            }
        }
        ROS_INFO("Motor prepared.");
        return true;
    }
    
    bool initializeMotor() {
        unsigned int error_code = 0;
        ROS_INFO("Activating Profile Position Mode...");
        if (!VCS_ActivateProfilePositionMode(key_handle_, node_id_, &error_code)) {
             ROS_ERROR("VCS_ActivateProfilePositionMode failed. Error: 0x%X", error_code);
             return false;
        }

        if (!VCS_SetPositionProfile(key_handle_, node_id_, 5000, 90000, 90000, &error_code)) {
            ROS_ERROR("VCS_SetPositionProfile failed. Error: 0x%X", error_code);
            return false;
        }
        
        ROS_INFO("Homing motor to position 0...");
        if (!VCS_MoveToPosition(key_handle_, node_id_, 0, 1, 1, &error_code)) {
            ROS_ERROR("Initial VCS_MoveToPosition (homing) failed. Error: 0x%X", error_code);
            return false;
        }
        ros::Duration(1.0).sleep(); 
        ROS_INFO("Motor initialized and at home position.");
        return true;
    }

    void stateCallback(const std_msgs::String::ConstPtr& msg) {
        if (msg->data == "shutdown") {
            ROS_INFO("Shutdown command received. Closing motor node!");
            ros::shutdown();
        }
    }

    void motorSetFreqCallback(const std_msgs::String::ConstPtr& msg) {
        std::string cmd = msg->data;
        ROS_INFO("Received command: %s", cmd.c_str());

        is_sum_of_sines_ = false;
        is_gain_mode_ = false;
        is_open_loop_ = false;
        is_closed_loop_ = false;

        if (cmd == "sum") {
            is_sum_of_sines_ = true;
            ROS_INFO("Mode switched to: Sum of Sines");
        } else if (cmd == "openloop") {
            is_open_loop_ = true;
            ROS_INFO("Mode switched to: Open Loop");
        } else if (cmd == "closedloop") {
            is_closed_loop_ = true;
            ROS_INFO("Mode switched to: Closed Loop");
        } else if (cmd.rfind("gain:", 0) == 0) {
            is_gain_mode_ = true;
            reafferent_gain_ = std::stod(cmd.substr(5));
            ROS_INFO("Mode switched to: Gain Control with gain = %.2f", reafferent_gain_);
        } else {
            try {
                frequency_ = std::stod(cmd);
                is_sum_of_sines_ = false;
                ROS_INFO("Mode switched to: Single Sine with frequency = %.2f Hz", frequency_);
            } catch (const std::exception& e) {
                ROS_WARN("Could not parse '%s' as a command or frequency.", cmd.c_str());
            }
        }
    }

    void imageCallback(const rectrial::pub_data::ConstPtr& msg)
    {
        if (msg->finish_c == "start") {
            loop_start_time_ = std::chrono::steady_clock::now();
            last_refuge_position_ = 0.0;
            frame_counter_ = 0;
            ROS_INFO("START command received. Resetting position and timers.");
        }
        
        if (msg->finish_c == "end") {
            ROS_INFO("END command received. Homing motor.");
            unsigned int error_code = 0;
            VCS_HaltVelocityMovement(key_handle_, node_id_, &error_code);
            initializeMotor();
            return;
        }

        frame_counter_++;
        double target_position_float = 0.0;

        if (is_gain_mode_ || is_closed_loop_) 
        {
            // <<< FIX 2: Correctly parse the fish position from the "x,y" string in data_e.
            std::stringstream ss(msg->data_e);
            std::string x_str;
            double fish_position_x = 0.0;
            if (std::getline(ss, x_str, ',')) {
                 try {
                    fish_position_x = std::stod(x_str);
                 } catch (const std::exception& e) {
                    ROS_ERROR("Failed to parse x-position from data_e: %s", msg->data_e.c_str());
                 }
            }
            
            target_position_float = (fish_position_x * reafferent_gain_) + last_refuge_position_;
            last_refuge_position_ = target_position_float;
        }
        else if (is_sum_of_sines_)
        {
            double time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - loop_start_time_).count();
            target_position_float = calculateSumOfSines(time_ms);
        }
        else // Default to single sine
        {
            double time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - loop_start_time_).count();
            target_position_float = (a_pos_ / (2.0 * M_PI * frequency_)) * sin(2.0 * M_PI * frequency_ * time_ms / 1000.0);
        }

        unsigned int error_code = 0;
        long target_position = static_cast<long>(target_position_float);
        if (!VCS_MoveToPosition(key_handle_, node_id_, target_position, 1, 1, &error_code)) {
            ROS_ERROR("VCS_MoveToPosition failed. Error: 0x%X", error_code);
        }

        geometry_msgs::PointStamped refuge_msg;
        refuge_msg.header.stamp = msg->image_e.header.stamp;
        refuge_msg.header.frame_id = msg->image_e.header.frame_id;
        refuge_msg.point.x = target_position_float;
        refuge_msg.point.y = 0;
        refuge_msg.point.z = 0;
        refuge_state_pub_.publish(refuge_msg);
    }

    double calculateSumOfSines(double time_ms) {
        static const double freqs[NUM_SINE_FREQUENCIES] = {0.1, 0.15, 0.25, 0.35, 0.55, 0.65, 0.85, 0.95, 1.15, 1.45, 1.55, 1.85, 2.05};
        static const double velPH[NUM_SINE_FREQUENCIES] = {0.2162, 2.5980, 0.0991, 0.2986, 2.9446, 2.5794, 2.8213, 0.0693, 1.4556, 0.3746, 1.0430, 2.9469, 2.6706};
        
        double position = 0.0;
        for (int i = 0; i < NUM_SINE_FREQUENCIES; ++i) {
            position += (a_pos_ / (2.0 * M_PI * freqs[i])) * sin(2.0 * M_PI * freqs[i] * time_ms / 1000.0 + velPH[i] + (M_PI / 2.0));
        }
        return position;
    }
    
    double calculateGainComponent(double fish_pos_x) {
        double pos_x = -reafferent_gain_ * fish_pos_x / REAFFERENT_SCALING_FACTOR;
        pos_x = std::max(-gain_limit_, std::min(gain_limit_, pos_x));
        return a_pos_ * pos_x;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "epos_node");
    MaxonMotorController controller;
    ros::spin();
    return 0;
}
