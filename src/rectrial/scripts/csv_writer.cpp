// updated data_logger_node.cpp

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <rectrial/pub_data.h> // Custom message header

#include <fstream>
#include <string>
#include <iomanip>
#include <algorithm>
#include <sstream>

namespace DataLogger
{

class DataLoggerNode
{
public:
    DataLoggerNode(ros::NodeHandle& nh);
    ~DataLoggerNode();

private:
    // We now have two separate callbacks
    void fishCallback(const rectrial::pub_data::ConstPtr& msg);
    void refugeCallback(const rectrial::pub_data::ConstPtr& msg);
    
    void stateCallback(const std_msgs::String::ConstPtr& msg);
    void tryLogData(); // The new function to handle logging
    std::string createLogFile();

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber fish_sub_;
    ros::Subscriber static_refuge_sub_;

    // Member variables to store the latest message from each topic
    rectrial::pub_data::ConstPtr last_fish_msg_;
    rectrial::pub_data::ConstPtr last_refuge_msg_;
    bool has_new_fish_data_ = false;
    bool has_new_refuge_data_ = false;

    // Logging
    std::ofstream log_file_;
    std::string log_file_path_;
    ros::Time last_loop_time_;
    bool first_message_ = true;
};

DataLoggerNode::DataLoggerNode(ros::NodeHandle& nh) : nh_(nh)
{
    log_file_path_ = createLogFile();
    if (log_file_path_.empty()) { ros::shutdown(); return; }
    log_file_.open(log_file_path_);
    if (!log_file_.is_open()) { ROS_ERROR("Failed to open log file: %s", log_file_path_.c_str()); ros::shutdown(); return; }
    
    // <<< FIX: Add the new columns to the CSV header
    log_file_ << "raw_fish_pos_x,filtered_fish_pos_x,refuge_pos_x,loop_time_ms\n";
    ROS_INFO("Logging data to: %s", log_file_path_.c_str());

    // Initialize two separate, standard ROS subscribers
    fish_sub_ = nh_.subscribe("/imager", 10, &DataLoggerNode::fishCallback, this); // Listen to the final controller output
    static_refuge_sub_ = nh_.subscribe("/refuge_data", 10, &DataLoggerNode::refugeCallback, this);
    state_sub_ = nh_.subscribe("/set_state", 10, &DataLoggerNode::stateCallback, this);
    
    ROS_INFO("Data Logger node initialized. Waiting for data from both topics...");
}

DataLoggerNode::~DataLoggerNode() { if (log_file_.is_open()) { log_file_.close(); ROS_INFO("Log file closed."); } }

void DataLoggerNode::fishCallback(const rectrial::pub_data::ConstPtr& msg)
{
    last_fish_msg_ = msg;
    has_new_fish_data_ = true;
    tryLogData(); // Attempt to log after receiving fish data
}

void DataLoggerNode::refugeCallback(const rectrial::pub_data::ConstPtr& msg)
{
    last_refuge_msg_ = msg;
    has_new_refuge_data_ = true;
    tryLogData(); // Attempt to log after receiving refuge data
}

void DataLoggerNode::tryLogData()
{
    if (!has_new_fish_data_ || !has_new_refuge_data_) {
        return;
    }

    ROS_INFO_ONCE("First data pair received. Logging started.");

    ros::Time current_time = ros::Time::now();
    double loop_duration_ms = 0.0;
    if (!first_message_) {
        ros::Duration delta_t = current_time - last_loop_time_;
        loop_duration_ms = delta_t.toSec() * 1000.0;
    }
    last_loop_time_ = current_time;
    first_message_ = false;

    // <<< FIX: Parse the new, more complex data string
    std::string fish_data_str = last_fish_msg_->data_e;
    std::string refuge_data_str = last_refuge_msg_->data_e;
    std::stringstream ss(fish_data_str);
    std::string final_pos_part, raw_pos_part, filtered_pos_part;

    std::getline(ss, final_pos_part, ';');
    std::getline(ss, raw_pos_part, ';');
    std::getline(ss, filtered_pos_part, ';');

    std::stringstream ss_r(refuge_data_str);
    std::string refuge_data_x;
    std::getline(ss_r, refuge_data_x, ',');

    std::replace(final_pos_part.begin(), final_pos_part.end(), ',', ',');

    if (log_file_.is_open()) {
        log_file_ << raw_pos_part << ","         // raw_fish_x
                  << filtered_pos_part << ","    // filtered_fish_x
                  << refuge_data_x << ","
                  << std::fixed << std::setprecision(4) << loop_duration_ms << "\n";   
    }

    has_new_fish_data_ = false;
    has_new_refuge_data_ = false;
}

std::string DataLoggerNode::createLogFile()
{
    std::string package_path = ros::package::getPath("rectrial");
    if (package_path.empty()) {
        ROS_ERROR("Could not find package 'rectrial'.");
        return "";
    }
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    
    return package_path + "/logs/log_" + ss.str() + ".csv";
}

void DataLoggerNode::stateCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "shutdown") {
        ROS_INFO("Shutdown command received. Closing Data Logger Node!");
        ros::shutdown();
    }
}

} // namespace DataLogger

int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_writer_node");
    ros::NodeHandle nh;

    std::string package_path = ros::package::getPath("rectrial");
    if (!package_path.empty()) {
        std::string log_dir = package_path + "/logs";
        system(("mkdir -p " + log_dir).c_str());
    }

    DataLogger::DataLoggerNode node(nh);
    ros::spin();
    return 0;
}
