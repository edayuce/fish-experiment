// data_logger_node.cpp

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

// Message Filters for synchronizing topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Custom message header
#include <rectrial/pub_data.h>

#include <fstream> // For file I/O (std::ofstream)
#include <string>
#include <iomanip> // For std::put_time

namespace DataLogger
{

class DataLoggerNode
{
public:
    DataLoggerNode(ros::NodeHandle& nh);
    ~DataLoggerNode();

private:
    // The single callback for synchronized messages
    void dataCallback(const rectrial::pub_data::ConstPtr& fish_msg, const rectrial::pub_data::ConstPtr& refuge_msg);
    void stateCallback(const std_msgs::String::ConstPtr& msg);
    std::string createLogFile();

    // ROS Communication
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;

    // Message Filters Subscribers
    typedef message_filters::Subscriber<rectrial::pub_data> DataSubscriber;
    DataSubscriber fish_sub_;
    DataSubscriber refuge_sub_;

    // Sync Policy and Synchronizer
    typedef message_filters::sync_policies::ApproximateTime<rectrial::pub_data, rectrial::pub_data> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;

    // Logging
    std::ofstream log_file_;
    std::string log_file_path_;
    ros::Time last_loop_time_;
    bool first_message_ = true;
};

DataLoggerNode::DataLoggerNode(ros::NodeHandle& nh) : nh_(nh)
{
    // --- 1. Create and open the log file ---
    log_file_path_ = createLogFile();
    if (log_file_path_.empty()) {
        ros::shutdown();
        return;
    }
    log_file_.open(log_file_path_);
    if (!log_file_.is_open()) {
        ROS_ERROR("Failed to open log file: %s", log_file_path_.c_str());
        ros::shutdown();
        return;
    }
    // Write the header row to the CSV file
    log_file_ << "fish_pos_x,fish_pos_y,refuge_pos_x,refuge_pos_y,loop_time_ms\n";
    ROS_INFO("Logging data to: %s", log_file_path_.c_str());

    // --- 2. Setup Subscribers using Message Filters ---
    // Initialize subscribers but do not assign a callback yet
    fish_sub_.subscribe(nh_, "/imager_processed", 10);
    refuge_sub_.subscribe(nh_, "/refuge_data", 10);

    // --- 3. Setup Synchronizer ---
    // Policy: Approx time, queue size of 10
    sync_.reset(new Synchronizer(MySyncPolicy(10), fish_sub_, refuge_sub_));
    // Register the combined callback function
    sync_->registerCallback(boost::bind(&DataLoggerNode::dataCallback, this, _1, _2));

    // --- 4. Setup State Subscriber ---
    state_sub_ = nh_.subscribe("/set_state", 10, &DataLoggerNode::stateCallback, this);

    ROS_INFO("Data Logger node initialized. Waiting for synchronized data...");
}

DataLoggerNode::~DataLoggerNode()
{
    if (log_file_.is_open()) {
        log_file_.close();
        ROS_INFO("Log file closed.");
    }
}

// Generates a unique filename for the log file.
std::string DataLoggerNode::createLogFile()
{
    // Get the path to the current package
    std::string package_path = ros::package::getPath("rectrial");
    if (package_path.empty()) {
        ROS_ERROR("Could not find package 'rectrial'. Make sure it's in your ROS_PACKAGE_PATH.");
        return "";
    }

    // Get current time for the filename
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    
    return package_path + "/logs/log_" + ss.str() + ".csv";
}


// This callback is triggered only when a message from BOTH topics with a similar timestamp arrives.
void DataLoggerNode::dataCallback(const rectrial::pub_data::ConstPtr& fish_msg, const rectrial::pub_data::ConstPtr& refuge_msg)
{
    ROS_INFO_ONCE("First synchronized data received. Logging started.");

    // --- Calculate Loop Time ---
    ros::Time current_time = ros::Time::now();
    double loop_duration_ms = 0.0;
    
    if (!first_message_) {
        ros::Duration delta_t = current_time - last_loop_time_;
        loop_duration_ms = delta_t.toSec() * 1000.0;
    }
    last_loop_time_ = current_time;
    first_message_ = false;

    // --- Extract Data ---
    // The data_e field is expected to be a string like "x,y"
    std::string fish_pos = fish_msg->data_e;
    std::string refuge_pos = refuge_msg->data_e;

    // To get separate x and y, we replace the comma with a comma for the CSV format.
    // This is a bit of a shortcut. A more robust solution would parse the
    // string into two numbers and then write them.
    std::replace(fish_pos.begin(), fish_pos.end(), ',', ',');
    std::replace(refuge_pos.begin(), refuge_pos.end(), ',', ',');

    // --- Write to File ---
    if (log_file_.is_open()) {
        log_file_ << fish_pos << ","
                  << refuge_pos << ","
                  << std::fixed << std::setprecision(4) << loop_duration_ms << "\n";             
    }
}

void DataLoggerNode::stateCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "shutdown")
    {
        ROS_INFO("Shutdown command received. Closing Data Logger Node!");
        // The destructor will handle closing the file.
        ros::shutdown();
    }
}

} // namespace DataLogger

int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_writer_node");
    ros::NodeHandle nh;

    // Create a 'logs' directory in your package if it doesn't exist
    std::string package_path = ros::package::getPath("rectrial");
    if (!package_path.empty()) {
        std::string log_dir = package_path + "/logs";
        // This is a simple way to create a directory. A more robust solution
        // would use filesystem libraries.
        const int dir_err = system(("mkdir -p " + log_dir).c_str());
        if (dir_err == -1) {
            ROS_WARN("Could not create log directory.");
        }
    }

    DataLogger::DataLoggerNode node(nh);
    ros::spin();
    return 0;
}
