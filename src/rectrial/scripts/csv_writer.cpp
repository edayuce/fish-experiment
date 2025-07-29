#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <string>
#include <chrono>

// For synchronizing topic subscriptions
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Message types we need to subscribe to
#include <rectrial/pub_data.h> // For fish position from the tracker
#include <std_msgs/Float64.h>   // For refuge position from the motor node
#include <geometry_msgs/PointStamped.h>

class CSVWriterNode
{
public:
    CSVWriterNode(ros::NodeHandle& nh) : nh_(nh), first_message_(true)
    {
        // Get the output file path from a ROS parameter for flexibility
        std::string default_path = ros::package::getPath("rectrial") + "/data/experiment_log.csv";
        nh_.param<std::string>("output_file_path", output_file_path_, default_path);

        // Open the file for writing
        file_handle_.open(output_file_path_, std::ios::out | std::ios::trunc);
        if (!file_handle_.is_open())
        {
            ROS_ERROR("Could not open file for writing: %s", output_file_path_.c_str());
            ros::shutdown();
            return;
        }

        // Write the header row to the CSV file
        file_handle_ << "fish_position,refuge_position,iteration_time_ms\n";
        ROS_INFO("Logging data to: %s", output_file_path_.c_str());

        // Initialize subscribers using message_filters
        fish_sub_.subscribe(nh_, "/imager_processed", 1);
        refuge_sub_.subscribe(nh_, "/refuge_state", 1);

        // Use an ApproximateTime synchronizer to get paired messages from the two topics
        sync_.reset(new Sync(MySyncPolicy(100), fish_sub_, refuge_sub_));
        sync_->setMaxIntervalDuration(ros::Duration(0.04));
        sync_->registerCallback(boost::bind(&CSVWriterNode::dataCallback, this, _1, _2));
        
        // Initialize the timer
        last_callback_time_ = std::chrono::steady_clock::now();
    }

    ~CSVWriterNode()
    {
        if (file_handle_.is_open())
        {
            file_handle_.close();
            ROS_INFO("Closed log file.");
        }
    }

private:
    // This callback is triggered when a synchronized pair of messages arrives
    void dataCallback(const rectrial::pub_data::ConstPtr& fish_msg, const geometry_msgs::PointStamped::ConstPtr& refuge_msg)
    {
        ROS_INFO("Datacallback triggired");

        auto now = std::chrono::steady_clock::now();
        long long iteration_time_ms = 0;

        // Don't calculate a duration for the very first message
        if (!first_message_)
        {
            iteration_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_callback_time_).count();
        }
        
        last_callback_time_ = now;
        first_message_ = false;

        // Extract the data from the messages
        double fish_position = fish_msg->target_pos_x;
        double refuge_position = refuge_msg->point.x;

        // Write the data as a new row in the CSV file
        file_handle_ << fish_position << "," << refuge_position << "," << iteration_time_ms << "\n";
    }

    // Define the synchronizer policy
    typedef message_filters::sync_policies::ApproximateTime<rectrial::pub_data, geometry_msgs::PointStamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;

    ros::NodeHandle nh_;
    std::ofstream file_handle_;
    std::string output_file_path_;
    
    // Subscribers and Synchronizer
    message_filters::Subscriber<rectrial::pub_data> fish_sub_;
    message_filters::Subscriber<geometry_msgs::PointStamped> refuge_sub_;
    boost::shared_ptr<Sync> sync_;

    // Timing variables
    std::chrono::steady_clock::time_point last_callback_time_;
    bool first_message_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_writer_node");
    ros::NodeHandle nh("~"); // Private node handle for parameters

    CSVWriterNode writer(nh);

    ros::spin();

    return 0;
}