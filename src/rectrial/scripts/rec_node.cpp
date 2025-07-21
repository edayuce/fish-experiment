#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

// Custom message - ensure this path is correct
#include <rectrial/pub_data.h>

#include <string>
#include <fstream>
#include <vector>
#include <iostream>

// Use a namespace for better organization
namespace ExperimentRecorder
{

class RecorderNode
{
public:
    RecorderNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~RecorderNode(); // Destructor to ensure files are closed

private:
    // A single, unified callback
    void messageCallback(const rectrial::pub_data::ConstPtr& msg);
    void stateCallback(const std_msgs::String::ConstPtr& msg);

    // Helper function to manage file opening/closing
    void startRecording(const std::string& video_name, const cv::Size& frame_size);
    void stopRecording();

    // Member Variables
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_; // Private node handle for parameters

    ros::Subscriber data_sub_;
    ros::Subscriber state_sub_;

    // Configuration loaded from parameter server
    std::string output_video_path_;
    std::string output_csv_path_;
    bool record_csv_;

    // State
    cv::VideoWriter video_writer_;
    std::ofstream csv_file_;
    std::string current_recording_name_;
    bool is_recording_;
};

RecorderNode::RecorderNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), is_recording_(false)
{
    // --- 1. Load parameters from the ROS parameter server ---
    std::string home_path = getenv("HOME");
    pnh_.param<std::string>("output_video_path", output_video_path_, home_path + "/ros_recordings/videos");
    pnh_.param<std::string>("output_csv_path", output_csv_path_, home_path + "/ros_recordings/csv");
    pnh_.param<bool>("record_csv", record_csv_, true);

    std::string topic_to_subscribe;
    pnh_.param<std::string>("topic", topic_to_subscribe, "/imager");

    ROS_INFO_STREAM("Recorder configured for topic: " << topic_to_subscribe);
    ROS_INFO_STREAM("Video output path: " << output_video_path_);
    ROS_INFO_STREAM("CSV output path: " << output_csv_path_);

    // --- 2. Setup Subscribers ---
    // The queue size is set to a more reasonable value.
    data_sub_ = nh_.subscribe(topic_to_subscribe, 50, &RecorderNode::messageCallback, this);
    state_sub_ = nh_.subscribe("/set_state", 10, &RecorderNode::stateCallback, this);
}

// The destructor is crucial for ensuring files are closed properly if the node is killed.
RecorderNode::~RecorderNode()
{
    if (is_recording_)
    {
        ROS_WARN("Node shutting down while recording was active. Attempting to close files.");
        stopRecording();
    }
}

void RecorderNode::messageCallback(const rectrial::pub_data::ConstPtr& msg)
{
    // --- State-change logic ---
    if (msg->finish_c == "start")
    {
        if (is_recording_) {
            // If we get a "start" while already recording, close the old files first.
            ROS_WARN("Received 'start' for a new experiment while another was running. Closing previous files.");
            stopRecording();
        }
        
        try {
            cv::Mat temp_frame = cv_bridge::toCvShare(msg->image_e, msg, "mono8")->image;
            startRecording(msg->video_name_p, temp_frame.size());
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception on start: %s. Recording aborted.", e.what());
            return;
        }
    }

    if (!is_recording_) {
        // If we get a message but haven't been told to start, ignore it.
        ROS_WARN_THROTTLE(5.0, "Receiving data but not in recording state. Waiting for a 'start' message.");
        return;
    }

    // --- Data writing logic ---
    try
    {
        cv::Mat frame_mono = cv_bridge::toCvShare(msg->image_e, msg, "mono8")->image;
        cv::Mat frame_bgr;
        // VideoWriter needs a 3-channel image
        cv::cvtColor(frame_mono, frame_bgr, cv::COLOR_GRAY2BGR);

        video_writer_.write(frame_bgr);

        if (record_csv_ && csv_file_.is_open())
        {
            csv_file_ << msg->data_e << "\n";
        }
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception during recording: %s", e.what());
    }
    
    // --- Stop logic ---
    if (msg->finish_c == "end")
    {
        ROS_INFO("Received 'end' message. Finalizing recording.");
        stopRecording();
    }
}

void RecorderNode::startRecording(const std::string& video_name, const cv::Size& frame_size)
{
    // Sanitize the name and create full paths
    current_recording_name_ = video_name;
    // Remove potential trailing newline characters from the name
    current_recording_name_.erase(std::remove(current_recording_name_.begin(), current_recording_name_.end(), '\n'), current_recording_name_.end());

    std::string video_filepath = output_video_path_ + "/" + current_recording_name_ + ".mp4";
    std::string csv_filepath = output_csv_path_ + "/" + current_recording_name_ + ".csv";

    ROS_INFO("Starting new recording: %s", current_recording_name_.c_str());

    // Open Video Writer
    // Using 'mp4v' is often more compatible than 'DIVX'
    video_writer_.open(video_filepath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 25, frame_size, true);
    if (!video_writer_.isOpened())
    {
        ROS_FATAL("Failed to open video writer for: %s", video_filepath.c_str());
        ros::shutdown();
        return;
    }

    // Open CSV file if in the correct mode
    if (record_csv_)
    {
        csv_file_.open(csv_filepath);
        if (!csv_file_.is_open())
        {
            ROS_ERROR("Failed to open CSV file for: %s", csv_filepath.c_str());
            // We might continue with just video recording, so this is not fatal.
        }
    }
    
    is_recording_ = true;
}

void RecorderNode::stopRecording()
{
    if (video_writer_.isOpened())
    {
        video_writer_.release();
        ROS_INFO("Video file for '%s' closed.", current_recording_name_.c_str());
    }
    if (csv_file_.is_open())
    {
        csv_file_.close();
        ROS_INFO("CSV file for '%s' closed.", current_recording_name_.c_str());
    }
    is_recording_ = false;
    current_recording_name_ = "";
}

void RecorderNode::stateCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "shutdown")
    {
        ROS_INFO("Shutdown command received. Closing Recorder Node!");
        ros::shutdown(); // The destructor will handle closing files.
    }
}

} // namespace ExperimentRecorder

int main(int argc, char** argv)
{
    ros::init(argc, argv, "recorder_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // Private node handle for parameters

    ExperimentRecorder::RecorderNode node(nh, pnh);

    ros::spin();

    return 0;
}

