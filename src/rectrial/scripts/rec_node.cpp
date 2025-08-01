// video_recorder_node.cpp

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

// Custom message for the processed feed
#include <rectrial/pub_data.h>

#include <string>
#include <iomanip> // For std::put_time
#include <chrono>

namespace VideoRecorder
{

class VideoRecorderNode
{
public:
    VideoRecorderNode(ros::NodeHandle& nh);
    ~VideoRecorderNode();

private:
    // Callbacks
    void rawImageCallback(const rectrial::pub_data::ConstPtr& msg);
    void processedImageCallback(const rectrial::pub_data::ConstPtr& msg);

    // Helper functions
    void startRecording();
    void stopRecording();
    std::string getTimestampString();

    // ROS Communication
    ros::NodeHandle nh_;
    ros::Subscriber raw_image_sub_;
    ros::Subscriber processed_image_sub_;
    
    // Video Recording
    cv::VideoWriter raw_video_writer_;
    cv::VideoWriter processed_video_writer_;
    bool is_recording_ = false;
    ros::Time recording_start_time_;
    cv::Size frame_size_;
    bool frame_size_initialized_ = false;
    double fps_ = 30.0; // Default frames per second

    // File paths
    std::string video_directory_;
};

VideoRecorderNode::VideoRecorderNode(ros::NodeHandle& nh) : nh_(nh)
{
    // --- 1. Get Parameters ---
    nh_.param<double>("fps", fps_, 30.0);
    
    // --- 2. Setup Video Directory ---
    std::string package_path = ros::package::getPath("rectrial");
    if (package_path.empty()) {
        ROS_ERROR("Could not find package 'rectrial'. Shutting down.");
        ros::shutdown();
        return;
    }
    video_directory_ = package_path + "/videos";
    const int dir_err = system(("mkdir -p " + video_directory_).c_str());
    if (dir_err == -1) {
        ROS_WARN("Could not create video directory at: %s", video_directory_.c_str());
    } else {
        ROS_INFO("Saving videos to: %s", video_directory_.c_str());
    }

    // --- 3. Setup Subscribers ---
    //image_transport::ImageTransport it(nh_);
    // Subscribe to the raw camera feed
    raw_image_sub_ = nh_.subscribe("/imager_c", 5, &VideoRecorderNode::rawImageCallback, this);
    // Subscribe to the final processed feed from the controller, which contains the start/end signals
    processed_image_sub_ = nh_.subscribe("/imager", 5, &VideoRecorderNode::processedImageCallback, this);

    ROS_INFO("Video Recorder node initialized. Waiting for 'start' signal from controller.");
}

VideoRecorderNode::~VideoRecorderNode()
{
    // Ensure videos are properly closed on shutdown
    if (is_recording_) {
        stopRecording();
    }
}

// Callback for the raw, unprocessed video feed
void VideoRecorderNode::rawImageCallback(const rectrial::pub_data::ConstPtr& msg)
{
    // Initialize frame size from the first message
    if (!frame_size_initialized_) {
        frame_size_ = cv::Size(msg->image_p.width, msg->image_p.height);
        frame_size_initialized_ = true;
        ROS_INFO("Video frame size set to: %d x %d", frame_size_.width, frame_size_.height);
    }

    if (is_recording_) {
        try {
            // Convert to BGR8 for color video saving
            cv::Mat frame = cv_bridge::toCvShare(msg-> image_p, msg, "bgr8")->image;
            if (raw_video_writer_.isOpened()) {
                raw_video_writer_.write(frame);
            }
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (raw): %s", e.what());
        }
    }
}

// Callback for the final, processed video feed from the controller
void VideoRecorderNode::processedImageCallback(const rectrial::pub_data::ConstPtr& msg)
{
    // --- Check for start/stop signals from the controller ---
    if (msg->finish_c == "start" && !is_recording_) {
        startRecording();
    } 
    else if (msg->finish_c == "end" && is_recording_) {
        ROS_INFO("Experiment 'end' signal received. Stopping recording.");
        stopRecording();
        return; // Stop processing to prevent writing one last frame
    }

    if (is_recording_) {
        // Automatically stop after 1 minute
        if ((ros::Time::now() - recording_start_time_).toSec() >= 60.0) {
            ROS_INFO("1 minute recording limit reached. Stopping automatically.");
            stopRecording();
            return; // Stop processing this frame
        }

        try {
            // The image is inside the custom message. Convert to BGR8 for color video.
            cv::Mat frame = cv_bridge::toCvShare(msg->image_e, msg, "bgr8")->image;
            if (processed_video_writer_.isOpened()) {
                processed_video_writer_.write(frame);
            }
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (processed): %s", e.what());
        }
    }
}


void VideoRecorderNode::startRecording()
{
    if (!frame_size_initialized_) {
        ROS_WARN("Cannot start recording, frame size not yet initialized. Waiting for first image...");
        return;
    }

    is_recording_ = true;
    recording_start_time_ = ros::Time::now();
    std::string timestamp = getTimestampString();

    std::string raw_path = video_directory_ + "/raw_" + timestamp + ".avi";
    std::string processed_path = video_directory_ + "/processed_" + timestamp + ".avi";

    // Using MJPG codec for wide compatibility
    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

    raw_video_writer_.open(raw_path, codec, fps_, frame_size_, true);
    processed_video_writer_.open(processed_path, codec, fps_, frame_size_, true);

    if (!raw_video_writer_.isOpened() || !processed_video_writer_.isOpened()) {
        ROS_ERROR("Failed to open one or both video writers! Check permissions and codecs.");
        is_recording_ = false;
        return;
    }
    
    ROS_INFO("Recording started. Duration: up to 60 seconds.");
    ROS_INFO(" -> Raw video: %s", raw_path.c_str());
    ROS_INFO(" -> Processed video: %s", processed_path.c_str());
}

void VideoRecorderNode::stopRecording()
{
    is_recording_ = false;
    if (raw_video_writer_.isOpened()) {
        raw_video_writer_.release();
    }
    if (processed_video_writer_.isOpened()) {
        processed_video_writer_.release();
    }
    ROS_INFO("Recording stopped.");
}

std::string VideoRecorderNode::getTimestampString()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

} // namespace VideoRecorder

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_recorder_node");
    ros::NodeHandle nh("~");
    VideoRecorder::VideoRecorderNode node(nh);
    ros::spin();
    return 0;
}
