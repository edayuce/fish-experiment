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
    void rawThrottledCallback(const rectrial::pub_data::ConstPtr& msg);
    void processedCallback(const rectrial::pub_data::ConstPtr& msg);
    
    void startRecording();
    void stopRecording();
    std::string getTimestampString();
    
    ros::NodeHandle nh_;
    ros::Subscriber raw_sub_;
    ros::Subscriber processed_sub_;
    
    cv::VideoWriter raw_video_writer_;
    cv::VideoWriter processed_video_writer_;
    
    // <<< FIX: New flag to manage the race condition
    bool recording_requested_ = false;
    bool is_recording_ = false;
    
    ros::Time recording_start_time_;
    cv::Size frame_size_;
    bool frame_size_initialized_ = false;
    double fps_ = 25.0;
    std::string video_directory_;
};

VideoRecorderNode::VideoRecorderNode(ros::NodeHandle& nh) : nh_(nh)
{
    nh_.param<double>("fps", fps_, 25.0);
    
    std::string package_path = ros::package::getPath("rectrial");
    video_directory_ = package_path + "/videos";
    system(("mkdir -p " + video_directory_).c_str());

    raw_sub_ = nh_.subscribe("/imager_c_throttled", 5, &VideoRecorderNode::rawThrottledCallback, this);
    processed_sub_ = nh_.subscribe("/imager", 5, &VideoRecorderNode::processedCallback, this);

    ROS_INFO("Video Recorder node initialized. Waiting for 'start' signal.");
}

VideoRecorderNode::~VideoRecorderNode()
{
    if (is_recording_) {
        stopRecording();
    }
}

// Callback for the throttled raw video feed
void VideoRecorderNode::rawThrottledCallback(const rectrial::pub_data::ConstPtr& msg)
{
    // This is the first thing that happens on a new frame
    if (!frame_size_initialized_) {
        frame_size_ = cv::Size(msg->image_p.width, msg->image_p.height);
        frame_size_initialized_ = true;
        ROS_INFO("Video frame size set to: %d x %d", frame_size_.width, frame_size_.height);
    }

    // <<< FIX: Check if a recording was requested and if we have the frame size
    if (recording_requested_ && !is_recording_ && frame_size_initialized_) {
        startRecording();
    }

    if (is_recording_) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg->image_p, msg, "bgr8")->image;
            if (raw_video_writer_.isOpened()) {
                raw_video_writer_.write(frame);
            }
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception (raw): %s", e.what());
        }
    }
}

// Callback for the final, processed video feed
void VideoRecorderNode::processedCallback(const rectrial::pub_data::ConstPtr& msg)
{
    // <<< FIX: This callback now only sets a flag or stops the recording.
    if (msg->finish_c == "start" && !is_recording_) {
        recording_requested_ = true;
    } 
    else if (msg->finish_c == "end" && is_recording_) {
        stopRecording();
    }

    if (is_recording_) {
        if ((ros::Time::now() - recording_start_time_).toSec() >= 60.0) {
            stopRecording();
            return;
        }

        try {
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
    ROS_INFO("Start signal and first frame received. Initializing video writers...");

    is_recording_ = true;
    recording_requested_ = false; // Reset the request flag
    recording_start_time_ = ros::Time::now();
    std::string timestamp = getTimestampString();

    std::string raw_path = video_directory_ + "/raw_" + timestamp + ".mp4";
    std::string processed_path = video_directory_ + "/processed_" + timestamp + ".mp4";

    int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');

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
    if (!is_recording_) return;
    
    is_recording_ = false;
    recording_requested_ = false;
    
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