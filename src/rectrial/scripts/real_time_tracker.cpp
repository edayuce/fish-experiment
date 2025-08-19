#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
//#include <opencv2/highgui/highgui.hpp>


// Include your custom message header
#include <rectrial/pub_data.h>
#include <cmath>

// Use a namespace to keep the code organized
namespace OnlineTracker
{

// A clear state machine is much better than using integers or multiple booleans.
enum class NodeState
{
    WAITING_FOR_FIRST_FRAME,
    SELECTING_ROI, // ROI = Region of Interest
    TRACKING
};

// A simple struct to pass data to the mouse callback function.
struct MouseParams
{
    bool clicked = false;
    cv::Point point;
};

// Standard OpenCV mouse callback function.
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        auto* params = static_cast<MouseParams*>(userdata);
        params->clicked = true;
        params->point = cv::Point(x, y);
    }
}

class OnlineTrackerNode
{
public:
    OnlineTrackerNode(ros::NodeHandle& nh);
    ~OnlineTrackerNode();

private:
    // Callbacks
    void imageCallback(const rectrial::pub_data::ConstPtr& msg);
    void stateCallback(const std_msgs::String::ConstPtr& msg);

    // Core logic for processing each frame
    void processFrame(const cv::Mat& frame, const std_msgs::Header& header);
    
    // Initialization logic that runs only once
    void initializeTracker(const cv::Mat& frame);

    // ROS Communication
    ros::NodeHandle nh_;
    ros::Subscriber data_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher processed_pub_;

    // State Management
    NodeState state_;
    cv::Ptr<cv::Tracker> tracker_;
    cv::Rect2d tracking_bbox_;

    // Configuration
    int BBOX_WIDTH_ = 50;
    int BBOX_HEIGHT_ = 50;
    const std::string SELECTION_WINDOW_NAME_ = "Select Fish";
    const std::string TRACKING_WINDOW_NAME_ = "Tracking";
};

OnlineTrackerNode::OnlineTrackerNode(ros::NodeHandle& nh)
    : nh_(nh), state_(NodeState::WAITING_FOR_FIRST_FRAME)
{
    // --- Get ROS Parameters ---
    nh_.param<int>("bbox_width", BBOX_WIDTH_, 50);
    nh_.param<int>("bbox_height", BBOX_HEIGHT_, 50);

    // --- Setup Subscribers and Publishers ---
    // For real-time tracking, the subscriber queue size should be small
    // to ensure we are always processing the latest available frame.
    data_sub_ = nh_.subscribe<rectrial::pub_data>("/imager_c", 10, &OnlineTrackerNode::imageCallback, this);
    state_sub_ = nh_.subscribe("/set_state", 10, &OnlineTrackerNode::stateCallback, this);
    
    // This publisher will send out the processed image and tracking data.
    processed_pub_ = nh_.advertise<rectrial::pub_data>("/imager_processed", 10);

    ROS_INFO("Online Tracker node initialized. Waiting for the first image...");
}

OnlineTrackerNode::~OnlineTrackerNode()
{
    // Ensure all OpenCV windows are closed when the node shuts down.
    cv::destroyAllWindows();
}

void OnlineTrackerNode::imageCallback(const rectrial::pub_data::ConstPtr& msg)
{
    try
    {
        // Convert the ROS image message to an OpenCV Mat.
        cv::Mat frame = cv_bridge::toCvShare(msg->image_p, msg, "mono8")->image;
        if (frame.empty())
        {
            ROS_WARN("Received an empty image frame.");
            return;
        }
        // Process the frame using our state machine.
        processFrame(frame, msg->image_p.header);
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void OnlineTrackerNode::processFrame(const cv::Mat& frame, const std_msgs::Header& header) {
    switch (state_) {
        case NodeState::WAITING_FOR_FIRST_FRAME:
            initializeTracker(frame);
            break;

        case NodeState::SELECTING_ROI:
            break;

        case NodeState::TRACKING: {
            bool ok = tracker_->update(frame, tracking_bbox_);

            // Convert the grayscale frame to a color frame for drawing
            cv::Mat display_frame;
            cv::cvtColor(frame, display_frame, cv::COLOR_GRAY2BGR);

            if (ok) {
                // Draw a white box for the fish
                cv::rectangle(display_frame, tracking_bbox_, cv::Scalar(255, 255, 255), 2);
            } else {
                cv::putText(display_frame, "Tracking Failure", cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
            }
            
            // This rounds to the nearest pixel, making the result stable
            cv::Point center(std::round(tracking_bbox_.x + tracking_bbox_.width / 2), std::round(tracking_bbox_.y + tracking_bbox_.height / 2));
            // Draw a black dot in the center
            cv::circle(display_frame, center, 4, cv::Scalar(0, 0, 0), -1);

            rectrial::pub_data processed_msg;
            // Publish the image as "bgr8" (color)
            processed_msg.image_e = *cv_bridge::CvImage(header, "bgr8", display_frame).toImageMsg();
            processed_msg.data_e = std::to_string(center.x) + "," + std::to_string(center.y);
            processed_pub_.publish(processed_msg);

            // Display the result in this node's own window
            cv::imshow(TRACKING_WINDOW_NAME_, display_frame);
            cv::waitKey(1);
            break;
        }
    }
}

void OnlineTrackerNode::initializeTracker(const cv::Mat& frame) {
    state_ = NodeState::SELECTING_ROI;
    cv::namedWindow(SELECTION_WINDOW_NAME_, cv::WINDOW_NORMAL | cv::WINDOW_GUI_EXPANDED);
    MouseParams mouse_params;
    cv::setMouseCallback(SELECTION_WINDOW_NAME_, onMouse, &mouse_params);

    ROS_INFO("Please select the fish to track by clicking on it.");
    
    while (!mouse_params.clicked && ros::ok()) {
        cv::imshow(SELECTION_WINDOW_NAME_, frame);
        if (cv::waitKey(10) == 27) { ros::shutdown(); return; }
    }
    
    tracking_bbox_ = cv::Rect2d(mouse_params.point.x - (BBOX_WIDTH_ / 2.0),
                                  mouse_params.point.y - (BBOX_HEIGHT_ / 2.0),
                                  BBOX_WIDTH_, BBOX_HEIGHT_);

    cv::TrackerCSRT::Params params;
    params.psr_threshold = 0.2;
    params.use_segmentation = false;   // segmentation can cause expansion
    params.scale_lr = 0.0;             // no learning for scale
    params.number_of_scales = 1;        // only evaluate single scale


    tracker_ = cv::TrackerCSRT::create();
    tracker_->init(frame, tracking_bbox_);

    cv::destroyWindow(SELECTION_WINDOW_NAME_);
    ROS_INFO("Tracker initialized. Starting continuous tracking.");
    state_ = NodeState::TRACKING;
    cv::namedWindow(TRACKING_WINDOW_NAME_);
}

void OnlineTrackerNode::stateCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "shutdown")
    {
        ROS_INFO("Shutdown command received. Closing Main Node!");
        ros::shutdown();
    }
}

} // namespace OnlineTracker

int main(int argc, char** argv)
{

    ros::init(argc, argv, "real_time_tracker_node");
    ros::NodeHandle nh("~"); // Use private node handle for parameters

    // Optional: Check for CUDA devices, though CSRT tracker is CPU-based.
    if (cv::cuda::getCudaEnabledDeviceCount() > 0)
    {
        cv::cuda::setDevice(0);
        cv::cuda::printCudaDeviceInfo(cv::cuda::getDevice());
    }

    OnlineTracker::OnlineTrackerNode node(nh);

    ros::spin();

    return 0;
}
