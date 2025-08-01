// refuge_tracker.cpp

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Include your custom message header
#include <rectrial/pub_data.h>

// Use a namespace to keep the code organized
namespace RefugeTracker
{

// State machine for the node's operation
enum class NodeState
{
    WAITING_FOR_FIRST_FRAME,
    SELECTING_ROI, // ROI = Region of Interest
    PUBLISHING_DATA
};

// A simple struct to hold data for the mouse callback.
// This is now identical to the one in the online_tracker.
struct MouseParams
{
    bool clicked = false;
    cv::Point point;
};

// Standard OpenCV mouse callback function for single-click selection.
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        auto* params = static_cast<MouseParams*>(userdata);
        params->clicked = true;
        params->point = cv::Point(x, y);
    }
}


class RefugeTrackerNode
{
public:
    RefugeTrackerNode(ros::NodeHandle& nh);
    ~RefugeTrackerNode();

private:
    // Callbacks
    void imageCallback(const rectrial::pub_data::ConstPtr& msg);
    void stateCallback(const std_msgs::String::ConstPtr& msg);

    // Core logic
    void processFrame(const cv::Mat& frame, const std_msgs::Header& header);
    void initializeRefuge(const cv::Mat& frame);

    // ROS Communication
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher refuge_data_pub_;

    // State Management
    NodeState state_;
    cv::Rect2d refuge_bbox_; // The selected bounding box for the refuge

    // Configuration
    int BBOX_WIDTH_ = 50;
    int BBOX_HEIGHT_ = 50;
    const std::string SELECTION_WINDOW_NAME_ = "Select Center of Refuge";
    const std::string DISPLAY_WINDOW_NAME_ = "Refuge Location";
};

RefugeTrackerNode::RefugeTrackerNode(ros::NodeHandle& nh)
    : nh_(nh), state_(NodeState::WAITING_FOR_FIRST_FRAME)
{
    // --- Get ROS Parameters for the refuge size ---
    nh_.param<int>("refuge_width", BBOX_WIDTH_, 50);
    nh_.param<int>("refuge_height", BBOX_HEIGHT_, 50);

    // --- Setup Subscribers and Publishers ---
    //ros::NodeHandle it(nh_);
    image_sub_ = nh_.subscribe<rectrial::pub_data>("/imager_c", 1, &RefugeTrackerNode::imageCallback, this);
    state_sub_ = nh_.subscribe("/set_state", 10, &RefugeTrackerNode::stateCallback, this);
    
    refuge_data_pub_ = nh_.advertise<rectrial::pub_data>("/refuge_data", 10);

    ROS_INFO("Refuge Tracker node initialized. Waiting for the first image...");
}

RefugeTrackerNode::~RefugeTrackerNode()
{
    cv::destroyAllWindows();
}

void RefugeTrackerNode::imageCallback(const rectrial::pub_data::ConstPtr& msg)
{
    try
    {
        cv::Mat frame = cv_bridge::toCvShare(msg-> image_p, msg, "mono8")->image;
        if (frame.empty())
        {
            ROS_WARN("Received an empty image frame.");
            return;
        }
        processFrame(frame, msg->image_p.header);
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void RefugeTrackerNode::processFrame(const cv::Mat& frame, const std_msgs::Header& header)
{
    switch (state_)
    {
        case NodeState::WAITING_FOR_FIRST_FRAME:
            ROS_INFO("First frame received. Proceeding to refuge selection.");
            initializeRefuge(frame);
            break;

        case NodeState::SELECTING_ROI:
            ROS_WARN("Still in selection state during callback, this should not happen.");
            break;

        case NodeState::PUBLISHING_DATA:
        {
            cv::Mat display_frame = frame.clone();

            cv::rectangle(display_frame, refuge_bbox_, cv::Scalar(0, 255, 0), 2);
            
            cv::Point center(refuge_bbox_.x + refuge_bbox_.width / 2, refuge_bbox_.y + refuge_bbox_.height / 2);
            cv::circle(display_frame, center, 4, cv::Scalar(0, 255, 0), -1);
            cv::putText(display_frame, "Refuge", cv::Point(refuge_bbox_.x, refuge_bbox_.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 2);

            rectrial::pub_data refuge_msg;
            refuge_msg.image_e = *cv_bridge::CvImage(header, "mono8", display_frame).toImageMsg();
            refuge_msg.data_e = std::to_string(center.x) + "," + std::to_string(center.y);
            refuge_data_pub_.publish(refuge_msg);

            cv::imshow(DISPLAY_WINDOW_NAME_, display_frame);
            cv::waitKey(1);
            break;
        }
    }
}

void RefugeTrackerNode::initializeRefuge(const cv::Mat& frame)
{
    state_ = NodeState::SELECTING_ROI;

    cv::namedWindow(SELECTION_WINDOW_NAME_, cv::WINDOW_AUTOSIZE);
    MouseParams mouse_params;
    cv::setMouseCallback(SELECTION_WINDOW_NAME_, onMouse, &mouse_params);

    ROS_INFO("Please select the refuge to track by clicking on its center.");
    
    // This loop waits for the user to click.
    while (!mouse_params.clicked && ros::ok())
    {
        cv::imshow(SELECTION_WINDOW_NAME_, frame);
        if (cv::waitKey(10) == 27) // ESC key
        {
            ROS_INFO("Refuge selection cancelled by user. Shutting down.");
            ros::shutdown();
            return;
        }
    }
    
    // The user has clicked. Create a rectangle centered on the click point.
    refuge_bbox_ = cv::Rect2d(mouse_params.point.x - (BBOX_WIDTH_ / 2.0),
                                mouse_params.point.y - (BBOX_HEIGHT_ / 2.0),
                                BBOX_WIDTH_,
                                BBOX_HEIGHT_);

    cv::destroyWindow(SELECTION_WINDOW_NAME_);
    ROS_INFO("Refuge area selected. Starting continuous publishing.");
    state_ = NodeState::PUBLISHING_DATA;
    cv::namedWindow(DISPLAY_WINDOW_NAME_);
}

void RefugeTrackerNode::stateCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "shutdown")
    {
        ROS_INFO("Shutdown command received. Closing Refuge Tracker Node!");
        ros::shutdown();
    }
}

} // namespace RefugeTracker

int main(int argc, char** argv)
{
    ros::init(argc, argv, "refuge_tracker_node");
    ros::NodeHandle nh("~"); // Use private node handle for parameters

    RefugeTracker::RefugeTrackerNode node(nh);

    ros::spin();

    return 0;
}
