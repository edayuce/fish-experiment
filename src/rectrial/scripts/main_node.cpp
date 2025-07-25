#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include <ros/package.h> // For finding package paths
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>


// Custom message - ensure this path is correct
#include <rectrial/pub_data.h>

#include "GLWindow.h" // Assuming GLWindow.h is in the include path

#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <chrono>
#include <iomanip> // For formatting time
#include <mutex>

// Use a namespace to avoid polluting the global scope
namespace FishTracker
{

// Use an enum for clearer state management
enum class NodeState
{
    WAITING_FOR_SELECTION,
    WAITING_FOR_START,
    TRACKING,
    EXPERIMENT_DONE
};

struct MouseParams
{
    bool clicked = false;
    cv::Point point;
};

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        auto* params = static_cast<MouseParams*>(userdata);
        params->clicked = true;
        params->point = cv::Point(x, y);
    }
}

class TrackerNode
{
public:
    TrackerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~TrackerNode();

    void spin();

private:
    // Callbacks
    void imageCallback(const rectrial::pub_data::ConstPtr& msg);
    void stateCallback(const std_msgs::String::ConstPtr& msg);

    // Core Logic
    void processImage(const cv::Mat& frame);
    void initializeTracker(const cv::Mat& frame);
    void updateTracking(const cv::Mat& frame);
    void publishData(const sensor_msgs::ImageConstPtr& original_msg, const cv::Mat& processed_frame);
    std::string getCurrentTimestamp();

    // Member Variables
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_; // Private node handle for parameters

    // ROS Communication
    ros::Publisher experiment_pub_;
    ros::Publisher processed_pub_;
    ros::Subscriber image_sub_;
    ros::Subscriber state_sub_;

    // State Management
    NodeState state_;
    std::string experiment_id_;
    int csv_index_;
    std::vector<std::string> csv_lines_;

    // Tracking
    cv::Ptr<cv::Tracker> tracker_;
    cv::Rect2d tracking_bbox_;
    cv::Point2f initial_tracked_pos_;
    cv::Point2f current_offset_;

    // Configuration
    std::string csv_file_path_;
    int BBOX_WIDTH_ = 50;
    int BBOX_HEIGHT_ = 50;
    
    // Display
    GLWindow* gl_window_ = nullptr;
    const std::string SELECTION_WINDOW_NAME = "Select Object to Track";

    std::mutex frame_mutex_;
    rectrial::pub_data::ConstPtr latest_msg_;
    bool new_frame_available_ = false;

};

TrackerNode::TrackerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), state_(NodeState::WAITING_FOR_SELECTION), csv_index_(0)
{
    // --- 1. Load parameters using the private node handle ---
    pnh_.param<std::string>("csv_file_path", csv_file_path_, "positions.csv");
    pnh_.param<int>("bbox_width", BBOX_WIDTH_, 50);
    pnh_.param<int>("bbox_height", BBOX_HEIGHT_, 50);

    // --- 2. Setup Publishers and Subscribers ---
    experiment_pub_ = nh_.advertise<rectrial::pub_data>("start_experiment", 10);
    processed_pub_ = nh_.advertise<rectrial::pub_data>("imager_processed", 10);
    
    image_sub_ = nh_.subscribe("/imager_c", 1, &TrackerNode::imageCallback, this);
    state_sub_ = nh_.subscribe("set_state", 10, &TrackerNode::stateCallback, this);

    // --- 3. Load CSV data ---
    std::ifstream fin(csv_file_path_);
    if (!fin.is_open())
    {
        ROS_FATAL_STREAM("Could not open positions CSV file: " << csv_file_path_);
        ros::shutdown();
        return;
    }

    std::string line;
    while (std::getline(fin, line))
    {
        if (!line.empty()) {
            csv_lines_.push_back(line);
        }
    }
    fin.close();

    if (csv_lines_.empty()) {
        ROS_WARN("CSV file is empty. Experiment may not run correctly.");
    }

    ROS_INFO("TrackerNode initialized. Waiting for image and object selection...");
    
    // --- 4. Initialize Display ---
    gl_window_ = new GLWindow(0, 0);
    gl_window_->setTitle("Experiment Screen");
}

TrackerNode::~TrackerNode()
{
    // Clean up resources
    if (gl_window_)
    {
        delete gl_window_;
        gl_window_ = nullptr;
    }
    cv::destroyAllWindows();
}

/* void TrackerNode::imageCallback(const rectrial::pub_data::ConstPtr& msg)
{
    try
    {
        // Convert ROS image from the custom message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image_p, sensor_msgs::image_encodings::MONO8);
        cv::Mat frame = cv_ptr->image;

        if (frame.empty()) {
            ROS_WARN("Received an empty image frame in callback.");
            return;
        }
        processImage(frame.clone()); // Use a clone to avoid modifying the original
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception in callback: %s", e.what());
    }
} */

/* void TrackerNode::imageCallback(const rectrial::pub_data::ConstPtr& msg)
{
    try
    {
        // This is the only place we convert the message to a CV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image_p, sensor_msgs::image_encodings::MONO8);
        
        // Use a lock to safely write to the shared frame variable
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = cv_ptr->image.clone();
        new_frame_available_ = true;
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception in callback: %s", e.what());
    }
} */

void TrackerNode::imageCallback(const rectrial::pub_data::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_msg_ = msg;
    new_frame_available_ = true;
}

void TrackerNode::processImage(const cv::Mat& frame)
{
    cv::Mat display_frame = frame.clone();
    
    switch (state_)
    {
        case NodeState::WAITING_FOR_SELECTION:
            initializeTracker(display_frame);
            break;

        case NodeState::WAITING_FOR_START:
            cv::putText(display_frame, "Press Space to Start", cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(200, 200, 200), 2);
            break;

        case NodeState::TRACKING:
            updateTracking(display_frame);
            // The actual publishing is now triggered by the main loop `spin()`
            break;

        case NodeState::EXPERIMENT_DONE:
            cv::putText(display_frame, "Experiment Done. Press Space to Restart.", cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(200, 200, 200), 2);
            break;
    }

    // Update the GL window with the latest frame
    if (gl_window_)
    {
        // Assuming GL_RED is for single-channel mono8 images
        gl_window_->updateImage(display_frame.ptr(0), display_frame.cols, display_frame.rows, GL_RED);
    }
}


void TrackerNode::initializeTracker(const cv::Mat& frame) {
    ROS_INFO("Please select the object to track in the '%s' window.", SELECTION_WINDOW_NAME.c_str());
    
    cv::namedWindow(SELECTION_WINDOW_NAME);
    MouseParams mouse_params;
    cv::setMouseCallback(SELECTION_WINDOW_NAME, onMouse, &mouse_params);

    while (!mouse_params.clicked && ros::ok()) {
        cv::imshow(SELECTION_WINDOW_NAME, frame);
        if (cv::waitKey(10) == 27) { // 27 = ESC key
            ROS_INFO("Object selection cancelled by user. Shutting down.");
            ros::shutdown();
            return;
        }
    }
    cv::destroyWindow(SELECTION_WINDOW_NAME);

    tracking_bbox_ = cv::Rect2d(mouse_params.point.x - (BBOX_WIDTH_ / 2.0),
                                mouse_params.point.y - (BBOX_HEIGHT_ / 2.0),
                                BBOX_WIDTH_,
                                BBOX_HEIGHT_);

    tracker_ = cv::TrackerCSRT::create();
    tracker_->init(frame, tracking_bbox_);
    
    // Store initial position
    initial_tracked_pos_ = cv::Point2f(tracking_bbox_.x + tracking_bbox_.width / 2.0, 
                                       tracking_bbox_.y + tracking_bbox_.height / 2.0);

    ROS_INFO("Object selected. Tracker initialized. Waiting for spacebar to start experiment.");
    state_ = NodeState::WAITING_FOR_START;
}

void TrackerNode::updateTracking(const cv::Mat& frame)
{
    bool ok = tracker_->update(frame, tracking_bbox_);

    if (ok)
    {
        // Calculate current center and offset
        cv::Point2f current_center(tracking_bbox_.x + tracking_bbox_.width / 2.0,
                                   tracking_bbox_.y + tracking_bbox_.height / 2.0);
        
        // Calculate offset from the image center, relative to the initial offset
        cv::Point2f image_center(frame.cols / 2.0, frame.rows / 2.0);
        cv::Point2f initial_offset = initial_tracked_pos_ - image_center;
        cv::Point2f current_pos_offset = current_center - image_center;
        
        // Final offset for publishing. Y is inverted to match screen coordinates (up is negative).
        current_offset_.x = current_pos_offset.x - initial_offset.x;
        current_offset_.y = -(current_pos_offset.y - initial_offset.y);

        // Draw visuals
        cv::rectangle(frame, tracking_bbox_, cv::Scalar(175, 0, 0), 4);
        cv::circle(frame, current_center, 4, cv::Scalar(0, 255, 0), -1);
    }
    else
    {
        cv::putText(frame, "Tracking Failure", cv::Point(25, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
        ROS_WARN_THROTTLE(1.0, "Tracker failed to update.");
    }
    
    cv::putText(frame, "Frame: " + std::to_string(csv_index_), cv::Point(25, 25), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(200, 200, 200), 2);
}

void TrackerNode::publishData(const sensor_msgs::ImageConstPtr& original_msg, const cv::Mat& processed_frame)
{
    if (csv_lines_.empty() || csv_index_ >= csv_lines_.size()) {
        ROS_WARN("Attempted to publish data but CSV data is exhausted.");
        state_ = NodeState::EXPERIMENT_DONE;
        return;
    }

     // --- Publish Experiment Control Message (to the Motor Node) ---
     rectrial::pub_data experiment_msg;
     experiment_msg.video_name_p = experiment_id_;
     experiment_msg.data_e = csv_lines_[csv_index_];
     experiment_msg.image_p = *original_msg;
 
     // ===================================================================
     // V V V V V V V V V  THE CRITICAL CHANGE IS HERE  V V V V V V V V V V V
     //
     // Populate the target_pos_x field with the live tracking data.
     // This sends the fish's position to the motor node for closed-loop control.
     experiment_msg.target_pos_x = current_offset_.x;
     //
     // ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^
     // ===================================================================
     
     if (csv_index_ == 0) {
         experiment_msg.finish_c = "start";
     } else if (csv_index_ >= csv_lines_.size() - 1) {
         experiment_msg.finish_c = "end";
     } else {
         experiment_msg.finish_c = "null";
     }
     // This is the publisher the motor node listens to.
     experiment_pub_.publish(experiment_msg);
 
     // --- Publish Processed Data Message (for visualization/debugging) ---
     // This part remains the same.
     rectrial::pub_data processed_msg;
     processed_msg.video_name_p = experiment_id_ + "_processed";
     processed_msg.data_e = std::to_string(current_offset_.x) + "," + std::to_string(current_offset_.y);
     processed_msg.target_pos_x = current_offset_.x;
     processed_msg.finish_c = experiment_msg.finish_c;
     
     std_msgs::Header header;
     header.stamp = ros::Time::now();
     sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "mono8", processed_frame).toImageMsg();
     processed_msg.image_e = *image_msg;
 
     processed_pub_.publish(processed_msg);

    // --- Update state for next iteration ---
    csv_index_++;
    if (csv_index_ >= csv_lines_.size()) {
        ROS_INFO("Experiment finished.");
        state_ = NodeState::EXPERIMENT_DONE;
    }
}


void TrackerNode::stateCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "shutdown")
    {
        ROS_INFO("Shutdown command received. Closing node.");
        ros::shutdown();
    }
}

std::string TrackerNode::getCurrentTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

/* void TrackerNode::spin()
{
    ros::Rate rate(25); // Control the loop rate to 25 Hz
    rectrial::pub_data::ConstPtr last_custom_msg;

    while (ros::ok())
    {

        // =========================================================
        // 1. Record the start time at the beginning of the loop
        auto start_time = std::chrono::steady_clock::now();
        // =========================================================

        // Get the latest image message that the callback has received
        last_custom_msg = ros::topic::waitForMessage<rectrial::pub_data>("/imager_c", nh_, ros::Duration(1.0));

        if (last_custom_msg) {
            // Create a shared_ptr to the image part of the message to pass to other functions
            sensor_msgs::ImageConstPtr last_image_msg = boost::make_shared<const sensor_msgs::Image>(last_custom_msg->image_p);

            // Convert and process the image to update state and visuals
            try {
                cv::Mat frame = cv_bridge::toCvShare(last_image_msg, "mono8")->image;
                if (!frame.empty()) {
                    processImage(frame.clone());
                }
            } catch (const cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception in main loop: %s", e.what());
            }

            // Handle publishing based on state
            if (state_ == NodeState::TRACKING) {
                 cv::Mat processed_frame = cv_bridge::toCvShare(last_image_msg, "mono8")->image.clone();
                 updateTracking(processed_frame); // Redraw tracking on the frame to be published
                 publishData(last_image_msg, processed_frame);
            }
        } else {
            ROS_WARN_THROTTLE(5.0, "No image received on /imager_c for over 1 second.");
        }

        // Handle keyboard input from SDL
        SDL_Event sdl_event;
        while (SDL_PollEvent(&sdl_event))
        {
            if (sdl_event.type == SDL_KEYUP && sdl_event.key.keysym.scancode == SDL_SCANCODE_SPACE)
            {
                if (state_ == NodeState::WAITING_FOR_START) {
                    ROS_INFO("Spacebar pressed. Starting experiment.");
                    state_ = NodeState::TRACKING;
                    csv_index_ = 0; // Reset for this run
                    experiment_id_ = getCurrentTimestamp(); // Generate a unique ID for this run
                } else if (state_ == NodeState::EXPERIMENT_DONE) {
                    ROS_INFO("Spacebar pressed. Restarting.");
                    state_ = NodeState::WAITING_FOR_START; // Go back to waiting state
                }
            }
            else if (gl_window_ && sdl_event.type == SDL_WINDOWEVENT && SDL_GetWindowFromID(sdl_event.window.windowID) == gl_window_->window)
            {
                gl_window_->processEvent(sdl_event);
            }
        }
        
        ros::spinOnce();

        // =========================================================
        // 2. Record the end time and calculate the duration
        auto end_time = std::chrono::steady_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        // 3. Print the processing time to the console
        ROS_INFO("Frame processed in: %ld ms", duration_ms);
        // =========================================================

        rate.sleep();
    }
} */


void TrackerNode::spin()
{
    ros::Rate rate(25); // Target 40 ms / 25 Hz

    while (ros::ok())
    {
        auto start_time = std::chrono::steady_clock::now();

        rectrial::pub_data::ConstPtr msg_to_process;
        bool frame_to_process = false;

        // Safely check for and retrieve the new message
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (new_frame_available_)
            {
                msg_to_process = latest_msg_;
                frame_to_process = true;
                new_frame_available_ = false; // Reset the flag, we've consumed the message
            }
        }

        // Only do work if we actually got a new message
        if (frame_to_process)
        {
            try
            {
                // 1. Convert image for processing
                sensor_msgs::ImageConstPtr image_ptr = boost::make_shared<sensor_msgs::Image>(msg_to_process->image_p);
                cv::Mat frame = cv_bridge::toCvShare(image_ptr, "mono8")->image.clone();
                

                // 2. Process the image (update visuals, tracking state)
                processImage(frame);

                // 3. If tracking, publish data to the motor
                if (state_ == NodeState::TRACKING)
                {
                    // Pass the necessary parts of the original message and the processed frame
                    sensor_msgs::ImageConstPtr original_image_msg =
                        boost::make_shared<const sensor_msgs::Image>(msg_to_process->image_p);

                    publishData(original_image_msg, frame);
                }
            }
            catch (const cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception in main loop: %s", e.what());
            }
        }

        // Handle SDL keyboard input
        SDL_Event sdl_event;
        while (SDL_PollEvent(&sdl_event))
        {
            // ... (Your existing spacebar and window event logic here) ...
            if (sdl_event.type == SDL_KEYUP && sdl_event.key.keysym.scancode == SDL_SCANCODE_SPACE)
            {
                if (state_ == NodeState::WAITING_FOR_START) {
                    ROS_INFO("Spacebar pressed. Starting experiment.");
                    state_ = NodeState::TRACKING;
                    csv_index_ = 0;
                    experiment_id_ = getCurrentTimestamp();
                } else if (state_ == NodeState::EXPERIMENT_DONE) {
                    ROS_INFO("Spacebar pressed. Restarting.");
                    state_ = NodeState::WAITING_FOR_START;
                }
            }
            else if (gl_window_ && sdl_event.type == SDL_WINDOWEVENT && SDL_GetWindowFromID(sdl_event.window.windowID) == gl_window_->window)
            {
                gl_window_->processEvent(sdl_event);
            }
        }

        ros::spinOnce(); // Process callbacks to receive new images

        auto end_time = std::chrono::steady_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        ROS_INFO("Loop iteration time: %ld ms", duration_ms);

        rate.sleep();
    }
}

} // namespace FishTracker

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // Private node handle

    // It's good practice to check if CUDA is available
    if (cv::cuda::getCudaEnabledDeviceCount() == 0)
    {
        ROS_WARN("No CUDA-enabled device found. OpenCV will run on CPU.");
    }
    else
    {
        cv::cuda::setDevice(0);
        cv::cuda::printCudaDeviceInfo(cv::cuda::getDevice());
    }

    FishTracker::TrackerNode node(nh, pnh);
    node.spin(); // Use a dedicated spin method in the class

    return 0;
}