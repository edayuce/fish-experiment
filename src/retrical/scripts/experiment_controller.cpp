//old my_subscriber.cpp

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Custom message - ensure this path is correct
#include <rectrial/pub_data.h>

#include "GLWindow.h" // Assuming GLWindow.h is in the include path

#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <chrono>
#include <iomanip> // For formatting time

// Use a namespace to keep the code organized
namespace ExperimentController
{

// An enum provides a much clearer way to manage the node's state
enum class NodeState
{
    WAITING_FOR_START,
    PUBLISHING_DATA,
    EXPERIMENT_DONE
};

class ControllerNode
{
public:
    ControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~ControllerNode();

    void spin(); // Main loop for handling events and state

private:
    // Callbacks
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void stateCallback(const std_msgs::String::ConstPtr& msg);

    // Core Logic
    void processImage(const cv::Mat& frame);
    void publishData(const sensor_msgs::ImageConstPtr& original_msg);
    std::string getCurrentTimestamp();

    // Member Variables
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_; // Private node handle for parameters

    // ROS Communication
    ros::Publisher experiment_data_pub_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber state_sub_;

    // State Management
    NodeState state_;
    std::string experiment_id_;
    int csv_index_;
    std::vector<std::string> csv_lines_;
    sensor_msgs::ImageConstPtr last_image_msg_; // Store the latest image

    // Configuration
    std::string csv_file_path_;
    
    // Display
    GLWindow* gl_window_ = nullptr;
};

ControllerNode::ControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), state_(NodeState::WAITING_FOR_START), csv_index_(0)
{
    // --- 1. Load parameters from the ROS parameter server ---
    // This makes the node reusable without changing the code.
    pnh_.param<std::string>("csv_file_path", csv_file_path_, "positions.csv");
    ROS_INFO_STREAM("Loading positions from: " << csv_file_path_);

    // --- 2. Setup Publishers and Subscribers ---
    experiment_data_pub_ = nh_.advertise<rectrial::pub_data>("imager", 10);
    
    image_transport::ImageTransport it(nh_);
    // The image callback just stores the latest message. The main loop processes it.
    image_sub_ = it.subscribe("/imager_c", 1, &ControllerNode::imageCallback, this);
    state_sub_ = nh_.subscribe("set_state", 10, &ControllerNode::stateCallback, this);

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
        ROS_WARN("CSV file is empty. The node will run but not publish any data.");
    }

    ROS_INFO("ControllerNode initialized. Waiting for images and user input...");
    
    // --- 4. Initialize Display ---
    gl_window_ = new GLWindow(0, 0);
    gl_window_->setTitle("Experiment Control Screen");
}

ControllerNode::~ControllerNode()
{
    // Properly clean up dynamically allocated memory
    if (gl_window_)
    {
        delete gl_window_;
        gl_window_ = nullptr;
    }
}

// This callback is now very simple: it just saves the latest message pointer.
void ControllerNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    last_image_msg_ = msg;
}

void ControllerNode::processImage(const cv::Mat& frame)
{
    // This function is now only responsible for drawing the correct overlay.
    switch (state_)
    {
        case NodeState::WAITING_FOR_START:
            cv::putText(frame, "Press Space to Start Experiment", cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(200, 200, 200), 2);
            break;

        case NodeState::PUBLISHING_DATA:
            cv::putText(frame, "Frame: " + std::to_string(csv_index_), cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(200, 200, 200), 2);
            break;

        case NodeState::EXPERIMENT_DONE:
            cv::putText(frame, "Experiment Done. Press Space to Restart.", cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(200, 200, 200), 2);
            break;
    }

    // Update the GL window with the annotated frame
    if (gl_window_)
    {
        // Assuming GL_RED is for single-channel mono8 images
        gl_window_->updateImage(frame.ptr(0), frame.cols, frame.rows, GL_RED);
    }
}

void ControllerNode::publishData(const sensor_msgs::ImageConstPtr& original_msg)
{
    if (csv_lines_.empty() || csv_index_ >= csv_lines_.size()) {
        ROS_WARN("Attempted to publish data but CSV data is exhausted.");
        state_ = NodeState::EXPERIMENT_DONE;
        return;
    }

    rectrial::pub_data experiment_msg;
    experiment_msg.video_name_p = experiment_id_;
    experiment_msg.data_e = csv_lines_[csv_index_];
    experiment_msg.image_e = *original_msg; // Pass original image
    
    if (csv_index_ == 0) {
        experiment_msg.finish_c = "start";
    } else if (csv_index_ >= csv_lines_.size() - 1) {
        experiment_msg.finish_c = "end";
    } else {
        experiment_msg.finish_c = "null";
    }
    
    experiment_data_pub_.publish(experiment_msg);

    // Update state for the next iteration
    csv_index_++;
    if (csv_index_ >= csv_lines_.size()) {
        ROS_INFO("Finished publishing all data for this run.");
        state_ = NodeState::EXPERIMENT_DONE;
    }
}

void ControllerNode::stateCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "shutdown")
    {
        ROS_INFO("Shutdown command received. Closing node.");
        ros::shutdown();
    }
}

std::string ControllerNode::getCurrentTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

void ControllerNode::spin()
{
    ros::Rate rate(60); // Control the main loop rate

    while (ros::ok())
    {
        // Process any incoming ROS messages (e.g., image callback)
        ros::spinOnce();

        // Only process and publish if we have received an image
        if (last_image_msg_) {
            // If we are in the publishing state, publish the data
            if (state_ == NodeState::PUBLISHING_DATA) {
                 publishData(last_image_msg_);
            }

            // Always draw the latest visuals, regardless of state
            try {
                cv::Mat frame = cv_bridge::toCvShare(last_image_msg_, "mono8")->image.clone();
                if (!frame.empty()) {
                    processImage(frame);
                }
            } catch (const cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception in main loop: %s", e.what());
            }
        } else {
            ROS_WARN_THROTTLE(5.0, "No image received on /imager_c yet...");
        }

        // Handle keyboard input from SDL in the main loop
        SDL_Event sdl_event;
        while (SDL_PollEvent(&sdl_event))
        {
            if (sdl_event.type == SDL_KEYUP && sdl_event.key.keysym.scancode == SDL_SCANCODE_SPACE)
            {
                if (state_ == NodeState::WAITING_FOR_START) {
                    ROS_INFO("Spacebar pressed. Starting experiment publishing.");
                    state_ = NodeState::PUBLISHING_DATA;
                    csv_index_ = 0; // Reset index for the new run
                    experiment_id_ = getCurrentTimestamp(); // Generate a unique ID for this run
                } else if (state_ == NodeState::EXPERIMENT_DONE) {
                    ROS_INFO("Spacebar pressed. Resetting for a new experiment.");
                    state_ = NodeState::WAITING_FOR_START;
                }
            }
            else if (gl_window_ && sdl_event.type == SDL_WINDOWEVENT && SDL_GetWindowFromID(sdl_event.window.windowID) == gl_window_->window)
            {
                gl_window_->processEvent(sdl_event);
            }
        }
        
        rate.sleep();
    }
}

} // namespace ExperimentController

int main(int argc, char** argv)
{
    ros::init(argc, argv, "experiment_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // Use a private node handle for parameters

    ExperimentController::ControllerNode node(nh, pnh);
    node.spin(); // The main loop is now controlled by the class

    return 0;
}
