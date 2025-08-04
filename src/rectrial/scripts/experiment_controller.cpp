// old my_subscriber.cpp

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PointStamped.h>

// Custom message
#include <rectrial/pub_data.h>

#include "GLWindow.h" // Assuming GLWindow.h is in the include path

#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include <iomanip> // For formatting time and float values
#include <sstream> // Required for parsing strings
#include <cmath>   // For std::sqrt

namespace ExperimentController
{

// Defines the overall state of the node (controlled by spacebar)
enum class NodeState
{
    WAITING_FOR_DATA, // New initial state
    WAITING_FOR_START,
    EXPERIMENT_RUNNING,
    EXPERIMENT_DONE
};

// Defines the behavior during the experiment (controlled by the GUI)
enum class ControlMode
{
    INACTIVE,
    CLOSED_LOOP_GAIN, // The mode that uses: new_pos = (fish_pos * gain) + prev_pos
    OPEN_LOOP_GAIN,
    FIXED_FREQ,
    SUM_OF_SINES
};


class ControllerNode
{
public:
    ControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~ControllerNode();

    void spin(); // Main loop

private:
    // Callbacks
    void fishTrackerCallback(const rectrial::pub_data::ConstPtr& msg);
    void refugeStateCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void stateCallback(const std_msgs::String::ConstPtr& msg);
    void motorCommandCallback(const std_msgs::String::ConstPtr& msg);

    // Core Logic
    void publishCalculatedData(bool is_stopping);
    void processAndDraw(cv::Mat& frame);
    std::string getCurrentTimestamp();
    std::string controlModeToString();
    cv::Point2f parsePosition(const std::string& data);

    // Member Variables
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // ROS Communication
    ros::Publisher experiment_data_pub_;
    ros::Subscriber fish_tracker_sub_;
    ros::Subscriber refuge_tracker_sub_; // New subscriber for the refuge
    ros::Subscriber state_sub_;
    ros::Subscriber motor_command_sub_;

    // State Management
    NodeState node_state_;
    ControlMode control_mode_;
    std::string experiment_id_;
    long frame_counter_;
    rectrial::pub_data::ConstPtr last_fish_tracker_msg_; // Latest message from the fish tracker
    bool has_fish_data_ = false;
    bool has_refuge_data_ = false;

    // Calculation Parameters
    double gain_;
    double fixed_frequency_;
    cv::Point2f fish_pos_;      // Position received from the fish tracker
    cv::Point2f refuge_pos_;    // Position received from the refuge tracker
    cv::Point2f prev_pos_;      // The result from the previous step
    cv::Point2f new_pos_;       // The newly calculated position to be published

    // Display
    GLWindow* gl_window_ = nullptr;
};

ControllerNode::ControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), 
      pnh_(pnh), 
      node_state_(NodeState::WAITING_FOR_DATA), // Start in a state waiting for initial data
      control_mode_(ControlMode::FIXED_FREQ),
      frame_counter_(0), 
      gain_(0.5),
      fixed_frequency_(0.2)
{
    // --- 1. Setup Publishers and Subscribers ---
    experiment_data_pub_ = nh_.advertise<rectrial::pub_data>("imager", 10);
    
    // Subscribe to the fish tracker
    fish_tracker_sub_ = nh_.subscribe("/imager_processed", 5, &ControllerNode::fishTrackerCallback, this);
    // Subscribe to the NEW refuge tracker
    refuge_tracker_sub_ = nh_.subscribe("/refuge_state", 5, &ControllerNode::refugeStateCallback, this);
    
    state_sub_ = nh_.subscribe("set_state", 10, &ControllerNode::stateCallback, this);
    motor_command_sub_ = nh_.subscribe("set_motor_freq", 10, &ControllerNode::motorCommandCallback, this);

    // --- 2. Initialize Calculation Variables ---
    prev_pos_ = cv::Point2f(0.0f, 0.0f);

    ROS_INFO("ControllerNode initialized. Waiting for data from fish and refuge trackers...");
    
    // --- 3. Initialize Display ---
    gl_window_ = new GLWindow(0, 0);
    gl_window_->setTitle("Experiment Control Screen");
}

ControllerNode::~ControllerNode()
{
    if (gl_window_)
    {
        delete gl_window_;
        gl_window_ = nullptr;
    }
}

// Helper function to parse "x,y" string into a Point2f
cv::Point2f ControllerNode::parsePosition(const std::string& data) {
    cv::Point2f pos(0,0);
    std::stringstream ss(data);
    std::string x_str, y_str;
    
    if (std::getline(ss, x_str, ',') && std::getline(ss, y_str)) {
        try {
            pos.x = std::stof(x_str);
            pos.y = std::stof(y_str);
        } catch(const std::exception& e) {
            ROS_ERROR("Failed to parse position from data string: '%s'", data.c_str());
        }
    } else {
        ROS_WARN("Received malformed position data: '%s'", data.c_str());
    }
    return pos;
}

// Callback for data from the OnlineTrackerNode (fish)
void ControllerNode::fishTrackerCallback(const rectrial::pub_data::ConstPtr& msg)
{
    last_fish_tracker_msg_ = msg;
    fish_pos_ = parsePosition(msg->data_e);
    if (!has_fish_data_) {
        has_fish_data_ = true;
        ROS_INFO("Received first data packet from fish tracker.");
    }
}

// NEW Callback for data from the RefugeTrackerNode
/* void ControllerNode::refugeTrackerCallback(const rectrial::pub_data::ConstPtr& msg)
{
    refuge_pos_ = parsePosition(msg->data_e);
    if (!has_refuge_data_) {
        has_refuge_data_ = true;
        ROS_INFO("Received first data packet from refuge tracker.");
    }
} */

void ControllerNode::refugeStateCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // The position is directly available in the message, no parsing needed
    refuge_pos_.x = msg->point.x;
    refuge_pos_.y = msg->point.y;

    if (!has_refuge_data_) {
        has_refuge_data_ = true;
        ROS_INFO("Received first data packet from refuge state (epos_node).");
    }
}

void ControllerNode::motorCommandCallback(const std_msgs::String::ConstPtr& msg)
{
    const std::string& cmd = msg->data;
    ROS_INFO_STREAM("Received command from GUI: " << cmd);

    if (cmd.rfind("gain:", 0) == 0) {
        try {
            std::string val_str = cmd.substr(5);
            gain_ = std::stod(val_str);
            control_mode_ = ControlMode::CLOSED_LOOP_GAIN;
            ROS_INFO("Set mode to CLOSED_LOOP_GAIN with gain = %f", gain_);
        } catch (const std::exception& e) { ROS_ERROR("Failed to parse gain: %s", cmd.c_str()); }
    } 
    else if (cmd.rfind("ol_gain:", 0) == 0) {
        try {
            std::string val_str = cmd.substr(8);
            gain_ = std::stod(val_str);
            control_mode_ = ControlMode::OPEN_LOOP_GAIN;
            ROS_INFO("Set mode to OPEN_LOOP_GAIN with gain = %f", gain_);
        } catch (const std::exception& e) { ROS_ERROR("Failed to parse gain: %s", cmd.c_str()); }
    }
    else if (cmd == "closedloop") {
        control_mode_ = ControlMode::CLOSED_LOOP_GAIN;
        ROS_INFO("Set mode to CLOSED_LOOP_GAIN (gain: %f)", gain_);
    }
    else if (cmd == "openloop") {
        control_mode_ = ControlMode::OPEN_LOOP_GAIN;
        ROS_INFO("Set mode to OPEN_LOOP_GAIN (gain: %f)", gain_);
    }
    else if (cmd == "sum") {
        control_mode_ = ControlMode::SUM_OF_SINES;
        ROS_INFO("Set mode to SUM_OF_SINES");
    }
    else { 
        try {
            fixed_frequency_ = std::stod(cmd);
            control_mode_ = ControlMode::FIXED_FREQ;
            ROS_INFO("Set mode to FIXED_FREQ with freq = %f", fixed_frequency_);
        } catch (const std::exception& e) { ROS_WARN("Unknown command/frequency: %s", cmd.c_str()); }
    }
}

// Draws the current state and data onto the image for display.
void ControllerNode::processAndDraw(cv::Mat& frame)
{
    std::stringstream ss;
    cv::Scalar text_color = CV_RGB(200, 200, 200);
    
    // Draw refuge position if available
    if(has_refuge_data_) {
        cv::putText(frame, "Refuge: (" + std::to_string((int)refuge_pos_.x) + "," + std::to_string((int)refuge_pos_.y) + ")", 
                    cv::Point(frame.cols - 250, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(0, 255, 0), 2);
    }

    switch (node_state_)
    {
        case NodeState::WAITING_FOR_DATA:
            cv::putText(frame, "Waiting for tracker data...", cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);
            break;
        case NodeState::WAITING_FOR_START:
            cv::putText(frame, "Press Space to Start Experiment", cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);
            break;
        case NodeState::EXPERIMENT_RUNNING:
            ss << std::fixed << std::setprecision(2)
               << "Mode: " << controlModeToString() << " | Gain: " << gain_ 
               << " | Fish: (" << (int)fish_pos_.x << ", " << (int)fish_pos_.y << ")"
               << " | New Pos: (" << (int)new_pos_.x << ", " << (int)new_pos_.y << ")";
            cv::putText(frame, ss.str(), cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(50, 255, 50), 2);
            cv::putText(frame, "Frame: " + std::to_string(frame_counter_), cv::Point(25, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2);
            cv::putText(frame, "Press Space to STOP", cv::Point(25, 130), cv::FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(255, 100, 100), 2);
            break;
        case NodeState::EXPERIMENT_DONE:
            cv::putText(frame, "Experiment Done. Press Space to Restart.", cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2);
            break;
    }

    if (gl_window_)
    {
        gl_window_->updateImage(frame.ptr(0), frame.cols, frame.rows, GL_RED);
    }
}

// Performs the calculation and publishes the custom message.
void ControllerNode::publishCalculatedData(bool is_stopping = false)
{
    if (!last_fish_tracker_msg_) return;

    rectrial::pub_data experiment_msg;
    experiment_msg.video_name_p = experiment_id_;
    
    std::stringstream ss;
    ss << new_pos_.x << "," << new_pos_.y;
    experiment_msg.data_e = ss.str();

    experiment_msg.image_e = last_fish_tracker_msg_->image_e;
    
    if (frame_counter_ == 0 && !is_stopping) {
        experiment_msg.finish_c = "start";
    } else if (is_stopping) {
        experiment_msg.finish_c = "end";
    } else {
        experiment_msg.finish_c = "null";
    }
    
    experiment_data_pub_.publish(experiment_msg);
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

std::string ControllerNode::controlModeToString()
{
    switch(control_mode_) {
        case ControlMode::INACTIVE:         return "Inactive";
        case ControlMode::CLOSED_LOOP_GAIN: return "ClosedLoop";
        case ControlMode::OPEN_LOOP_GAIN:   return "OpenLoop";
        case ControlMode::FIXED_FREQ:       return "FixedFreq";
        case ControlMode::SUM_OF_SINES:     return "SumOfSines";
        default:                            return "Unknown";
    }
}

void ControllerNode::spin()
{
    ros::Rate rate(30);

    while (ros::ok())
    {
        ros::spinOnce();

        // <<< FIX: The condition to start is now ONLY dependent on the fish tracker.
        if (node_state_ == NodeState::WAITING_FOR_DATA && has_fish_data_) {
            node_state_ = NodeState::WAITING_FOR_START;
            ROS_INFO("Fish tracker data received. Ready to start experiment.");
        }

        if (last_fish_tracker_msg_) {
            try {
                cv::Mat frame = cv_bridge::toCvShare(last_fish_tracker_msg_->image_e, last_fish_tracker_msg_, "mono8")->image.clone();
                if (!frame.empty()) {
                    
                    if (node_state_ == NodeState::EXPERIMENT_RUNNING) {
                        
                        if (control_mode_ == ControlMode::CLOSED_LOOP_GAIN) {
                            new_pos_ = (fish_pos_ * gain_) + prev_pos_;
                        } else {
                            new_pos_ = prev_pos_; 
                        }
                        
                        publishCalculatedData(false);
                        
                        prev_pos_ = new_pos_;
                        frame_counter_++;
                    }
                    
                    processAndDraw(frame);
                }
            } catch (const cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        } else if (node_state_ == NodeState::WAITING_FOR_DATA) {
             ROS_WARN_THROTTLE(5.0, "Waiting for data from fish tracker on /imager_processed...");
        }

        SDL_Event sdl_event;
        while (SDL_PollEvent(&sdl_event))
        {
            if (sdl_event.type == SDL_KEYUP && sdl_event.key.keysym.scancode == SDL_SCANCODE_SPACE)
            {
                if (node_state_ == NodeState::WAITING_FOR_START) {
                    ROS_INFO("Spacebar pressed. Starting experiment.");
                    node_state_ = NodeState::EXPERIMENT_RUNNING;
                    frame_counter_ = 0;
                    prev_pos_ = cv::Point2f(0.0f, 0.0f);
                    new_pos_ = cv::Point2f(0.0f, 0.0f);
                    experiment_id_ = getCurrentTimestamp();
                } else if (node_state_ == NodeState::EXPERIMENT_RUNNING) {
                    ROS_INFO("Spacebar pressed. Stopping experiment.");
                    publishCalculatedData(true);
                    node_state_ = NodeState::EXPERIMENT_DONE;
                } else if (node_state_ == NodeState::EXPERIMENT_DONE) {
                    ROS_INFO("Spacebar pressed. Resetting for a new experiment.");
                    node_state_ = NodeState::WAITING_FOR_START;
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
    ros::NodeHandle pnh("~");

    ExperimentController::ControllerNode node(nh, pnh);
    node.spin();

    return 0;
}