// old my_subscriber.cpp

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PointStamped.h>

#include <memory>
#include <fstream> // For CSV file reading

#include "adaptive_filter.h"
#include "rectrial/pub_data.h"
#include "GLWindow.h"

#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>

namespace ExperimentController
{

enum class NodeState { WAITING_FOR_DATA, WAITING_FOR_START, EXPERIMENT_RUNNING, EXPERIMENT_DONE };
enum class ControlMode { CLOSED_LOOP_GAIN, OPEN_LOOP, SUM_OF_SINES, FIXED_FREQ, INACTIVE };

class ControllerNode
{
public:
    ControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~ControllerNode();
    void spin();

private:
    // Callbacks
    void fishTrackerCallback(const rectrial::pub_data::ConstPtr& msg);
    //void refugeStateCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void refugeTrackerCallback(const rectrial::pub_data::ConstPtr& msg);
    void stateCallback(const std_msgs::String::ConstPtr& msg);
    void motorCommandCallback(const std_msgs::String::ConstPtr& msg);
    void filterStateCallback(const std_msgs::Bool::ConstPtr& msg);

    // Core Logic
    void publishCalculatedData(const cv::Mat& final_frame, bool is_stopping, double raw_pos, double filtered_pos);
    void drawVisuals(cv::Mat& frame, bool draw_text_overlays);
    std::string getCurrentTimestamp();
    std::string controlModeToString();
    cv::Point2f parsePosition(const std::string& data);
    bool loadCsvFile(const std::string& path);
    double calculateSumOfSines(double time_ms);

    // Member Variables
    std::unique_ptr<AdaptiveFilter> m_filter_x;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher experiment_data_pub_;
    //ros::Publisher elapsed_pub_;
    ros::Subscriber fish_tracker_sub_;
    ros::Subscriber refuge_state_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber motor_command_sub_;
    ros::Subscriber filter_state_sub_;

    NodeState node_state_;
    ControlMode control_mode_;
    std::string experiment_id_;
    long frame_counter_;
    rectrial::pub_data::ConstPtr last_fish_tracker_msg_;
    rectrial::pub_data::ConstPtr last_refuge_tracker_msg;
    bool has_fish_data_ = false;
    bool has_refuge_data_ = false;
    bool m_is_filter_enabled = false;

    // Parameters
    double gain_;
    double fixed_frequency_;
    cv::Point2f fish_pos_;
    cv::Point2f refuge_pos_;
    cv::Point2f prev_pos_;
    cv::Point2f new_pos_;
    int refuge_bbox_width_;
    int refuge_bbox_height_;
    
    double m_count_per_mm;
    double m_pixels_per_mm;
    double m_amplitude_pixels; // Amplitude for sine waves, now in pixels
    
    double home_position_ = 0.0;
    
    double m_avg_fish_pos; // Stores the running average of the fish's position
    double m_avg_alpha;    // The smoothing factor for the running average

    // CSV Open Loop
    std::string csv_file_path_;
    std::vector<double> csv_positions_;
    int csv_index_ = 0;

    // Sine Wave Calculation
    double a_pos_ = 0.0; // Amplitude in motor counts
    std::chrono::steady_clock::time_point trial_start_time_;
    // <<< NEW: ROS-based timer for the experiment duration
    ros::Time trial_start_time_ros_;

    GLWindow* gl_window_ = nullptr;
};

ControllerNode::ControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), node_state_(NodeState::WAITING_FOR_DATA), control_mode_(ControlMode::FIXED_FREQ),
      frame_counter_(0), gain_(0.5), fixed_frequency_(0.2)
{
    pnh_.param<int>("refuge_bbox_width", refuge_bbox_width_, 50);
    pnh_.param<int>("refuge_bbox_height", refuge_bbox_height_, 50);

    // --- Filter Setup ---
    double filter_fs = 30.0;
    int filter_taps = 128;
    std::vector<double> filter_freqs;
    pnh_.param<double>("filter_fs", filter_fs, 30.0);
    pnh_.param<int>("filter_taps", filter_taps, 128);
    pnh_.getParam("filter_freqs", filter_freqs);
    m_filter_x.reset(new AdaptiveFilter(filter_fs, filter_taps, filter_freqs));
    
    // --- CSV Open Loop Setup ---
    pnh_.param<std::string>("csv_file_path", csv_file_path_, "");
    if (!csv_file_path_.empty()) {
        loadCsvFile(csv_file_path_);
    }

    pnh_.param<double>("pixels_per_mm", m_pixels_per_mm, 10.0);
    if (m_pixels_per_mm <= 0) {
        ROS_ERROR("pixels_per_mm must be a positive value!");
        m_pixels_per_mm = 10.0;
    }

    // <<< MODIFIED: Calculate sine wave amplitude in PIXELS, not motor counts
    double amp_mm = 10.0;
    pnh_.param<double>("amplitude_mm", amp_mm, 10.0);
    m_amplitude_pixels = amp_mm * m_pixels_per_mm;

    
    // Load the alpha for the running average filter.
    // A small value like 0.01 provides smooth averaging.
    pnh_.param<double>("averaging_alpha", m_avg_alpha, 0.01);
    m_avg_fish_pos = 0.0; // Initialize average to zero

    // --- ROS Communication ---
    experiment_data_pub_ = nh_.advertise<rectrial::pub_data>("imager", 10);
    //elapsed_pub_= nh.advertise<std_msgs::Float64>("loop_elapsed_time", 10);
    fish_tracker_sub_ = nh_.subscribe("/imager_processed", 5, &ControllerNode::fishTrackerCallback, this);
    refuge_state_sub_ = nh_.subscribe("/refuge_data", 5, &ControllerNode::refugeTrackerCallback, this);
    state_sub_ = nh_.subscribe("set_state", 10, &ControllerNode::stateCallback, this);
    motor_command_sub_ = nh_.subscribe("set_motor_freq", 10, &ControllerNode::motorCommandCallback, this);
    filter_state_sub_ = nh_.subscribe("/set_filter_state", 1, &ControllerNode::filterStateCallback, this);
    

    //prev_pos_ = cv::Point2f(0.0f, 0.0f);
    //refuge_pos_ = cv::Point2f(0.0f, 0.0f);
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

bool ControllerNode::loadCsvFile(const std::string& path) {
    csv_positions_.clear();
    csv_index_ = 0;
    std::ifstream file(path);
    if (!file.is_open()) { ROS_ERROR("Failed to open CSV file: %s", path.c_str()); return false; }
    std::string line;
    while (std::getline(file, line)) {
        try { csv_positions_.push_back(std::stod(line)); }
        catch (const std::exception& e) { ROS_WARN("Could not parse line in CSV: '%s'", line.c_str()); }
    }
    file.close();
    if (csv_positions_.empty()) { ROS_ERROR("CSV file is empty or invalid: %s", path.c_str()); return false; }
    ROS_INFO("Successfully loaded %zu positions from %s", csv_positions_.size(), path.c_str());
    return true;
}

void ControllerNode::filterStateCallback(const std_msgs::Bool::ConstPtr& msg)
{
    m_is_filter_enabled = msg->data;
    if (m_is_filter_enabled) {
        ROS_INFO("Adaptive filter has been ENABLED.");
    } else {
        ROS_INFO("Adaptive filter has been DISABLED.");
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
void ControllerNode::refugeTrackerCallback(const rectrial::pub_data::ConstPtr& msg)
{
    last_refuge_tracker_msg = msg;
    refuge_pos_ = parsePosition(msg->data_e);
    if (!has_refuge_data_) {
        has_refuge_data_ = true;
        ROS_INFO("Received first data packet from refuge tracker.");
    }
}

/* void ControllerNode::refugeStateCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // The position is directly available in the message, no parsing needed
    refuge_pos_.x = msg->point.x;
    refuge_pos_.y = msg->point.y;

    if (!has_refuge_data_) {
        has_refuge_data_ = true;
        ROS_INFO("Received first data packet from refuge state (epos_node).");
    }
} */

void ControllerNode::motorCommandCallback(const std_msgs::String::ConstPtr& msg) {
    
    std::string cmd = msg->data;
    std::vector<double> freqs_to_set; // Create a list for the new frequencies

    if (cmd == "sum") {
        
        control_mode_ = ControlMode::SUM_OF_SINES;
        ROS_INFO("Set mode to SUM_OF_SINES");
        
        static const double sum_freqs[13] = {0.1, 0.15, 0.25, 0.35, 0.55, 0.65, 0.85, 0.95, 1.15, 1.45, 1.55, 1.85, 2.05};
        freqs_to_set.assign(sum_freqs, sum_freqs + 13);
        ROS_INFO("Filter updated to suppress all 13 Sum-of-Sines frequencies.");
        
    
    } else if (cmd == "openloop") {
        
        control_mode_ = ControlMode::OPEN_LOOP;
        ROS_INFO("Set mode to OPEN_LOOP");

        pnh_.getParam("csv_file_path", csv_file_path_);
        if (!csv_file_path_.empty()) loadCsvFile(csv_file_path_);

    
    } else if (cmd == "closedloop" || cmd.rfind("gain:", 0) == 0) {
        control_mode_ = ControlMode::CLOSED_LOOP_GAIN;

        try {
            // This line correctly parses the gain value from the string
            gain_ = std::stod(cmd.substr(5));
            ROS_INFO("Gain set to: %.2f", gain_);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to parse gain value from command: %s", cmd.c_str());
        }
    
    } else {
        try {            
            control_mode_ = ControlMode::FIXED_FREQ;
            fixed_frequency_ = std::stod(cmd);

            freqs_to_set.push_back(fixed_frequency_);
            ROS_INFO("Filter updated to suppress single frequency: %.2f Hz", fixed_frequency_);

        } catch (const std::exception& e) {
            ROS_WARN("Could not parse '%s' as a command or frequency.", cmd.c_str());
        }
    }
    ROS_INFO("Switched to mode: %s", controlModeToString().c_str());

    if (m_filter_x) {
        m_filter_x->updateFrequencies(freqs_to_set);
    }

}

// Draws the current state and data onto the image for display.
void ControllerNode::drawVisuals(cv::Mat& frame, bool draw_text_overlays) {
    // --- Part 1: Always draw the tracking boxes ---
    if(has_refuge_data_) {
        cv::Rect2d refuge_box(refuge_pos_.x - refuge_bbox_width_ / 2.0,
                              refuge_pos_.y - refuge_bbox_height_ / 2.0,
                              refuge_bbox_width_, refuge_bbox_height_);
        cv::rectangle(frame, refuge_box, cv::Scalar(255, 255, 255), 2);
        cv::Point refuge_center(std::round(refuge_box.x + refuge_box.width / 2), std::round(refuge_box.y + refuge_box.height / 2));
        cv::circle(frame, refuge_center, 4, cv::Scalar(0, 0, 0), -1);
    }
    
    // --- Part 2: Conditionally draw the status text ---
    if (draw_text_overlays) {
        std::stringstream ss;
        cv::Scalar text_color = CV_RGB(200, 200, 200);
        
        switch (node_state_) {
            case NodeState::WAITING_FOR_DATA:
                cv::putText(frame, "Waiting for fish tracker data...", cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);
                break;
            case NodeState::WAITING_FOR_START:
                cv::putText(frame, "Press Space to Start Experiment", cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);
                break;
            case NodeState::EXPERIMENT_RUNNING:
                ss << std::fixed << std::setprecision(2)
                   << "Mode: " << controlModeToString() << " | Gain: " << gain_ 
                   << " | Fish: (" << (int)fish_pos_.x << ", " << (int)fish_pos_.y << ")";
                cv::putText(frame, ss.str(), cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, CV_RGB(50, 255, 50), 2);
                cv::putText(frame, "Frame: " + std::to_string(frame_counter_), cv::Point(25, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2);
                cv::putText(frame, "Press Space to STOP", cv::Point(25, 130), cv::FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(255, 100, 100), 2);
                break;
            case NodeState::EXPERIMENT_DONE:
                cv::putText(frame, "Experiment Done. Press Space to Restart.", cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2);
                break;
        }
    }
}

// Performs the calculation and publishes the custom message.
void ControllerNode::publishCalculatedData(const cv::Mat& final_frame, bool is_stopping, double raw_pos, double filtered_pos)
{
    if (!last_fish_tracker_msg_) return;

    rectrial::pub_data experiment_msg;
    experiment_msg.video_name_p = experiment_id_;
    
    std::stringstream ss;
    // <<< FIX: Create the new data format: "final_x,final_y;raw_fish_x;filtered_fish_x"
    ss << new_pos_.x << "," << new_pos_.y << ";" << raw_pos << ";" << filtered_pos;
    experiment_msg.data_e = ss.str();

    experiment_msg.image_e = *cv_bridge::CvImage(last_fish_tracker_msg_->image_e.header, "bgr8", final_frame).toImageMsg();
    
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
        case ControlMode::OPEN_LOOP:        return "OpenLoop";
        case ControlMode::FIXED_FREQ:       return "FixedFreq";
        case ControlMode::SUM_OF_SINES:     return "SumOfSines";
        default:                            return "Unknown";
    }
}

void ControllerNode::spin()
{
    //ros::WallDuration desired_loop_time(0.04);
    ros::Rate rate(30);
    while (ros::ok())
    {
        //ros::WallTime start_time = ros::WallTime::now();
        ros::spinOnce();
        if (node_state_ == NodeState::WAITING_FOR_DATA && has_fish_data_) {
            node_state_ = NodeState::WAITING_FOR_START;
        }

        if (last_fish_tracker_msg_) {
            try {
                cv::Mat recording_frame = cv_bridge::toCvShare(last_fish_tracker_msg_->image_e, last_fish_tracker_msg_, "bgr8")->image.clone();
                if (!recording_frame.empty()) {
                    cv::Mat display_frame = recording_frame.clone();
                    if (node_state_ == NodeState::EXPERIMENT_RUNNING) {

                        if ((ros::Time::now() - trial_start_time_ros_).toSec() >= 60.0) {
                            ROS_INFO("Experiment time limit (60s) reached. Stopping automatically.");
                            drawVisuals(recording_frame, false);
                            publishCalculatedData(recording_frame, true, 0,0); // Publish "end" message
                            node_state_ = NodeState::EXPERIMENT_DONE;
                            // Use 'continue' to skip the rest of this loop iteration
                            // and prevent a double-stop if space is pressed simultaneously.
                            continue; 
                        }
                        
                        // --- Centralized Calculation Logic ---
                        double raw_fish_pos_x = fish_pos_.x;
                        double filtered_fish_pos_x = raw_fish_pos_x; // Default to raw if filter is off

                        if (m_is_filter_enabled) {

                            filtered_fish_pos_x = m_filter_x->measurement(raw_fish_pos_x);

                        }
                        
                        cv::Point2f processed_fish_pos(filtered_fish_pos_x, fish_pos_.y);

                        switch (control_mode_) {
                            case ControlMode::OPEN_LOOP:
                                if (!csv_positions_.empty()) {
                                    // <<< MODIFIED: Convert position from mm (in CSV) to pixels
                                    double position_mm = csv_positions_[csv_index_];
                                    new_pos_.x = position_mm * m_pixels_per_mm;
                                    new_pos_.y = 0;
                                    csv_index_ = (csv_index_ + 1) % csv_positions_.size();
                                } else { new_pos_ = cv::Point2f(0,0); }
                                break;
                            
                            case ControlMode::CLOSED_LOOP_GAIN: {
                                // 1. Update the running average of the fish's position
                                m_avg_fish_pos = (m_avg_alpha * raw_fish_pos_x) + ((1.0 - m_avg_alpha) * m_avg_fish_pos);

                                // 2. Calculate the fish's deviation from its average (the AC component)
                                double ac_fish_pos_x = raw_fish_pos_x - m_avg_fish_pos;

                                // 3. Filter this deviation signal if the filter is enabled
                                double filtered_ac_pos_x = ac_fish_pos_x; // Default to unfiltered
                                if (m_is_filter_enabled) {
                                    filtered_ac_pos_x = m_filter_x->measurement(ac_fish_pos_x);
                                }
                                
                                // 4. Calculate the feedback component from the filtered deviation
                                double feedback_pos = filtered_ac_pos_x * gain_;

                                // 5. Calculate the baseline single sine wave stimulus
                                double time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - trial_start_time_).count();
                                double baseline_pos = (m_amplitude_pixels / (2.0 * M_PI * fixed_frequency_)) * sin(2.0 * M_PI * fixed_frequency_ * time_ms / 1000.0);

                                // 6. The new position is the sum of the baseline and the feedback
                                new_pos_.x = baseline_pos + feedback_pos;
                                new_pos_.y = 0;
                                break;
}

                            case ControlMode::SUM_OF_SINES: {
                                // <<< MODIFIED: This calculation now uses pixel amplitude
                                double time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - trial_start_time_).count();
                                new_pos_.x = calculateSumOfSines(time_ms); // This function is now pixel-based
                                new_pos_.y = 0;
                                break;
                            }
                            case ControlMode::FIXED_FREQ: {
                                // <<< MODIFIED: This calculation now uses pixel amplitude
                                double time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - trial_start_time_).count();
                                new_pos_.x = (m_amplitude_pixels / (2.0 * M_PI * fixed_frequency_)) * sin(2.0 * M_PI * fixed_frequency_ * time_ms / 1000.0);
                                new_pos_.y = 0;
                                break;
                            }
                        }
                        
/*                         double max_displacement_mm = 100.0; // ±10 cm
                        double displacement_from_home = new_pos_.x - home_position_;

                        if (std::abs(displacement_from_home) > max_displacement_mm) {
                            ROS_WARN_STREAM("Stopping experiment: Motor exceeded ±10 cm from home ("
                                            << displacement_from_home << " mm)");
                            publishCalculatedData(recording_frame, true, raw_fish_pos_x, filtered_fish_pos_x);
                            new_pos_.x = home_position_;
                        } */
                        
                        drawVisuals(recording_frame, false); 
                        publishCalculatedData(recording_frame, false, raw_fish_pos_x, filtered_fish_pos_x);
                        prev_pos_ = new_pos_;
                        
                        frame_counter_++;
                    }
                    drawVisuals(display_frame, true);

                    if (gl_window_) {
                        gl_window_->updateImage(display_frame.ptr(0), display_frame.cols, display_frame.rows, GL_BGR);
                    }
                }
            } catch (const cv_bridge::Exception& e) { ROS_ERROR("cv_bridge exception: %s", e.what()); }
        } 
        
        SDL_Event sdl_event;
        while (SDL_PollEvent(&sdl_event)) {
             if (sdl_event.type == SDL_KEYUP && sdl_event.key.keysym.scancode == SDL_SCANCODE_SPACE) {
                if (node_state_ == NodeState::WAITING_FOR_START) {
                    node_state_ = NodeState::EXPERIMENT_RUNNING;
                    frame_counter_ = 0;
                    csv_index_ = 0;
                    prev_pos_ = refuge_pos_;
                    new_pos_ = cv::Point2f(0.0f, 0.0f);
                    experiment_id_ = getCurrentTimestamp();
                    trial_start_time_ = std::chrono::steady_clock::now();
                    // <<< NEW: Record the ROS time when the experiment starts
                    trial_start_time_ros_ = ros::Time::now();

                    if (m_filter_x) {
                        m_filter_x->primeBuffer(fish_pos_.x);
                        ROS_INFO("Adaptive filter buffer primed with initial fish position.");
                    }

                    m_avg_fish_pos = fish_pos_.x;

                } else if (node_state_ == NodeState::EXPERIMENT_RUNNING) {
                    cv::Mat frame_fish = cv_bridge::toCvShare(last_fish_tracker_msg_->image_e, last_fish_tracker_msg_, "bgr8")->image.clone();
                    drawVisuals(frame_fish, true);
                    publishCalculatedData(frame_fish, true, 0, 0);
                    node_state_ = NodeState::EXPERIMENT_DONE;
                } else if (node_state_ == NodeState::EXPERIMENT_DONE) {
                    node_state_ = NodeState::WAITING_FOR_START;
                }
             }
        }
       rate.sleep();
    }
}

double ControllerNode::calculateSumOfSines(double time_ms) {
    static const int NUM_SINE_FREQUENCIES = 13;
    static const double freqs[NUM_SINE_FREQUENCIES] = {0.1, 0.15, 0.25, 0.35, 0.55, 0.65, 0.85, 0.95, 1.15, 1.45, 1.55, 1.85, 2.05};
    static const double velPH[NUM_SINE_FREQUENCIES] = {0.2162, 2.5980, 0.0991, 0.2986, 2.9446, 2.5794, 2.8213, 0.0693, 1.4556, 0.3746, 1.0430, 2.9469, 2.6706};
    double position = 0.0;
    for (int i = 0; i < NUM_SINE_FREQUENCIES; ++i) {
        position += (m_amplitude_pixels / (2.0 * M_PI * freqs[i])) * sin(2.0 * M_PI * freqs[i] * time_ms / 1000.0 + velPH[i] + (M_PI / 2.0));
    }
    return position;
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