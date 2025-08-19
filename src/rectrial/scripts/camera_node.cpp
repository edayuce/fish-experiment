// updated basler_camera.cpp

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float64.h>

// Include the Pylon API
#include <pylon/PylonIncludes.h>

// Include your custom message header
#include <rectrial/pub_data.h>

#include <chrono>

// Use the Pylon namespace
using namespace Pylon;

class BaslerCameraNode
{
public:
    BaslerCameraNode(ros::NodeHandle& nh);
    ~BaslerCameraNode();
    void spin();

private:
    ros::NodeHandle nh_;
    ros::Publisher data_pub_;
    ros::Publisher loop_time_pub_; 
    CInstantCamera camera_;
    std::string frame_id_;
};

BaslerCameraNode::BaslerCameraNode(ros::NodeHandle& nh)
    : nh_(nh)
{
    // --- Get ROS Parameters ---
    nh_.param<std::string>("frame_id", frame_id_, "camera_frame");
    ROS_INFO("Publishing with frame_id: %s", frame_id_.c_str());

    // --- Initialize Pylon ---
    PylonInitialize();
    ROS_INFO("Pylon initialized.");

    try
    {
        // --- Setup Publisher ---
        data_pub_ = nh_.advertise<rectrial::pub_data>("/imager_c", 10);
        loop_time_pub_ = nh_.advertise<std_msgs::Float64>("/camera_trigger_interval", 10);

        // --- Create and Attach Camera ---
        camera_.Attach(CTlFactory::GetInstance().CreateFirstDevice());
        ROS_INFO("Using device: %s", camera_.GetDeviceInfo().GetModelName().c_str());
        
        // Open Camera
        camera_.Open();

        // --- Configure Camera for Software Trigger ---
        GenApi::INodeMap& nodemap = camera_.GetNodeMap();
        CEnumParameter(nodemap, "TriggerSelector").SetValue("FrameStart");
        // Turn TriggerMode ON
        CEnumParameter(nodemap, "TriggerMode").SetValue("On");
        // Set the trigger source to Software
        CEnumParameter(nodemap, "TriggerSource").SetValue("Software");
        CEnumParameter(nodemap, "AcquisitionMode").SetValue("Continuous");

        // --- Start Grabbing ---
        // The camera is now waiting for software triggers.
        camera_.StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByUser);
        ROS_INFO("Camera is now configured for software trigger mode.");
    }
    catch (const GenericException& e)
    {
        ROS_FATAL_STREAM("An exception occurred during Pylon setup: " << e.GetDescription());
        ros::shutdown();
    }
}

BaslerCameraNode::~BaslerCameraNode()
{
    ROS_INFO("Shutting down the camera node.");
    try
    {
        if (camera_.IsGrabbing()) { camera_.StopGrabbing(); }
        if (camera_.IsOpen()) { camera_.Close(); }
    }
    catch (const GenericException& e)
    {
        ROS_ERROR_STREAM("An exception occurred during camera shutdown: " << e.GetDescription());
    }

    PylonTerminate();
    ROS_INFO("Pylon terminated.");
}

void BaslerCameraNode::spin()
{
    // --- High-Precision Timer Setup ---
    // Use steady_clock for monotonic time, which is best for timing loops.
    auto lastTimeTarget = std::chrono::steady_clock::now();
    // 40ms in nanoseconds
    const auto interval = std::chrono::nanoseconds(40000000); 
    auto timeTarget = lastTimeTarget + interval;

    CGrabResultPtr ptrGrabResult;
    auto lastGrabTime = std::chrono::steady_clock::now();

    while (ros::ok())
    {
        // --- 1. Wait for the precise 40ms interval ---
        // This is a "busy-wait" loop for maximum precision.
        while (std::chrono::steady_clock::now() < timeTarget) {
            // Do nothing, just wait.
        }

        try
        {
            // --- 2. Trigger and Retrieve the Image ---
            if (camera_.IsGrabbing()) {
                // Send the software trigger command to the camera
                camera_.ExecuteSoftwareTrigger();

                // Wait for the grab result, with a timeout (e.g., 500ms)
                if (camera_.RetrieveResult(500, ptrGrabResult, TimeoutHandling_ThrowException))
                {
                    if (ptrGrabResult->GrabSucceeded())
                    {
                        
                        auto now = std::chrono::steady_clock::now();
                        auto delta_t = std::chrono::duration_cast<std::chrono::nanoseconds>(now - lastGrabTime);
                        lastGrabTime = now;
                        
                        std_msgs::Float64 time_msg;
                        time_msg.data = delta_t.count() / 1e6; // Convert nanoseconds to milliseconds
                        loop_time_pub_.publish(time_msg);

                        // --- 3. Process and Publish the Image ---
                        cv::Mat image_mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1);
                        memcpy(image_mat.ptr(), ptrGrabResult->GetBuffer(), ptrGrabResult->GetPayloadSize());

                        rectrial::pub_data msg;
                        std_msgs::Header header;
                        header.stamp = ros::Time::now();
                        header.frame_id = frame_id_;
                        
                        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "mono8", image_mat).toImageMsg();
                        msg.image_e = *image_msg; // <<< IMPORTANT: Put image in the correct field
                        msg.image_p = *image_msg; // Also put in the old field for compatibility

                        auto nanosec_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                        msg.data_e = std::to_string(nanosec_since_epoch);

                        data_pub_.publish(msg);
                    }
                    else
                    {
                        ROS_ERROR_STREAM("Pylon image grab failed: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription());
                    }
                }
            }
        }
        catch (const GenericException& e)
        {
            ROS_ERROR_STREAM("An exception occurred while grabbing: " << e.GetDescription());
        }

        // --- 4. Schedule the next trigger time ---
        lastTimeTarget = timeTarget;
        timeTarget = timeTarget + interval;
        
        // --- 5. Handle ROS events ---
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "basler_camera_node");
    ros::NodeHandle nh("~");

    BaslerCameraNode node(nh);
    node.spin(); // This now contains our high-precision loop

    return 0;
}
