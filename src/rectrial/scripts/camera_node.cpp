//this is old basler_camera.cpp file


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

// Include the Pylon API
#include <pylon/PylonIncludes.h>

// Include your custom message header
#include <rectrial/pub_data.h>

#include <chrono>

// Use the Pylon namespace
using namespace Pylon;

// Use the C++ standard namespace
using namespace std;

class BaslerCameraNode
{
public:
    BaslerCameraNode(ros::NodeHandle& nh);
    ~BaslerCameraNode();
    void spin();

private:
    // This class handles the image grabbing event from the camera.
    class ImageEventHandler : public CImageEventHandler
    {
    public:
        // The node is passed to the handler so it can access the publisher.
        ImageEventHandler(BaslerCameraNode* node) : node_(node) {}

        // This method is called by the Pylon SDK whenever a new image is available.
        virtual void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
        {
            if (ptrGrabResult->GrabSucceeded())
            {
                // Create a cv::Mat to hold the image data.
                // CV_8UC1 is for a single-channel 8-bit image (grayscale).
                cv::Mat image_mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1);

                // Copy the image data from the Pylon buffer to the cv::Mat.
                memcpy(image_mat.ptr(), ptrGrabResult->GetBuffer(), ptrGrabResult->GetPayloadSize());

                // --- Create and populate the custom message ---
                rectrial::pub_data msg;

                // 1. Populate the image field (image_p)
                std_msgs::Header header; // Create a header
                header.stamp = ros::Time::now();
                header.frame_id = node_->frame_id_;
                
                // Convert the cv::Mat to a sensor_msgs::Image message
                sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "mono8", image_mat).toImageMsg();
                msg.image_p = *image_msg;

                // 2. Populate the data field (data_e) with a high-resolution timestamp
                auto nanosec_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                msg.data_e = std::to_string(nanosec_since_epoch);      

                // The other fields (finish_c, video_name_p, image_e) are left empty,
                // as they are intended to be filled by downstream nodes.

                // Publish the custom message.
                node_->data_pub_.publish(msg);
            }
            else
            {
                ROS_ERROR_STREAM("Pylon image grab failed: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription());
            }
        }
    private:
        BaslerCameraNode* node_;
    };

    // ROS NodeHandle
    ros::NodeHandle nh_;

    // A standard ROS publisher for our custom message type
    ros::Publisher data_pub_;

    // Pylon camera object
    CInstantCamera camera_;

    // Parameters
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
        // Publish the custom message to the '/imager_c' topic
        data_pub_ = nh_.advertise<rectrial::pub_data>("/imager_c", 10);

        // --- Create and Camera ---
        camera_.Attach(CTlFactory::GetInstance().CreateFirstDevice());
        ROS_INFO("Using device: %s", camera_.GetDeviceInfo().GetModelName().c_str());
        
        // --- Register Event Handler ---
        camera_.RegisterImageEventHandler(new ImageEventHandler(this), RegistrationMode_ReplaceAll, Cleanup_Delete);

        // Open Camera
        camera_.Open();

        // --- Configure Camera ---
        GenApi::INodeMap& nodemap = camera_.GetNodeMap();
        CEnumParameter(nodemap, "TriggerSelector").SetValue("FrameStart");
        CEnumParameter(nodemap, "TriggerMode").SetValue("Off"); // 'Off' for continuous/freerun
        CEnumParameter(nodemap, "AcquisitionMode").SetValue("Continuous");

        // --- Start Grabbing ---
        camera_.StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByInstantCamera);
        ROS_INFO("Camera is now grabbing continuously and publishing to /imager_c.");
    }
    catch (const GenericException& e)
    {
        ROS_FATAL_STREAM("An exception occurred in the Pylon camera node: " << e.GetDescription());
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
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "basler_camera_node");
    ros::NodeHandle nh("~");

    BaslerCameraNode node(nh);
    node.spin();

    return 0;
}
