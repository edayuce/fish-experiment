// throttler_node.cpp

#include <ros/ros.h>
#include <rectrial/pub_data.h> // Your custom message

/**
 * @brief This node subscribes to a fast-publishing topic and republishes
 * the last received message at a fixed, slower rate (e.g., 30 Hz).
 */
class ThrottlerNode
{
public:
    ThrottlerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        // Get the desired output framerate from the parameter server
        double rate = 30.0;
        pnh.param<double>("rate", rate, 30.0);
        loop_rate_ = new ros::Rate(rate);

        // Publisher for the new, throttled topic
        pub_ = nh.advertise<rectrial::pub_data>("/imager_c_throttled", 10);
        
        // Subscriber to the original, fast camera topic
        sub_ = nh.subscribe("/imager_c", 10, &ThrottlerNode::callback, this);

        ROS_INFO("Throttler node initialized. Republishing from /imager_c to /imager_c_throttled at %.1f Hz.", rate);
    }

    ~ThrottlerNode()
    {
        delete loop_rate_;
    }

    void spin()
    {
        while (ros::ok())
        {
            ros::spinOnce(); // Process incoming messages to get the latest one
            
            // Publish the last message we received at the fixed rate
            if (last_msg_) {
                pub_.publish(last_msg_);
            }
            
            loop_rate_->sleep();
        }
    }

private:
    void callback(const rectrial::pub_data::ConstPtr& msg)
    {
        // Just store the latest message
        last_msg_ = msg;
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Rate* loop_rate_;
    rectrial::pub_data::ConstPtr last_msg_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "throttler_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ThrottlerNode node(nh, pnh);
    node.spin();
    return 0;
}