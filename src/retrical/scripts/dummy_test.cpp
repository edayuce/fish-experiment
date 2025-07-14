#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rectrial/pub_data.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_publisher_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Publisher pub_data_pub = nh.advertise<rectrial::pub_data>("imager", 10);
    image_transport::Publisher image_pub = it.advertise("dummy_image_raw", 1);

    ros::Rate loop_rate(25);

    int frame_count = 0;

    while (ros::ok())
    {
        // Görüntü oluştur
        cv::Mat image = cv::Mat::zeros(400, 1152, CV_8UC1);
        cv::putText(image, "Dummy Frame: " + std::to_string(frame_count), cv::Point(50,200),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 2);

        // Image msg oluştur
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();

        // pub_data msg oluştur
        rectrial::pub_data msg;
        msg.data_e = std::to_string(0.1 * frame_count);
        msg.finish_c = (frame_count == 0) ? "start" : (frame_count == 100 ? "end" : "middle");
        msg.video_name_p = "dummy_video";
        msg.target_pos_x = "5.0";
        msg.image_e = *img_msg;

        pub_data_pub.publish(msg);
        image_pub.publish(img_msg);

        frame_count++;
        if (frame_count > 100)
            frame_count = 0;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
