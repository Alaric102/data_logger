#include <ros/ros.h>

#include <string_view>
#include <string>

namespace {

constexpr std::string_view NODE_NAME = "cloud_logger_node";

}

int main(int argc, char** argv){
    ros::init(argc, argv, std::string(NODE_NAME));
    ros::NodeHandle private_nh("~");
    ros::Rate rate(1.);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO_STREAM("Spin");
    }
    return 0;
}