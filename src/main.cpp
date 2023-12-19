#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "com_stream.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "light_flow");
    ros::NodeHandle nh("~");

    std::string device;
    nh.param<std::string>("device", device, "/dev/ttyUSB0");
    ROS_INFO("get device is %s", device.c_str());

    ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>("/optical_flow", 10);

    ComStream reader;
    ComParams params = {
        .rate = B115200,
    };

    if (!reader.open(device.c_str(), params))
    {
        ROS_ERROR("Failed to open device [%s]", device.c_str());
        ros::Duration(1).sleep();
        return -1;
    }
    ROS_INFO("success open device, start light flow node.");

    const int bufferSize = 1024;
    uint8_t buffer[bufferSize];
    int bytesRead = 0;
    ros::Rate rate(120);

    std_msgs::UInt8MultiArray msg;
    msg.data.resize(bufferSize);
    while (ros::ok())
    {
        ros::spinOnce();

        if (reader.hasHandleError())
        {
            ROS_ERROR("device lost");
            break;
        }

        bytesRead = reader.read(msg.data.data(), bufferSize);
        if (bytesRead == -1 && (errno != EINTR && errno != EAGAIN))
        {
            ROS_ERROR("read data error, [ret = %d, errno = %d] will exit...", bytesRead, errno);
            ros::Duration(1).sleep();
            break;
        }

        msg.data.resize(bytesRead);
        ROS_INFO("bytesRead is %d, data size is %zu", bytesRead, msg.data.size());

        pub.publish(msg);
        rate.sleep();
    }

    reader.close();

    return 0;
}
