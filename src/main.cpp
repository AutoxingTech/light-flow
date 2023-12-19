#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "com_stream.h"
#include <string>

void test()
{
    uint8_t arr[] = {85, 80, 0, 1, 2, 3, 0, 220, 244};
    std::string str;

    for (size_t i = 0; i < sizeof(arr); i++)
    {
        str.push_back(arr[i]);
    }
    printf("str length is %zu, str is %s\n", str.length(), str.c_str());

    std::string str2(str);
    std::vector<uint8_t> arr2;
    arr2.resize(str2.length());
    memcpy(arr2.data(), (void*)(&str2[0]), str2.length());

    for (size_t i = 0; i < arr2.size(); i++)
    {
        printf("%d, ", arr2[i]);
    }
    printf("\n");
}

void test2()
{
    uint8_t buffer[] = {85, 80, 0, 1, 2, 3, 0, 220, 244};
    size_t bufferSize = sizeof(buffer);

    std_msgs::String msg;
    msg.data.resize(bufferSize);
    memcpy((void*)msg.data.data(), (void*)(&buffer[0]), bufferSize);

    ROS_INFO("bytesRead is %zu, data size is %zu", bufferSize, msg.data.size());
    for (size_t i = 0; i < msg.data.size(); i++)
    {
        printf("%d, ", (uint8_t)msg.data[i]);
    }
    printf("\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "light_flow");
    ros::NodeHandle nh("~");

    std::string device;
    nh.param<std::string>("device", device, "/dev/ttyUSB0");
    ROS_INFO("get device is %s", device.c_str());

    ros::Publisher pub = nh.advertise<std_msgs::String>("/optical_flow", 10);

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

    std_msgs::String msg;
    msg.data.resize(bufferSize);
    while (ros::ok())
    {
        ros::spinOnce();

        if (reader.hasHandleError())
        {
            ROS_ERROR("device lost");
            break;
        }

        bytesRead = reader.read(&buffer[0], bufferSize);
        if (bytesRead == -1 && (errno != EINTR && errno != EAGAIN))
        {
            ROS_ERROR("read data error, [ret = %d, errno = %d] will exit...", bytesRead, errno);
            ros::Duration(1).sleep();
            break;
        }

        msg.data.resize(bytesRead);
        memcpy((void*)msg.data.data(), (void*)(&buffer[0]), bytesRead);

        ROS_INFO("bytesRead is %d, data size is %zu", bytesRead, msg.data.size());

        pub.publish(msg);
        rate.sleep();
    }

    reader.close();

    return 0;
}
