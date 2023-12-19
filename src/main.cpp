#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "com_stream.h"
#include "light_flow/OpticalFlowPack.h"

struct __attribute__((packed)) BottomSensorPacket
{
    char header[2];
    uint16_t length;
    uint16_t crc16;
    uint8_t payload[0];
};

struct __attribute__((packed)) OpticalFlowPack
{
    BottomSensorPacket header;
    int16_t flow_x_integral;
    int16_t flow_y_integral;
    uint16_t integration_timespan; // unit: us
    uint16_t ground_distance;
    uint8_t valid; // 0x00 invalid; 0xf5 valid
    uint8_t version;
};

uint16_t calculateCRC16(const uint8_t *p, int len)
{
    uint16_t crc = 0xffff;            /// init value
    const uint16_t CRC_MASK = 0xA001; /// high and low bit flipping of '0x8005'
    int i, j;
    for (j = 0; j < len; j++)
    {
        crc ^= p[j];
        for (i = 0; i < 8; i++)
        {
            if ((crc & 0x0001) > 0)
            {
                crc = (crc >> 1) ^ CRC_MASK;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

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
    int bytesRead;
    ros::Rate rate(120);

    std_msgs::UInt8MultiArray msg;
    msg.data.resize(bufferSize);
    while (ros::ok())
    {
        msg.data.clear();
        ros::spinOnce();

        if (reader.hasHandleError())
        {
            ROS_ERROR("device lost");
            break;
        }

        bytesRead = reader.read(buffer, bufferSize);
        if (bytesRead == -1 && (errno != EINTR && errno != EAGAIN))
        {
            ROS_ERROR("read data error, [ret = %d, errno = %d] will exit...", bytesRead, errno);
            ros::Duration(1).sleep();
            break;
        }

        ROS_INFO("bytesRead is %d", bytesRead);
        for (int i = 0; i < bytesRead; i++)
        {
            msg.data.push_back(buffer[i]);
        }

        pub.publish(msg);
        rate.sleep();
    }

    reader.close();

    return 0;
}
