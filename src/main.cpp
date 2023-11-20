#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
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
    ros::NodeHandle nh;

    std::string device;
    nh.param<std::string>("device", device, "/dev/ttyUSB0");

    ros::Publisher pub = nh.advertise<light_flow::OpticalFlowPack>("/optical_flow", 10);

    ComStream reader;
    ComParams params = {
        .rate = B115200,
    };

    if (!reader.open(device.c_str(), params))
    {
        ROS_ERROR("Failed to open device [%s]", device.c_str());
        return -1;
    }
    ROS_INFO("success open device, start light flow node.");

    uint8_t buffer[1024];
    int bufferIndex = 0;
    int bytesRead;
    ros::Rate rate(120);

    while (ros::ok())
    {
        ros::spinOnce();

        bufferIndex = 0;
        bytesRead = reader.read(&buffer[bufferIndex], 1);
        if (bytesRead == -1 && (errno != EINTR && errno != EAGAIN))
        {
            ROS_ERROR("read data error, [ret = %d, errno = %d] will exit...", bytesRead, errno);
            ros::Duration(1).sleep();
            break;
        }
        if (buffer[bufferIndex] != 'U')
            continue;

        bufferIndex = 1;
        bytesRead = reader.read(&buffer[bufferIndex], 1);
        if (bytesRead == -1 && (errno != EINTR && errno != EAGAIN))
        {
            ROS_ERROR("read data error, [ret = %d, errno = %d] will exit...", bytesRead, errno);
            ros::Duration(1).sleep();
            break;
        }
        if (buffer[bufferIndex] != 'P')
            continue;

        int packLength = (int)sizeof(OpticalFlowPack);
        bufferIndex = 2;
        while (ros::ok() && bufferIndex < packLength)
        {
            bytesRead = reader.read(&buffer[bufferIndex], packLength - bufferIndex);
            // bytesRead = ::read(fd, &buffer[bufferIndex], packLength - bufferIndex);
            if (bytesRead > 0)
            {
                bufferIndex += bytesRead;
            }
            else
            {
                ROS_WARN_THROTTLE(1, "read error. [ret = %d, errno = %d], retry", bytesRead, errno);
            }
        }

        // crc
        const OpticalFlowPack *pack = (OpticalFlowPack *)buffer;
        if (pack->header.length != 10)
        {
            ROS_WARN("length error, length = %d", pack->header.length);
            continue;
        }
        uint16_t crc = calculateCRC16(&pack->header.payload[0], pack->header.length);
        if (crc != pack->header.crc16)
        {
            ROS_WARN("CRC error");
            continue;
        }

        light_flow::OpticalFlowPack msg;
        msg.flow_x_integral = pack->flow_x_integral;
        msg.flow_y_integral = pack->flow_y_integral;
        msg.integration_timespan = pack->integration_timespan;
        msg.ground_distance = pack->ground_distance;
        msg.valid = pack->valid;
        msg.version = pack->version;
        pub.publish(msg);
        rate.sleep();
    }

    return 0;
}
