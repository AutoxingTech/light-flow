#include "com_stream.h"
#include <sys/ioctl.h>
#include <fcntl.h>
#include <ros/ros.h>

ComStream::ComStream()
{
    m_comHandle = -1;
}

ComStream::~ComStream()
{
    close();
}

bool ComStream::open(const char* port_name, const ComParams& params)
{
    bool isReopen = isOpen();
    close();

    m_params = params;
    m_timeout = timespec{0, m_params.readTimeoutMs * 1000000};
    m_portName = port_name;
    int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

    m_comHandle = ::open(port_name, flags);
    if (-1 == m_comHandle)
    {
        ROS_DEBUG("ComStream::Open open error!");
        return false;
    }

    // if you've entered a standard baud the function below will return it
    speed_t speed = params.rate;

    // get port options
    struct termios options;
    if (tcgetattr(m_comHandle, &options) == -1)
    {
        close();
        ROS_DEBUG("ComStream::Open tcgetattr error!");
        return false;
    }

    if (params.flowControlCRTSCTS)
        options.c_cflag |= (tcflag_t)(CLOCAL | CREAD | CS8 | CRTSCTS);
    else
        options.c_cflag |= (tcflag_t)(CLOCAL | CREAD | CS8 | ~CRTSCTS);

    options.c_cflag &= (tcflag_t) ~(CSTOPB | PARENB | PARODD);
    options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN); //|ECHOPRT
    options.c_oflag &= (tcflag_t) ~(OPOST);
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | INLCR | IGNCR | ICRNL | IGNBRK);

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (params.enableParityCheck)
    {
        if (params.parityCheckOdd)
            options.c_cflag |= (tcflag_t)(PARENB);
        else
            options.c_cflag |= (tcflag_t)(PARENB | PARODD);
    }

    cfsetispeed(&options, speed ?: B38400);
    cfsetospeed(&options, speed ?: B38400);

    if (tcsetattr(m_comHandle, TCSANOW, &options) < 0)
    {
        ROS_ERROR("ComStream::Open tcsetattr error!");
        close();
        return false;
    }

    tcflush(m_comHandle, TCIFLUSH);

    return true;
}

bool ComStream::isOpen()
{
    return m_comHandle != -1;
}

bool ComStream::hasHandleError()
{
    int count;
    return ioctl(m_comHandle, TIOCINQ, &count) == -1;
}

void ComStream::close()
{
    if (m_comHandle != -1)
    {
        ::close(m_comHandle);
        m_comHandle = -1;
    }
}

int ComStream::read(uint8_t* buffer, int bufferSize)
{
    if (m_comHandle == -1)
        return -1;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(m_comHandle, &read_fds);
    int r = pselect(m_comHandle + 1, &read_fds, NULL, NULL, &m_timeout, NULL);
    if (r < 0)
    {
        // Select was interrupted
        if (errno == EINTR)
        {
            ROS_INFO_THROTTLE(1, "ComStream pselect() failed. rtn = EINTR");
            return -1;
        }
        else
        {
            ROS_INFO_THROTTLE(1, "ComStream pselect() failed. rtn = %d", r);
        }
    }
    else if (r == 0) // timeout
    {
        ROS_DEBUG_THROTTLE(1, "ComStream pselect() timeout. rtn = %d", r);
        return 0;
    }
    else
    {
        ROS_DEBUG_THROTTLE(1, "ComStream pselect() succeeded. rtn = %d", r);
    }

    if (FD_ISSET(m_comHandle, &read_fds))
    {
        int len = (int)::read(m_comHandle, buffer, bufferSize);
        if (len == -1)
        {
            ROS_INFO_THROTTLE(1, "ComStream read() failed. ");
        }
        else if (len == 0)
        {
            ROS_INFO_THROTTLE(1, "ComStream read() reached EOF.");
        }
        else
        {
            ROS_DEBUG_THROTTLE(1, "ComStream read() succeeded. bytes = %d", len);
        }

        return len;
    }
    else
    {
        return -1;
    }
}

int ComStream::write(const void* buffer, int bufferSize)
{
    ssize_t len = ::write(m_comHandle, buffer, bufferSize);
    return (int)len;
}

int ComStream::writeText(const char* buffer)
{
    int strLength = (int)strlen(buffer);
    ssize_t len = ::write(m_comHandle, buffer, strLength);
    return (int)len;
}