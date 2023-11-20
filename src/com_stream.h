#pragma once
#include <termios.h>
#include <linux/serial.h>
#include <stdint.h>
#include <string>


struct ComParams
{
    speed_t rate = 0;          // 9600, 19200, 115200 etc. See definition of speed_t in "termios.h"
    int readTimeoutMs = 0; // will not block read().
                           // WARNING: when timeout == 0, ros::Rate() is needed to avoid high CPU usage.
    bool enableParityCheck = false;
    bool parityCheckOdd = false; // Only valid when enableParityCheck == true. True for even check, false for odd check.
    bool flowControlCRTSCTS = false;
};

/**
 * Read/Write from COM port
 */
class ComStream
{
public:
    ComStream();
    ~ComStream();

    bool open(const char *portName, const ComParams &params);
    void close();
    bool isOpen();

    int read(uint8_t *buffer, int bufferSize);
    int write(const void *buffer, int bufferSize);
    int writeText(const char *buffer);
    bool hasHandleError();

protected:
    ComParams m_params;
    timespec m_timeout{0, 0};
    int m_comHandle;
    std::string m_portName;
};