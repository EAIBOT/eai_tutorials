#pragma once
#include <v8stdint.h>
#include <string>

enum {
    DRIVER_TYPE_SERIALPORT = 0x0,
    DRIVER_TYPE_UDP = 0x1,
    DRIVER_TYPE_TCP = 0x2,

};

class ChannelDevice
{
public:
    virtual bool bindport(const char*, uint32_t ) = 0;
    virtual bool open() = 0;
    virtual bool isOpen() = 0;
    virtual void closefd() = 0;
    virtual void flush() = 0;
    virtual int waitfordata(size_t data_count,uint32_t timeout = -1, size_t * returned_size = NULL) = 0;
    virtual int writedata(const uint8_t * data, size_t size) = 0;
    virtual int readdata(unsigned char * data, size_t size) = 0;
    virtual bool setDTR(bool level = true) {return true;}
    virtual int getByteTime() { return 0;}
    virtual void setParity(int) {return;}
    virtual size_t readline (std::string &buffer, size_t size = 65536, std::string eol = "\n") {
        return 0;
    }
    virtual void setTimeoutMs(int) {return;}
    virtual int get_serial_error() = 0;
};
