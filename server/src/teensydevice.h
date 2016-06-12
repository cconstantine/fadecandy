
#pragma once
#include "usbdevice.h"
#include "opc.h"
#include <set>


class TeensyDevice : public USBDevice
{
public:
    TeensyDevice(libusb_device *device, bool verbose);
    virtual ~TeensyDevice();

    static bool probe(libusb_device *device);

    virtual int open();
    virtual void loadConfiguration(const Value &config);
    virtual void writeMessage(const OPC::Message &msg);
    virtual std::string getName();
    virtual void flush();

    
    // Write color LUT from parsed JSON
    virtual void writeColorCorrection(const Value &color);


    static LIBUSB_CALL void completeTransfer(libusb_transfer *transfer);

private:
    char mSerialBuffer[256];
    unsigned char lut[3][255];
    const Value *mConfigMap;
    int dev;

    const unsigned int show = 1;
    struct Packet {
        uint8_t strip_index;
        uint8_t pixel_index;
        uint8_t size;
        uint8_t flags;
        uint8_t data[60];
    };

    struct Transfer {
        Transfer(TeensyDevice *device, Packet &packet);
        ~Transfer();
        libusb_transfer *transfer;
        Packet bufferCopy;
        bool finished;
    };

    std::set<Transfer*> mPending;
    bool submitTransfer(Packet &packet);

    void opcSetPixelColors(const OPC::Message &msg);
    void opcMapPixelColors(const OPC::Message &msg, const Value &inst);
};
