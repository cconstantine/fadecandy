
#include "teensydevice.h"

#include "opc.h"
#include <sstream>
#include <iostream>


TeensyDevice::TeensyDevice(libusb_device *device, bool verbose)
    : USBDevice(device, "teensy", verbose)
{

    mSerialBuffer[0] = '\0';
    mSerialString = mSerialBuffer;
 }

TeensyDevice::~TeensyDevice()
{

}

bool TeensyDevice::probe(libusb_device *device)
{
    /*
     * Prior to opening the device, all we can do is look for an FT245 device.
     * We'll take a closer look in probeAfterOpening(), once we can see the
     * string descriptors.
     */

    libusb_device_descriptor dd;

    if (libusb_get_device_descriptor(device, &dd) < 0) {
        // Can't access descriptor?
        return false;
    }

    return dd.idVendor == 0x016c0 && dd.idProduct == 0x0486;
}

int TeensyDevice::open()
{
    libusb_device_descriptor dd;
    int r = libusb_get_device_descriptor(mDevice, &dd);
    if (r < 0) {
        return r;
    }

    r = libusb_open(mDevice, &mHandle);
    if (r < 0) {
        return r;
    }


    // Only relevant on linux; try to detach the FTDI driver.
    libusb_detach_kernel_driver(mHandle, 0);

    r = libusb_claim_interface(mHandle, 0);
    if (r < 0) {
        return r;
    }

    r = libusb_get_string_descriptor_ascii(mHandle, dd.iSerialNumber,
        (uint8_t*)mSerialBuffer, sizeof mSerialBuffer);
    if (r < 0) {
        return r;
    }

    return 0;
}

void TeensyDevice::loadConfiguration(const Value &config)
{
    mConfigMap = findConfigMap(config);
}

std::string TeensyDevice::getName()
{
    std::ostringstream s;
    s << "Teensy";
    if (mSerialString[0]) {
        s << " (Serial# " << mSerialString << ")";
    }
    return s.str();
}

void TeensyDevice::writeMessage(const OPC::Message &msg)
{
    if(msg.command != OPC::SetPixelColors) {
        if (mVerbose) {
            std::clog << "Unsupported OPC command: " << unsigned(msg.command) << "\n";
        }
        return;
    }
    /*
     * Dispatch an incoming OPC command
     */
    int transfered;
    Packet p;
    p.strip_index = 1;
    p.pixel_index = 0;
    p.size = 60/3;
    p.flags = 0;
    memcpy(p.data, msg.data, 60);
    int ret = libusb_bulk_transfer(mHandle, 4, (unsigned char*)&p, sizeof(p), &transfered, 0);
}

void TeensyDevice::flush() { }


void TeensyDevice::opcSetPixelColors(const OPC::Message &msg)
{
    /*
     * Parse through our device's mapping, and store any relevant portions of 'msg'
     * in the framebuffer.
     */

    if (!mConfigMap) {
        // No mapping defined yet. This device is inactive.
        return;
    }

    const Value &map = *mConfigMap;
    for (unsigned i = 0, e = map.Size(); i != e; i++) {
        opcMapPixelColors(msg, map[i]);
    }
}

void TeensyDevice::opcMapPixelColors(const OPC::Message &msg, const Value &inst)
{
    /*
     * Parse one JSON mapping instruction, and copy any relevant parts of 'msg'
     * into our framebuffer. This looks for any mapping instructions that we
     * recognize:
     *
     *   [ OPC Channel, First OPC Pixel, First output pixel, Pixel count ]
     */

    // unsigned msgPixelCount = msg.length() / 3;

    // if (inst.IsArray() && inst.Size() == 4) {
    //     // Map a range from an OPC channel to our framebuffer

    //     const Value &vChannel = inst[0u];
    //     const Value &vFirstOPC = inst[1];
    //     const Value &vFirstOut = inst[2];
    //     const Value &vCount = inst[3];

    //     if (vChannel.IsUint() && vFirstOPC.IsUint() && vFirstOut.IsUint() && vCount.IsUint()) {
    //         unsigned channel = vChannel.GetUint();
    //         unsigned firstOPC = vFirstOPC.GetUint();
    //         unsigned firstOut = vFirstOut.GetUint();
    //         unsigned count = vCount.GetUint();

    //         if (channel != msg.channel) {
    //             return;
    //         }

    //         // Clamping, overflow-safe
    //         firstOPC = std::min<unsigned>(firstOPC, msgPixelCount);
    //         firstOut = std::min<unsigned>(firstOut, unsigned(256));
    //         count = std::min<unsigned>(count, msgPixelCount - firstOPC);
    //         count = std::min<unsigned>(count, 256 - firstOut);

    //         // Copy pixels
    //         const uint8_t *inPtr = msg.data + (firstOPC * 3);
    //         unsigned outIndex = firstOut;

    //         while (count--) {
    //             uint8_t *outPtr = fbPixel(outIndex++);
    //             outPtr[0] = inPtr[0];
    //             outPtr[1] = inPtr[1];
    //             outPtr[2] = inPtr[2];
    //             inPtr += 3;
    //         }

    //         return;
    //     }
    // }

    // // Still haven't found a match?
    // if (mVerbose) {
    //     rapidjson::GenericStringBuffer<rapidjson::UTF8<> > buffer;
    //     rapidjson::Writer<rapidjson::GenericStringBuffer<rapidjson::UTF8<> > > writer(buffer);
    //     inst.Accept(writer);
    //     std::clog << "Unsupported JSON mapping instruction: " << buffer.GetString() << "\n";
    // }
}
