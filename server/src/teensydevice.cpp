
#include "teensydevice.h"

#include "opc.h"
#include <sstream>
#include <iostream>
#include <math.h>

#include <sys/ioctl.h>
#include <linux/serial.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>

#define BAUD B115200
TeensyDevice::Transfer::Transfer(TeensyDevice *device, Packet &packet)
    : transfer(libusb_alloc_transfer(0)),finished(false)
{
    memcpy(&bufferCopy, &packet, sizeof(packet));
    libusb_fill_bulk_transfer(transfer, device->mHandle,
        4, (unsigned char*)&bufferCopy, sizeof(packet), TeensyDevice::completeTransfer, this, 2000);
}

TeensyDevice::Transfer::~Transfer()
{
    libusb_free_transfer(transfer);
}

TeensyDevice::TeensyDevice(libusb_device *device, bool verbose)
    : USBDevice(device, "teensy", verbose), dev(0)
{

    mSerialBuffer[0] = '\0';
    mSerialString = mSerialBuffer;
 }

TeensyDevice::~TeensyDevice()
{
    /*
     * If we have pending transfers, cancel them.
     * The Transfer objects themselves will be freed
     * once libusb completes them.
     */

    for (std::set<Transfer*>::iterator i = mPending.begin(), e = mPending.end(); i != e; ++i) {
        Transfer *fct = *i;
        libusb_cancel_transfer(fct->transfer);
    }
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
    fprintf(stderr, "dd.idVendor: %x\n", dd.idVendor);
    fprintf(stderr, "dd.idProduct: %x\n", dd.idProduct);

    return dd.idVendor == 0x016c0 && dd.idProduct == 0x0486 || dd.idVendor == 0x016c0 && dd.idProduct == 0x0483;
}

void die(const char * msg) {
    fprintf(stderr, "%s\n", msg);
    exit(1);
}
int derp(char* name) {
    struct termios tinfo;
    struct serial_struct kernel_serial_settings;

    fprintf(stderr, "opening: %s\n", name);
    int dev = open(name, O_RDWR| O_NOCTTY);
    if (dev < 0) die("unable to open port");
    if (tcgetattr(dev, &tinfo) < 0) die("unable to get serial parms\n");
    cfmakeraw(&tinfo);
    if (cfsetspeed(&tinfo, BAUD) < 0) die("error in cfsetspeed\n");
    if (tcsetattr(dev, TCSANOW, &tinfo) < 0) die("unable to set baud rate\n");
    int r = ioctl(dev, TIOCGSERIAL, &kernel_serial_settings);
    if (r >= 0) {
        kernel_serial_settings.flags |= ASYNC_LOW_LATENCY;
        r = ioctl(dev, TIOCSSERIAL, &kernel_serial_settings);
        if (r >= 0) printf("set linux low latency mode\n");
    }
    return dev;
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


    r = libusb_get_string_descriptor_ascii(mHandle, dd.iSerialNumber,
        (uint8_t*)mSerialBuffer, sizeof mSerialBuffer);
    if (r < 0) {
        return r;
    }

    char name[256];
    sleep(1);
    sprintf(name, "/dev/serial/by-id/usb-Teensyduino_USB_Serial_%s-if00", mSerialBuffer);
    dev = derp(name);
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
    opcSetPixelColors(msg);
}


bool TeensyDevice::submitTransfer(Packet &packet)
{
    int transfered = 0;
    
    //transfered = libusb_control_transfer(mHandle, 0x21, 9, 0x0200, 0,
    //                          (unsigned char *)&packet, sizeof(packet), 10000);

    char buffer[256];
    int bytes_available;
    ioctl(dev, FIONREAD, &bytes_available);
    if(bytes_available) {
        memset(buffer, sizeof(buffer), '\0');
        read(dev, &buffer, bytes_available);
        fprintf(stderr, "%s", buffer);
    }
    //fprintf(stderr, "sending\n");

    transfered = write(dev, &packet, sizeof(packet));
    //fprintf(stderr, "transfered: %d\n", transfered);

    return true;
    int ret = libusb_bulk_transfer(mHandle, 0x02, (unsigned char*)&packet, sizeof(packet), &transfered, 0);
    fprintf(stderr, "transfered: %d\n", transfered);
    if (ret < 0) {
        std::clog << "Error submitting USB transfer: " << libusb_strerror(libusb_error(ret)) << "\n";
        exit(1);
    }
    return true;
    Transfer *fct = new Transfer(this, packet);
    /*
     * Submit a new USB transfer. The Transfer object is guaranteed to be freed eventually.
     * On error, it's freed right away.
     */

    int r = libusb_submit_transfer(fct->transfer);

    if (r < 0) {
        if (mVerbose && r != LIBUSB_ERROR_PIPE) {
            std::clog << "Error submitting USB transfer: " << libusb_strerror(libusb_error(r)) << "\n";
        }
        delete fct;
        return false;

    } else {
        mPending.insert(fct);
        return true;
    }
}

void TeensyDevice::completeTransfer(libusb_transfer *transfer)
{
    TeensyDevice::Transfer *fct = static_cast<TeensyDevice::Transfer*>(transfer->user_data);
    fct->finished = true;
}

void TeensyDevice::flush()
{
    // Erase any finished transfers

    std::set<Transfer*>::iterator current = mPending.begin();
    while (current != mPending.end()) {
        std::set<Transfer*>::iterator next = current;
        next++;

        Transfer *fct = *current;
        if (fct->finished) {
            mPending.erase(current);
            delete fct;
        }

        current = next;
    }

    int transfered;

    Packet p;
    p.strip_index = 0;
    p.pixel_index = 0;
    p.size = 0;
    p.flags = 0x01;
    submitTransfer(p);

    //int ret = libusb_bulk_transfer(mHandle, 4, (unsigned char*)&p, sizeof(p), &transfered, 0);
}


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
     *   [ OPC Channel, First OPC Pixel, Strip index, Pixel count ]
     */

    unsigned msgPixelCount = msg.length() / 3;

    if (inst.IsArray() && inst.Size() == 4) {
        // Map a range from an OPC channel to our framebuffer

        const Value &vChannel = inst[0u];
        const Value &vFirstOPC = inst[1];
        const Value &vFirstOut = inst[2];
        const Value &vCount = inst[3];

        if (vChannel.IsUint() && vFirstOPC.IsUint() && vFirstOut.IsUint() && vCount.IsUint()) {
            unsigned channel = vChannel.GetUint();
            unsigned firstOPC = vFirstOPC.GetUint();
            unsigned strip_index = vFirstOut.GetUint();
            unsigned count = vCount.GetUint();

            // fprintf(stderr, "\nchannel:     %d\n", channel);
            // fprintf(stderr, "firstOPC:    %d\n", firstOPC);
            // fprintf(stderr, "strip_index: %d\n", strip_index);
            // fprintf(stderr, "count:       %d\n", count);

            if (channel != msg.channel) {
                return;
            }

            // Copy pixels
            const uint8_t *inPtr = msg.data + (firstOPC * 3);
            unsigned outIndex = 0;
            
            for(int i = 0; count > 0;i++)  {
                int transfered;
                Packet p;
                p.strip_index = strip_index;
                p.pixel_index = outIndex;
                p.size = 20;
                p.flags = 0x00;

                if(count < 20) {
                    p.size = count;
                }

                // fprintf(stderr, "p.strip_index = %d\n", p.strip_index);
                // fprintf(stderr, "p.pixel_index = %d\n", p.pixel_index);
                // fprintf(stderr, "p.size =        %d\n", p.size);
                // fprintf(stderr, "p.flags =       %d\n", p.flags);

                for(int j = 0;j < p.size;j++) {
                    p.data[j*3+0] = lut[0][inPtr[j*3+0]];
                    p.data[j*3+1] = lut[1][inPtr[j*3+1]];
                    p.data[j*3+2] = lut[2][inPtr[j*3+2]];

                    p.data[j*3+0] = inPtr[j*3+0]*0.1;
                    p.data[j*3+1] = inPtr[j*3+1]*0.1;
                    p.data[j*3+2] = inPtr[j*3+2]*0.1;
                }
//                memcpy(p.data, inPtr, p.size*3);
                //int ret = libusb_bulk_transfer(mHandle, 4, (unsigned char*)&p, sizeof(p), &transfered, 0);

                submitTransfer(p);

                outIndex += p.size;
                inPtr += p.size*3;
                count -= p.size;
            }

            return;
        }
    }

}


void TeensyDevice::writeColorCorrection(const Value &color)
{
    fprintf(stderr, "writeColorCorrection\n");
    /*
     * Populate the color correction table based on a JSON configuration object,
     * and send the new color LUT out over USB.
     *
     * 'color' may be 'null' to load an identity-mapped LUT, or it may be
     * a dictionary of options including 'gamma' and 'whitepoint'.
     *
     * This calculates a compound curve with a linear section and a nonlinear
     * section. The linear section, near zero, avoids creating very low output
     * values that will cause distracting flicker when dithered. This isn't a problem
     * when the LEDs are viewed indirectly such that the flicker is below the threshold
     * of perception, but in cases where the flicker is a problem this linear section can
     * eliminate it entierly at the cost of some dynamic range.
     *
     * By default, the linear section is disabled (linearCutoff is zero). To enable the
     * linear section, set linearCutoff to some nonzero value. A good starting point is
     * 1/256.0, correspnding to the lowest 8-bit PWM level.
     */

    // Default color LUT parameters
    double gamma = 1.0;                         // Power for nonlinear portion of curve
    double whitepoint[3] = {1.0, 1.0, 1.0};     // White-point RGB value (also, global brightness)
    double linearSlope = 1.0;                   // Slope (output / input) of linear section of the curve, near zero
    double linearCutoff = 0.0;                  // Y (output) coordinate of intersection of linear and nonlinear curves

    /*
     * Parse the JSON object
     */

    if (color.IsObject()) {
        const Value &vGamma = color["gamma"];
        const Value &vWhitepoint = color["whitepoint"];
        const Value &vLinearSlope = color["linearSlope"];
        const Value &vLinearCutoff = color["linearCutoff"];

        if (vGamma.IsNumber()) {
            gamma = vGamma.GetDouble();
        } else if (!vGamma.IsNull() && mVerbose) {
            std::clog << "Gamma value must be a number.\n";
        }

        if (vLinearSlope.IsNumber()) {
            linearSlope = vLinearSlope.GetDouble();
        } else if (!vLinearSlope.IsNull() && mVerbose) {
            std::clog << "Linear slope value must be a number.\n";
        }

        if (vLinearCutoff.IsNumber()) {
            linearCutoff = vLinearCutoff.GetDouble();
        } else if (!vLinearCutoff.IsNull() && mVerbose) {
            std::clog << "Linear slope value must be a number.\n";
        }

        if (vWhitepoint.IsArray() &&
            vWhitepoint.Size() == 3 &&
            vWhitepoint[0u].IsNumber() &&
            vWhitepoint[1].IsNumber() &&
            vWhitepoint[2].IsNumber()) {
            whitepoint[0] = vWhitepoint[0u].GetDouble();
            whitepoint[1] = vWhitepoint[1].GetDouble();
            whitepoint[2] = vWhitepoint[2].GetDouble();
        } else if (!vWhitepoint.IsNull() && mVerbose) {
            std::clog << "Whitepoint value must be a list of 3 numbers.\n";
        }

    } else if (!color.IsNull() && mVerbose) {
        std::clog << "Color correction value must be a JSON dictionary object.\n";
    }

    /*
     * Calculate the color LUT, stowing the result in an array of USB packets.
     */

    for (unsigned entry = 0; entry <= 0xFF; entry++) {    
        for (unsigned channel = 0; channel < 3; channel++) {
            double output;

            /*
             * Normalized input value corresponding to this LUT entry.
             * Ranges from 0 to slightly higher than 1. (The last LUT entry
             * can't quite be reached.)
             */
            double input = (double)entry / 0xFF;

            // Scale by whitepoint before anything else
            input *= whitepoint[channel];

            // Is this entry part of the linear section still?
            if (input * linearSlope <= linearCutoff) {

                // Output value is below linearCutoff. We're still in the linear portion of the curve
                output = input * linearSlope;

            } else {

                // Nonlinear portion of the curve. This starts right where the linear portion leaves
                // off. We need to avoid any discontinuity.

                double nonlinearInput = input - (linearSlope * linearCutoff);
                double scale = 1.0 - linearCutoff;
                output = linearCutoff + pow(nonlinearInput / scale, gamma) * scale;
            }
            if(output > 1) {
                output = 1;
            }
            // Store LUT entry, little-endian order.
            lut[channel][entry] = uint8_t(output * 255 + 0.5);

            //fprintf(stderr, "%4d -> %4d\n", entry, lut[channel][entry]);
        }
        //fprintf(stderr, "\n");
    }
}
