// Copyright (c) 2021 Alfi Maulana
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>

#include <iostream>
#include <memory>

int main(int /*argc*/, char ** /*argv*/)
{
  libfreenect2::Freenect2 freenect2;

  // Check available devices
  if (freenect2.enumerateDevices() <= 0) {
    std::cerr << "No device found!" << std::endl;
    return 1;
  }

  auto serial = freenect2.getDefaultDeviceSerialNumber();
  auto pipeline = new libfreenect2::CpuPacketPipeline();

  auto device = freenect2.openDevice(serial, pipeline);

  libfreenect2::SyncMultiFrameListener listener(
    libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

  device->setColorFrameListener(&listener);
  device->setIrAndDepthFrameListener(&listener);

  // Trying to start the device
  if (!device->start()) {
    std::cerr << "Failed to start the device!" << std::endl;
    return 1;
  }

  std::cout << "Device serial: " << device->getSerialNumber() << std::endl;
  std::cout << "Device firmware: " << device->getFirmwareVersion() << std::endl;

  libfreenect2::FrameMap frames;
  while (true) {
    if (!listener.waitForNewFrame(frames, 10 * 1000)) {
      std::cerr << "Timeout waiting for new frames!" << std::endl;
      break;
    }

    listener.release(frames);
  }

  // Stop and close the device
  device->stop();
  device->close();

  return 0;
}
