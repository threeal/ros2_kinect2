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

#include <kinect2/kinect2_node.hpp>
#include <kinect2/utility.hpp>

using namespace std::chrono_literals;

namespace kinect2
{

Kinect2Node::Kinect2Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("kinect2", options),
  SyncMultiFrameListener(
    libfreenect2::Frame::Color | libfreenect2::Frame::Ir)
{
  // Check available devices
  if (freenect2.enumerateDevices() <= 0) {
    throw std::runtime_error("no device found");
  }

  pipeline = new libfreenect2::CpuPacketPipeline();
  device = freenect2.openDevice(freenect2.getDefaultDeviceSerialNumber(), pipeline);

  device->setColorFrameListener(this);
  device->setIrAndDepthFrameListener(this);

  // Trying to start the device
  if (!device->start()) {
    throw std::runtime_error("failed to start the device");
  }

  RCLCPP_INFO_STREAM(get_logger(), "Device serial: " << device->getSerialNumber());
  RCLCPP_INFO_STREAM(get_logger(), "Device firmware: " << device->getFirmwareVersion());

  // Initialize publishers
  rgb_image_publisher = create_publisher<sensor_msgs::msg::Image>("/kinect2/image_raw", 10);
  depth_image_publisher = create_publisher<sensor_msgs::msg::Image>("/kinect2/depth/image_raw", 10);
}

Kinect2Node::~Kinect2Node()
{
  // Stop and close the device
  if (device) {
    device->stop();
    device->close();
  }
}

bool Kinect2Node::onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame * frame)
{
  RCLCPP_DEBUG(get_logger(), "Received new frames!");

  switch (type) {
    case libfreenect2::Frame::Color: {
        auto color_image = rgb_frame_to_image(frame);
        rgb_image_publisher->publish(*color_image);
        break;
      }

    case libfreenect2::Frame::Ir: {
        auto ir_image = ir_frame_to_image(frame);
        depth_image_publisher->publish(*ir_image);
        break;
      }

    default:
      break;
  }

  return false;
}

}  // namespace kinect2
