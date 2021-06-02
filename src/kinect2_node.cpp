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
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

namespace kinect2
{

Kinect2Node::Kinect2Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("kinect2", options),
  listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth)
{
  // Check available devices
  if (freenect2.enumerateDevices() <= 0) {
    throw std::runtime_error("no device found");
  }

  pipeline = new libfreenect2::CpuPacketPipeline();
  device = freenect2.openDevice(freenect2.getDefaultDeviceSerialNumber(), pipeline);

  device->setColorFrameListener(&listener);
  device->setIrAndDepthFrameListener(&listener);

  // Trying to start the device
  if (!device->start()) {
    throw std::runtime_error("failed to start the device");
  }

  RCLCPP_INFO_STREAM(get_logger(), "Device serial: " << device->getSerialNumber());
  RCLCPP_INFO_STREAM(get_logger(), "Device firmware: " << device->getFirmwareVersion());

  // Initialize publisher
  rgb_image_publisher = create_publisher<sensor_msgs::msg::Image>("/kinect2/rgb_image", 10);

  // Initialize capture timer
  capture_timer = create_wall_timer(
    1ms, [this]() {
      if (!listener.waitForNewFrame(frames, 3 * 1000)) {
        RCLCPP_WARN(get_logger(), "Timeout waiting for new frames!");
        return;
      }

      RCLCPP_DEBUG(get_logger(), "Received new frames!");

      auto frame = frames[libfreenect2::Frame::Color];

      auto image = std::make_shared<sensor_msgs::msg::Image>();

      image->height = frame->height;
      image->width = frame->width;

      image->encoding = sensor_msgs::image_encodings::BGRA8;
      image->step = frame->width * frame->bytes_per_pixel;

      image->data.resize(image->step * image->height);
      std::copy_n(frame->data, image->step * image->height, image->data.begin());

      rgb_image_publisher->publish(*image);

      listener.release(frames);
    });
}

Kinect2Node::~Kinect2Node()
{
  // Stop and close the device
  if (device) {
    device->stop();
    device->close();
  }
}

}  // namespace kinect2
