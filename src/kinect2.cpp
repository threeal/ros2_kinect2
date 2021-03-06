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

#include <kinect2/kinect2.hpp>
#include <kinect2/utility.hpp>

using namespace std::chrono_literals;

namespace kinect2
{

Kinect2::Options::Options()
: enable_rgb(false),
  enable_depth(false),
  width(-1),
  height(-1)
{
}

unsigned int Kinect2::Options::get_frame_types() const
{
  unsigned int frame_types = 0;

  if (enable_rgb) {
    frame_types |= libfreenect2::Frame::Color;
  }

  if (enable_depth) {
    frame_types |= libfreenect2::Frame::Ir;
  }

  return frame_types;
}

Kinect2::Kinect2(const Kinect2::Options & options)
: rclcpp::Node("kinect2", options),
  SyncMultiFrameListener(options.get_frame_types()),
  options(options)
{
  // Check available devices
  if (freenect2.enumerateDevices() <= 0) {
    throw std::runtime_error("no device found");
  }

  pipeline = new libfreenect2::OpenGLPacketPipeline();
  device = freenect2.openDevice(freenect2.getDefaultDeviceSerialNumber(), pipeline);

  device->setColorFrameListener(this);
  device->setIrAndDepthFrameListener(this);

  // Trying to start the device
  if (!device->start()) {
    throw std::runtime_error("failed to start the device");
  }

  RCLCPP_INFO_STREAM(get_logger(), "Device serial: " << device->getSerialNumber());
  RCLCPP_INFO_STREAM(get_logger(), "Device firmware: " << device->getFirmwareVersion());

  if (options.enable_rgb) {
    rgb_image_publisher = create_publisher<sensor_msgs::msg::Image>("/kinect2/image_raw", 10);
    rgb_camera_info_publisher = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/kinect2/camera_info", 10);
  }

  if (options.enable_depth) {
    depth_image_publisher = create_publisher<sensor_msgs::msg::Image>(
      "/kinect2/depth/image_raw", 10);

    depth_camera_info_publisher = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/kinect2/depth/camera_info", 10);
  }
}

Kinect2::~Kinect2()
{
  // Stop and close the device
  if (device) {
    device->stop();
    device->close();

    delete device;
  }

  if (pipeline) {
    delete pipeline;
  }
}

bool Kinect2::onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame * frame)
{
  RCLCPP_DEBUG(get_logger(), "Received new frames!");

  switch (type) {
    case libfreenect2::Frame::Color: {
        if (rgb_image_publisher) {
          auto color_image = rgb_frame_to_image(frame, options.width, options.height);

          color_image->header.stamp = now();
          color_image->header.frame_id = "camera";

          rgb_image_publisher->publish(*color_image);

          if (rgb_camera_info_publisher) {
            auto camera_info = camera_info_from_image(*color_image);

            auto param = device->getColorCameraParams();

            camera_info.k = {
              param.fx, 0, param.cx,
              0, param.fy, param.cy,
              0, 0, 1
            };

            camera_info.p = {
              param.fx, 0, param.cx, 0,
              0, param.fy, param.cy, 0,
              0, 0, 1, 0
            };

            rgb_camera_info_publisher->publish(camera_info);
          }
        }

        break;
      }

    case libfreenect2::Frame::Ir: {
        if (depth_image_publisher) {
          auto ir_image = ir_frame_to_image(frame, options.width, options.height);

          ir_image->header.stamp = now();
          ir_image->header.frame_id = "camera";

          depth_image_publisher->publish(*ir_image);

          if (depth_camera_info_publisher) {
            auto camera_info = camera_info_from_image(*ir_image);

            auto param = device->getIrCameraParams();

            camera_info.distortion_model = "plum_bob";
            camera_info.d = {param.k1, param.k2, param.p1, param.p2, param.k3};

            camera_info.k = {
              param.fx, 0, param.cx,
              0, param.fy, param.cy,
              0, 0, 1
            };

            camera_info.p = {
              param.fx, 0, param.cx, 0,
              0, param.fy, param.cy, 0,
              0, 0, 1, 0
            };

            depth_camera_info_publisher->publish(camera_info);
          }
        }

        break;
      }

    default:
      break;
  }

  return false;
}

}  // namespace kinect2
