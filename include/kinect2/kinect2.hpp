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

#ifndef KINECT2__KINECT2_HPP_
#define KINECT2__KINECT2_HPP_

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace kinect2
{

class Kinect2 : public rclcpp::Node, public libfreenect2::SyncMultiFrameListener
{
public:
  struct Options : public rclcpp::NodeOptions
  {
    Options();

    unsigned int get_frame_types() const;

    bool enable_rgb;
    bool enable_depth;
  };

  explicit Kinect2(const Options & options = Options());
  ~Kinect2();

  bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame * frame);

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher;

  libfreenect2::Freenect2 freenect2;

  libfreenect2::PacketPipeline * pipeline;
  libfreenect2::Freenect2Device * device;
};

}  // namespace kinect2

#endif  // KINECT2__KINECT2_HPP_
