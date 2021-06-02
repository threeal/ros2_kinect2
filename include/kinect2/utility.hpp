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

#ifndef KINECT2__UTILITY_HPP_
#define KINECT2__UTILITY_HPP_

#include <libfreenect2/frame_listener.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

namespace kinect2
{

std::shared_ptr<sensor_msgs::msg::Image> frame_info_to_image(libfreenect2::Frame * frame);
std::shared_ptr<sensor_msgs::msg::Image> rgb_frame_to_image(libfreenect2::Frame * frame);
std::shared_ptr<sensor_msgs::msg::Image> ir_frame_to_image(libfreenect2::Frame * frame);

}  // namespace kinect2

#endif  // KINECT2__UTILITY_HPP_