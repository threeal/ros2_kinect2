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

#include <kinect2/utility.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <memory>
#include <string>

namespace kinect2
{

cv::Mat frame_to_mat(libfreenect2::Frame * frame, const int & type)
{
  cv::Mat mat(frame->height, frame->width, type);

  auto size = frame->height * frame->width * frame->bytes_per_pixel;
  std::copy_n(frame->data, size, mat.data);

  return mat;
}

std::shared_ptr<sensor_msgs::msg::Image> mat_to_image(cv::Mat mat)
{
  auto image = std::make_shared<sensor_msgs::msg::Image>();

  image->height = mat.rows;
  image->width = mat.cols;
  image->step = mat.step;

  image->encoding = cv::typeToString(mat.type());

  image->data.resize(image->step * image->height);
  std::copy(mat.datastart, mat.dataend, image->data.begin());

  return image;
}

cv::Mat crop_mat(cv::Mat mat, const int & width, const int & height)
{
  cv::Rect roi;
  roi.x = (mat.cols - width) / 2;
  roi.y = (mat.rows - height) / 2;
  roi.width = width;
  roi.height = height;

  return mat(roi);
}

cv::Mat resize_mat(cv::Mat mat, int width, int height)
{
  if (width <= 0 && height <= 0) {
    return mat;
  }

  if (width <= 0) {
    width = (mat.cols * height) / mat.rows;
  }

  if (height <= 0) {
    height = (mat.rows * width) / mat.cols;
  }

  // Crop image to keep the aspect ratio
  if (static_cast<double>(width) / height < static_cast<double>(mat.cols) / mat.rows) {
    mat = crop_mat(mat, (width * mat.rows) / height, mat.rows);
  } else {
    mat = crop_mat(mat, mat.cols, (height * mat.cols) / width);
  }

  cv::resize(mat, mat, cv::Size(width, height));

  return mat;
}

std::shared_ptr<sensor_msgs::msg::Image> rgb_frame_to_image(
  libfreenect2::Frame * frame, const int & width, const int & height)
{
  auto mat = frame_to_mat(frame, CV_8UC4);
  auto resized_mat = resize_mat(mat, width, height);

  auto image = mat_to_image(resized_mat);
  image->encoding = sensor_msgs::image_encodings::BGRA8;

  return image;
}

std::shared_ptr<sensor_msgs::msg::Image> ir_frame_to_image(
  libfreenect2::Frame * frame, const int & width, const int & height)
{
  auto mat = frame_to_mat(frame, CV_32FC1);
  auto resized_mat = resize_mat(mat, width, height);

  resized_mat.convertTo(resized_mat, CV_8UC1, 256.0 / 65535.0);

  auto image = mat_to_image(resized_mat);
  image->encoding = sensor_msgs::image_encodings::MONO8;

  return image;
}

sensor_msgs::msg::CameraInfo camera_info_from_image(const sensor_msgs::msg::Image & image)
{
  sensor_msgs::msg::CameraInfo camera_info;

  camera_info.header = image.header;
  camera_info.height = image.height;
  camera_info.width = image.width;

  return camera_info;
}

}  // namespace kinect2
