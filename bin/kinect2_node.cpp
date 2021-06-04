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

#include <argparse/argparse.hpp>
#include <rclcpp/rclcpp.hpp>
#include <kinect2/kinect2_node.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  auto program = argparse::ArgumentParser("kinect2_node", "0.1.0");

  program.add_argument("--disable-rgb")
  .help("disable RGB image capture and publication")
  .default_value(false)
  .implicit_value(true);

  program.add_argument("--disable-depth")
  .help("disable depth image capture and publication")
  .default_value(false)
  .implicit_value(true);

  kinect2::Kinect2Node::Options kinect2_options;

  // Try to parse arguments
  try {
    program.parse_args(argc, argv);

    kinect2_options.enable_rgb = !program.get<bool>("--disable-rgb");
    kinect2_options.enable_depth = !program.get<bool>("--disable-depth");
  } catch (const std::exception & e) {
    std::cout << e.what() << std::endl;
    std::cout << program;
    return 1;
  }

  rclcpp::init(argc, argv);

  // Try to start the Kinect V2
  try {
    auto kinect2_node = std::make_shared<kinect2::Kinect2Node>(kinect2_options);
    rclcpp::spin(kinect2_node);
  } catch (const std::exception & e) {
    std::cout << "Failed to initialize kinect2_node: " << e.what() << std::endl;
  }

  rclcpp::shutdown();

  return 0;
}
