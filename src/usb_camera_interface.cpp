/*!*******************************************************************************************
 *  \file       usb_camera_interface.cpp
 *  \brief      Aruco gate detector implementation file.
 *  \authors    David Perez Saura
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "usb_camera_interface.hpp"

UsbCameraInterface::UsbCameraInterface() : as2::Node("usb_camera_interface")
{
  loadParameters();

  img_transport_ = std::make_shared<as2::sensors::Camera>(camera_name_, this);

  setCameraInfo(camera_matrix_, dist_coeffs_);
  setupCamera();

  // create timer for image capture
  static auto image_capture_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&UsbCameraInterface::captureImage, this));
};

void UsbCameraInterface::loadParameters()
{
  this->declare_parameter<std::string>("video_device");
  this->declare_parameter<std::string>("camera_name");
  this->declare_parameter<int>("image_width");
  this->declare_parameter<int>("image_height");
  this->declare_parameter<double>("framerate");
  this->declare_parameter<std::string>("distortion_model");
  this->declare_parameter<std::vector<double>>("camera_matrix.data");
  this->declare_parameter<std::vector<double>>("distortion_coefficients.data");

  this->get_parameter("video_device", device_port_);
  this->get_parameter("camera_name", camera_name_);
  this->get_parameter("image_width", image_width_);
  this->get_parameter("image_height", image_height_);
  this->get_parameter("framerate", framerate_);
  this->get_parameter("distortion_model", distortion_model_);

  rclcpp::Parameter cm_param = this->get_parameter("camera_matrix.data");
  rclcpp::Parameter dc_param = this->get_parameter("distortion_coefficients.data");

  std::vector<double> cm_param_vec = cm_param.as_double_array();
  std::vector<double> dc_param_vec = dc_param.as_double_array();

  camera_matrix_ = cv::Mat(3, 3, CV_64F, cm_param_vec.data()).clone();
  dist_coeffs_ = cv::Mat(1, dc_param_vec.size(), CV_64F, dc_param_vec.data()).clone();

  RCLCPP_INFO(this->get_logger(), "Video device: %s", device_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "Camera name: %s", camera_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Image width: %d", image_width_);
  RCLCPP_INFO(this->get_logger(), "Image height: %d", image_height_);
  RCLCPP_INFO(this->get_logger(), "Framerate: %f", framerate_);
  RCLCPP_INFO(this->get_logger(), "Distortion model: %s", distortion_model_.c_str());

  std::cout << camera_matrix_ << std::endl;
  std::cout << dist_coeffs_ << std::endl;
}

void UsbCameraInterface::setCameraInfo(const cv::Mat &_camera_matrix, const cv::Mat &_dist_coeffs)
{
  RCLCPP_INFO(get_logger(), "Setting camera info");
  sensor_msgs::msg::CameraInfo camera_info;

  camera_info.width = image_width_;
  camera_info.height = image_height_;

  camera_info.k[0] = _camera_matrix.at<double>(0, 0);
  camera_info.k[1] = _camera_matrix.at<double>(0, 1);
  camera_info.k[2] = _camera_matrix.at<double>(0, 2);
  camera_info.k[3] = _camera_matrix.at<double>(1, 0);
  camera_info.k[4] = _camera_matrix.at<double>(1, 1);
  camera_info.k[5] = _camera_matrix.at<double>(1, 2);
  camera_info.k[6] = _camera_matrix.at<double>(2, 0);
  camera_info.k[7] = _camera_matrix.at<double>(2, 1);
  camera_info.k[8] = _camera_matrix.at<double>(2, 2);

  for (int i = 0; i < _dist_coeffs.cols; i++)
  {
    camera_info.d.emplace_back(_dist_coeffs.at<double>(0, i));
  }

  img_transport_->setParameters(camera_info);

  RCLCPP_INFO(get_logger(), "Camera info set");
}

void UsbCameraInterface::setupCamera()
{
  cap_.open(device_port_);
  if (!cap_.isOpened())
  {
    RCLCPP_ERROR(get_logger(), "Cannot open device");
    return;
  }

  cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
  cap_.set(cv::CAP_PROP_FPS, framerate_);
}

void UsbCameraInterface::captureImage()
{
  // Capture image in device with opencv2

  cv::Mat frame;
  if (!cap_.read(frame))
  {
    RCLCPP_ERROR(get_logger(), "Cannot read image");
    return;
  }
  // cv::imshow("frame", frame);
  // cv::waitKey(1);
  // Convert to sensor_msgs::msg::Image
  sensor_msgs::msg::Image img_msg;
  cv_bridge::CvImage cv_img;
  cv_img.header.frame_id = "camera_link"; // TODO: check tf2 link name
  cv_img.header.stamp = this->now();
  cv_img.encoding = sensor_msgs::image_encodings::BGR8;
  cv_img.image = frame;
  cv_img.toImageMsg(img_msg);

  // Publish image
  img_transport_->updateData(img_msg);

  // imshow
}
