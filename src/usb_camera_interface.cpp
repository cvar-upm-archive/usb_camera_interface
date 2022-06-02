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
#include <memory>

double cm[3][3] = {{519.9198295939999, 0.0, 659.1468132131484},
                   {0.0, 502.46760776515197, 357.57506142772786},
                   {0.0, 0.0, 1.0}};
double dc[3][3] = {-0.021303744666207214, -0.006628539603283135, -0.007097678030316164,
                   0.002559386685475455};

UsbCameraInterface::UsbCameraInterface() : as2::Node("usb_camera_interface") {
  std::string ns = this->get_namespace();
  device_port_ = "/dev/video2";

  // loadParameters();
  // setCameraInfo(camera_matrix_, dist_coeffs_);
  
  img_transport_ = std::make_shared<as2::sensors::Camera>("camera", this);
  setupCamera();

  // create timer for image capture
  static auto image_capture_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&UsbCameraInterface::captureImage, this));
};

void UsbCameraInterface::loadParameters() {
  camera_matrix_ = cv::Mat(3, 3, CV_64F, &cm);
  dist_coeffs_ = cv::Mat(1, 5, CV_64F, &dc);
}

void UsbCameraInterface::setCameraInfo(const cv::Mat &_camera_matrix, const cv::Mat &_dist_coeffs) {
  RCLCPP_INFO(get_logger(), "Setting camera info");
  sensor_msgs::msg::CameraInfo camera_info;

  camera_info.k[0] = _camera_matrix.at<double>(0, 0);
  camera_info.k[1] = _camera_matrix.at<double>(0, 1);
  camera_info.k[2] = _camera_matrix.at<double>(0, 2);
  camera_info.k[3] = _camera_matrix.at<double>(1, 0);
  camera_info.k[4] = _camera_matrix.at<double>(1, 1);
  camera_info.k[5] = _camera_matrix.at<double>(1, 2);
  camera_info.k[6] = _camera_matrix.at<double>(2, 0);
  camera_info.k[7] = _camera_matrix.at<double>(2, 1);
  camera_info.k[8] = _camera_matrix.at<double>(2, 2);

  camera_info.d.emplace_back(_dist_coeffs.at<double>(0, 0));
  camera_info.d.emplace_back(_dist_coeffs.at<double>(0, 1));
  camera_info.d.emplace_back(_dist_coeffs.at<double>(0, 2));
  camera_info.d.emplace_back(_dist_coeffs.at<double>(0, 3));
  camera_info.d.emplace_back(_dist_coeffs.at<double>(0, 4));

  img_transport_->setParameters(camera_info);
}

void UsbCameraInterface::setupCamera() {
  cap_.open(device_port_);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(get_logger(), "Cannot open device");
    return;
  }

  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  // cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
  // cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap_.set(cv::CAP_PROP_FPS, 30);
}

void UsbCameraInterface::captureImage() {
  // Capture image in device with opencv2

  cv::Mat frame;
  if (!cap_.read(frame)) {
    RCLCPP_ERROR(get_logger(), "Cannot read image");
    return;
  }
  // cv::imshow("frame", frame);
  // cv::waitKey(1);
  // Convert to sensor_msgs::msg::Image
  sensor_msgs::msg::Image img_msg;
  cv_bridge::CvImage cv_img;
  cv_img.header.frame_id = "camera_link"; // TODO: check tf2 link name
  cv_img.header.stamp =this->now();
  cv_img.encoding = sensor_msgs::image_encodings::BGR8;
  cv_img.image = frame;
  cv_img.toImageMsg(img_msg);

  // Publish image
  img_transport_->updateData(img_msg);

  // imshow
}

