/*
 *  Copyright (c) 2024, Northeastern University
 *  All rights reserved.
 *
 *  Author: Jibo Wang
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Northeastern University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include "wheel_encoder_processing.hpp"

WheelEncoderProcess::WheelEncoderProcess() {}

WheelEncoderProcess::~WheelEncoderProcess() {}

void WheelEncoderProcess::set_extrinsic(const Eigen::Matrix3d &rot, const Eigen::Vector3d &transl)
{
  Wheel_encoder_R_wrt_IMU = rot;
  Wheel_encoder_T_wrt_IMU = transl;
}
void WheelEncoderProcess::set_wheel_encoder_frequency(const int frequency)
{
  wheel_encoder_frequency_ = frequency;
}


void WheelEncoderProcess::set_wheel_encoder_pulse_resolution(const double pulse_resolution)
{
  wheel_encoder_pulse_resolution_ = pulse_resolution;
}


void WheelEncoderProcess::Process(const LidarMeasureGroup &meas, Eigen::Vector3d &wheel_encoder_delta_distance_in_WHEEL, Eigen::Vector3d &wheel_encoder_latest_vel_in_WHEEL)
{
  if(meas.wheel_encoder.empty()) {return;};
  ROS_ASSERT(meas.lidar != nullptr);

  auto v_wheel_encoder = meas.wheel_encoder;

  const double &imu_beg_time = v_wheel_encoder.front()->header.stamp.toSec();
  const double &imu_end_time = v_wheel_encoder.back()->header.stamp.toSec();

  int sum_wheel_encoder = 0;
  double latest_vel = 0;
  for (auto it_wheel_encoder = v_wheel_encoder.begin(); it_wheel_encoder < v_wheel_encoder.end(); it_wheel_encoder++)
  {
    auto &&cur_encoder = *(it_wheel_encoder);
    // cur_encoder->left_count = 0;
    sum_wheel_encoder += cur_encoder->right_count;
    if(it_wheel_encoder == v_wheel_encoder.end()-1)
    {
      latest_vel = cur_encoder->right_count * 1.0 * wheel_encoder_pulse_resolution_ * wheel_encoder_frequency_;
    }

  }
  
  double wheel_encoder_distance = sum_wheel_encoder * 1.0 *  wheel_encoder_pulse_resolution_;

  delta_distance_in_WHEEL =  Eigen::Vector3d(0, wheel_encoder_distance, 0);
  latest_vel_in_WHEEL =  Eigen::Vector3d(0, latest_vel, 0);

  wheel_encoder_delta_distance_in_WHEEL = delta_distance_in_WHEEL;
  wheel_encoder_latest_vel_in_WHEEL = latest_vel_in_WHEEL;

}
