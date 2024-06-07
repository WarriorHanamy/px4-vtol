/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <lib/adv_control_lib/butterworth_filter.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

#include <lib/matrix/matrix/math.hpp>
#include <lib/rate_control/rate_control.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
// #include <uORB/topics/vehicle_
// #include <uORB/topics/vehithrus
#include <uORB/topics/vehicle_thrust_acc_setpoint.h>

#include <AttitudeControl.hpp>

// #include <uORB/topics/vehilce_thrust_acc_setpoint.h>
using namespace time_literals;

class ThrustAccControl : public ModuleBase<ThrustAccControl>,
                         public ModuleParams,
                         public px4::WorkItem {
 public:
  ThrustAccControl();
  ~ThrustAccControl() override;

  /** @see ModuleBase */
  static int task_spawn(int argc, char *argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char *argv[]);

  /** @see ModuleBase */
  static int print_usage(const char *reason = nullptr);

  bool init();

 private:
  void Run() override;
  void resetButterworthFilter();
  /**
   * initialize some vectors/matrices from parameters
   */
  void parameters_updated();

  float get_u_inverse_model(float target_at);

  uORB::SubscriptionData<vehicle_thrust_acc_setpoint_s>
      _vehicle_thrust_acc_setpoint_sub{ORB_ID(vehicle_thrust_acc_setpoint)};
  uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
  uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
  uORB::SubscriptionData<vehicle_attitude_s> _vehicle_attitude_sub{
      ORB_ID(vehicle_attitude)};
  //   uORB::SubscriptionData<
  uORB::SubscriptionData<sensor_gyro_s> _acc_b_sub{ORB_ID(sensor_gyro)};
  uORB::SubscriptionData<vehicle_acceleration_s> _vacc_sub{
      ORB_ID(vehicle_acceleration)};

  uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update),
                                                   1_s};
  uORB::SubscriptionData<vehicle_thrust_setpoint_s>
      _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};
  uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{
      this, ORB_ID(vehicle_angular_velocity)};

  uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub{
      ORB_ID(vehicle_rates_setpoint)};

  orb_advert_t _mavlink_log_pub{nullptr};  ///< mavlink log pub

  RateControl _rate_control;

  vehicle_control_mode_s _vehicle_control_mode{};

  float thrust_sp;

  hrt_abstime _last_run{0};

  perf_counter_t _loop_perf; /**< loop duration performance counter */

  // keep setpoint values between updates
  matrix::Vector3f _rates_setpoint{};
  vehicle_thrust_acc_setpoint_s _thrust_acc_setpoint_msg;
  math::LowPassFilter2p<float> _thrust_sp_lpf{};
  float _u_prev = 0.0;
  float _u = 0.0;
  float _a_curr;
  float _thrust_acc_sp{};
  matrix::Quaternionf _rotate_q{};
  float _thr_p;
  float _beta;
  float _thr_model_ff;
  // we assumes the model of thrust is quadratic, i.e. a_t = a*u
  float _delta_thr_bound;

  float _timeout_acc = 9.81;
  uint64_t _timeout_time = 2;
  //   ButterworthFilter2nd _thrust_splpf;
  bool _is_sim = false;

  AttitudeControl _attitude_control;

  DEFINE_PARAMETERS(
      (ParamFloat<px4::params::MC_ROLL_P>)_param_mc_roll_p,
      (ParamFloat<px4::params::MC_PITCH_P>)_param_mc_pitch_p,
      (ParamFloat<px4::params::MC_YAW_P>)_param_mc_yaw_p,
      (ParamFloat<px4::params::MC_YAW_WEIGHT>)_param_mc_yaw_weight,
      (ParamFloat<px4::params::MC_ROLLRATE_MAX>)_param_mc_rollrate_max,
      (ParamFloat<px4::params::MC_PITCHRATE_MAX>)_param_mc_pitchrate_max,
      (ParamFloat<px4::params::MC_YAWRATE_MAX>)_param_mc_yawrate_max,

      (ParamFloat<px4::params::THR_P>)_param_thr_p,
      (ParamFloat<px4::params::THR_TMO_ACC>)_param_thr_timeout_acc,
      (ParamFloat<px4::params::GYROX_CUTOFF>)_param_imu_gyro_cutoff,
      (ParamFloat<px4::params::THR_DELTA_BOUND>)_param_delta_thr_bound,
      (ParamFloat<px4::params::THR_LPF_CUTOFF>)_param_thr_lpf_cutoff_frq,
      (ParamFloat<px4::params::THR_BETA>)_param_beta,
      (ParamBool<px4::params::THR_SIM>)_param_thr_sim,
      (ParamInt<px4::params::THR_TMO_TIME>)_param_sys_timeout_time)
};
