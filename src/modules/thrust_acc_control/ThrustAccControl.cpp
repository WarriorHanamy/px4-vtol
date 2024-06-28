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

#include "ThrustAccControl.hpp"

#include <drivers/drv_hrt.h>
// #include <circuit_breaker/circuit_breaker.h>
#include <px4_platform_common/events.h>

#include <mathlib/math/Functions.hpp>
#include <mathlib/math/Limits.hpp>

using namespace matrix;
using namespace time_literals;
using math::radians;

ThrustAccControl::ThrustAccControl()
    : ModuleParams(nullptr),
      WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
      _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")) {
  parameters_updated();
}

ThrustAccControl::~ThrustAccControl() { perf_free(_loop_perf); }

bool ThrustAccControl::init() {
  if (!_vehicle_angular_velocity_sub.registerCallback()) {
    PX4_ERR("callback registration failed");
    return false;
  }

  return true;
}

void ThrustAccControl::resetButterworthFilter() {
  // _lpf.initButterSysLowpass(0, 0, 10, 250);
  _thrust_sp_lpf.set_cutoff_frequency(_param_imu_gyro_cutoff.get(),
                                      _param_thr_lpf_cutoff_frq.get());

  _thrust_sp_lpf.reset(_u_prev);
}

void ThrustAccControl::parameters_updated() {
  // rate control parameters
  // The controller gain K is used to convert the parallel (P + I/s + sD) form
  // to the ideal (K * [1 + 1/sTi + sTd]) form
  // const Vector3f rate_k = Vector3f(_param_thr_p.get(), 0., 0.);
  _thr_p = _param_thr_p.get();
  _delta_thr_bound = _param_delta_thr_bound.get();
  _timeout_acc = _param_thr_timeout_acc.get();
  _timeout_time = _param_sys_timeout_time.get() * 1e5;
  _is_sim = _param_thr_sim.get();
  _beta = _param_beta.get();

  _acc_limit = _param_thr_sft_acc.get();
  _rate_limit = _param_thr_sft_rate.get();

  _attitude_control.setProportionalGain(
      Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(),
               _param_mc_yaw_p.get()),
      _param_mc_yaw_weight.get());

  // angular rate limits
  using math::radians;
  _attitude_control.setRateLimit(
      Vector3f(radians(_param_mc_rollrate_max.get()),
               radians(_param_mc_pitchrate_max.get()),
               radians(_param_mc_yawrate_max.get())));
  resetButterworthFilter();
}

float ThrustAccControl::get_u_inverse_model(float target_at) {
  // TODO check max and min
  return target_at / (float)(9.8) * (float)(0.62063);
}

void ThrustAccControl::Run() {
  if (should_exit()) {
    _vehicle_angular_velocity_sub.unregisterCallback();
    exit_and_cleanup();
    return;
  }

  perf_begin(_loop_perf);

  // Check if parameters have changed
  if (_parameter_update_sub.updated()) {
    // clear update
    parameter_update_s param_update;
    _parameter_update_sub.copy(&param_update);

    updateParams();
    parameters_updated();
  }

  // IMPORTANT
  _vehicle_thrust_acc_setpoint_sub.update();
  _vehicle_thrust_setpoint_sub.update();
  _vehicle_attitude_sub.update();
  // float last_thrust_sp = 0.0;
  if (_vehicle_angular_velocity_sub.updated()) {
    _vehicle_control_mode_sub.update(&_vehicle_control_mode);
    // // must enable thrust_acc_control to allow it control VehicleThrust
    _vehicle_thrust_acc_setpoint_sub.update();
    _vacc_sub.update();
    _can_run_offboard = (_vehicle_control_mode.flag_control_offboard_enabled &&
                         _vehicle_control_mode.flag_control_rates_enabled);
    if (_last_can_run && _can_run_offboard) {
      _last_run = _vehicle_thrust_acc_setpoint_sub.get().timestamp;
      _thrust_acc_sp = _vehicle_thrust_acc_setpoint_sub.get().thrust_acc_sp;
      _rates_setpoint =
          matrix::Vector3f(_vehicle_thrust_acc_setpoint_sub.get().rates_sp);
      _thr_model_ff = _vehicle_thrust_acc_setpoint_sub.get().model_ff;
      if (!safeCheck()) {
        static uint64_t printer_counter_saft = 0;
        if (printer_counter_saft % 10 == 0) {
          printer_counter_saft++;
          PX4_WARN("Safety Check Failed");
        }
        safeAttitudeHolder();
      }
      if (hrt_elapsed_time(&_last_run) > _timeout_time) {
        static uint64_t print_counter = 0;
        if (print_counter % 10 == 0) {
          print_counter++;
          PX4_WARN("MPC Timeout");
        }
        safeAttitudeHolder();
      }

      _u_prev = -_vehicle_thrust_setpoint_sub.get().xyz[2];
      // change to FLU setting
      _a_curr = -_vacc_sub.get().xyz[2];
      float _du = (_thrust_acc_sp - _a_curr) * _thr_p;
      math::constrain(_du, -_delta_thr_bound, +_delta_thr_bound);
      _u = _du + _u_prev;

      // TODO MPC phase compensation
      // if (_thrust_acc_sp - last_thrust_sp > 0) {
      //   if (_a_curr < _thrust_acc_sp + (float)(.3 * 1e-1)) {
      //     _u += (float)(0.0005);
      //   }
      // } else if (_thrust_acc_sp - last_thrust_sp < 0) {
      //   _u -= _u;
      // } else {
      //   _u = _u;
      // }
      // last_thrust_sp = _thrust_acc_sp;

      _u = (1 - _beta) * _u + _beta * _thr_model_ff;
      _u = math::constrain<float>(_u, 0.0, 1.0);
      vehicle_rates_setpoint_s vehicle_rates_setpoint{};
      vehicle_rates_setpoint.thrust_body[2] = -_u;
      vehicle_rates_setpoint.roll = _rates_setpoint(0);
      vehicle_rates_setpoint.pitch = _rates_setpoint(1);
      vehicle_rates_setpoint.yaw = _rates_setpoint(2);
      vehicle_rates_setpoint.timestamp = hrt_absolute_time();
      _vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
    } else {
      _u_prev = -_vehicle_thrust_setpoint_sub.get().xyz[2];
      _u = _u_prev;
    }
    _last_can_run = _can_run_offboard;

    // use rates setpoint topic

    // mavlink_log_info(&_mavlink_log_pub, "%f",
    // (double)vehicle_rates_setpoint.roll);
  }

  perf_end(_loop_perf);
}

void ThrustAccControl::safeAttitudeHolder() {
  _thrust_acc_sp = _timeout_acc;
  // identity setpoint
  matrix::Quatf q_cur(_vehicle_attitude_sub.get().q);
  const float yaw = Eulerf(q_cur).psi();
  matrix::Quatf q_sp = Eulerf(0.0, 0.0, yaw);
  _attitude_control.setAttitudeSetpoint(q_sp, 0.0);
  _rates_setpoint = _attitude_control.update(q_cur);
}

bool ThrustAccControl::safeCheck() {
  // position check
  // TODO map limit

  // TODO trajectory error too large

  // TODO rates too high
  vehicle_angular_velocity_s _vehicle_angular_velocity;
  _vehicle_angular_velocity_sub.copy(&_vehicle_angular_velocity);
  matrix::Vector3f rates{_vehicle_angular_velocity.xyz};

  if (rates.norm() > _rate_limit) {
    return false;
  }
  // TODO accleration is too high
  matrix::Vector3f acc{_vacc_sub.get().xyz[0], _vacc_sub.get().xyz[1],
                       _vacc_sub.get().xyz[2] + 9.81f};
  if (acc.norm() > _acc_limit) {
    return false;
  }
  return true;
}

int ThrustAccControl::task_spawn(int argc, char *argv[]) {
  ThrustAccControl *instance = new ThrustAccControl();

  if (instance) {
    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->init()) {
      return PX4_OK;
    }

  } else {
    PX4_ERR("alloc failed");
  }

  delete instance;
  _object.store(nullptr);
  _task_id = -1;

  return PX4_ERROR;
}

int ThrustAccControl::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int ThrustAccControl::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
This implements the thrust accleration control. It takes rate thrust_acc as inputs and vehilce_angular_rates_sepoint.

The controller has a PID loop for thrust acc error.

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("thrust_acc_control", "controller");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

extern "C" __EXPORT int thrust_acc_control_main(int argc, char *argv[]) {
  return ThrustAccControl::main(argc, argv);
}
