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
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

ThrustAccControl::ThrustAccControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{

	parameters_updated();
}

ThrustAccControl::~ThrustAccControl()
{
	perf_free(_loop_perf);
}

bool
ThrustAccControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void ThrustAccControl::resetButterworthFilter(){
_lpf.initButterSysLowpass(0,0,10,250);
}


void
ThrustAccControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	// const Vector3f rate_k = Vector3f(_param_thr_p.get(), 0., 0.);
	_thr_p = _param_thr_p.get();
	_thr_cur_a = _param_thr_curve_a.get();
	_thr_cur_b = _param_thr_curve_b.get();
	_timeout_acc = _param_thr_timeout_acc.get();
	_timeout_time = _param_sys_timeout_time.get() * 1e5;

}


float ThrustAccControl::get_u_inverse_model(float target_at) {
	// TODO check max and min
	return (-_thr_cur_b + sqrtf(_thr_cur_b * _thr_cur_b - 4 * _thr_cur_a * (-target_at))) / (2 * _thr_cur_a);
}


void
ThrustAccControl::Run()
{
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
	if (_vehicle_attitude_sub.updated()){
	vehicle_attitude_s vehicle_attitude;
	_vehicle_attitude_sub.update(&vehicle_attitude);
	matrix::Quaternionf q_now(vehicle_attitude.q[0],vehicle_attitude.q[1],vehicle_attitude.q[2],vehicle_attitude.q[3]);
	_rotate_q = q_now.inversed();
	// _rotate_q.print();
	}
	/* run controller on gyro changes */
	// vehicle_angular_velocity_s linear_acc_b;
	resetButterworthFilter();
	// Accordiing sensor gyro
	if (_vehicle_angular_velocity_sub.updated()) {

		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		// // must enable thrust_acc_control to allow it control VehicleThrust
		if (_vehicle_control_mode.flag_control_thrust_acc_enabled == true) {
		_vehicle_thrust_acc_setpoint_sub.update();
		_last_run = _vehicle_thrust_acc_setpoint_sub.get().timestamp;
		_thrust_acc_sp = _vehicle_thrust_acc_setpoint_sub.get().thrust_acc_sp;
		_rates_setpoint = matrix::Vector3f(_vehicle_thrust_acc_setpoint_sub.get().rates_sp);

		if (_last_run && hrt_absolute_time()  -  _last_run > _timeout_time)
		{
			PX4_WARN("Haven't Received Thrust Acc Setpoint Messages! Restored to Hold mode");
			PX4_WARN("Timestamp: %ld", (int64_t)(hrt_absolute_time()));
			_thrust_acc_sp = _timeout_acc;
			_rates_setpoint(0) = 0.0;
			_rates_setpoint(1) = 0.0;
			_rates_setpoint(2) = 0.0;
		}

		// _thrust_acc_setpoint_msg.timestamp
		// if(_thrust_acc_sp.timestamp)
		float normalized_u = get_u_inverse_model(_thrust_acc_sp);


		// adding P controller
		_vacc_sub.update();
		// change to FLU setting
		float acc_body_z = -_vacc_sub.get().xyz[2];
		normalized_u += (_thrust_acc_sp - acc_body_z) * _thr_p;
		normalized_u = math::constrain<float>(normalized_u, 0.0, 1.0);
		// 	// mavlink_log_critical(&_mavlink_log_pub, "in thrust_acc mode");
		// _vehicle_thrust_acc_setpoint_sub.update();
		// // vehicle_thrust_acc_setpoint_s tmp = _vehicle_thrust_acc_setpoint_sub.get();
		vehicle_rates_setpoint_s vehicle_rates_setpoint{};
		// // tmp.thrust_acc_sp = 10;
		// if(tmp.thrust_acc_sp - acc_t(2) > 0 )
		// vehicle_rates_setpoint.thrust_body[2] =  -math::constrain<float>(tmp.thrust_acc_sp / _thr_slope + (tmp.thrust_acc_sp - acc_t(2)) * _thr_p / _thr_slope, 0.0, 1.0);
		// else
		// vehicle_rates_setpoint.thrust_body[2] =  -math::constrain<float>(tmp.thrust_acc_sp / _thr_slope , 0.0, 1.0);

		// vehicle_rates_setpoint.thrust_body[2] =  - 0.8;
		// PX4_INFO("%f thrust", (double) vehicle_rates_setpoint.thrust_body[2]);
		// PX4_INFO("%f thurust_acc_sp", (double) tmp.thrust_acc_sp);
		// PX4_INFO("%f first part ", (double) (tmp.thrust_acc_sp / _thr_slope ));
		// PX4_INFO("%f tmp.thrust_acc_sp - acc_t(2) ", (double) (tmp.thrust_acc_sp - acc_t(2))  );
		// PX4_INFO("%f tmp.thrust_acc_sp - acc_t(2) ", (double) (tmp.thrust_acc_sp )  );
		// PX4_INFO("%f tmp.thrust_acc_sp - acc_t(2) ", (double) (normalized_u )  );
		// PX4_INFO("%f (tmp.thrust_acc_sp - acc_t(2)) * _thr_p / _thr_slope ", (double)((tmp.thrust_acc_sp - acc_t(2)) * _thr_p / _thr_slope ));
		vehicle_rates_setpoint.thrust_body[2] = -normalized_u;
		vehicle_rates_setpoint.roll = _rates_setpoint(0);
		vehicle_rates_setpoint.pitch = _rates_setpoint(1);
		vehicle_rates_setpoint.yaw = _rates_setpoint(2);
		vehicle_rates_setpoint.timestamp = hrt_absolute_time();
		_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);
		}
		// use rates setpoint topic

		// mavlink_log_info(&_mavlink_log_pub, "%f", (double)vehicle_rates_setpoint.roll);


	}

	perf_end(_loop_perf);
}



int ThrustAccControl::task_spawn(int argc, char *argv[])
{

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

int ThrustAccControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ThrustAccControl::print_usage(const char *reason)
{
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

extern "C" __EXPORT int thrust_acc_control_main(int argc, char *argv[])
{
	return ThrustAccControl::main(argc, argv);
}
