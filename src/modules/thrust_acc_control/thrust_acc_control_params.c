/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

/**
 * @file thrust_acc_control_params.c
 * Parameters for thrust_acc control
 *
 * @author WarriorHanamy rongerch@mail2.sysu.edu.cn
 */

/**
 * thrust accleration error P gain
 *
 * [at_sp - hat(at)] * K * (P+I*1/s+D*s) + thrust_ff = Thrust_setpoint
 *
 * @min 0.001
 * @max 0.008
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(THR_P, 0.005f);


/**
 * thrust setpoint low pass filter cutoff frequency 
 *
 * thrust setpoint low pass filter cutoff frequency 
 *
 * @min 0.00
 * @max 100.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(THR_LPF_CUTOFF, 50.0f);

/**
 * thrust curve over-confident SLOPE
 *
 * [at_sp - hat(at)] * K * (P+I*1/s+D*s) + thrust_ff = Thrust_setpoint
 *
 * @min 20.0
 * @max 200.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(THR_CUR_LIN_K, 150.0f);






/**
 * thrust contrl timeout acc when mpc is lost
 *
 * thrust contrl timeout acc when mpc is lost
 *
 * @min 9.8
 * @max 12.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(THR_TMO_ACC, 10.0f);



/**
 * thrust contrl timeout time when mpc is lost  [0.1s]
 *
 * thrust contrl timeout time when mpc is lost, for example,  5 is 0.5s.
 *
 * @min 1
 * @max 10
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(THR_TMO_TIME, 2);
