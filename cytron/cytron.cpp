/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file Cytron.cpp
 *
 * Cytron Motor Driver
 *
 * references:
 * http://downloads.orionrobotics.com/downloads/datasheets/motor_controller_robo_claw_R0401.pdf
 *
 */

#include "cytron.hpp"
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h>

// The Cytron has a serial communication timeout of 10ms.
// Add a little extra to account for timing inaccuracy
#define TIMEOUT_US 10500

// If a timeout occurs during serial communication, it will immediately try again this many times
#define TIMEOUT_RETRIES 1

// If a timeout occurs while disarmed, it will try again this many times. This should be a higher number,
// because stopping when disarmed is pretty important.
#define STOP_RETRIES 10

// Number of bytes returned by the Cytron when sending command 78, read both encoders
#define ENCODER_MESSAGE_SIZE 10

// Number of bytes for commands 18 and 19, read speeds.
#define ENCODER_SPEED_MESSAGE_SIZE 7

bool Cytron::taskShouldExit = false;

Cytron::Cytron(const char *deviceName, const char *baudRateParam):
	_uart(0),
	_uart_set(),
	_uart_timeout{.tv_sec = 0, .tv_usec = TIMEOUT_US},
	_actuatorsSub(-1),
	_motorSpeeds{0}

{
	_param_handles.actuator_write_period_ms = 	param_find("CYTR_WRITE_PER");
	_param_handles.serial_baud_rate = 			param_find(baudRateParam);
	_param_handles.address = 					param_find("CYTR_ADDRESS");

	_parameters_update();

	// start serial port
	_uart = open(deviceName, O_RDWR | O_NOCTTY);

	if (_uart < 0) { err(1, "could not open %s", deviceName); }

	int ret = 0;
	struct termios uart_config {};
	ret = tcgetattr(_uart, &uart_config);

	if (ret < 0) { err(1, "failed to get attr"); }

	uart_config.c_oflag &= ~ONLCR; // no CR for every LF
	ret = cfsetispeed(&uart_config, _parameters.serial_baud_rate);

	if (ret < 0) { err(1, "failed to set input speed"); }

	ret = cfsetospeed(&uart_config, _parameters.serial_baud_rate);

	if (ret < 0) { err(1, "failed to set output speed"); }

	ret = tcsetattr(_uart, TCSANOW, &uart_config);

	if (ret < 0) { err(1, "failed to set attr"); }

	FD_ZERO(&_uart_set);
}

Cytron::~Cytron()
{
	setMotorDutyCycle(MOTOR_1, 0.0);
	close(_uart);
}

void Cytron::taskMain()
{
	uint8_t rbuff[4];

	// This main loop performs two different tasks, asynchronously:
	// - Send actuator_controls_0 to the Cytron as soon as they are available
	// - Read the encoder values at a constant rate
	// To do this, the timeout on the poll() function is used.
	// waitTime is the amount of time left (int microseconds) until the next time I should read from the encoders.
	// It is updated at the end of every loop. Sometimes, if the actuator_controls_0 message came in right before
	// I should have read the encoders, waitTime will be 0. This is fine. When waitTime is 0, poll() will return
	// immediately with a timeout. (Or possibly with a message, if one happened to be available at that exact moment)
	//uint64_t encoderTaskLastRun = 0;
	int waitTime = 0;

	uint64_t actuatorsLastWritten = 0;

	_actuatorsSub = orb_subscribe(ORB_ID(actuator_controls_0));
	orb_set_interval(_actuatorsSub, _parameters.actuator_write_period_ms);

	_armedSub = orb_subscribe(ORB_ID(actuator_armed));
	_paramSub = orb_subscribe(ORB_ID(parameter_update));

	pollfd fds[3];
	fds[0].fd = _paramSub;
	fds[0].events = POLLIN;
	fds[1].fd = _actuatorsSub;
	fds[1].events = POLLIN;
	fds[2].fd = _armedSub;
	fds[2].events = POLLIN;




	while (!taskShouldExit) {

		int pret = poll(fds, sizeof(fds) / sizeof(pollfd), waitTime / 1000);

		bool actuators_timeout = int(hrt_absolute_time() - actuatorsLastWritten) > 2000 * _parameters.actuator_write_period_ms;

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(parameter_update), _paramSub, &_paramUpdate);
			_parameters_update();
		}

		// No timeout, update on either the actuator controls or the armed state
		if (pret > 0 && (fds[1].revents & POLLIN || fds[2].revents & POLLIN || actuators_timeout)) {
			orb_copy(ORB_ID(actuator_controls_0), _actuatorsSub, &_actuatorControls);
			orb_copy(ORB_ID(actuator_armed), _armedSub, &_actuatorArmed);

			int drive_ret = 0;
			int turn_ret = 0;

			const bool disarmed = !_actuatorArmed.armed || _actuatorArmed.lockdown || _actuatorArmed.manual_lockdown
					      || _actuatorArmed.force_failsafe || actuators_timeout;

			if (disarmed) {
				// If disarmed, I want to be certain that the stop command gets through.
				int tries = 0;

				while (tries < STOP_RETRIES && ((drive_ret = drive(0.0)) <= 0 || (turn_ret = turn(0.0)) <= 0)) {
					PX4_ERR("Error trying to stop: Drive: %d, Turn: %d", drive_ret, turn_ret);
					tries++;
					px4_usleep(TIMEOUT_US);
				}

			} else {
				drive_ret = setMotorDutyCycle(_actuatorControls.control[actuator_controls_s::INDEX_THROTTLE]);

				if (drive_ret <= 0) {
					PX4_ERR("Error controlling Cytron. Drive err: %d. ", drive_ret);
				}
			}

			actuatorsLastWritten = hrt_absolute_time();
	}

	orb_unsubscribe(_actuatorsSub);
	orb_unsubscribe(_armedSub);
	orb_unsubscribe(_paramSub);
	}
}

void Cytron::showStatus(char *string, size_t n)
{
	snprintf(string, n, "spd1: %10.2f \n",
		 double(getMotorSpeed(MOTOR_1)));
}

float Cytron::getMotorSpeed(e_motor motor)
{
	if (motor == MOTOR_1) {
		return _motorSpeeds[0];
	} else {
		warnx("Unknown motor value passed to Cytron::getMotorPosition");
		return NAN;
	}
}

int Cytron::setMotorDutyCycle(float value)
{
	return _sendUnsigned7Bit(value);
}

int Cytron::_sendUnsigned7Bit(float data)
{
	data = fabs(data);

	auto byte = (uint8_t)(data * INT8_MAX);
	uint8_t recv_byte;
	return _transaction(&byte, 1, &recv_byte, 1);
}

int Cytron::_sendSigned16Bit(e_command command, float data)
{
	if (data > 1.0f) {
		data = 1.0f;

	} else if (data < -1.0f) {
		data = -1.0f;
	}

	auto buff = (uint16_t)(data * INT16_MAX);
	uint8_t recv_buff;
	return _transaction(command, (uint8_t *) &buff, 2, &recv_buff, 1);
}

int Cytron::_sendNothing(e_command command)
{
	uint8_t recv_buff;
	return _transaction(command, nullptr, 0, &recv_buff, 1);
}

uint8_t Cytron::_calcCRC(const uint8_t *buf, size_t n, uint8_t init)
{
	uint8_t crc = init;
	//Header + Address + Command
	for (size_t byte = 0; byte < (n - 1) ; byte++) {
		crc += buf[byte];
	}

	return crc;
}

int Cytron::_transaction(uint8_t *wbuff, size_t wbytes,
			   uint8_t *rbuff, size_t rbytes, bool send_checksum, bool recv_checksum)
{
	int err_code = 0;

	// WRITE
	tcflush(_uart, TCIOFLUSH); // flush  buffers
	uint8_t buf[wbytes + 3];
	buf[0] = 85;
	buf[1] = 0x00;

	//Value
	if (wbuff) {
		memcpy(&buf[2], wbuff, wbytes);
	}

	if (send_checksum) {
		uint8_t sum = _calcCRC(buf, 4);
		buf[3] = sum;
	}

	int count = write(_uart, buf, wbytes);

	if (count < (int) wbytes) { // Did not successfully send all bytes.
		PX4_ERR("Only wrote %d out of %zu bytes", count, wbytes);
		return -1;
	}

	// READ

	FD_ZERO(&_uart_set);
	FD_SET(_uart, &_uart_set);

	uint8_t *rbuff_curr = rbuff;
	size_t bytes_read = 0;

	// select(...) returns as soon as even 1 byte is available. read(...) returns immediately, no matter how many
	// bytes are available. I need to keep reading until I get the number of bytes I expect.
	while (bytes_read < rbytes) {
		// select(...) may change this timeout struct (because it is not const). So I reset it every time.
		_uart_timeout.tv_sec = 0;
		_uart_timeout.tv_usec = TIMEOUT_US;
		err_code = select(_uart + 1, &_uart_set, nullptr, nullptr, &_uart_timeout);

		// An error code of 0 means that select timed out, which is how the Cytron indicates an error.
		if (err_code <= 0) {
			return err_code;
		}

		err_code = read(_uart, rbuff_curr, rbytes - bytes_read);

		if (err_code <= 0) {
			return err_code;

		} else {
			bytes_read += err_code;
			rbuff_curr += err_code;
		}
	}

	//TODO: Clean up this mess of IFs and returns

	if (recv_checksum) {
		if (bytes_read < 2) {
			return -1;
		}

		// The checksum sent back by the cytron is calculated based on the address and command bytes as well
		// as the data returned.
		uint16_t checksum_calc = _calcCRC(buf, 2);
		checksum_calc = _calcCRC(rbuff, bytes_read - 2, checksum_calc);
		uint16_t checksum_recv = (rbuff[bytes_read - 2] << 8) + rbuff[bytes_read - 1];

		if (checksum_calc == checksum_recv) {
			return bytes_read;

		} else {
			return -10;
		}

	} else {
		if (bytes_read == 1 && rbuff[0] == 0xFF) {
			return 1;

		} else {
			return -11;
		}
	}
}

void Cytron::_parameters_update()
{
	param_get(_param_handles.actuator_write_period_ms, &_parameters.actuator_write_period_ms);
	param_get(_param_handles.address, &_parameters.address);

	if (_actuatorsSub > 0) {
		orb_set_interval(_actuatorsSub, _parameters.actuator_write_period_ms);
	}

	int32_t baudRate;
	param_get(_param_handles.serial_baud_rate, &baudRate);

	switch (baudRate) {
	case 2400:
		_parameters.serial_baud_rate = B2400;
		break;

	case 9600:
		_parameters.serial_baud_rate = B9600;
		break;

	case 19200:
		_parameters.serial_baud_rate = B19200;
		break;

	case 38400:
		_parameters.serial_baud_rate = B38400;
		break;

	case 57600:
		_parameters.serial_baud_rate = B57600;
		break;

	case 115200:
		_parameters.serial_baud_rate = B115200;
		break;

	case 230400:
		_parameters.serial_baud_rate = B230400;
		break;

	case 460800:
		_parameters.serial_baud_rate = B460800;
		break;

	default:
		_parameters.serial_baud_rate = B2400;
	}
}
