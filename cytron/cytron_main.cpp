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
 * @file cytron_main.cpp
 *
 * cytron Motor Driver
 *
 * references:
 * http://downloads.ionmc.com/docs/roboclaw_user_manual.pdf
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <parameters/param.h>


#include "cytron.hpp"

static bool thread_running = false;     /**< Deamon status flag */
px4_task_t deamon_task;

/**
 * Deamon management function.
 */
extern "C" __EXPORT int cytron_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int cytron_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage();

static void usage()
{
	PRINT_MODULE_USAGE_NAME("cytron", "driver");

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description

This driver communicates over UART with the [Cytron motor driver](https://docs.google.com/document/d/1ECKsH1WfgZLomwFoPQ8PlkUzVEm88A6Sup9X0wDBHKA/view).
It performs one task:

- Control the motors based on the `actuator_controls_0` UOrb topic.

In order to use this driver, the Cytron should be put into Packet Serial mode (see the linked documentation), and
your flight controller's UART port should be connected to the Cytron as shown in the documentation. For Pixhawk 4,
use the `UART & I2C B` port, which corresponds to `/dev/ttyS3`.

### Implementation

The main loop of this module (Located in `Cytron.cpp::task_main()`) performs 1 task:

 1. Write `actuator_controls_0` messages to the Roboclaw as they become available

Because of the latency of UART, this driver does not write every single `actuator_controls_0` message to the Roboclaw
immediately. Instead, it is rate limited based on the parameter `RBCLW_WRITE_PER`.

On startup, this driver will attempt to read the status of the Roboclaw to verify that it is connected. If this fails,
the driver terminates immediately.

### Examples

The command to start this driver is:

```
$ cytron start <device> <baud>
```

- `<device>` is the name of the UART port. On the Pixhawk 4, this is `/dev/ttyS3`.
- `<baud>` is the baud rate.

All available commands are:

- `$ cytron start <device> <baud>`
- `$ cytron status`
- `$ cytron stop`
	)DESCR_STR");
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int cytron_main(int argc, char *argv[])
{

	if (argc < 4) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("cytron already running\n");
			/* this is not an error */
			return 0;
		}

		Cytron::taskShouldExit = false;
		deamon_task = px4_task_spawn_cmd("cytron",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 10,
						 2000,
						 cytron_thread_main,
						 (char *const *)argv);
		return 0;

	} else if (!strcmp(argv[1], "stop")) {

		Cytron::taskShouldExit = true;
		return 0;

	} else if (!strcmp(argv[1], "status")) {

		if (thread_running) {
			printf("\tcytron app is running\n");

		} else {
			printf("\tcytron app not started\n");
		}

		return 0;
	}

	usage();
	return 1;
}

int cytron_thread_main(int argc, char *argv[])
{
	printf("[cytron] starting\n");

	// skip parent process args
	argc -= 2;
	argv += 2;

	if (argc < 2) {
		printf("usage: cytron start <device> <baud>\n");
		return -1;
	}

	const char *deviceName = argv[1];
	const char *baudRate = argv[2];

	// start
	Cytron cytron(deviceName, baudRate);

	thread_running = true;

	cytron.taskMain();

	// exit
	printf("[cytron] exiting.\n");
	thread_running = false;
	return 0;
}
