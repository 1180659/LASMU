#include "cytron.h"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/uORB.h>

#include <cstdint>
#include <cstring>
#include <poll.h>
#include <unistd.h>

// Set the minimum and maximum duty cycle values for the motor
const float MIN_DUTY_CYCLE = 0.0f;
const float MAX_DUTY_CYCLE = 1.0f;

Cytron::Cytron(int pwm_index, int pwm_frequency) :
    _pwm_index(pwm_index),
    _pwm_frequency(pwm_frequency),
    _pwm_fd(-1),
    _actuator_controls_sub(-1),
    _actuator_outputs_pub(nullptr),
    _duty_cycle(MIN_DUTY_CYCLE)
{
}

int Cytron::init()
{
    // Open the PWM device
    char pwm_dev_path[16];
    snprintf(pwm_dev_path, sizeof(pwm_dev_path), PWM_OUTPUT_DEVICE_PATH, _pwm_index);
    _pwm_fd = px4_open(pwm_dev_path, 0);
    if (_pwm_fd < 0)
    {
        return -1;
    }

    // Set the PWM frequency for the motor
    if (ioctl(_pwm_fd, PWM_SERVO_SET_UPDATE_RATE, _pwm_frequency) < 0)
    {
        return -1;
    }

    // Set the initial duty cycle for the motor
    set_duty_cycle(_duty_cycle);

    // Subscribe to the actuator_controls topic
    _actuator_controls_sub = orb_subscribe(ORB_ID(actuator_controls));

    // Set the update rate for the actuator_controls topic
    orb_set_interval(_actuator_controls_sub, 1000 / _pwm_frequency);

    // Publish the initial value of the actuator_outputs topic
    actuator_outputs_s actuator_outputs = {};
    actuator_outputs.output[0] = _duty_cycle;
     _actuator_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &actuator_outputs);

    return 0;
}

void Cytron::set_duty_cycle(float duty_cycle)
{
    // Clamp the duty cycle to the minimum and maximum values
    if (duty_cycle < MIN_DUTY_CYCLE)
    {
        duty_cycle = MIN_DUTY_CYCLE;
    }
    else if (duty_cycle > MAX_DUTY_CYCLE)
    {
        duty_cycle = MAX_DUTY_CYCLE;
    }

    // Update the duty cycle for the motor
    _duty_cycle = duty_cycle;

    // Write the duty cycle to the PWM device
    uint32_t pwm_value = static_cast<uint32_t>(_duty_cycle * PWM_HIGHEST_MAX);
    ioctl(_pwm_fd, PWM_SERVO_SET(0), pwm_value);
}

void Cytron::start()
{
    // Create a thread to run the motor control loop
    px4_task_spawn_cmd("cytron",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       1500,
                       (px4_main_t)&motor_control_loop_trampoline,
                       nullptr);
}

void *Cytron::motor_control_loop_trampoline(void *context)
{
    // Cast the context to a Cytron instance and run the motor control loop
    Cytron *self = static_cast<Cytron *>(context);
    self->motor_control_loop();
    return nullptr;
}

void Cytron::motor_control_loop()
{
    // Set the thread name
    px4_prctl(PR_SET_NAME, "cytron", px4_getpid());

    // Poll for new data on the actuator_controls topic
    actuator_controls_s actuator_controls = {};
    pollfd fds[1] = {};
    fds[0].fd = _actuator_controls_sub;
    fds[0].events = POLLIN;
    int ret = 0;

    while (!px4_task_should_exit() && (ret = poll(fds, 1, 1000)) >= 0)
    {
        // Check for updated data on the actuator_controls topic
        if (ret > 0 && (fds[0].revents & POLLIN))
        {
            orb_copy(ORB_ID(actuator_controls), _actuator_controls_sub, &actuator_controls);

            // Update the duty cycle for the motor
            set_duty_cycle(actuator_controls.control[0]);

            // Publish the updated value of the actuator_outputs topic
            actuator_outputs_s actuator_outputs = {};
            actuator_outputs.output[0] = _duty_cycle;
            orb_publish(ORB_ID(actuator_outputs), _actuator_outputs_pub, &actuator_outputs);
        }
    }
}
