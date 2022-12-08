#pragma once

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/uORB.h>

#include <drivers/drv_pwm_output.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <cstdint>

class Cytron
{
public:
    /**
     * Constructor
     *
     * @param pwm_index: Index of the PWM output channel to use for the motor
     * @param pwm_frequency: PWM frequency to use for the motor
     */
    Cytron(int pwm_index, int pwm_frequency);

    /**
     * Initialize the driver
     *
     * @return 0 on success, -1 on error
     */
    int init();

    /**
     * Set the duty cycle for the motor
     *
     * @param duty_cycle: Duty cycle for the motor, from 0.0 to 1.0
     */
    void set_duty_cycle(float duty_cycle);

    /**
     * Start the motor control loop
     */
    void start();

private:
    // Index of the PWM output channel to use for the motor
    int _pwm_index;

    // PWM frequency to use for the motor
    int _pwm_frequency;

    // File descriptor for the PWM device
    int _pwm_fd;

    // Handle for the actuator_controls subscription
    int _actuator_controls_sub;

    // Handle for the actuator_outputs publication
    orb_advert_t _actuator_outputs_pub;

    // Current duty cycle for the motor
    float _duty_cycle;

    // Static function to run the motor control loop in a separate thread
    static void *motor_control_loop_trampoline(void *context);

    // Function to run the motor control loop
    void motor_control_loop();
};
