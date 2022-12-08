#include "cytron.h"

#include <cstdlib>
#include <lib/parameters/param.h>

// Set the default values for the driver parameters
static constexpr int PWM_INDEX = 0;
static constexpr int PWM_FREQUENCY = 50;

int cytron_main(int argc, char *argv[])
{
    // Load the driver parameters from the parameter system
    int pwm_index = PWM_INDEX;
    param_get(param_find("PWM_INDEX"), &pwm_index);
    int pwm_frequency = PWM_FREQUENCY;
    param_get(param_find("PWM_FREQUENCY"), &pwm_frequency);

    // Create an instance of the Cytron driver
    Cytron cytron(pwm_index, pwm_frequency);

    // Initialize the driver
    if (cytron.init() < 0)
    {
        return EXIT_FAILURE;
    }

    // Start the motor control loop
    cytron.start();

    // Run the driver until it is asked to exit
    while (!px4_task_should_exit())
    {
        px4_usleep(10000);
    }

    return EXIT_SUCCESS;
}
