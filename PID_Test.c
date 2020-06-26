#include <stdio.h>
#include <stdlib.h>

#include "PID.h"

/* Controller parameters */
#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.25f

#define PID_TAU 0.02f

#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 4.0f

/* Simulated dynamical system (first order) */
float TestSystem_Update(float inp);

int main()
{
    /* Initialise PID controller */
    PIDController pid = { PID_KP, PID_KI, PID_KD,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
			  PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S };

    PIDController_Init(&pid);

    /* Simulate response using test system */
    float setpoint = 1.0f;

    printf("Time (s)\tSystem Output\tControllerOutput\r\n");
    for (float t = 0.0f; t <= SIMULATION_TIME_MAX; t += SAMPLE_TIME_S) {

        /* Get measurement from system */
        float measurement = TestSystem_Update(pid.out);

        /* Compute new control signal */
        PIDController_Update(&pid, setpoint, measurement);

        printf("%f\t%f\t%f\r\n", t, measurement, pid.out);

    }

    return 0;
}

float TestSystem_Update(float inp) {

    static float output = 0.0f;
    static const float alpha = 0.02f;

    output = (SAMPLE_TIME_S * inp + output) / (1.0f + alpha * SAMPLE_TIME_S);

    return output;
}
