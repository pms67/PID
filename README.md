# PID
 PID controller implementation written in C.
 
 Note: If you want to use derivative-on-measurement, you will need to negate the 'D'-gain you would typically use in a 'standard derivative-on-error' PID controller - I did not mention that in the video. Since the 'error signal' effectively going into the differentiator does not depend on the setpoint: e[n] = 0 - measurement, and therefore (e[n] - e[n - 1]) = (0 - measurement) - (0 - prevMeasurement) = -Kd * (measurement - prevMeasurement).
So, for example, if you require a controller with a D-gain of 10, you would set pid->Kd = -10. This is a slight quirk of the derivative-on-measurement form - however, the code can of course be adapted to include the minus sign.
 
 Dynamic integrator clamping: https://e2e.ti.com/blogs_/b/industrial_strength/archive/2013/04/13/teaching-your-pi-controller-to-behave-part-vii
