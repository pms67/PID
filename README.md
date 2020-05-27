# PID
 PID controller implementation written in C.
 
 Note on 'derivative-on-measurement': Since the 'error signal' effectively going into the differentiator does not depend on the setpoint: e[n] = 0 - measurement,
 and therefore (e[n] - e[n - 1]) = (0 - measurement) - (0 - prevMeasurement) = -Kd * (measurement - prevMeasurement). (Note the minus sign compared to derivative-on-error!)
 
 Dynamic integrator clamping: https://e2e.ti.com/blogs_/b/industrial_strength/archive/2013/04/13/teaching-your-pi-controller-to-behave-part-vii
