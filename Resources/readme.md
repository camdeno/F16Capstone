Created a folder to store resources provided by Galois and ECE 412 professors. 

# NOTES

## RealFligth & SITL

Good news is that RealFlight contains a Flight Dynamics Model of the RC F16, and has so called *Flight Axis* interface for data input/output.
And Ardupilot implements/supports this interface, so it is (at least theoretically) possible to use RealFlight+Ardupilot for SITL.

The bad news is that FlightAxis uses obscure SOAP XML, which is pretty horrendous. Perhaps a thin Python module on top of aero bench would be a better solution, should the RealFligth simulator show promising results and a realistic feel.

It is yet to be seen whether their model matches the real airplane.

* [RealFlight 9.5](https://www.realflight.com/index.php) contains the scaled down F16 model (but needs Interlink compatible RC controller)
* [RealFlight PDF manual](https://www.horizonhobby.com/on/demandware.static/Sites-horizon-us-Site/Sites-horizon-master/default/Manuals/RFL1100_RFL1101-Manual-EN.pdf)
* [Ardupilot FlightAxis implementation](https://github.com/ArduPilot/ardupilot/blob/03bb3237eff79eea3429f6fc1d559a5130e87a49/libraries/SITL/SIM_FlightAxis.h)
* [Ardupilot using RealFlight for SITL](https://ardupilot.org/dev/docs/sitl-with-realflight.html)
* Overview of [FlightAxis](https://www.knifeedge.com/forums/index.php?threads/flightaxis-link-q-a.32854/)
* What is [SOAP XML](https://www.w3.org/TR/2000/NOTE-SOAP-20000508/#_Toc478383490)
