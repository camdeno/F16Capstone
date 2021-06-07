# Software-in-the-loop (SITL)

The SITL simulation allow capstone team to run flight simulation without any hardware. Connect the simulator to QGroundControl (QGC), use QGC to develop a simulated flight plan, view simulated flight, and record simulated flight data. When running in SITL the sensor data comes from a flight dynamics model in a flight simulator. For more information, Please check the [simulation section](https://docs.px4.io/master/en/simulation/) of PX4 User Guid.

The Capstone team use [Flightgear](https://docs.px4.io/master/en/simulation/flightgear.html) and [Gezabo](https://docs.px4.io/master/en/simulation/gazebo.html) as a simulator for stand plane SITL simulation. 

The software in the loop file contains information relating to the capstone team's exploration of SITL. The team attempted to implement a F16 model in SITL, but production of that model was stopped short in March. 

The folder contains:
* [Flight Path File](https://github.com/camdeno/F16Capstone/tree/main/SITL/Flight%20Path%20File)
  * A folder for flight paths when the model was upfitted to fly autonomously - this currently holds one racetrack pattern
* [Practice SITL Log Files](https://github.com/camdeno/F16Capstone/tree/main/SITL/Practice%20SITL%20Log%20Files)
  * A collection of log files from the early explorations of SITL- these were all explored in ubuntu with flight gear and qgroundcontrol 
* [RC F16 FDM](https://github.com/camdeno/F16Capstone/tree/main/SITL/RC%20F-16%20FDM)
  * Attempt to implement F16 Model in SITL
