# F16Capstone
<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/camdeno/F16Capstone/blob/main/Resources/Images/F16%20In%20the%20Field.jpg">
    <img src="https://github.com/camdeno/F16Capstone/blob/main/Resources/Images/F16%20In%20the%20Field.jpg" alt="Logo" width="180" height="160">
  </a>

  <h3 align="center">F-16 Capstone: Modeling and Simulation</h3>

  <p align="center">
   The objective of this project is to develop a dynamic model of a scaled-down Radio Controlled F-16 Falcon jet and perform flight simulation as well as a real flight.
    <br />
    <a href="https://github.com/camdeno/F16Capstone"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/camdeno/F16Capstone/tree/main/Mathematical%20Model/Matlab%20Script">Nonlinear Model Script</a>
    <a href="https://github.com/camdeno/F16Capstone/tree/main/Testing%20Documentation">Testing Documentation</a>
    <a href="https://github.com/camdeno/F16Capstone/issues">Report Bug</a>
  </p>
</p>


<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#flight-axis">Flight Axis</a>
    </li>
    <li><a href="#flight-data">Flight Data</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

![F16](./Resources/f16.jpeg)

The objective of this project is to develop a dynamic model of a scaled-down Radio Controlled F-16 Falcon jet and perform flight simulation as well as a real flight. Galois, the industry sponsor, plans to use models to circumvent the inconvenience of flight testing real aircraft. This project is open-source, so the Cyber-Physical System Research Community and some hobbyists can also use the data and information our team collects for flight simulation. In this project, our team needs to use the supplied Pixhawk software stack to determine the flight dynamics of a small-scale model RC F-16 aircraft and develop trusted flight control software.

<!-- ABOUT THE PROJECT -->
### Built With

Matlab was used for primary system identification and the nonlinear model. Flightaxis is used to extract data from the Realflight 9.5 simulator. 
* [Matlab]( https://www.mathworks.com/products/matlab.html)
* [Flightaxis]( https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_FlightAxis.cpp)

<!-- Flight Axis -->
## Flight Axis

Flight Axis provides a link to RealFlight 9.5, the RC Plane simulator, to output a csv file of the states and controls of the RC plane. Flight Axis is provided by Ardupilot free software for the development of autonomous autopilot systems. 

<!-- Flight Data -->
## Flight Data

Flight Data provides information on the physical flight testing done by the capstone team. This should contain videos of flights, ulog files from qgroundcontrol and the pixhawk, and links to the flight log analysis from [Pixhawk](https://logs.px4.io/)
