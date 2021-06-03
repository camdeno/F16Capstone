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
      <details open="open">
      <summary>Project Documents Overview</summary>
      <ol>
          <li>
           <a href="#flight-axis">Flight Axis</a>
         </li>
          <li><a href="#flight-data">Flight Data</a></li>
          <li><a href="#literature">Literature</a></li>
          <li><a href="#mathematical-model">Mathematical Model</a></li>
          <li><a href="#literature">literature</a></li>
          <li><a href="#meeting-agenda">Meeting Agenda</a></li>
          <li><a href="#project-requirements">Project Requirements</a></li>
          <li><a href="#project-schedule">Project Schedule</a></li>
          <li><a href="#rc-physical-model">RC Physical Model</a></li>
          <li><a href="#resources">Resources</a></li>
          <li><a href="#sitl">SITL</a></li>
          <li><a href="#testing-documentation">Testing Documentation</a></li>
      </ol>  
    </details>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#license">License</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

![F16](./Resources/f16.jpeg)

Galois, the industry sponsor, plans to use models to circumvent the inconvenience of flight testing real aircraft. This project is open-source, so the Cyber-Physical System Research Community and some hobbyists can also use the data and information our team collects for flight simulation. In this project, our team needs to use the supplied Pixhawk software stack to determine the flight dynamics of a small-scale model RC F-16 aircraft and develop trusted flight control software.

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

<!-- Literature-->
## Literature

The Literature folder provides a large array of literature used in this capstone project. Each piece of literature is accompanied by a summary of the document. Some documents are also accompanied by the importance to our project, denoted by reccomended or interesting read. The main reference papers we used are:

* Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems: Dynamics, Controls Design, and Autonomous Systems
  * This text was used to develop the Mathematical Model and for general research purposes   
* System Identification for Small, Low-Cost, Fixed-Wing Unmanned Aircraft
  * This text was used to develop flight tests, moment of inertia tests, as well as general research purposes
* Development and Validation of a Flight-Identified Full Envelope Business Jet Simulation Model Using a Stitching Architecture
  * This text was used to develop flight tests, understand system identification, and provide a linear model for system identification 


<!-- Mathematical Model -->
## Mathematical Model

The Mathematical Model folder holds the primary work on the github. This folder contains the following sub folders and current states: 

* Learjet 
  * Provides an example system identification of a known model from the learjet paper   
* Matlab Script
  * Mat files were used to send signals to FlightAxis to use in System Identification
  * F16Constants is used to call the simulink models and provide them initial conditions
    * Testsimulation.slx is the full scale simulink model, it includes control limits, and the state derivative file
    * F16Simulation_SC.slx is the small scale simulink model, it includes control limes, and the small scale state derivative file
  * F16sixDegreeFreedom.m and SC.m is the large and small scale state derivatives vector, respectively
* Python Script
  * This was used at the start of the project and was a start at converting the fortran state derivatives model to python. This work was stopped early in the project, but has been archived in the repository   
* Simulation Results
  * Stores the output of the cost function of the small and large scale files
  * Stores steady state and impulse responses of the large and small scale files
* System Identification
  * Outputs of FlightAxis are stored as MAT and CSV files 
  * F16_sysid is an initial attempt at system Identification
  * FlightLogAnalysis and ReadCSV have been replaced by prep_flight_data and should not be used to handle CSV and Matfiles
  * prep_flight_data loads a MAT or a CSV file from FlightAxis and allows the user to use the brushing tool to save the file as a specified name
  * inputsdlg is a necessary file with the prep_flight_data to load a file
* Variables 
  * Stores the Variables and Coefficient Definitions of the Mathematical Model   

<!-- Meeting Agenda -->
## Meeting Agenda

Holds the meeting agenda and tasks from each week of the Capstone, Jan 2021 - June 2021. These agendas were provided by the Capstone Team, Advisor, and Sponsor. 

<!-- Project Requirements -->
## Project Requirements

Holds the Project Requirements in the form of the Project Proposal, this is what the team set out in January 2021 to accomplish. 

<!-- Project Schedule -->
## Project Schedule 

Contains a MS Project file of the teams initial project schedule. The PDF version can be found in the Project Requirements folder. 

<!-- Rc Physical Model -->
## RC Physical Model

The RC Physical Model Folder is primarily where information on sensors and hardwares are stored. It contains the following folders and documents:
* AOA Sensor
  *  3D Model of the Angle of Attack Wind Vane that was attempted be mounted on the plane
  *  Code to connect the Angle of Attack to the PX4
  *  Relevant information to the Angle of Attack Sensor
* Sensor Noise Analysis Report
* Physical measurements recorded of the Small Scale F16 
* Relating Documents to the PX4 Mini

<!-- Resources -->
## Resources

The Resources folder provides pivotal information for the students and Galois.
* ECE 412 Documents contains references for the students from the ECE 412 Professors, Andrew Greenberg and Mark Faust
* Galois Documents provides the initial Capstone Proposal from Galois to the students and an inital SITL/HITL information
* Images contains all the images taken of the physical model during the duration of the capstone project
* Student Documents contains notes and references used by the Capstone Team 


<!-- SITL-->
## SITL

The software in the loop file contains information relating to the capstone team's exploration of SITL. The team attempted to implement a F16 model in SITL, but production of that model was stopped short in March. 

The folder contains:
* Flight Path File
  * A folder for flight paths when the model was upfitted to fly autonomously - this currently holds one racetrack pattern
* Practice SITL Log Files
  * A collection of log files from the early explorations of SITL- these were all explored in ubuntu with flight gear and qgroundcontrol  
* RC F16 FDM
  * Attempt to implement F16 Model in SITL

<!-- Testing Documentation-->
## Testing Documentation



<!-- ROADMAP -->
## Roadmap

See the [open issues](https://github.com/camdeno/F16Capstone/issues) for a list of proposed features (and known issues).





<!-- License -->
## License 
Distributed under the MIT License. See `LICENSE` for more information.

<!-- -->
[Readme Template](https://github.com/othneildrew/Best-README-Template)






