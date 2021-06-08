# Learjet
Provides an example system identification of a known model from the learjet paper.
It contains a script taken from Development and Validation of a Flight-Identified Full-Envelope Business Jet Simulation Model Using a Stitching Architecture book.
And an additional script demostrating the use of MATLAbs SYS ID tool box to develop a model of the lear jet system. Written by Ethan Lew.

# Matlab Script
Mat files were used to send signals to FlightAxis to use in System Identification
Contains Trimmer function script
  * F16Constants is used to call the simulink models and provide them initial conditions
    * Testsimulation.slx is the full scale simulink model, it includes control limits, and the state derivative file
    * F16Simulation_SC.slx is the small scale simulink model, it includes control limits, and the small scale state derivative file
  * F16sixDegreeFreedom.m and SC.m is the large and small scale state derivatives vector, respectively

# Python Script
This was used at the start of the project and was a start at converting the fortran state derivatives model to python. This work was stopped early in the project, but has been archived in the repository

# Simulation Results
Large Scale
  * Contains simulation results from the full scale f-16 RC model

Small Scale
  * Contains simulation results from the small scale f-16 RC model
 
Comparison for Simulink and Realflight
  * Report adressing the differences between the Simulink model and the behaviors of the Realflight 9.5 sim

Trimmer Function Outputs
  * Is an excel file that contains the outputs from the trimmer function used to dial in 

# System Identification
The System Identification folder is used to store the outputs of FlightAxis
  * F16_sysid is an intial attempt at sys ID
  * FlightLogAnalysis and ReadCSV have been replaced by Prep_fligt_data and should not be used to handle CSV and Mat files.
  * Prep_flight_data loads a CSV or Mat file from FlightAxis and allows the user to use the brushing tool to save the file as disered
  * Inputsdlg is a necessary file with the prep_flight_data used to load a file


# Variables
Aerodynamic Coefficients 
  * is used to store the updated Coefficients developed in the Model from testing and data gathered

Variable Definitions 
  * contains the history of equations and reviesions for the variables defined in the mathmatical model.

