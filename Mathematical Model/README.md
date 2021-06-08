# Learjet
Provides an example system identification of a known model from the learjet paper.
It contains a script taken from Development and Validation of a Flight-Identified Full-Envelope Business Jet Simulation Model Using a Stitching Architecture book.
And an additional script demostrating the use of MATLAbs SYS ID tool box to develop a model of the lear jet system. Written by Ethan Lew.

# Matlab Script
Mat files were used to send signals to FlightAxis to use in System Identification
  * F16Constants is used to call the simulink models and provide them initial conditions
    * Testsimulation.slx is the full scale simulink model, it includes control limits, and the state derivative file
    * F16Simulation_SC.slx is the small scale simulink model, it includes control limits, and the small scale state derivative file
  * F16sixDegreeFreedom.m and SC.m is the large and small scale state derivatives vector, respectively

# Python Script
This was used at the start of the project and was a start at converting the fortran state derivatives model to python. This work was stopped early in the project, but has been archived in the repository

# Simulation Results
Stores the output of the cost function of the small and large scale files
Stores steady state and impulse responses of the large and small scale files

# System Identification
The System Identification folder is used to store the outputs of FlightAxis
  * F16_sysid is an intial attempt at sys ID
  * FlightLogAnalysis and ReadCSV have been replaced by Prep_fligt_data and should not be used to handle CSV and Mat files.
  * Prep_flight_data loads a CSV or Mat file from FlightAxis and allows the user to use the brushing tool to save the file as disered
  * Inputsdlg is a necessary file with the prep_flight_data used to load a file


# Variables
Stores the Variables and Coefficient Definitions of the Mathematical Model

