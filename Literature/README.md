# Literature review

A collection of various papers, and other relevant documents.

## Research papers



## Parameter Estimation

### [Aerodynamic modeling of the Skywalker X8 Fixed-Wing Unmanned Aerial Vehicle](https://ieeexplore.ieee.org/abstract/document/8453370)
*[Interesting read]* Show use of a real wind tunnel to measure aerodynamic coefficients. Not applicable to our case.

### [Combining model-free and model-based Angle of Attack estimation for small fixed-wing UAVs using a standard sensor suite](https://ieeexplore.ieee.org/abstract/document/7502583)
Uses Extended Kalman Filter (EKF) and no prior knowledge of the UAV parameters. Claims estimating aerodynamic coefficients, Angle of Attack, and even wind. Not impressed by the results, mostly because they were only in simulation (so you had a reference to compare the estimates to) and the aerodynamic coefficient estimates didn't seem to converge very well.

### [Parameterized, Multi-fidelity Aircraft Geometry and Analysis for MDAO Studies using CAPS](https://acdl.mit.edu/ESP/Publications/AIAApaper2019-2230.pdf)
Uses AVL to calculate various parameters of a real F16. The results match wind tunnel measurements only for a small flight envelope (small alpha), which is not particularly convincing.

### [Estimation of Aircraft Aerodynamic Derivatives Using Extended Kalman Filter](https://www.scielo.br/scielo.php?script=sci_arttext&pid=S0100-73862000000200001#apx01)
Old paper, shows that it is possible to use EKF to estimate aerodynamic coefficients. More literature review is need to see if it was succesfully used for UAVs and if the results are accurate enough.

### [Calculation and Identification of the AerodynamicParameters for Small-Scaled Fixed-Wing UAVs](https://www.researchgate.net/publication/322512468_Calculation_and_Identification_of_the_Aerodynamic_Parameters_for_Small-Scaled_Fixed-Wing_UAVs)
This paper seems to capture what we are looking for (calculate aerodynamic parameters), but the accuracy of the results is not particularly convincing. A more careful review is needed.

### [Aerodynamic Characterization of an Off-the-Shelf Aircraft via Flight Test and Numerical Simulation](https://www.laas.fr/projects/skyscanner/sites/www.laas.fr.projects.skyscanner/files/u45/Aerodynamic%20Characterization%20of%20an%20Off-the-Shelf%20Aircraft%20via%20Flight%20Test%20and%20Numerical%20Simulation.pdf)
*[Interesting read]* Determining aerodynamic parameters both numerically and experimentally. Significant differences, leading to some good questions about the methodology and accuracy.

## System Identification

### [System Identification for Small, Low-Cost, Fixed-Wing Unmanned Aircraft](https://arc.aiaa.org/doi/abs/10.2514/1.C032065?journalCode=ja)
**[Recommended read]** Nicely described system identificaton for a UAV. More control-theoretical, but the results are encouraging.

### [Development of a Full Flight Envelope F-16 VISTA Simulation Model from Closed-loop Flight Data](https://www.researchgate.net/profile/Marit-Knapp/publication/322309590_Development_of_a_Full_Envelope_Flight_Identified_F-16_Simulation_Model/links/5c7877ada6fdcc4715a3ebd5/Development-of-a-Full-Envelope-Flight-Identified-F-16-Simulation-Model.pdf)
*[Interesting read]* System ID of a real F16 using stiching and other advanced methods. Very good results.

### [Development and Validation of a Flight-Identified Full-Envelope Business Jet Simulation Model Using a Stitching Architecture](https://arc.aiaa.org/doi/abs/10.2514/6.2017-1550)
*[Interesting read]* Describes using the stitching method on a Learjet-25 airplane. Again, very promising results.

### [Nonlinear System Identification: A User-Oriented Road](https://arxiv.org/pdf/1902.00683.pdf)
Overview of nonlinear system ID methods. An extensive paper (120 pages!).

## Tools

### [XFoil](https://web.mit.edu/drela/Public/web/xfoil/)
For airfoil analysis. Very old (last update 2013).


### [AVL](https://web.mit.edu/drela/Public/web/avl/)
For aerodynamic analysis of an aircraft. Looks promising, but it is quite old (last update 2017).

## Various / parking lot
### [Design of A High-Performance Tailless MAV Through Planform Optimization](https://arc.aiaa.org/doi/abs/10.2514/6.2015-2879)
Designing a UAV platform at ENAC.

### [Performance Testing of RNR’s SBXC Using a Piccolo Autopilot](http://www.xcsoaring.com/techPicts/Edwards%20performance%20test.pdf)
Designing and testing a glider.