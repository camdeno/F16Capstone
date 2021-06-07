# F-16 RC Physical Model
This folder contains all the files about RC Physical Model. It contains the instructions for using Pixhawk4 and sensors, the measurement data of the model, and the introduction of some assembly.

To check all the hardwares used in the physical model, plase view [F-16 Model Hardwares List](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/F-16%20Model%20Hardwares%20List%20.xlsx)

To view the assembly of all components in the model, please view [Instrumentation.pdf](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/Instrumentation.pdf)


## Pixhawk4
This folder include the Pixhawk4 instruction files. All these files downloaded from Pixhawk4 offical webside. To get more information, please go to [Pixhawk4 offical webside](https://docs.px4.io/v1.9.0/en/)

* [Pixhawk4 Quick Start Guide](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/Pixhawk%204/Pixhawk4-quickstartguide.pdf)
* [Pixhawk4 Datasheet](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/Pixhawk%204/Pixhawk4-DataSheet.pdf)
* [Pixhawk4 Pinouts](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/Pixhawk%204/Pixhawk4-Pinouts.pdf)
* [Pixhawk4 GPS Quick Start Guide](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/Pixhawk%204/Pixhawk4-GPS-Quick-Start-Guide.pdf)

## AoA Sensor
AoA (Angle of Attack) sensor is used to detect the direction of airflow in flight. In this F-16 physical model, the AoA sensor is composed of an PWM signal input angle sensor equipped with a vane. There are two guides in the AoA sensor folder explain how to use the AoA sensor.

These are the fileds include in the AoA sensor folder:

* [MA3_datasheet.pdf](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/AOA%20Sensor/MA3_datasheet.pdf): Datasheet for angle sensor dowmloaded from MA3 angle sensor [offical product website](https://www.usdigital.com/products/encoders/absolute/shaft/MA3).
* [Angle Sensor Connection Guide.pdf](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/AOA%20Sensor/Angle%20Sensor%20Connection%20Guide.pdf): Introduced the wire connection and test method of angle sensor.
* [AOA Log Data Recording Guide.pdf](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/AOA%20Sensor/AOA%20Log%20Data%20Recording%20Guide.pdf): Introduce how to record the testing data from AoA sensor.
* 3D Model
  * [MA3-X-125-B.f3z](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/AOA%20Sensor/3D%20Model/MA3-X-125-B.f3z): The 3D model of angle sensor dowmloaded from MA3 angle sensor [offical product website](https://www.usdigital.com/products/encoders/absolute/shaft/MA3).
  * [Vane v3.f3d](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/AOA%20Sensor/3D%20Model/Vane%20v3.f3d): The 3D modle of vane made by Fusion 360.
  * [Vane v3.stl](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/AOA%20Sensor/3D%20Model/Vane%20v3.stl): The 3D modle of vane used for 3D print.
  * [MA3 with Vane.png](https://github.com/camdeno/F16Capstone/blob/main/RC%20Physical%20Model/AOA%20Sensor/3D%20Model/MA3%20with%20Vane.png)：The 3D diagram of AoA composed of angle sensor and vane
