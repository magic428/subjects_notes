# aeroblk_HL20 模块

**术语**
Incidence：入射角   
Forces and Moments: 力矩   
Sideslip：横移   
discontinued: 停产    

## Introduction
This section introduces a NASA HL-20 lifting body airframe model that uses blocks from the Aerospace Blockset™ software to simulate the airframe of a NASA HL-20 lifting body, in conjunction with other Simulink® blocks.

The model simulates the NASA HL-20 lifting body airframe approach and landing flight phases using an automatic-landing controller.

For more information on this model, see NASA HL-20 Lifting Body Airframe.

## What This Example Illustrates
The NASA HL-20 lifting body airframe example illustrates the following features of the blockset:
Representing bodies and their degrees of freedom with the Equations of Motion library blocks
Using the Aerospace Blockset blocks with other Simulink blocks
Feeding Simulink signals to and from Aerospace Blockset blocks with Actuator and Sensor blocks
Encapsulating groups of blocks into subsystems
Visualizing an aircraft with Simulink 3D Animation™ and Gauges Blockset™ library blocks.
Open the Model
To open the NASA HL-20 airframe example, type the example name, aeroblk_HL20, at the MATLAB® command line. The model opens.


## 1. 打开`aeroblk_HL20`模型   
type the example name, aeroblk_HL20, at the MATLAB® command line.    
```matlab
>> aeroblk_HL20
```

## Key Subsystems
The model implements the airframe using the following subsystems:     
使用以下的子系统来实现本模型：    
The `6DOF (Euler Angles)` subsystem implements the `6DOF (Euler Angles)` block along with other Simulink blocks.    
The `Environment Models` subsystem implements the `WGS84 Gravity Model` and `COESA Atmosphere Model` blocks. It also contains a `Wind Models` subsystem that implements a number of wind blocks.    
The `Alpha`, `Beta`, `Mach` subsystem implements the `Incidence`, `Sideslip` & `Airspeed`, `Mach Number`, and `Dynamic Pressure` blocks. These blocks calculate aerodynamic coefficient values and lookup functionality.    
The `Forces and Moments` subsystem implements the Aerodynamic Forces and Moments block. This subsystem calculates body forces and body moments.    
The `Aerodynamic Coefficients` subsystem implements several subsystems to calculate six aerodynamic coefficients.    
## NASA HL-20 Example
Running an example lets you observe the model simulation in real time. After you run the example, you can examine the resulting data in plots, graphs, and other visualization tools. To run this model, follow these steps:    
 If it is not already open, open the aeroblk_HL20 example.     
 From the Simulation menu, select Start. On Microsoft® Windows® systems, you can also click the Start button in the model window toolbar. The simulation proceeds until the aircraft lands.     

## Modify the Model
You can adjust the airframe model settings and examine the effects on simulation performance. Here is one modification that you can try. It changes the camera point of view for the landing animation.   
### Change the Animation Point of View    
By default, the airframe animation viewpoint is Rear position, which means the view tracks with the airframe flight path from the rear. You can change the animation point of view by selecting another viewpoint from the Simulink 3D Animation viewer:    
(1) Open the aeroblk_HL20 model, and click the Simulink 3D Animation viewer.   
(2) From the list of existing viewpoints, change the viewpoint to Fixed Position.   


## 遇到的问题：    
1. Failed to load library 'gaugeslibv2'.     
这是因为，As of Release 2016a, Gauges Blockset is discontinued and no longer available for purchase. You can continue to use existing product licenses, or you can use alternative products.    
In place of Gauges Blockset, consider using the graphical controls and displays included in the Dashboard block library in Simulink. This library contains controls such as knobs and slider switches, and it has lamps and semicircular gauges. You can add these blocks to your model to run interactive desktop simulations.    
