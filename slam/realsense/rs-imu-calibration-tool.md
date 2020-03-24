# rs-imu-calibration Tool 

这种标定方法只适用于商用化的简单标定, 在 SLAM 应用中无法满足精度要求.  

https://github.com/IntelRealSense/librealsense/tree/development/tools/rs-imu-calibration#rs-imu-calibration-tool

## Goal
The tool is intended to calibrate the IMU built in D435i cameras

## Description
D435i cameras arrive from the factory without IMU calibration. Hence the values may be slightly off.
In order to improve accuracy, a calibration procedure should be done.
The rs-imu-calibration tool walks you through the calibration steps and saves the calibration coefficients to the EEPROM, to be applied automatically by the driver.

## Command Line Parameters

|Flag   |Description   |Default|
|-----|---|---|
|`-h `|Show help. ||
|`-i <accel_filename> [gyro_filename]`| Load previously saved results to EEPROM| |
|`-s serial_no`| calibrate device with given serial_no| calibrate first found device|
|`-g `|show graph of data before and after calibration| ||

# Calibration Procedure:
Running:

`python rs-imu-calibration.py`

The script runs you through the 6 main orientations of the camera.
For each direction there are the following steps:
*	**Rotation:**<br>
  *	The script prints the following line, describing how to orient the camera:<br>
`Align to direction:  [ 0. -1.  0.]   Upright facing out`<br>
  *	Then it prints the status (rotate) and the difference from the desired orientation:<br>
  `Status.rotate:           [ 1.0157 -0.1037  0.9945]:                 [False False False]`<br>
  *	You have to bring the numbers to [0,0,0] and then you are in the right direction and the script moves on to the next status.<br><br>

*	**Wait to Stablize:**<br>
  *	The script waits for you to be stable for 3 seconds. Meanwhile there is a countdown message:<br>
  `Status.wait_to_stable: 2.8 (secs)`<br>
  *	When waited for 3 seconds, the script begin to <b>collect data:<br><br>

*	**Collecting data:**<br>
  *	Status line is a line of dots. When reaching 20 dots, enough data is collected and script carries on to next orientation.<br>
  *	If camera is no longer in the precise orientation, data is not collected.
  *	If camera is moved too much, going back to <b>Rotation status.

When done all 6 orientations, the following message appears, suggesting you to save the raw data gathered:<br>
`Would you like to save the raw data? Enter footer for saving files (accel_<footer>.txt and gyro_<footer>.txt)`<br>
`Enter nothing to not save raw data to disk. >`<br>

Either press “Enter” not to save or an ending (like 2 or <serial_no>_1) to save the accel.txt, gyro.txt.<br>

The files can be loaded using the “-i” option.

Then the script will ask to save the data to the Device.<br>
Choose Y (or any other combination with Y or y in it) to save.

That’s it. At the end a confirmation message appears:<br>
`SUCCESS: saved calibration to camera.`

**NOTICE:**<br>
CTRL-C isn’t working so press CTRL-Z and then `kill -9 %1` if you want to terminate in the middle…

**Appendix A:**<br>
I found it a little challenging to hold the camera in the precise 90 degrees angle needed.
Therefor I used my original camera's box as a camera holder.
I cut a little piece so the cord may come out, and that's it:<br>
***open box image:***<br>
![open box image](images/IMG_4918.jpg)<br>

***camera in box:***<br>
![camera in box](images/IMG_4919.JPG)<br>

***face down position:***<br>
![face down position](images/IMG_4920.JPG)<br>

***cord up position:***<br>
![cord up position](images/IMG_4921.JPG)<br>

***upright facing out (or upside down facing out, depends on which side the cord is..):***<br>
![upright facing out](images/IMG_4922.JPG)<br>

***cord down position:***<br>
![cord down position](images/IMG_4923.JPG)<br>

***upside down (or upright) facing out:***<br>
![upside down facing out](images/IMG_4924.JPG)



## Issues   

**1\) 一直打印: Status.collect_dataWARNING: MOVING**  

When I run the program python3 rs-imu-calibration.py in the first position according to the instructions, the program keeps repeating WARNING: MOVING and Status.wait_to_stable, but cannot perform calibration.as follow,

```bash
$ python3 rs-imu-calibration.py
Start interactive mode:
FOUND GYRO with fps=200
FOUND ACCEL with fps=63
*** Press ESC to Quit ***
Align to direction: [ 0. -1. 0.] Upright facing out
Status.rotate: [-0.0031 -0.0023 -0.0681]: [ True T Status.collect_dataWARNING: MOVING
Status.rotate: [-0.0052 -0.0021 -0.064 ]: [ True T Status.collect_dataWARNING: MOVING
Status.rotate: [-0.0072 -0.0021 -0.064 ]: [ True T Status.collect_dataWARNING: MOVING
Status.rotate: [-0.0052 -0.0021 -0.0641]: [ True T Status.collect_dataWARNING: MOVING
Status.rotate: [-0.0052 -0.0019 -0.0619]: [ True T
```

When printing the gyroscope values in the program, I found the gyroscope The value of the meter has always been kept large, and the gyroscope data cannot be collected, which makes it impossible to calibrate. How to solve this problem?

collect gyroscope values code as follow:
if pr.stream_type() == rs.stream.gyro:
self.collected_data_gyro.append(np.append(frame.get_timestamp(), data_np))
is_moving = any(abs(data_np) > self.rotating_threshold)

gyroscope values is about [ 6.98131695e-03 5.67720680e+01 -2.15338726e+01]
Please try to uninstall the *pip package-pyrealsense2* and try to install with librealsense source code. Do not use pip install pyrealsense2 to install python programs.

