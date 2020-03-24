# Realsense D435i examples  









```bash
./realsense-viewer/realsense-viewer



```

## 2. Firmware Update Tool (rs-fw-update)

> 固件下载地址: https://dev.intelrealsense.com/docs/firmware-update-tool  


rs-fw-update tool is a console application for updating depth camera firmware.

Prerequisites
In order to update a depth camera firmware, a signed image file is required.
The latest firmware for D400 cameras is available here.
The firmware is packed into zip file and contains a file with "bin" extension with the following naming convension: "Signed_ImageUVC<firmware_version>.bin"

Usage
After installing librealsense run rs-fw-update -l to launch the tool and print a list of connected devices.
An example for output for a D415 camera is:

connected devices:
1) Name: Intel RealSense D415, serial number: 725112060411, ASIC serial number: 012345678901, firmware version: 05.11.01.100, USB type: 3.2
Then we will provid the serial number to identify the device together with the path to firmware file that we want to update `rs-fw-update -s 725112060411 -f Signed_Image_UVC_5_11_6_250.bin`.
An example for the expected output is:

search for device with serial number: 725112060411

update to FW: Signed_Image_UVC_5_11_6_250.bin

updating device:
Name: Intel RealSense D415, serial number: 725112060411, ASIC serial number: 012345678901, firmware version: 05.11.01.100, USB type: 3.2

firmware update started

firmware update progress: 100[%]

firmware update done

device 725112060411 successfully updated to FW: 05.11.06.250
In case only one camera is connected you can simply run rs-fw-update -f Signed_Image_UVC_5_11_6_250.bin.

A camera/s might be in a recovery state, in such case listing the devices will output the following:

connected devices:
1) Name: Intel RealSense D4xx Recovery, serial number: unknown, ASIC serial number: 012345678901, firmware version: unknown, USB type: unknown
In such case we can use the recovery flag and run rs-fw-update -r -f Signed_Image_UVC_5_11_6_250.bin
An example for the expected output is:

update to FW: Signed_Image_UVC_5_11_6_250.bin

recovering device:
Name: Intel RealSense D4xx Recovery, serial number: unknown, ASIC serial number: 012345678901, firmware version: unknown, USB type: unknown

firmware update started

firmware update progress: 100[%]

firmware update done

recovery done
Command Line Parameters
Flag	Description
-s	The serial number of the device to be update, this is mandetory if more than one device is connected
-f	Path of the firmware image file
-r	Recover all connected devices which are in recovery mode
-l	List all available devices and exits
-v	Displays version information and exits
-h	Displays usage information and exits
| None| List supported streaming modes|




## Issues  

1. @matlabninja ,
There is a typo in the line

#define FPS 20 // Defines the rate of frames per second //

which is not a valid FPS for D435. Switch to 30 or 15, if you're using USB2, and retry.

To obtain the full range of supported fps modes and resolutions you can run rs-enumerate-devices  

