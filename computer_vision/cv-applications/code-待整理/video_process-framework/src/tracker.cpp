/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 10 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <cmath>

#include "featuretracker.h"

int main()
{
    // Create video procesor instance
    VideoProcessor processor;

    // Create feature tracker instance
    FeatureTracker tracker;

    // Open video file
    processor.setInput("../bike.avi");

    // set frame processor
    processor.setFrameProcessor(&tracker);

    // Declare a window to display the video
    processor.displayOutput("Tracked Features");

    // Play the video at the original frame rate
    double fps = processor.getFrameRate(); 
    fps = isnan(fps) ? 30 : fps;

    processor.setDelay(1000./fps);

    // Start the process
    processor.run();

    cv::waitKey();
    
}