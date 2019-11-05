#!/usr/bin/env python 
#coding=utf-8

import cv2
import os

data = '/home/klm/work/gitwork/work_note/dev_tools/opencv/calibrate/chessboards/'

#output_dir = 'chessboards/'
#cap_video(video, output_dir)

files = os.listdir(data)
i = 0
for f in files:
	i += 1
	os.rename(data+f, data+"chessboard{:>02d}.jpg".format(i))