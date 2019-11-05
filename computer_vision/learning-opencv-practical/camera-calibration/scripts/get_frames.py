#!/usr/bin/env python 
#coding=utf-8

import cv2
import os


def cap_video(video, output_dir):
	video_2 = os.path.join(video, "board_2.avi")  
	cap = cv2.VideoCapture(video_2)
	if not os.path.exists(output_dir):
		os.makedirs(output_dir)
	else:
		for f in os.listdir(output_dir):
			os.remove(os.path.join(output_dir, f))

	i = 0
	while cap.isOpened():
		ret, frame = cap.read()
		if i % 10 == 0 and ret:
			cv2.imwrite(output_dir+"chessboard0"+ str(i) + '.jpg', frame, (cv2.IMWRITE_JPEG_QUALITY, 100))
			cv2.imshow("demo", frame)
			cv2.waitKey(30)
		i += 1
	cap.release() # 释放摄像头

def cap_test_video(video, output_dir):

	video_test = os.path.join(video, "2.avi")
	cap = cv2.VideoCapture(video_test)
	if not os.path.exists(output_dir):
		os.makedirs(output_dir)

	i = 0
	while cap.isOpened():
		ret, frame = cap.read()
		if i % 10 == 0 and ret:
			cv2.imwrite(output_dir+"test"+ str(i) + '.jpg', frame, (cv2.IMWRITE_JPEG_QUALITY, 100))
			cv2.imshow("demo", frame)
			cv2.waitKey(30)
		i += 1
	cap.release() # 释放摄像头


if __name__ == '__main__':
	video = '/home/klm/work/videos/chessboard'

	output_dir = 'chessboards/'
	cap_video(video, output_dir)
	
	#test_output_dir = 'test/'
	#cap_test_video(video, test_output_dir)

	cv2.destroyAllWindows()# 删除建立的全部窗口