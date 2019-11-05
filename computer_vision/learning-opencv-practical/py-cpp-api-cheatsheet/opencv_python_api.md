# OpenCV 的 Python API  

```python
cv2.imread(filename[, flags]) → retval

```





## 保存图片 - imwrite()   

```python
cv2.imwrite(filename, img[, params]) → retval 

# eg. 
cv2.imwrite("img.jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
```

params – 指定图片的保存格式, 支持的格式有:  

(1) JPEG, CV_IMWRITE_JPEG_QUALITY 从 0 到 100, 默认值是 95.   
(2) PNG, CV_IMWRITE_PNG_COMPRESSION 表示压缩程度, 从 0 到 9. 默认值是 3.  

## RGB 通道交换为 BGR 

```py
img_reverse = img[..., [2, 1, 0]]

# 或  

img = np.transpose(img[0], (1,2,0))
```

## 图像深拷贝   

```python
import copy

convas = copy.deepcopy(frame)
```

## Video 相关  

### 1. Video 读取

```python
cv2.VideoCapture(filename)    
VideoCapture.isOpened()   
VideoCapture.get()   
ret, frame = VideoCapture.read()
VideoCapture.release()
```


### 2. Video 保存

```python
cv2.VideoWriter(filename, fourcc, fps, (width, height))
cv2.VideoWriter_fourcc('M', 'P', '4', '2')   
VideoWriter.write(frame)
VideoWriter.release()
```

get 函数中常用的参数:   

* CAP_PROP_FRAME_WIDTH;    
* CAP_PROP_FRAME_HEIGHT;    
* CAP_PROP_FRAME_COUNT;  // 获取视频总的帧数, 可用于遍历视频流  
* CAP_PROP_FPS;    
* CAP_PROP_FOURCC;    

所有支持的参数.   

```
CV_CAP_PROP_POS_MSEC Current position of the video file in milliseconds or video capture timestamp.
CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
CV_CAP_PROP_POS_AVI_RATIO Relative position of the video file: 0 - start of the film, 1 - end of the film.
CV_CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
CV_CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
CV_CAP_PROP_FPS Frame rate.
CV_CAP_PROP_FOURCC 4-character code of codec.
CV_CAP_PROP_FRAME_COUNT Number of frames in the video file.
CV_CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
CV_CAP_PROP_MODE Backend-specific value indicating the current capture mode.
CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
CV_CAP_PROP_HUE Hue of the image (only for cameras).
CV_CAP_PROP_GAIN Gain of the image (only for cameras).
CV_CAP_PROP_EXPOSURE Exposure (only for cameras).
CV_CAP_PROP_CONVERT_RGB Boolean flags indicating whether images should be converted to RGB.
CV_CAP_PROP_WHITE_BALANCE_U The U value of the whitebalance setting (note: only supported by DC1394 v 2.x backend currently)
CV_CAP_PROP_WHITE_BALANCE_V The V value of the whitebalance setting (note: only supported by DC1394 v 2.x backend currently)
CV_CAP_PROP_RECTIFICATION Rectification flag for stereo cameras (note: only supported by DC1394 v 2.x backend currently)
CV_CAP_PROP_ISO_SPEED The ISO speed of the camera (note: only supported by DC1394 v 2.x backend currently)
CV_CAP_PROP_BUFFERSIZE Amount of frames stored in internal buffer memory (note: only supported by DC1394 v 2.x backend currently)
```

## 图像中显示文字   

cv2.putText(img, text, orgin, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]]) -> img

eg:  

cv2.putText(frame, capture, (10, 10), cv2.FONT_HERSHEY_COMPLEX, 0.3, (0, 255, 0))


## 监听用户按键动作   

cv2.waitKey(1)   
key == ord('c')
cv2.destroyAllWindows()   


