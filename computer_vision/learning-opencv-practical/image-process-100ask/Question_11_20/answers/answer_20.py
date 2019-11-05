import cv2
import numpy as np
import matplotlib.pyplot as plt

# Read image as gray
img = cv2.imread("imori_dark.jpg", 0) 
H, W = img.shape

bins = np.zeros(256, dtype=np.int)
    
for y in range(H):
    for x in range(W):
        bins[img[y, x]] += 1

# Display histogram with plt.hist()  
# plt.hist(img.ravel(), bins=255, rwidth=0.8, range=(0, 255))

plt.bar(range(256), bins)  
plt.savefig("out.png")
plt.show()