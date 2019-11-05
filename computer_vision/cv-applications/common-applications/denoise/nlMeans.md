# PIMA-NL-Means

基于非本地图像过滤的M1项目  



projet M1 based on non local means filter.   

## Compilation

Le projet seras dévelloper en C++ avec l'utilisation de la librairie CImg (distribué sous la licence CeCILL) pour modifier et afficher les images.
http://cimg.eu/

linux 平台: 
```
mkdir build && cd build
cmake ..
make
```

额外的说明, 如果你使用的图片格式是 jpg, 那么需要在 Cimg.h 前面加上 `#define cimg_use_jpeg`, 并且在编译命令中加上 `-ljpeg`.   
同样, 如果你使用的图片格式是 png, 那么需要在 Cimg.h 前面加上 `#define cimg_use_png`, 并且在编译命令中加上 `-lpng`.  

```cpp
#define cimg_use_png 1
#define cimg_use_jpeg
#include "CImg.h"
...

```

然后重新编译.   

## 使用方法   

pour lancer le projet : 
```
./nlmeans -i image -s patchSize (-n noise si l'image n'est pas bruité && -h pour le sigma du bruit)
```
valeur -n : 0 -> gaussian, 1 -> uniforme, 2 -> Salt & Pepper, 3 -> Poisson, 4 -> Rician   

```
./morpho -a 0.4 -s 3 -n 1 -seuil 10 -h 40   
```
option : -a = apha, -n = noise, -sig = sigma noise, -i = picture, -h = filtering param, -s = patchSize, -seuil = threshold   

# Bibliographie

Antoni Buades, Bartomeu Coll, Jean-Michel Morel. A non-local algorithm for image denoising. Computer Vision and Pattern Recognition, 2005.
https://perso.telecom-paristech.fr/bloch/P6Image/Projets/NLMeans.pdf

Connelly Barnes, Eli Shechtaman, Adam Finkelstein, Dan B Goldman. PatchMatch: A Randomized Correspondence Algorithm for Structural Image Editing. ACM Transactions on Graphics (Proc. SIGGRAPH) 28(3), August 2009.
http://gfx.cs.princeton.edu/pubs/Barnes_2009_PAR/patchmatch.pdf

# Result  

---- NLMEANS ----  

Example :    

./nlmeans -i Picture/barbara.png -h 10 -n 0 -s 7   

![ScreenShot](/snapshots/Lena_h5_sig10_p7_with_CImg.png)  
