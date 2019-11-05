# PIL 图像操作 API   

## 1. 图片读取操作及其基本属性  

```py

```


## 2. 图像处理操作  

```py
from PIL import Image, ImageOps
```

1) padding 操作  

```py
padding = 2
img = ImageOps.expand(img, border=padding, fill=0)
```

2) resize 

```py
img = img.resize((tw, th), Image.BILINEAR) 
```

3) crop  

```py
x1 = random.randint(0, w - tw)
y1 = random.randint(0, h - th)
img = img.crop((x1, y1, x1 + tw, y1 + th)) 
```

4) flip  

```py
img = img.transpose(Image.FLIP_LEFT_RIGHT) 
```

5) PIL Image 转换成 array  

```py
img = np.asarray(image)
```

需要注意的是，如果出现read-only错误，并不是转换的错误，一般是你读取的图片的时候，默认选择的是"r","rb"模式有关。

修正的办法:　手动修改图片的读取状态

  img.flags.writeable = True  # 将数组改为读写模式
 

6) array 转换成 PIL.Image  

```py
Image.fromarray(np.uint8(img))
```

7) save()  

保存图片.  

```py
img.save("result.jpg", quality=95)
```

8) RGB 通道交换为 BGR   

```py
img_reverse = img[:, :, ::-1]
```