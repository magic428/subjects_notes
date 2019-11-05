# OpenCV 使用用户自定义 kernel 实现卷积操作   

```cpp
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

Mat get_emboss_kernel()
{
    Mat kernel = (Mat_<float>(3, 3));
    kernel.ptr<float>(0)[0] = -2;
    kernel.ptr<float>(0)[1] = -1;
    kernel.ptr<float>(0)[2] = 0;

    kernel.ptr<float>(1)[0] = -1;
    kernel.ptr<float>(1)[1] = 1;
    kernel.ptr<float>(1)[2] = 1;

    kernel.ptr<float>(2)[0] = 0;
    kernel.ptr<float>(2)[1] = 1;
    kernel.ptr<float>(2)[2] = 2;

    return kernel;
}

int main()
{
    cv::Mat srcImage = imread("yjl.JPG");
    if (srcImage.empty())
    {
        return -1;
    }

    Mat kernel = get_emboss_kernel();
    cv::Mat emboss;
    // 初始化滤波器参数
    Point anchor = Point(-1, -1);
    double delta = 0;
    int ddepth = -1;
    //将核设置好之后，使用函数 filter2D 就可以生成滤波器：
    filter2D(srcImage, emboss, ddepth, kernel, anchor, delta, BORDER_DEFAULT);
}
```


