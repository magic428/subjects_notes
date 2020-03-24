# 折半插入排序  

```java
待排序数据：2，1，6，7，4
private void binaryInsertSort(int arr[])
{
    int low = 0;
    int high = 0;
    int m = 0;// 中间位置

    for(int i = 1; i < arr.length; i++){
        low = 0;
        high = i-1;

        while(low <= high){

            m = (low+high)/2;

            if(arr[m] > arr[i])
                high = m - 1;
            else
                low = m + 1;
        }

        //统一移动元素，将待排序元素插入到指定位置
        temp = arr[i];
        for(int j=i; j > high+1; j--){
            arr[j] = arr[j-1];
        }

        arr[high+1] = temp;

    }

}
```