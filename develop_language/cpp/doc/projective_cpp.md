# 让自己的 cpp 工程更加"工程化"   

## 1. 使用日志系统    


## 2. 使用迭代器    


## 3. 使用 map 关联容器的排序操作    

[map 使用实例](./cpp_primer_5nd/map_instance.md)对 map 的使用做了很详细的记录.   

## 4. 使用类封装思想   



## 5. 使用模板   


## 6. 使用命名空间    


## 7. 多线程优化   
在多线程中调用 `cv::imshow()` 函数会导致如下错误:   
```
(tracks:4201): Gtk-WARNING **: gtk_disable_setlocale() must be called before gtk_init()
[xcb] Unknown request in queue while dequeuing
[xcb] Most likely this is a multi-threaded client and XInitThreads has not been called
[xcb] Aborting, sorry about that.
detection: ../../src/xcb_io.c:179: dequeue_pending_request: Assertion `!xcb_xlib_unknown_req_in_deq' failed.
Aborted (core dumped)
```

