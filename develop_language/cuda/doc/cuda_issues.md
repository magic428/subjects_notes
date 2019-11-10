# Cuda Unknown Error(ErrNo: 30) on cudaMalloc()  

nvidia_uvm kernel module is required for CUDA to work.  

使用下面的代码查看 nvidia_uvm 模块是否已经加载.  

```
$ lsmod |grep nv

nvidia              10744914  65 
nvram                  14362  1 thinkpad_acpi
drm                   310919  6 i915,drm_kms_helper,nvidia
```

```
$ sudo apt-get install nvidia-modprobe nvidia-384-uvm
$ sudo modprobe nvidia_uvm
```

If you don't want to reboot after installing nvidia-modprobe, you can try to run your program as root (e.g. sudo ./a.out) — module should be loaded during run as root.  


