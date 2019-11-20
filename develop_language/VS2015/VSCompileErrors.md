# VS2015 编译错误集锦  

## 1) unresolved external symbol main referenced in function  

这个错误是由于 .exe 文件中缺少 main() 函数, 检查 main() 函数是否拼写正确.  

```bash
MSVCRT.lib(exe_main.obj) : error LNK2019: unresolved external symbol main referenced in function "int __cdecl __scrt_common_main_seh(void)" (?__scrt_common_main_seh@@YAHXZ)
xxx.exe : fatal error LNK1120: 1 unresolved externals
```

