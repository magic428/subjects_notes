# cmake 中的 list 操作  

和 if 类似, list 中操作变量时不应该使用变量引用符号 "${}".  

## Append 尾部插入操作  

```cmake
set (LIBS "")

list(APPEND LIBS)
```

## REMOVE_ITEM 移除 list 中的某个条目  


