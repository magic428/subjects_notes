# short 类型在vs中打印是占4字节

`unsigned short`和`short`的区别
```
	printf("head=%x, size=%d\n", head, sizeof(head));
	if (head  == 0x4d42)
		type = FileType_BMP;
	else if (head  == 0xd8ff)
		type = FileType_JPG;
```

使用`short`的话，对于高位为`1`的整数，可能会出现位数错误情形。特别是用于`==`比较的情况下。比如`d8ff`会被当做`ffffd8ff`，而`424d`由于高位并不是`1`,所以不会出现这种情况。

另外可以在打印时，在`int的格式化字符`前加`h`：
`short`对应的`%hd`，`unsigned short`对应`%hu（十进制） %ho（八进制） %hx（十六进制）`