# CMake 控制指令 - IF/WHILE/FOREACH   

## (1) IF 指令

其基本语法为:  

```cmake
IF(expression_r)
  COMMAND1(ARGS ...)
ELSE()
  COMMAND2(ARGS ...)
ENDIF()
```

把握一个原则: 凡是出现 IF 的地方一定要有对应的 ENDIF; 出现 ELSEIF 的地方, ENDIF 是可选的。  

IF (表达式) 的使用方法如下:  

```
IF(var), 如果变量不是: "空, 0, N, NO, OFF, FALSE, NOTFOUND 或 <var>_NOTFOUND" 时, 表达式为真。  
IF(NOT var ), 与上述条件相反。  
IF(var1 AND var2), 当两个变量都为真是为真。  
IF(var1 OR var2), 当两个变量其中一个为真时为真。  
IF(COMMAND cmd), 当给定的 cmd 确实是命令属性并可以调用时为真。  
IF(EXISTS dir) 或者 IF(EXISTS file), 当目录名或者文件名存在时为真。  
IF(file1 IS_NEWER_THAN file2), 当 file1 比 file2 新, 或者 file1/file2 其中有一个不存在时为真, 文件名请使用完整路径。  
IF(IS_DIRECTORY dirname), 当 dirname 是目录时, 为真。  
IF(variable | string MATCHES regex) 当给定的变量或者字符串能够匹配正则表达式 regex 时为真。  
IF(DEFINED variable), 如果变量被定义为真。   
```

比如:  

```cmake
IF("hello" MATCHES "ell")
MESSAGE("true")
ENDIF()
```

变量和数字/字符串比较表达式:  

```
IF(variable LESS number)
IF(string LESS number)
IF(variable GREATER number)
IF(string GREATER number)
IF(variable EQUAL number)
IF(string EQUAL number)
```

按照字母序的排列进行比较: 

```
IF(variable STRLESS string)
IF(string STRLESS string)
IF(variable STRGREATER string)
IF(string STRGREATER string)
IF(variable STREQUAL string)
IF(string STREQUAL string)
```

## (2) WHILE 指令  

WHILE 指令的语法是:

```cmake
WHILE(condition)
  COMMAND1(ARGS ...)
ENDWHILE()
```

其真假判断条件可以参考 IF 指令。

## (3) FOREACH 指令  

FOREACH 指令的使用方法有三种形式:   

3.1) 列表

```cmake
FOREACH(loop_var arg1 arg2 ...),  loop_var 每次获取一个列表 arg1 arg2 ... 中的元素;  
FOREACH(loop_var RANGE total), loop_var 从 0 到 total 以 1 为步进;  
FOREACH(loop_var RANGE start stop [step]), loop_var 从 start 开始到 stop 结束, 以 step 为步进;   
```

这个指令需要注意的是, 直到遇到 ENDFOREACH 指令,整个语句块才会得到真正的执行。

特别需要注意的是, 在 IF 控制语句"条件"中使用变量, 不能用 ${} 引用, 而是直接应用变量名。  
 
掌握了以上的各种控制指令, 你应该完全可以通过 cmake 管理复杂的程序了。

