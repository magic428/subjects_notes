# shell中特殊变量的涵义   
## $0 $1 $# $$ $! $? 
1. `$0`   
执行脚本的名字   
2. `$*` 和 `$@`    
将所有参数返回.   
3. `$#`:   
参数的个数   
4. `$_`:   
代表上一个命令的最后一个参数   
5. `$$`:   
代表所在命令的PID   
6. `$!`:   
代表最后执行的后台命令的PID   
7. `$?`:   
代表上一个命令执行是否成功的标志，如果执行成功则 `$?` 为 0，否则不为 `0`.  

## 脚本测试
```bash
# test1.sh
#!/bin/sh
echo "number:$#"
echo "name:$0"
echo "first:$1"
echo "second:$2"
echo "argure1:$@"
ret="$@"
ret1=$@
echo "$ret"
echo "$ret1"
echo "argure2:$*"
echo "arg:$_"
echo "result:$?"
echo "PID:$$"
echo "pid:$!"
```
更改文件的执行权限后运行脚本文件.   
```bash
$ chmod +x test1.sh 
$ ./test1.sh a b c
```
输出如下:   
```
number:3
name:./test1.sh
first:a
second:b
argure1:a b c
a b c
a b c
argure2:a b c
arg:argure2:a b c
result:0
PID:14973
pid:
```
[wang@localhost 桌面]$ ./test1.sh "a" "b" "c"
除了PID不一样，其他结果是相同的。

经测试对以下两个变量仍有疑问：   
`$*`: 以一对双引号给出参数列表   
`$@`: 将各个参数分别加双引号返回   

**Note**：
`$0`在 `shell` 和 `awk` 中的涵义不同。    
`shell`: 命令行的第一个单词  
`awk`:当前记录     