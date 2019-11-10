
shell中单中括号与双中括号的区别
转载 2014年05月30日 20:39:34 标签：linux shell /shell command 2375
转载于http://ending123.blog.51cto.com/2376140/673419
1.单括号TEST命令要对变量进行单词分离，当变量值包含空白符时，要用引号将变量括起来；而双括号的TEST命令不会对变量进行单词分离。
以下情况分为变量为单个单词，含有空格的词组。
[root@www yansan]# echo $name       
hello
[root@www yansan]# [ $name = "hello" ]
[root@www yansan]# echo $?
0
[root@www yansan]# [[ $name = "hello" ]]
[root@www yansan]# echo $?             
0
[root@www yansan]#
得出结论，因为变量只有一个单词嘛，单双括号的单词分离都没有影响到这个变量了。
[root@www yansan]# echo $name1       
hello world
[root@www yansan]# [ $name1 = "hello world" ]
-bash: [: too many arguments
[root@www yansan]# echo $?
2
[root@www yansan]# [ "$name1" = "hello world" ]
[root@www yansan]# echo $?
0
[root@www yansan]# [[ $name1 = "hello world" ]]
[root@www yansan]# echo $?
0
[root@www yansan]#
这次变量是一个词组了，带空格那种了。当我们要测试时，结果发现BASH回应参数太多了，这是怎么回事呢？原来是单引号TEST命令对变量进行单词分离了，结果也就变成了
[ hello world = "hello world" ]，多了那个字符串hello，成了字符串world和字符串hello world之间的比较了。因此如果在单括号的TEST命令中的变量含有空格，但它还得和字符串比较，那就必须给变量加个双引号了，这时它就不会发生错误了。而在下面的双括号的TEST命令中，即使变量含有空格又何妨，因为它不能对变量单词进行分离嘛。
 
2．单方括号的TEST命令，通常用内置的TEST命令来测试表达式的值，TEST命令也被链接到方括号上。这样，既可以使用单独的TEST命令，也可以通过把表达式用单方括号括起来，来测试表达式的值。
[root@www yansan]# ll abc
-rw-r--r-- 1 root root 0 Sep 24 08:59 abc
[root@www yansan]# test -r abc ; echo $?
0
[root@www yansan]# [ -r abc ] ; echo $?             
0
[root@www yansan]#
 
3.单括号的TEST命令不对SHELL元字符进行扩展的，而双括号TEST命令则会对SHELL元字符进行扩展的了。
[root@www yansan]# name=tom
[root@www yansan]# [ $name = t?? ]
[root@www yansan]# echo $?
1
[root@www yansan]# [[ $name = t?? ]]
[root@www yansan]# echo $?          
0
[root@www yansan]#
 
4.在双括号的TEST命令当中，如果一个字符串（不管含不含有空格）仅仅是在表达式中作为一个普通字符串，而不是一个模式的一部分，则它也必须用引号括起来。
我的理解是如果一个字符串值（右边那个的了）不加双引号，那这个字符串就是模式来的，如果它里面有含有SHELL元字符，BASH会对它进行扩展。如果字符串加了双引号，那它就是一个很普通的字符串的，即便字符串里面里面含有特殊字符，也就是当普通内容来处理。
 
[root@www yansan]# echo $name
tom
[root@www yansan]# echo $name1
tomm
 [root@www yansan]# [[ $name = tom ]]      
[root@www yansan]# echo $?
0
[root@www yansan]# [[ $name = tom? ]]
[root@www yansan]# echo $?          
1
[root@www yansan]# [[ $name1 = tom ]]
[root@www yansan]# echo $?          
1
[root@www yansan]# [[ $name1 = tom? ]]
[root@www yansan]# echo $?           
0
[root@www yansan]#
[root@www yansan]# [[ $name = "tom" ]]
[root@www yansan]# echo $?           
0
 [root@www yansan]# [[ $name = "tom?" ]]
[root@www yansan]# echo $?            
1
 [root@www yansan]# [[ $name1 = "tom" ]]
[root@www yansan]# echo $?             
1
[root@www yansan]# [[ $name1 = "tom?" ]]
[root@www yansan]# echo $?             
1
 
5 [ express1 –a express2 ] 这是放在单括号的TEST命令中的，因为单的不支持元字符扩展，因此就只能叫做表达式了，它们可以组合构成逻辑测试的，不过与或非使用( -a –o !)的形式。
[root@www yansan]# ll mm
-r--r--r-- 2 root root 644 Sep 24 08:48 mm
[root@www yansan]# [ -r mm -a -x mm ]
[root@www yansan]# echo $?
1
[root@www yansan]#
 
[[pattern1 –a pattern1]] 这是放在双括号TEST命令中的，因为单条式子可以支持元字符嘛，所以可以叫做模式表达式了，也就可以用于复合逻辑了，不过与非或使用（&& || !）的形式了。
[root@www yansan]# echo $name
tom
[root@www yansan]# [[ $name = [tT]om && $name = t?? ]]
[root@www yansan]# echo $?
0
[root@www yansan]#


## 2 . bash下的单方括号和双方括号
发表于2017/7/18 15:54:23  291人阅读
分类： Linux学习

bash下的单方括号和双方括号概念以及用法：

一、bash[ ] 单括号 
[ ]两个符号左右都要有空格分隔 
内部操作与操作变量之间要有空格： [ val−gt10]字符串比较，><要进行转义，>\<[]中字符串或者{}变量尽量使用”“双引号扩住，避免值未定义引用而出错。 
[ ]中可以使用 -a -o进行逻辑运算 
[ ]是bash的内置命令 
下面列举几个单方括号的使用例子：

这里写图片描述 
这里写图片描述

成功条件满足返回0，失败条件不满足返回1； 
这里写图片描述 
[]单括号可以用来做字符串测试，例如判断两个字符串是否相等；一个字符串是否为空，字符串不等，字符串的大小（注意要加转义字符\） 
str1\str2 str1>str2

这里写图片描述

二、bash [[ ]] 双方括号 
特点： 
1、 [[ “ch”=”h” ]]内部操作符与操作变量之间要有空格 
2、字符串比较中，可以直接使用> < 无需转义 
3、 [[ ]]内部可以使用逻辑与 && 和逻辑或 ||

[ ]和[[ ]]都可以和!配合使用 
优先级： ！ > && > || 
逻辑运算符 < 关系运算符 
关系运算符： < > \< > == != -eq -ne -gt -ge -lt -le 
三、两者比较： 
相对而言[[ ]]比[ ] 更好一些： 
[[是bash程序的关键字，并不是一个命令，[[ ]]比[ ]更加通用，在[[ ]]之间所有的字符都不会发生文件名扩展或者单词分割，但是会发生参数扩展和命令替换。 
支持字符串的模式匹配，使用=~操作时甚至支持shell的正则表达式。字符串比较时可以把右边的作为一个模式，而不仅仅是一个字符串。 
使用[[…]]条件判断结构，可以防止脚本中的许多逻辑错误。比如&&、||、<和>操作符可以正常存在于[[]]条件判断语句中，如果在单括号[]结构中会出现错误。 
bash中把双括号中的表达式看做一个孤独的元素，并返回一个退出码状态。