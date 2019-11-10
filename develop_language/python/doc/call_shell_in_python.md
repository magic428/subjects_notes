# Python 中如何调用 Linux 命令

## 一、使用 os 模块

```python
import os   
os.system('ls')
a = os.system('ls')   ## 得到的是执行的命令的返回值，并不是执行结果
print(a)   #执行ls命令的返回值，成功，为0
b = os.popen('ls').readlines()    # 将得到的结果直接赋值给 b 列表
print(b)   # ls 执行结果输出给 b 
```

os 模块中常见方法(Linux命令)

os.remove()：删除文件

os.rename()：重命名文件

os.walk()：生成目录树下的所有文件名

os.chdir()：改变目录

os.mkdir/makedirs：创建目录/多层目录

os.rmdir/removedirs:删除目录/多层目录

os.listdir()：列出指定目录的文件

os.getcwd()：取得当前工作目录

os.chmod()：改变目录权限

os.path.basename()：去掉目录路径，返回文件名

os.path.dirname()：去掉文件名，返回目录路径

os.path.join()：将分离的各部分组合成一个路径名

os.path.getsize()：返回文件大小

os.path.exists()：是否存在

os.path.isabs()：是否为绝对路径

os.path.isdir()：是否为目录

os.path.isfile()：是否为文件

 

## 二、使用 commands 模块

```python
import commands  
c = commands.getoutput('ls')  # ls命令执行结果，字符串形式赋值给c变量
print(c)
d = c.split('\n')    # 对变量c，指定分隔符\n分隔，列表形式赋值给d
print(d)
```
输出结果为:   

    'anaconda-ks.cfg\nepel-release-7-5.noarch.rpm\nipython-4.1.2\nipython-4.1.2.tar.gz\npip-8.1.2\npip-8.1.2.tar.gz#md5=87083c0b9867963b29f7aba3613e8f4a.gz'

    ['anaconda-ks.cfg',
    'epel-release-7-5.noarch.rpm',
    'ipython-4.1.2',
    'ipython-4.1.2.tar.gz',
    'pip-8.1.2',
    'pip-8.1.2.tar.gz#md5=87083c0b9867963b29f7aba3613e8f4a.gz']
 

## 三、read、readline、readlines 区别

1、read() 全部取出，放到字符串里  

2、readline() 方法会将内存空间里的内容一次性只读一行，放到一个字符串里  

3、readlines() 方法会将内存空间里的内容一次性全部取出来，放到一个列表里  

 
## 四、Python 脚本示例

编写一个 python 脚本实现输出 linux 里的所有的用户的信息，格式如下：  

```bash
username is root     uid is 0
username is xiaojin     uid is 200
username is bin     uid is 10
```

方式一：  

```python
#!/usr/bin/python3
import commands
user_str = commands.getoutput('cat /etc/passwd')
user_list = user_str.splitlines() # 列表形式分隔文件内容(默认按行分隔)

for i in user_list:
    u_info = i.split(':')
    print ("username is {}, uid is {}\n".format(u_info[0], u_info[2]))
```

方式二：  

```python
#!/usr/bin/python3
import os
userlines=os.popen("cat /etc/passwd").readlines()
for i in userlines:
    u_info=i.split(":")
    print ("username is {}, uid is {}\n".format(u_info[0], u_info[2]))
```

2、数值脚本  

1.提醒用户输入内容范围是 0-100  
2.判断用户输入的内容，如果不是数字给予提醒  
3.输出用户输入的内容  

```python
#!/usr/bin/python3
u_grade = input("please input your grade:")

if u_grade.isdigit(): # isdigit() 判断字符串内容是否是数字
    if 0 <= int(u_grade) <= 100:
        print "your grade is",u_grade
    else:
        print "please input the range of number 0-100"
    else:
        print "It's not a valid number,try again"
```
