# virtualenv 创建独立的 Python 环境   

## python 版本   

在开发 `Python` 应用程序的时候，系统安装的 `Python3` 只有一个版本：`3.4`。所有第三方的包都会被 `pip` 安装到系统版本 `Python3` 的 `site-packages` 目录下。  

如果我们要同时开发多个应用程序，那这些应用程序都会共用一个 `Python`， 即安装在系统的 `Python 3`。如果 `应用A` 需要 `jinja 2.7`， 而 `应用B` 需要 `jinja 2.6` 怎么办？  

这种情况下，每个应用可能需要各自拥有一套“独立”的 `Python 运行环境`。`virtualenv` 就是用来为一个应用创建一套 `“隔离”的Python` 运行环境。  

### 1. 使用 virtualenv 创建一个独立的 Python 环境    

首先，我们用 `pip` 安装 `virtualenv`： 

~~~
$ pip3 install virtualenv
~~~

然后，假定我们要开发一个新的项目，需要一套独立的 Python 运行环境，可以这么做：   

第一步，创建目录：   
~~~
$ mkdir myproject
$ cd myproject/
~~~

第二步，创建一个独立的 Python 运行环境，命名为 venv_test：     
~~~
$ python3 -m virtualenv --no-site-packages venv_test

Using base prefix '/usr/local/.../Python.framework/Versions/3.4'
New python executable in venv_test/bin/python3.4
Also creating executable in venv_test/bin/python
Installing setuptools, pip, wheel...done.
~~~

命令 `python3 -m virtualenv` 就可以创建一个独立的 Python 运行环境，我们还加上了参数 `--no-site-packages`，这样，已经安装到系统 Python 环境中的所有第三方包都不会复制过来，这样，我们就得到了一个不带任何第三方包的“干净” 的 Python 运行环境。  

--distribute 选项使 virtualenv 使用新的基于发行版的包管理系统而不是 setuptools 获得的包。 --distribute 选项会自动在新的虚拟环境中安装 pip 。  

### 2. 使用这个新建的 Python 环境  

新建的 Python 环境被放到当前目录下的 venv_test 目录。 有了 venv_test 这个 Python 环境，可以用 `source` 进入该环境：    

~~~
$ source venv_test/bin/activate source venv_test/bin/activate
(venv_test)$
~~~

注意到命令提示符变了，有个 (venv_test) 前缀， 表示当前环境是一个名为 venv_test 的 Python 环境。  

下面正常安装各种第三方包，并运行 python 命令：   

~~~bash
(venv_test)$ pip install jinja2
...
Successfully installed jinja2-2.7.3 markupsafe-0.23
~~~

在 venv_test 环境下，用 `pip` 安装的包都被安装到 venv_test 这个环境下，系统 Python 环境不受任何影响。也就是说， venv_test 环境是专门针对 myproject 这个应用创建的。   

**Note:** 如果使用的是 `sudo pip`, 那么安装的包还是在系统目录下.   

### 3. 退出这个新建的 Python 环境  

退出当前的 venv_test 环境，使用 `deactivate` 命令：  

~~~bash
(venv_test)$ deactivate   
~~~

此时就回到了正常的环境，现在 `pip 或 python` 均是在系统 Python 环境下执行。  

完全可以针对每个应用创建独立的 Python 运行环境， 这样就可以对每个应用的 Python 环境进行隔离。   

**小结**

`virtualenv` 是如何创建“独立”的 Python 运行环境的呢？    

原理很简单，就是把系统 Python 复制一份到 `virtualenv` 的环境，用命令 `source venv_test/bin/activate` 进入一个 `virtualenv` 环境时，`virtualenv` 会修改相关环境变量，让命令 `python 和 pip` 均指向当前的 `virtualenv` 环境。    

`virtualenv` 为应用提供了隔离的 Python 运行环境，解决了不同应用间多版本的冲突问题。   

## conda - Anacodna 版本

conda 创建虚拟环境   

### 1. 查看包   

~~~bash
# 查看安装了哪些包
conda list 
# 查看有哪些虚拟环境
conda env list 
# 查看 conda 的版本
conda -V 
~~~

### 2.创建虚拟环境  

命名为 myflaskapp，n就是指name；并安装flask包。    
Note that the conda create command requires that you give it the name of a package to install in the new environment.   
conda命令创建虚拟环境时，必须指定一个或者几个你需要安装的package。    

~~~bash
conda create -n py2 python=2* anaconda
~~~

这样就会安装anaconda2版本。   

栗子1：   
这条命令安装了一个名为 myflaskapp 虚拟环境，安装 flask 包。   

~~~bash
conda create -n myflaskapp flask
~~~

栗子2：   
这个是克隆创建了一个和原系统一样的 python 环境，命名为 nb 。   

~~~bash
conda create -n nb --clone root
~~~ 

栗子3：     
这就不需指定具体包了  

~~~bash
conda create --name $ENVIRONMENT_NAME python
~~~

其他：   

~~~bash
$ conda create -n py3 python=3*
$ conda create -n py2 python=2*
~~~

This will create two environments, one with Python3 and the other with Python2. I typically set one of these as my default by adding source activate py3 to my terminal startup. Typically I only use these "named python" environments to run a Python REPL or do general Python tasks. I'll create another conda environment named specifically for each real project I work on.   

### 3. 切换环境   

Linux: `source activate myflaskapp`   
Windows: `activate myflaskapp`  

### 4. 退出环境    
    
Linux:  `deactivate`    
Windows: `deactivate`    

### 5. 给指定虚拟环境安装包   
    
conda install -n yourenvname [package]    

### 6. 移除虚拟环境    
    
移除某个环境中的包    
conda remove --name $ENVIRONMENT_NAME $PACKAGE_NAME    
移除某个虚拟环境    
conda remove -n yourenvname --all    
这些所有的虚拟环境，都在C:\Anaconda3\envs文件夹下。    
    
