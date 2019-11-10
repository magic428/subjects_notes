# 文件和目录操作   

python 中对文件、文件夹操作时经常用到的 os 模块和 shutil 模块常用方法。字符串前面加r，表示的意思是禁止字符串转义.     

## 1. 目录操作方法大全

|功能|函数|
|---|---|
|os.mkdir("file")|创建目录|
|shutil.copyfile("oldfile","newfile")| 复制文件,oldfile和newfile都只能是文件|
|shutil.copy("oldfile","newfile")|复制文件,oldfile只能是文件?，newfile可以是文件或目录|
|shutil.copytree("olddir","newdir")|复制文件夹,olddir和newdir都只能是目录，且newdir必须不存在|
|os.rename("oldname","newname")|重命名文件(目录),文件或目录都是使用这条命令|
|shutil.move("oldpos","newpos")|移动文件(目录)|
|os.remove("file")|删除文件|
|os.rmdir("dir")|删除目录.但只能删除空目录|
|shutil.rmtree("dir")|删除目录,空目录、有内容的目录都可以删|
|os.chdir("path")|切换路径|

关于以上操作的详细介绍请看下文.    
## 2. 文件操作方法  

|功能|函数|
|---|---|
|os.getcwd()|获取当前目录，即当前 Python 脚本所在的目录|
|os.listdir()|返回指定目录下的所有文件名(包含目录名)|
|os.remove()|删除一个文件|
|os.removedirs(r“c:\python”)|删除多个目录|
|os.path.isfile()|检验给定的路径是否是一个文件|
|os.path.isdir()|检验给定的路径是否是一个目录|
|os.path.isabs()|判断是否是绝对路径|
|os.path.exists()|检验给定的路径是否存在|
|os.path.split()|将给定的文件路径切分为目录名和文件名|
|os.path.splitext()|分离扩展名|
|os.path.dirname()|获取路径名|
|os.path.basename()|获取文件名|
|os.rename(old， new)|重命名|
|os.mkdir(“test”)|创建单个目录|
|os.makedirs(r“c:\python\test”)|创建多级目录|
|os.stat(file)|获取文件属性|
|os.chmod(file)|修改文件权限与时间戳|
|os.exit()|终止当前进程|
|os.path.getsize(filename)|获取文件大小|
|os.system()|运行shell命令|
|os.getenv()|读取环境变量|
|os.putenv()|设置环境变量|
|os.linesep|给出当前平台使用的行终止符,Windows使用'\r\n'，Linux使用'\n'|
|os.name|指示你正在使用的平台,对于Windows，它是'nt'，而对于Linux/Unix用户，它是'posix'|

**注意**:   
字符串前面加 `r` ，表示的意思是禁止字符串转义.    

## 3. 文件操作方法大全   

|操作|描述|
|------|---|
|os.mknod("test.txt")   |创建空文件|
|fp = open("test.txt",w)|直接打开一个文件，如果文件不存在则创建文件|
|fp.read([size])        |size为读取的长度，以byte为单位|
|fp.readline([size])    |读一行，如果定义了size，有可能返回的只是一行的一部分|
|fp.readlines([size])   |把文件每一行作为一个list的一个成员，并返回这个list。size是表示读取内容的总长|
|fp.write(str)          |把str写到文件中，write()并不会在str后加上一个换行符|
|fp.writelines(seq)     |把seq的内容全部写到文件中(多行一次性写入)。这个函数也只是忠实地写入，不会在每行后面加上任何东西|
|fp.close()             |关闭文件。如果一个文件在关闭后还对其进行操作会产生ValueError|
|fp.flush()             |把缓冲区的内容写入硬盘|
|fp.fileno()            |返回一个长整型的”文件标签“|
|fp.isatty()            |文件是否是一个终端设备文件（unix系统中的）|
|fp.tell()              |返回文件操作标记的当前位置，以文件的开头为原点|
|fp.next()              |返回下一行，并将文件操作标记位移到下一行|
|fp.seek(offset[,whence])|将文件打操作标记移到offset的位置|
|fp.truncate([size])    |把文件裁成规定的大小，默认裁到文件上次操作的位置。如果size大于文件的大小，加上随机内容|

**Note**   
3.1 fp.readlines([size])   
把文件每一行作为一个list的一个成员，并返回这个list。其实它的内部是通过循环调用readline()来实现的。如果提供size参数，size是表示读取内容的总长，也就是说可能只读到文件的一部分。  
3.2 fp.next()函数   
把一个file用于for … in file这样的语句时，就是调用next()函数来实现遍历的。  
3.3 fp.seek(offset[,whence])   
将文件打操作标记移到offset的位置。这个offset一般是相对于文件的开头来计算的，一般为正数。但如果提供了whence参数就不一定了，whence可以为0表示从头开始计算，1表示以当前位置为原点计算。2表示以文件末尾为原点进行计算。需要注意，如果文件以a或a+的模式打开，每次进行写操作时，文件操作标记会自动返回到文件末尾。  

**open 模式**   

|mode|说明|   
|----|---|
|w|以写方式打开|
|a|以追加模式打开 (从 EOF 开始, 必要时创建新文件)|
|r+|以读写模式打开|
|w+|以读写模式打开 (参见 w )|
|a+|以读写模式打开 (参见 a )|
|rb|以二进制读模式打开|
|wb|以二进制写模式打开 (参见 w )|
|ab|以二进制追加模式打开 (参见 a )|
|rb+|以二进制读写模式打开 (参见 r+ )|
|wb+|以二进制读写模式打开 (参见 w+ )|
|ab+|以二进制读写模式打开 (参见 a+ )|

## 4. 目录操作方法详细介绍    
**`shutil.copyfileobj**(fsrc, fdst[, length])`**    
Copy the contents of the file-like object fsrc to the file-like object fdst. The integer length, if given, is the buffer size. In particular, a negative length value means to copy the data without looping over the source data in chunks; by default the data is read in chunks to avoid uncontrolled memory consumption. Note that if the current file position of the fsrc object is not 0, only the contents from the current file position to the end of the file will be copied.   
**`shutil.copyfile(src, dst)`**   
Copy the contents (no metadata) of the file named src to a file named dst. dst must be the complete target file name; look at shutil.copy() for a copy that accepts a target directory path. If src and dst are the same files, Error is raised. The destination location must be writable; otherwise, an IOError exception will be raised. If dst already exists, it will be replaced. Special files such as character or block devices and pipes cannot be copied with this function. src and dst are path names given as strings.   
**`shutil.copymode(src, dst)`**   
Copy the permission bits from src to dst. The file contents, owner, and group are unaffected. src and dst are path names given as strings.   
**`shutil.copystat(src, dst)`**   
Copy the permission bits, last access time, last modification time, and flags from src to dst. The file contents, owner, and group are unaffected. src and dst are path names given as strings.   
**`shutil.copy(src, dst)`**   
Copy the file src to the file or directory dst. If dst is a directory, a file with the same basename as src is created (or overwritten) in the directory specified. Permission bits are copied. src and dst are path names given as strings.    
**`shutil.copy2(src, dst)`**    
Similar to shutil.copy(), but metadata is copied as well – in fact, this is just shutil.copy() followed by copystat(). This is similar to the Unix command cp -p.   
**`shutil.ignore_patterns(*patterns)`**   
This factory function creates a function that can be used as a callable for copytree()’s ignore argument, ignoring files and directories that match one of the glob-style patterns provided.   
**`shutil.copytree(src, dst, symlinks=False, ignore=None)`**   
Recursively copy an entire directory tree rooted at src. The destination directory, named by dst, must not already exist; it will be created as well as missing parent directories. Permissions and times of directories are copied with copystat(), individual files are copied using shutil.copy2().   
If symlinks is true, symbolic links in the source tree are represented as symbolic links in the new tree, but the metadata of the original links is NOT copied; if false or omitted, the contents and metadata of the linked files are copied to the new tree.    
If ignore is given, it must be a callable that will receive as its arguments the directory being visited by copytree(), and a list of its contents, as returned by os.listdir(). Since copytree() is called recursively, the ignore callable will be called once for each directory that is copied. The callable must return a sequence of directory and file names relative to the current directory (i.e. a subset of the items in its second argument); these names will then be ignored in the copy process. ignore_patterns() can be used to create such a callable that ignores names based on glob-style patterns.   
If exception(s) occur, an Error is raised with a list of reasons.   
The source code for this should be considered an example rather than the ultimate tool.   
Changed in version 2.3: Error is raised if any exceptions occur during copying, rather than printing a message.   
Changed in version 2.5: Create intermediate directories needed to create dst, rather than raising an error. Copy permissions and times of directories using copystat().   
Changed in version 2.6: Added the ignore argument to be able to influence what is being copied.   
**`shutil.rmtree(path[, ignore_errors[, onerror]])`**  
Delete an entire directory tree; path must point to a directory (but not a symbolic link to a directory). If ignore_errors is true, errors resulting from failed removals will be ignored; if false or omitted, such errors are handled by calling a handler specified by onerror or, if that is omitted, they raise an exception.   
If onerror is provided, it must be a callable that accepts three parameters: function, path, and excinfo. The first parameter, function, is the function which raised the exception; it will be os.path.islink(), os.listdir(), os.remove() or os.rmdir(). The second parameter, path, will be the path name passed to function. The third parameter, excinfo, will be the exception information return by sys.exc_info(). Exceptions raised by onerror will not be caught.   
Changed in version 2.6: Explicitly check for path being a symbolic link and raise OSError in that case.   

**`shutil.move(src, dst)`**    

Recursively move a file or directory (src) to another location (dst).   
If the destination is an existing directory, then src is moved inside that directory. If the destination already exists but is not a directory, it may be overwritten depending on os.rename() semantics.  

If the destination is on the current filesystem, then os.rename() is used. Otherwise, src is copied (using shutil.copy2()) to dst and then removed.   

**`exception shutil.Error`**   
This exception collects exceptions that are raised during a multi-file operation. For copytree(), the exception argument is a list of 3-tuples (srcname, dstname, exception).     


## 实例

```py
import os

data_dir = "~/data"
files = os.listdir(data_dir)

for f in files:
    f_new = f[2:]
    print f_new
    os.rename(os.path.join(data_dir, f), os.path.join(data_dir, f_new))
```