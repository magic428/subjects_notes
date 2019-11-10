# `sublime_text3`编译`matlab`      
`Tools -> Build System -> New Build System`，输入如下代码（注意替换你的`MATLAB`安装路径），然后保存为`MATLAB.sublime-build`文件：      
```
"cmd": ["gnome-terminal -x bash -c \"matlab -nosplash -nodesktop -r ${file_base_name}; exec bash\""],
"selector":"source.m",
"shell":"true",
"working_dir": "$file_path"
```