# cmake FIND_xxx 指令详解 

1. find_path¶  

A short-hand signature is:

find_path (<VAR> name1 [path1 path2 ...])
The general signature is:

find_path (
          <VAR>
          name | NAMES name1 [name2 ...]
          [HINTS path1 [path2 ... ENV var]]
          [PATHS path1 [path2 ... ENV var]]
          [PATH_SUFFIXES suffix1 [suffix2 ...]]
          [DOC "cache documentation string"]
          [NO_DEFAULT_PATH]
          [NO_PACKAGE_ROOT_PATH]
          [NO_CMAKE_PATH]
          [NO_CMAKE_ENVIRONMENT_PATH]
          [NO_SYSTEM_ENVIRONMENT_PATH]
          [NO_CMAKE_SYSTEM_PATH]
          [CMAKE_FIND_ROOT_PATH_BOTH |
           ONLY_CMAKE_FIND_ROOT_PATH |
           NO_CMAKE_FIND_ROOT_PATH]
         )

This command is used to find a directory containing the named file.

A cache entry named by <VAR> is created to store the result of this command. 

If the file in a directory is found the result is stored in the variable and the search will not be repeated unless the variable is cleared. If nothing is found, the result will be <VAR>-NOTFOUND, and the search will be attempted again the next time find_path is invoked with the same variable.

用来查找包含文件名为 <name1> 的目录.   

如果查找成功, 则将查找到的目录保存在变量 <VAR> 中; 否则, <VAR> 被置为 NOTFOUND.   

