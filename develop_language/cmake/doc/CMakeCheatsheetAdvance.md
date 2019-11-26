# CMake 高级语法  

> 待整理.....  

## get_filename_component

Get a specific component of a full filename.

```cpp
get_filename_component(<var> <FileName> <mode> [CACHE])

Sets <var> to a component of <FileName>, where <mode> is one of:

DIRECTORY = Directory without file name
NAME      = File name without directory
EXT       = File name longest extension (.b.c from d/a.b.c)
NAME_WE   = File name without directory or longest extension
LAST_EXT  = File name last extension (.c from d/a.b.c)
NAME_WLE  = File name without directory or last extension
PATH      = Legacy alias for DIRECTORY (use for CMake <= 2.8.11)
Paths are returned with forward slashes and have no trailing slashes. If the optional CACHE argument is specified, the result variable is added to the cache.
```

```cpp
get_filename_component(<var> <FileName> <mode> [BASE_DIR <dir>] [CACHE])
```

Sets <var> to the absolute path of <FileName>, where <mode> is one of:

ABSOLUTE  = Full path to file
REALPATH  = Full path to existing file with symlinks resolved
If the provided <FileName> is a relative path, it is evaluated relative to the given base directory <dir>. If no base directory is provided, the default base directory will be CMAKE_CURRENT_SOURCE_DIR.  

## file 

1) Path Conversion

```cpp
file(RELATIVE_PATH <variable> <directory> <file>)
Compute the relative path from a <directory> to a <file> and store it in the <variable>.

file(TO_CMAKE_PATH "<path>" <variable>)
file(TO_NATIVE_PATH "<path>" <variable>)

The TO_CMAKE_PATH mode converts a native <path> into a cmake-style path with forward-slashes (/). The input can be a single path or a system search path like $ENV{PATH}. A search path will be converted to a cmake-style list separated by ; characters.

The TO_NATIVE_PATH mode converts a cmake-style <path> into a native path with platform-specific slashes (\ on Windows and / elsewhere).

Always use double quotes around the <path> to be sure it is treated as a single argument to this command.
```

## set 

```cpp
```

## marco()  

Start recording a macro for later invocation as a command::  

```cpp
 macro(<name> [arg1 [arg2 [arg3 ...]]])
   COMMAND1(ARGS ...)
   COMMAND2(ARGS ...)
   ...
 endmacro(<name>)
```

Define a macro named ``<name>`` that takes arguments named ``arg1``,
``arg2``, ``arg3``, (...).
Commands listed after macro, but before the matching
``endmacro()``, are not invoked until the macro is invoked.
When it is invoked, the commands recorded in the macro are first
modified by replacing formal parameters (``${arg1}``) with the arguments
passed, and then invoked as normal commands.
In addition to referencing the formal parameters you can reference the
values ``${ARGC}`` which will be set to the number of arguments passed