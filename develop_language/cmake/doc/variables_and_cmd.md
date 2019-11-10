# CMake 变量和指令函数  

## 0. 目录  

- 一. Cmake 变量的定义和引用;  
- 二. cmake 系统内置变量;  
- 三. Cmake 调用操作系统中的环境变量;  
- 四. 主要的开关选项;  
- 五. 系统信息;  
- 六. Cmake 常用指令;  


## macro  
Start recording a macro for later invocation as a command:

macro(<name> [arg1 [arg2 [arg3 ...]]])
  COMMAND1(ARGS ...)
  COMMAND2(ARGS ...)
  ...
endmacro(<name>)
Define a macro named <name> that takes arguments named arg1, arg2, arg3, (…). Commands listed after macro, but before the matching endmacro(), are not invoked until the macro is invoked. When it is invoked, the commands recorded in the macro are first modified by replacing formal parameters (${arg1}) with the arguments passed, and then invoked as normal commands. In addition to referencing the formal parameters you can reference the values ${ARGC} which will be set to the number of arguments passed into the function as well as ${ARGV0}, ${ARGV1}, ${ARGV2}, … which will have the actual values of the arguments passed in. This facilitates creating macros with optional arguments. Additionally ${ARGV} holds the list of all arguments given to the macro and ${ARGN} holds the list of arguments past the last expected argument. Referencing to ${ARGV#} arguments beyond ${ARGC} have undefined behavior. Checking that ${ARGC} is greater than # is the only way to ensure that ${ARGV#} was passed to the function as an extra argument.

See the cmake_policy() command documentation for the behavior of policies inside macros.

Macro Argument Caveats
Note that the parameters to a macro and values such as ARGN are not variables in the usual CMake sense. They are string replacements much like the C preprocessor would do with a macro. Therefore you will NOT be able to use commands like:

if(ARGV1) # ARGV1 is not a variable
if(DEFINED ARGV2) # ARGV2 is not a variable
if(ARGC GREATER 2) # ARGC is not a variable
foreach(loop_var IN LISTS ARGN) # ARGN is not a variable
In the first case, you can use if(${ARGV1}). In the second and third case, the proper way to check if an optional variable was passed to the macro is to use if(${ARGC} GREATER 2). In the last case, you can use foreach(loop_var ${ARGN}) but this will skip empty arguments. If you need to include them, you can use:

set(list_var "${ARGN}")
foreach(loop_var IN LISTS list_var)
Note that if you have a variable with the same name in the scope from which the macro is called, using unreferenced names will use the existing variable instead of the arguments. For example:

macro(_BAR)
  foreach(arg IN LISTS ARGN)
    [...]
  endforeach()
endmacro()

function(_FOO)
  _bar(x y z)
endfunction()

_foo(a b c)
Will loop over a;b;c and not over x;y;z as one might be expecting. If you want true CMake variables and/or better CMake scope control you should look at the function command.


cmake中的宏(macro)和函数(function)都支持动态参数
变量ARGC记录传入的参数个数
变量ARGV0,ARGV1,...顺序代表传入的参数
变量ARGV则是一个包含所有传入参数的list
变量ARGN也是一个包含传入参数的list，但不是所有参数，而是指macro/function声明的参数之后的所有传入参数

写一个小程序就可以验证：

# 定义一个宏，显式声明了两个参数hello,world
macro(argn_test hello world)
	MESSAGE(STATUS ARGV=${ARGV})
	MESSAGE(STATUS ARGN=${ARGN})
	MESSAGE(STATUS ARGV0=${ARGV0})
	MESSAGE(STATUS ARGV1=${ARGV1})
	MESSAGE(STATUS ARGV2=${ARGV2})
	MESSAGE(STATUS ARGV3=${ARGV3})
endmacro()
# 调用宏时传入4个参数
argn_test(TOM JERRY SUSAN BERN)

cmake脚本执行输出结果，ARGN为声明参数之后的所有参数

-- ARGV=TOMJERRYSUSANBERN
-- ARGN=SUSANBERN
-- ARGV0=TOM
-- ARGV1=JERRY
-- ARGV2=SUSAN
-- ARGV3=BERN
 ———————————————— 