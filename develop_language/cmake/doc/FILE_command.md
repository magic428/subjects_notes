# cmake FILE 指令详解

> https://cmake.org/cmake/help/latest/command/file.html?highlight=file

## 一. Reading 指令  

### 1. FILE(READ <filename> <variable>...)

```cmake
FILE(READ <filename> <variable>
     [OFFSET <offset>] [LIMIT <max-in>] [HEX])
```

从文件名为 <filename> 的文件中读取内容, 保存在 <variable> 变量中;  

可选参数为: <offset> 可指定读取的起始偏移; <max-in> 指定要读取数据的最大长度; HEX 可以将二进制数据转换为十六进制.    

### 2. FILE(READ <filename> <variable>...)

file(STRINGS <filename> <variable> [<options>...])
Parse a list of ASCII strings from <filename> and store it in <variable>. Binary data in the file are ignored. Carriage return (\r, CR) characters are ignored. The options are:

LENGTH_MAXIMUM <max-len>
Consider only strings of at most a given length.
LENGTH_MINIMUM <min-len>
Consider only strings of at least a given length.
LIMIT_COUNT <max-num>
Limit the number of distinct strings to be extracted.
LIMIT_INPUT <max-in>
Limit the number of input bytes to read from the file.
LIMIT_OUTPUT <max-out>
Limit the number of total bytes to store in the <variable>.
NEWLINE_CONSUME
Treat newline characters (\n, LF) as part of string content instead of terminating at them.
NO_HEX_CONVERSION
Intel Hex and Motorola S-record files are automatically converted to binary while reading unless this option is given.
REGEX <regex>
Consider only strings that match the given regular expression.
ENCODING <encoding-type>
Consider strings of a given encoding. Currently supported encodings are: UTF-8, UTF-16LE, UTF-16BE, UTF-32LE, UTF-32BE. If the ENCODING option is not provided and the file has a Byte Order Mark, the ENCODING option will be defaulted to respect the Byte Order Mark.
For example, the code

file(STRINGS myfile.txt myfile)
stores a list in the variable myfile in which each item is a line from the input file.

file(<HASH> <filename> <variable>)
Compute a cryptographic hash of the content of <filename> and store it in a <variable>. The supported <HASH> algorithm names are those listed by the string(<HASH>) command.

file(TIMESTAMP <filename> <variable> [<format>] [UTC])
Compute a string representation of the modification time of <filename> and store it in <variable>. Should the command be unable to obtain a timestamp variable will be set to the empty string (“”).

See the string(TIMESTAMP) command for documentation of the <format> and UTC options.


