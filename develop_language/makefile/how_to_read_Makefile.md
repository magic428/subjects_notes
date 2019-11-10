## Makefile 语法分析


## patsubst - 替换通配符  

格式: $(patsubst <pattern>,<replacement>,<text> ) 

功能： 查找 <text> 中的单词（单词以 “空格”、“Tab” 或 “回车” “换行” 分隔）是否符合模式 <pattern>， 如果匹配的话， 则以 <replacement> 替换。这里， <pattern> 可以包括通配符 “%”， 表示任意长度的字串。如果 <replacement> 中也包含 “%”， 那么，<replacement> 中的这个 “%” 将是 <pattern> 中的那个 “%” 所代表的字串。（可以用 “\” 来转义，以 “\%” 来表示真实含义的 “%” 字符）   

返回：函数返回被替换过后的字符串。

示例：

$(patsubst %.c,%.o,x.c.c bar.c)

把字串 “x.c.c bar.c” 符合模式 [%.c] 的单词替换成 [%.o]，返回结果是 “x.c.o bar.o”   


## addprefix - 添加前缀

格式: $(addprefix PREFIX, NAMES...)   

函数功能：为 "NAMES..." 中的每一个文件名添加前缀 "PREFIX"。 参数 "NAMES..." 是空格分割的文件名序列， 将 "PREFIX" 添加到此序列的每一个文件名之前。  

返回值：以单空格分割的添加了前缀 "PREFIX" 的文件名序列。  

示例:  

$(addprefix src/, foo bar)

返回值为“src/foo src/bar”.  

## wildcard - 扩展通配符

格式: $(wildcard patterns...)  

示例:  
	
	SRC = $(wildcard *.c)

等于指定编译当前目录下所有 .c 文件，如果还有子目录，比如子目录为 inc，则再增加一个wildcard 函数， 象这样：  

	SRC = $(wildcard *.c) $(wildcard inc/*.c)

也可以指定汇编源程序：   

	ASRC = $(wildcard *.S)


## notdir - 去除路径 

格式: $(notdir NAMES...)   


## 获得文件所在的目录 

获取相对目录格式: $(dir filespath)   
获取绝对目录格式: $(abspath $(lastword $(MAKEFILE_LIST)))   

 
## 变量替换引用

对于一个已经定义的变量， 可以使用 “替换引用” 将其值中的后缀字符（串）使用指定的字符（字符串）替换。 

格式: “$(VAR:A=B)”（或者“${VAR:A=B}”）   

作用是，替换变量 “VAR” 中所有 “A” 字符结尾的字为 “B” 结尾的字。“结尾” 的含义是空格之前（变量值多个字之间使用空格分开）。而对于变量其它部分的 “A” 字符不进行替换。  

例如：

	foo := a.o b.o c.o  
	bar := $(foo:.o=.c)  

在这个定义中，变量 “bar” 的值就为 “a.c b.c c.c”。使用变量的替换引用将变量 “foo” 以空格分开的值中的所有的字的尾字符 “o” 替换为 “c”，其他部分不变。   

## Makefile中foreach使用

foreach 函数的语法是：

	$(foreach <var>,<list>,<text>)

把参数 <list> 中的单词逐一取出放到参数 <var> 所指定的变量中， 然后再执行 <text> 所包含的表达式。 每一次 <text> 会返回一个字符串， 循环过程中 <text> 返回的每个字符串会以空格分隔， 最后当整个循环结束时，<text> 所返回的每个字符串所组成的整个字符串（以空格分隔）就是 foreach 函数的返回值。  

所以，<var> 最好是一个变量名， <list> 可以是一个表达式， 而 <text> 中一般会使用<var> 这个参数来依次枚举 <list> 中的单词。举个例子： 

```makefile
names := a b c d
files := $(foreach n,$(names),$(n).o)
```

上面的例子中， $(name) 中的单词会被挨个取出，并存到变量 “n” 中， “$(n).o” 每次根据 “$(n)” 计算出一个值，这些值以空格分隔，最后作为 foreach 函数的返回， 所以 $(files) 的值是 “a.o b.o c.o d.o”。  

注意， foreach 中的 <var> 参数是一个临时的局部变量， foreach 函数执行完后，参数<var> 的变量将不复作用，其作用域只在 foreach 函数当中。  

## 打印变量值  

1、输出打印信息的方法是：$(warning xxxxx)，$(error xxxxx)
2、输出打印变量值的方法是：$(warning  $(XXX))

格式: $(error TEXT…)     

`函数功能`：产生致命错误，并提示“TEXT…”信息给用户，并退出 make的执行。需要说明的是：“error”函数是在函数展开式（函数被调用时）才提示信息并结束 make进程。因此如果函数出现在命令中或者一个递归的变量定义中时，在读取 Makefile 时不会出现错误。而只有包含“error”函数引用的命令被执行，或者定义中引用此函数的递归变量被展开时，才会提示致命信息“TEXT…”同时退出 make。   

`返回值`：空 

`函数说明`： “error” 函数一般不出现在直接展开式的变量定义中，否则在 make 读取Makefile 时将会提示致命错误。


示例 1:   

```
ERR = $(error found an error!) 
 
.PHONY: err 
err: ; $(ERR) 
```

这个例子， 在 make 读取 Makefile 时不会出现致命错误。只有目标 “err” 被作为一个目标被执行时才会出现。   

示例 2:   

```
ifeq ($(wildcard .git), )
	$(error YOU HAVE TO USE GIT TO DOWNLOAD THIS REPOSITORY. ABORTING.)
endif
```

1.ifeq 是用来判断前面的那个条目是否为空。
2.$(error TEXT…) 用于产生致命错误，并提示后边的”TEXT...“信息给用户。

## 规则语法

通常规则的语法格式如下：  

```makefile
TARGETS : PREREQUISITES
	COMMAND
...

# 或者：

TARGETS : PREREQUISITES ; COMMAND
	COMMAND
...
```

规则中 “TARGETS” 可以是空格分开的多个文件名，也可以是一个标签（例如：执行清空的“ clean”）。“ TARGETS”的文件名可以使用通配符，格式“ A(M)”表示档案文件（ Linux下的静态库.a文件）的成员“ M”（关于静态库的重建可参考 第十一章 使用make更新静态库文件）。通常规则只有一个目标文件（建议这么做），偶尔会在一个规则中需要多个目标。

书写规则是我们需要注意的几点：

1. 规则的命令部分有两种书写方式： a. 命令可以和目标：依赖描述放在同一行。命令在依赖文件列表后并使用分号（；）和依赖文件列表分开。 b. 命令在目标：依赖的描述的下一行，作为独立的命令行。 当作为独立的命令行时此行必须以[Tab]字符开始。在 Makefile 中，在第一个规则之后出现的所有以[Tab]字符开始的行都会被当作命令来处理。

2. Makefile 中符号“ $”有特殊的含义（表示变量或者函数的引用），在规则中需要使用符号“ $”的地方，需要书写两个连续的（“ $$”）。

3. 前边已提到过，对于 Makefile 中一个较长的行，我们可以使用反斜线“ \”将其书写到几个独立的物理行上。虽然 make 对 Makefile 文本行的最大长度是没有限制的，但还是建议这样做。不仅书写方便而且更有利于别人的阅读（这也是一个程序员修养的体现）。

依赖的类型
在 GNU make 的规则中可以使用两种不同类型的依赖：

1. 以前章节所提到的规则中使用的是常规依赖，这是书写 Makefile 规则时最常用的一种。

2. 另外一种在我们书写 Makefile 时不会经常使用，它比较特殊、称之为“ order-only”依赖。

一个规则的常规依赖（通常是多个依赖文件）表明了两件事：

首先，它决定了重建此规则目标所要执行规则（确切的说是执行命令）的顺序；表明在更新这个规则的目标（执行此规则的命令行）之前需要按照什么样的顺序、执行那些规则（命令）来重建这些依赖文件（对所有依赖文件的重建，使用明确或者隐含规则。就是说对于这样的规则： A:B C，那么在重建目标 A 之前，首先需要完成对它的依赖文件 B 和 C 的重建。重建 B 和 C 的过程就是执行 Makefile 中以文件 B 和 C 为目标的规则）。

其次，它确定了一个依存关系；规则中如果依赖文件的任何一个比目标文件新，则认为规则的目标已经过期而需要重建目标文件。

有时，需要定义一个这样的规则，在更新目标（ 目标文件已经存在）时只需要根据依赖文件中的部分来决定目标是否需要被重建，而不是在依赖文件的任何一个被修改后都重建目标。为了实现这一目的，相应的就需要对规则的依赖进行分类，一类是在这些依赖文件被更新后，需要更新规则的目标；另一类是更新这些依赖的，可不需要更新规则的目标。我们把第二类称为：“ order-only”依赖。书写规则时，“ order-only”依赖使用管道符号“ |”开始，作为目标的一个依赖文件。规则依赖列表中管道符号“ |”左边的是常规依赖，管道符号右边的就是“ order-only”依赖。这样的规则书写格式如下：

TARGETS : NORMAL-PREREQUISITES | ORDER-ONLY-PREREQUISITES

文件名使用通配符
Maekfile 中表示文件名时可使用通配符。可使用的通配符有：“ *”、“ ?”和“ […]”。在 Makefile 中通配符的用法和含义和 Linux（ unix）的 Bourne shell 完全相同。例如，“ *.c”代表了当前工作目录下所有的以“ .c”结尾的文件等。但是在 Makefile 中这些统配符并不是可以用在任何地方， Makefile 中统配符可以出现在以下两种场合：

1. 可以用在规则的目标、依赖中， make 在读取 Makefile 时会自动对其进行匹配处理（通配符展开）；

2. 可出现在规则的命令中，通配符的通配处理是在 shell 在执行此命令时完成的。除这两种情况之外的其它上下文中，不能直接使用通配符。而是需要通过函数“ wildcard”来实现。

如果规则的一个文件名包含统配字符（“ *”、“ .”等字符），在使用这样的文件时需要对文件名中的统配字符使用反斜线（ \）进行转义处理。例如“ foo\*bar”，在 Makefile中它表示了文件“ foo*bar”。 Makefile 中对一些特殊字符的转移和 B-SHELL 以及 C 语言中的基本上相同。

变量定义中使用的通配符不会被统配处理（因此在变量定义中不能使用通配符，否则在某些情况下会出现非预期的结果，下一小节将会详细讨论）。在 Makefile 有这样一个变量定义：“ objects = *.o”。它表示变量“ objects”的值是字符串“ *.o”（并不是期望的空格分开的.o 文件列表）。当需要变量“ objects”代表所有.o 文件列表示，需要使用函数“ wildcard”（ objects = $(wildcar *.o)）。

目录搜寻
一般搜索（变量VPATH）

GNU make 可以识别一个特殊变量“ VPATH”。通过变量“ VPATH”可以指定依赖文件的搜索路径，当规则的依赖文件在当前目录不存在时， make 会在此变量所指定的目录下去寻找这些依赖文件。通常我们都是用此变量来指定规则的依赖文件的搜索路径。其实“ VPATH”变量所指定的是 Makefile 中所有文件的搜索路径，包括了规则的依赖文件和目标文件。

定义变量“ VPATH”时，使用空格或者冒号（ :）将多个需要搜索的目录分开。 make搜索目录的顺序是按照变量“ VPATH”定义中的目录顺序进行的（当前目录永远是第一搜索目录）。

 选择性搜索（关键字vpath）

另一个设置文件搜索路径的方法是使用 make 的“ vpath”关键字（全小写的）。它不是一个变量，而是一个 make 的关键字，它所实现的功能和上一小节提到的“ VPATH”变量很类似，但是它更为灵活。它可以为不同类型的文件（由文件名区分）指定不同的搜索目录。它的使用方法有三种：

 1、 vpath PATTERN DIRECTORIES为所有符合模式“ PATTERN”的文件指定搜索目录“ DIRECTORIES”。多个目录使用空格或者冒号（：）分开。类似上一小节的“ VPATH”变量。

 2、 vpath PATTERN清除之前为符合模式“ PATTERN”的文件设置的搜索路径。

 3、 vpath清除所有已被设置的文件搜索路径。

vapth 使用方法中的“ PATTERN”需要包含模式字符“ %”。“ %”意思是匹配一个或者多个字符，例如，“ %.h”表示所有以“ .h”结尾的文件。如果在“ PATTERN”中没有包含模式字符“ %”，那么它就是一个明确的文件名，这样就是给定了此文件的所在目录，我们很少使用这种方式来为单独的一个文件指定搜索路径。在“ vpath”所指定的模式中我们可以使用反斜杠来对字符“ %”进行引用（和其他的特使字符的引用一样）。

Makefile伪目标
伪目标是这样一个目标：它不代表一个真正的文件名，在执行 make 时可以指定这个目标来执行其所在规则定义的命令，有时也可以将一个伪目标称为标签。使用伪目标有两点原因：

 1. 避免在我们的 Makefile 中定义的只执行命令的目标（此目标的目的为了执行执行一些列命令，而不需要创建这个目标）和工作目录下的实际文件出现名字冲突。

 2. 提高执行 make 时的效率，特别是对于一个大型的工程来说，编译的效率也许你同样关心。

如果我们需要书写这样一个规则：规则所定义的命令不是去创建目标文件，而是通过 make 命令行明确指定它来执一些特定的命令。像常见的 clean 目标：

clean:

rm *.o temp

规则中“ rm”不是创建文件“ clean”的命令，而是删除当前目录下的所有.o 文件和 temp文件。当工作目录下不存在“ clean”这个文件时，我们输入“ make clean”，“ rm *.otemp”总会被执行。 但是如果在当前工作目录下存在文件“ clean”，情况就不一样了，同样我们输入“ make clean”，由于这个规则没有任何依赖文件，所以目标被认为是最新的而不去执行规则所定义的命令，因此命令“ rm”将不会被执行。这并不是我们的初衷。为了解决这个问题，我们需要将目标“ clean”声明为伪目标。将一个目标声明为伪目标的方法是将它作为特殊目标.PHONY”的依赖。如下：

.PHONY : clean

这样目标“ clean”就被声明为一个伪目标，无论在当前目录下是否存在“ clean”这个文件。我们输入“ make clean”之后。“ rm”命令都会被执行。而且，当一个目标被声明为伪目标后， make 在执行此规则时不会去试图去查找隐含规则来创建它。这样也提高了 make 的执行效率，同时也不用担心由于目标和文件名重名而使我们的期望失败。

伪目标的另外一种使用场合是在 make 的并行和递归执行过程中。

强制目标（没有命令或依赖的规则）
如果一个规则没有命令或者依赖，并且它的目标不是一个存在的文件名。在执行此规则时，目标总会被认为是最新的。就是说：这个规则一旦被执行， make 就认为它的目标已经被更新过。这样的目标在作为一个规则的依赖时，因为依赖总被认为被更新过，因此作为依赖所在的规则中定义的命令总会被执行。看一个例子：

clean: FORCE

rm $(objects)

FORCE:

这个例子中，目标“ FORCE”符合上边的条件。它作为目标“ clean”的依赖，在执行make 时，总被认为被更新过。因此“ clean”所在规则在被执行时其所定义的命令总会被执行。这样的一个目标通常我们将其命名为“ FORCE”。

空目标文件
空目标文件是伪目标的一个变种；此目标所在规则执行的目的和伪目标相同——通过 make 命令行指定将其作为终极目标来执行此规则所定义的命令。和伪目标不同的是：这个目标可以是一个存在的文件，但文件的具体内容我们并不关心，通常此文件是一个空文件。

空目标文件只是用来记录上一次执行此规则命令的时间。在这样的规则中，命令部分都会使用“ touch”在完成所有命令之后来更新目标文件的时间戳，记录此规则命令的最后执行时间。 make 时通过命令行将此目标作为终极目标，当前目录下如果不存在这个文件，“ touch”会在第一次执行时创建一个空的文件（命名为空目标文件名）。

多目标
一个规则中可以有多个目标，规则所定义的命令对所有的目标有效。一个具有多目标的规则相当于多个规则。规则的命令对不同的目标的执行效果不同，因为在规则的命令中可能使用了自动环变量“ $@”。多目标规则意味着所有的目标具有相同的依赖文件。

虽然在多目标的规则中，可以根据不同的目标使用不同的命令（在命令行中使用自动化变量“ $@”）。但是，多目标的规则并不能做到根据目标文件自动改变依赖文件（像上边例子中使用自动化变量“ $@”改变规则的命令一样）。需要实现这个目的是，要用到make的静态模式。

 静态模式
静态模式规则是这样一个规则：规则存在多个目标，并且不同的目标可以根据目标文件的名字来自动构造出依赖文件。相当于是一个过滤器。

双冒号规则
双冒号规则就是使用“ ::”代替普通规则的“ :”得到的规则。当同一个文件作为多个规则的目标时，双冒号规则的处理和普通规则的处理过程完全不同（双冒号规则允许在多个规则中为同一个目标指定不同的重建目标的命令）。

首先需要明确的是： Makefile 中，一个目标可以出现在多个规则中。但是这些规则必须是同一类型的规则，要么都是普通规则， 要么都是双冒号规则。而不允许一个目标同时出现在两种不同类型的规则中。

双冒号规则和普通规则的处理的不同点表现在以下几个方面：

1. 双冒号规则中，当依赖文件比目标更新时。规则将会被执行。对于一个没有依赖而只有命令行的双冒号规则，当引用此目标时，规则的命令将会被无条件执行。而普通规则，当规则的目标文件存在时，此规则的命令永远不会被执行（目标文件永远是最新的）。

2. 当同一个文件作为多个双冒号规则的目标时。这些不同的规则会被独立的处理，而不是像普通规则那样合并所有的依赖到一个目标文件。这就意味着对这些规则的处理就像多个不同的普通规则一样。就是说多个双冒号规则中的每一个的依赖文件被改变之后， make 只执行此规则定义的命令，而其它的以这个文件作为目标的双冒号规则将不会被执行。

 