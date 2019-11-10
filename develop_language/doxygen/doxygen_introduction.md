# Doxygen 配置详解
[TOC]
注意：可以任意转载，但请注名出处

（翻译）
Doxygen 配置详解（1）
##Getting Started
　使用Doxygen文档开发工具时需要进行的配置：
　　doxygen 对原代码分析来生成文档. 请看 Doxygen usage 章节来获取更详细的使用帮助. Doxytag可执行文件---仅仅是用来实现帮助程序员生成不需要看原代码就能了解工程部署信息的doxygen文档的参考文档( 例如：那些使用doxygen生成的文档).请看Doxytag usage 章节来获得更多的使用帮助.
　　
![](https://raw.githubusercontent.com/gzj2013/markdown_src/master/Doxygen%20information%20flow.png)
<center>图1：Doxygen information flow</center > 
　
###Step 1: 创建一个配置文件
　　Doxygen使用一个配置文件来确定它所有的设置. 每个工程都应该有它自己的配置文件.
　一个工程可以只有一个源文件, 也可以是工程中所有源文件的递归扫描得到的源文件的树状视图。
　为了简化doxygen生成配置文件的工作, doxygen 可以为你提供一个模板化的配置文件. 
1. 为了创建一个模板化的配置文件，只需要调用doxygen并在命令行中敲入-g: 
```c
$ doxygen -g config-file
```

　其中 config-file是某个模板化的配置文件的文件名. 如果你省略了文件名, doxygen会为你生成一个默认的Doxyfile的配置文件. 如果config-file是一个已经存在的文件名, doxygen 在生成配置模板之前，将会生成一个 config-file. Bak的备份文件。
　
2. 如果你使用 - (例如：减号) 作为文件名doxygen将会把你从键盘输入的文字当作配置文件名。
配置文件的格式和Makefile相似.主要是：包含了很多的"标记"分配符 (tags): 
例如：
```c
TAGNAME = VALUE or 
TAGNAME = VALUE1 VALUE2 ... 
```

　在生成文档模板时，你可以使用默认这些标记的默认值（即：保留大多数的TAGS）详细信息请看 Configuration 这一章节来获取更多的信息. 
　
　如果你不想使用文本编辑工具来编写配置文件,你应该看看 doxywizard 章节的描述, 它是一个可以用来创建、读、写doxygen 配置文档的图形化工具，同时它也可以在路径中进行全路径配置来使doxygen正常工作。
3. 对于一个有很少的源文件和头文件组成的C/C++工程来说, 你可以保留<font color=#0000FF> INPUT </font>标记为"空" ，那么 doxygen 将会在当前路径下搜索源文件. 
4. 如果你的工程很大，你应该在INPUT标记中指定工程文件的"根目录"，需要添加到工程中的文件应该放到FILE_PATTERNS 标记之后(例如： *.cpp *.h). 至少是匹配了1项的文件才能被doxygen程序读入并分析(如果省略了这项设置，则会使用doxygen配置列表中的格式).
5. 如果想要递归对源文件树进行分析必须设置<font color=#0000FF>RECURSIVE</font>标记为 YES.
6. 想在doxygen中使用更多的自定义规则进行分析，必须使用<font color=#0000FF>EXCLUDE</font>标记和 <font color=#0000FF>EXCLUDE_PATTERNS</font>标记。
7. 想忽略所有的 test路径下的文件，使用下面的形式: 
``` c
 EXCLUDE_PATTERNS = */test/* 
```
8. 对于C/C++文件Doxygen通常直接进行分析。 如果文件有 .idl或 .odl 扩展名，则doxygen会把它视为 IDL文件。
8. 有.java 扩展名的将被视为Java文件.
9. 使用 .cs作为扩展名的文件将会视为C# 文件. 
10. 使用.php, .php4扩展名的文件和用.inc 或 .phtml 扩展名的文件将被视为PHP 源文件. 
11. 如果你想用doxygen为已经存在的工程生成文档。你首先要想象一下你的工程文档最终使用什么样的格式排版，为了实现这样的目标，你必须要设置<font color=#0000FF>EXTRACT_ALL </font>标记为YES. 然后，doxygen表现出来的是它知道了所有工程文件的配置目标。
注意：如果设置<font color=#0000FF>EXTRACT_ALL</font> 标记为YES 。则：undocumented members 之类的警告将不会再产生。 
12.   使用doxygen来分析一个现存的源代码的某个部分或全部文件可以更清晰的明白源代码各个功能模块的定义和要实现的功能以及它们之间的交叉引用。
13.   使用Doxygen 生成交叉参考必须设置<font color=#0000FF> SOURCE_BROWSER</font>标记为 YES。也可以直接通过设置<font color=#0000FF> INLINE_SOURCES</font> 标记为 YES 来实现把工程的所有源代码包含进文档中。(这样方便了代码的通览).

### Step 2: 运行 doxygen
To generate the documentation you can now enter: 
```c
$ doxygen config-file
```
   Doxygen会根据doxygen的设置在输出路径中创建html, rtf, latex或man目录。 路径和路径中的文件格式是对应的HTML,LATEX, RTF,XML和Unix-Man格式. 
   默认的路径是doxygen的安装路径。可以使用 <font color=#0000FF>OUTPUT_DIRECTORY, HTML_OUTPUT, RTF_OUTPUT, LATEX_OUTPUT</font>, 和<font color=#0000FF>MAN_OUTPUT</font>标记来自定义配置文档的输出路径。如果输出路径不存在doxygen将会为你创建一个输出路径，但是它不会像mkdir -p那样为你递归创建目录。 
   
#### HTML output
   生成的 HTML 可以通过使用浏览器浏览位于html路径下的 index.html 文件. 如果浏览器支持层叠样式表 (CSS) 那就更棒了。
   HTML的一些特性需要浏览器支持动态HTML和使能JSP(例如GENERATE_TREEVIEW 或是搜索引擎)。

#### LaTeX output
   生成的LaTeX文档必须要先用LaTeX编译器进行编译(我使用 teTeX 0.9版本，其中包含了 3.14159)。 为了简化编译文档的生成过程，doxygen在latex 路径下提供了一个Makefile (Windows 平台下生成的是 make.bat 批处理文件)。
   Makefile中的内容和目标依赖于<font color=#0000FF>USE_PDFLATEX</font>。如果它被设置为NO，在latex路径下执行 make 会生成一个refman.dvi文件，使用xdvi命令可以查看这个文件，或者通过执行make ps(这需要有dvips工具)把它转换成 refman.ps。
   执行make ps_2on1命令可以实现分成2页的效果。PostScript文件可以被发送到PostScript打印机输出。如果你没有PostScript打印机，你可以使用ghostscript 命令把PostScript文件格式转换成你的打印机能够识别的文件格式。
   如果你已经安装了ghostscript 解释程序，那么执行make pdf (或make pdf_2on1)指令就可以把文件转换成 PDF格式。
   如果想想生成PDF文件，需要把<font color=#0000FF> PDF_HYPERLINKS </font> 和 <font color=#0000FF>USE_PDFLATEX </font> 标记设置成YES。
   
#### RTF output

#### XML output

#### Man page output
   产生的man页面文件可以通过man程序来进行查看。但是，你必须确定man路径有相应的环境变量设置 (一般在 MANPATH环境变量中)。注意：man页面文件的格式有一些限制 ，所以有些信息(像：class图，交叉参考，公式等)将会丢失掉。

#### DocBook output

### Step 3: Documenting the sources ###
   尽管源代码编制文档的被作为第3步，但是，在某些新的工程中，这个是作为第1步来做的。 这里，假设你已经有了一些想用doxygen来对其进行文档化（描述API接口和作用）的源代码。
   如<font color=#0000FF> EXTRACT_ALL </font> 选项被设置成NO（默认情况下是NO ）那么doxygen只会为已经文档化的成员，文件，类和命名空间生成文档。如果你的文档属于这种情况，该怎么办呢？对于成员，类和命名空间有2种基本的设置：
1. 成员，类或命名空间的前面安排一个描述或定义的块。对于文件，类和命名空间成员来说，doxygen 允许直接在成员后面安排文档。你可以参考： Special documentation blocks 了解更多特殊块的设置。
2. 想在任何地方部署特殊的文档块（任何的文件或任何的路径）和在文档块中添加一个"结构化"的命令。"结构化"的命令用来设置一个可被编制成文档的链接。 (e.g. a member, class, namespace or file)。请看：Documentation at other places 了解更多的结构化命令的使用方法。
   第一个选项的优点在于你不需要 重复entity的名字；
   文件只能使用上面2中的方法进行设置，因为没有办法把一个文档块放到一个文件的前面。
   当然，文件成员（函数，变量，类型定义，define）不需要显式的使用"结构化"命令，只需要把特殊的文档块放到文件中的最前面或最后面就可以了。
   文档内部的文档块在输出为HTML格式或其他格式输出文件之前进行doxygen的语法分析：
它其实是在进行下面的步骤前进行分析：
文档内部的特殊"结构化"命令被执行的时候。请看： Special Commands 章节获取所有的命令参考信息。
如果某行中使用"空格+后面使用1个或多个*号"，或者是很多的"空格"符，则所有的空格和"*"号都会被删除。
所有的"空行"都会被视为"图形分隔符"。这项安排可以使你有"部署自定义图形分隔符"的能力，以产生更具可读性的文档。
Doxygen将会为所有已经归档的classes生成链接。
如果在文档中找到符合doxygen文档格式的成员，那么也会为members创建链接。请参考：Automatic link generation获取更多的如何自动化文档链接。
文档中的HTML标记被解释和转换成相应的输出。请看：HTML Commands章节获取更多的关于HTML标记的使用信息。



## Doxygen 的使用说明 ##
   Doxygen 是一个基于命令行的实用工具。在命令行使用doxygen --help可以显示关于各种命令的简短描述。所有的命令选项都包括前导字符-， 后面跟什么命令（1个字符或是多个字符）取决于你的选择。
   为工程产生一个手册，典型的情况----你需要经过下面几个步骤：
   1. 你想要在你的文档中使用特殊的块状标识 (请参考Special documentation blocks章节)。
   2. 可以通过使用doxygen 中的-g 来创建配置文件(请参考Configuration章节)：
   ```
   $   doxygen -g [config_file]
   ```
   3. 你需要针对你的工程编写相应的配置文件。在配置文件中你可以指定输入文件以及很多其它的信息。
   4. 通过使用下面的命令来对你配置过的文件产生相应的文档：
   ```
   $   doxygen [config_file]
   ```
   如果你之前使用老版本的doxygen生成过配置文件，你可以通过运行doxygen 中的 -u 命令来对其进行更新。
   ```
   $   doxygen -u [config_file]
   ```
   在原来配置文件中的所有信息都会被替换成新的配置文件。所有新版本doxygen中的配置都会使用默认值。但是，你改动前的原来的配置文件中添加过的所有注释都会被自动删除。 
   
### 对输出进行精细调整 ###

   如果你想让产生的文档能够按照某种你自定义的格式进行显示的话，你可以对doxygen生成的默认样式，头部和脚注形式的文件进行修改：

   - 在HTML输出中，使用下面的命令可以生成默认的头部HTML_HEADER、页脚HTML_FOOTER和样式表HTML_STYLESHEET：
   ```
   $   doxygen -w html header.html footer.html stylesheet.css
   ```
   - 在LaTeX输出中，使用下面的命令来生成refman.tex的LATEX_HEADER、LATEX_FOOTER和显示样式(通常是doxygen.sty)：
   ```
   $   doxygen -w latex header.tex doxygen.sty
   ```
   - 在RTF输出中, 使用下面的命令创建一个默认的样式格式配置文件 (请查看RTF_STYLESHEET_FILE获得更详细的信息)：
   ```
   $   doxygen -w rtf rtfstyle.cfg
   ```
**警告**


**注意:**

   - 如果你不想对配置文件中的每一项都进行编制，你可以使用***-s***命令。-s与-u同时使用，可以对已存在的配置文件增加或删除文档。如果你想向我报告BUG，请使用-s命令。
   - 想要使用doxygen从标准输入/输出(如：键盘/显示器)读/写，而不是从文件中读写，需要使用 **-** 命令（后面跟上你想指定的文件名）。


## Chapter 23 Doxygen 的配置 ##

### 格式 ###

   一个配置文件是一个和Makefile有着类似结构的ASCII 文本文件，默认的配置文件名是Doxyfile。它是经doxygen分析后生成的。这个配置文件以固定的格式包含标记和新行 。配置文件中的设置是"大小写敏感"的。注释可以放在文档中的任何的位置(内部使用的"引号"除外)。注释采用的是以"#"开始的 单行注释形式。
文 件中包含了很多"赋值语句"的形式。每条语句主要包含了一个后面跟着一个"="号的标记名TAG_NAME， 然后后面是1个或者多个值。如果相似的标记已经被多次的使用，那么最后使用的标记将会覆盖掉前面使用过的标记。下面列出了所有命令的列表，表中的" +=" 号 可以用来代替"=新值"。如果使用多个命令参数，它们之间是"不能有空格符"出现的。如果某个命令参数需要使用"空格符"，则必须使用""把这个命令参数 括起来。写成多行时，必须使用反斜杠 (/)做为该行的结束。环境变量可以通过使用$(ENV_VARIABLE_NAME)进行扩展。 
想包含别的配置文件中的相关内容，只需使用下面格式的 @INCLUDE 标记即可： 

@INCLUDE = config_file_name

被包含的文件通常先从当前路径搜索。你也可以通过使用路径指定自定义路径中被包含的配置文件。使用命令 @INCLUDE_PATH ，后面跟上被包含的配置文件的具体的路径，如下所示：

@INCLUDE_PATH = my_config_dir

配置选项可以被分成几类。下面给出的命令开关（或者说：配置选项）以字母顺序给出：


下面是需要涉及到的配置参数，通过对它们进行具体的设置能够实现想要的自定义效果.......

有些地方翻译的不太合适，请参照相关的链接来了解它所表示的意思........

通过用文本编辑工具打开通过Doxygen生成的配置文件，然后用Ctrl+F来查找相关的"设置词汇"，来进行自定义，得到想要的文档显示格式

配置选项可以被分成几类。下面给出的命令开关（或者说：配置选项）以字母顺序给出：











Doxytag可执行文件---仅仅是用来实现帮助程序员生成不需要看原代码就能了解工程部署信息的doxygen文档的参考文档( 例如：那些使用doxygen生成的文档).请看Doxytag usage 章节来获得更多的使用帮助.

Doxytag 使用说明

Doxytag 是一个基于命令行的小程序。它能够产生标记文件。这些标记文件能够被doxygen使用作为自己的标记文件。这些标记文件能够被用来为产生外部文档产生参考信息。 (例如：那些要被doxygen使用到，但是被包含在输入文件中的部分)。
一个标记文件包含关于这个文件中的classes，成员的相关信息。Doxytag能够直接从HTML文件中抽取这些信息。 这样就有了一个好处，不需要源代码就能够从doxytag中知道源代码到底有什么文件组成，每个文件的组成和它们的功能，以及文件之间的相互关系。
通过在doxygen中进行的配置文件中进行相应的配置，你能够很容易的在GENERATE_TAGFILE 之后放上相应的标记文件名。 
关键: 
如果你使用标记文件，通过doxygen生成links将会包含dummy（哑元）链接。你必须运行installdox脚本程序把这些dummy链接转化成真实的链接。请参考Installdox usage 章节获取更多的相关信息。Dummy链接看起来好象是没什么用处的，但实际上它非常的有用---如果你想要把外部文档移动到任何位置的的时候你会发现它很有用。只要你运行着installdox，文档不需要通过doxygen进行重新生成。
注意: 
因为，通常要在HTML文档中形成某种特定的结构，但是，通过doxygen生成的HTML文件中只有HTML或者是按照字母排序的classes视图形式。Doxytag只能读取HTML格式。 
Doxytag能够为一个在HTML目录下的所有的HTML文件建立标记。如果没有没有进行设置，Doxytag将从当前路径读取所有的扩展名为html的文件。 如果使用 doxytag中的-t命令就能生成一个标记文件。
例 1: 
假如下面列出的出现在examples目录下的example.cpp文件，以特殊块儿显示的文件形式，没有它的源文件。那么你就很幸运了，可以通过软件包中包含的分配器可以过doxygen生成包含HTML文档的文档形式显示的文件。

/** A Test class.

*  More details about this class.

*/


class Test

{

  public:

    /** An example member function.

     *  More details about this function.

     */

    void example();

};


 

void Test::example() {}


 

/** /example example_test.cpp

* This is an example of how to use the Test class.

* More details about this example.

*/

你只用使用下面的命令从HTML文件创建一个标记文件就能达到分析源文件由哪些部分组成，各个部分都是什么功能，以及它们之间有什么联系：

doxytag -t example.tag example/html

上面的examples是HTML文档原始的路径。这样，你就能在你自己的代码块儿中使用上面的标记文件了，如下所示照葫芦画瓢即可：

/*! A class that is inherited from the external class Test.

*/


 

class Tag : public Test

{

  public:

    /*! an overloaded member. */

    void example();

};

Doxygen能够为你的文档提供包含上所有指定过的链接选项。因为标记文件不能确定文档定位在哪里，你必须通过运行installdox脚本程序确定doxygen生成的文件放置到什么位置。(请参考 Installdox usage 章节获得更多的信息)。
注意：（不管你移动别人的文档或者是别人移动你的文档）把外部的文档移动到一个不同的路径下面或者改变链接设置，你只需简单的运行脚本就能实现所有连接的同步更新。 
点击 here 查看通过doxygen生成的相应的HTML文档和下面的第2个例子。 
例2: 
使用下面的命令行生成一个标记文档：

doxytag -t qt.tag $QTDIR/doc/html

 

 