# MarkDown 绘制代码流程图     

一个纯文本的语法怎么画图？
将流程图代码包含在```folw和```之间即可
例子

```flow 
st=>start: 开始 
e=>end: 登录 
io1=>inputoutput: 输入用户名密码 
sub1=>subroutine: 数据库查询子类 
cond=>condition: 是否有此用户 
cond2=>condition: 密码是否正确 
op=>operation: 读入用户信息
st->io1->sub1->cond 
cond(yes,right)->cond2 
cond(no)->io1(right) 
cond2(yes,right)->op->e 
cond2(no)->io1 
```
流程图代码分两块，上面一块是创建你的流程（创建元素），然后隔一行，创建流程的走向(连接元素)

创建流程（元素）：tag=>type: content:>url    
tag 是流程图中的标签，在第二段连接元素时会用到。名称可以任意，一般为流程的英文缩写和数字的组合。
type 用来确定标签的类型，=>后面表示类型。由于标签的名称可以任意指定，所以要依赖type来确定标签的类型
标签有6种类型：start end operation subroutine condition inputoutput
content 是流程图文本框中的描述内容，: 后面表示内容，中英文均可。特别注意，冒号与文本之间一定要有个空格
url是一个连接，与框框中的文本相绑定，:>后面就是对应的 url 链接，点击文本时可以通过链接跳转到 url 指定页面
指向流程(连接元素)：标识（类别）->下一个标识
使用 -> 来连接两个元素
对于condition类型，有yes和no两个分支，如示例中的cond(yes)和cond(no)
每个元素可以制定分支走向，默认向下，也可以用right指向右边，如示例中cond2(yes,right)。
Created with Raphaël 2.1.4开始输入用户名密码数据库查询子类是否有此用户密码是否正确读入用户信息登录yesnoyesno
流程图元素

开始 
st=>start: 开始
操作 
op1=>operation: 操作、执行说明
条件 
cond=>condition: 确认？
子程序 
sub1=>subroutine: 子程序操作说明
用户输入或输出 
io1=>inputoutput: 输入密码
结束 
e=>end: 结束
简单的流程图，几句MD语法代码即可解决，不必去各种拖拽~

