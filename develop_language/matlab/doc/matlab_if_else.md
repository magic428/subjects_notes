# `MATLAB`分支结构
决策结构要求程序员应指定要由程序评估计算或测试的一个或多个条件，以及条件确定为真时要执行的语句或语句，如果条件被确定为假时，可选地如果执行其他语句。   
以下是大多数编程语言中典型的决策结构的一般形式：   
![](../../pictures/if_else.png)     
MATLAB提供以下类型的决策语句。点击以下链接来查看它们的详细说明 -

|语句|描述|
|-----|-----|
|if…end 	|if ... end语句包含一个布尔表达式，后跟一个或多个语句。|
|if…else…end|if语句可以跟随一个可选的else语句，当布尔表达式为false时，else语句块将执行。|
|if…else if…else if…else…end	|if语句后面可以有一个(或多个)可选elseif ...和一个else语句，这对于测试各种条件非常有用。|
|嵌套if	|可以在一个if或elseif语句中使用另一个if或elseif语句。|
|switch	|switch语句用来测试一个变量与值列表的相等性。|
|嵌套switch	|可以在一个switch语句中使用一个switch语句。|

1. `if`语法    
在`MATLAB`中，`if`语句的语法是：   
```matlab
if <expression>
% statement(s) will execute if the boolean expression is true 
<statements>
end
```
如果表达式`(expression)`计算结果为`true`，则`if`语句中的代码块将被执行。如果表达式的计算结果为`false`，那么执行结束语句后的第一组代码。    
2. `switch`语法   
`switch`块有条件地执行来自多个选择的一组语句。每个选择由`case`语句指定。   
评估的`switch_expression`是一个标量或字符串或单元格数组。    
`switch`块测试每种情况，直到其中一种情况为真`(true)`。以下情况是真的：   
```
对于数字，eq(case_expression，switch_expression)。
对于字符串，strcmp(case_expression，switch_expression)。
对于对象，支持eq(case_expression，switch_expression)。
对于单元格数组case_expression至少有一个。
```
当情况`(case)为真`时，`MATLAB`会执行相应的语句，然后退出`switch`块。`otherwise`块是可选的，并且仅在`没有case为真`时执行。   
```
switch <switch_expression>
   case <case_expression>
      <statements>
   case <case_expression>
      <statements>
      ...
      ...
   otherwise
      <statements>
end
```
**注意**: `switch`这里没有`break`语句。     
