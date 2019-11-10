# `Matlab`循环   
一般来说，语句是按顺序执行的。首先执行函数中的第一个语句，然后执行第二个语句，依此类推。编程语言提供了允许更复杂的执行路径的各种控制结构。   
循环语句允许多次执行一个语句或一组语句，以下是大多数编程语言中循环语句的一般形式:   
![](../../pictures/loop.png)  

## 循环类型
`MATLAB`提供以下类型的循环来处理循环需求。

|循环类型	|描述|
|----|----|
|while循环|	在给定条件为真时，重复一个语句或一组语句。它在执行循环体之前测试状态。|
|for循环	|多次执行一系列语句，并缩写管理循环变量的代码。|
|嵌套循环	|在任何循环中使用另外一个或多个循环。|

1. `while`语法    
```matlab
while <expression>
   <statements>
end
```
只要表达式`(expression)`为`true`，`while`循环将重复执行程序语句`(statements)`。
当结果为非空并且包含所有非零元素(逻辑或实数)时，表达式`(expression)`为`true`。否则，表达式`(expression)`为`false`。    
>eg   
```matlab
a = 10;
% while loop execution 
while( a < 20 )
  fprintf('value of a: %d\n', a);
  a = a + 1;
end
```
2. `for`语法   
`for`循环是一种重复控制结构，可以让您有效地编写一个需要执行特定次数的循环。
```matlab
for index = values
   <program statements>
            ...
end
```
其中值`(values)`具有以下格式：    

|值格式|描述|
|----|----|
|initval:endval|	index变量从initval到endval每次递增1，并重复程序语句的执行，直到index大于endval。|
|initval:step:endval|	通过每次迭代值步长(step)增加索引(index)的值，或者当step为负时递减。|
|valArray|	在每个迭代中从数组valArray的后续列创建列向量索引。 例如，在第一次迭代中，index = valArray(:，1)。 循环最多执行n次，其中n是由numel(valArray，1，:)给出的valArray的列数。valArray可以是任何MATLAB数据类型，包括字符串，单元格数组或结构体。|

>eg.1
```matlab
for a = 10:20 
   fprintf('value of a: %d\n', a);
end
```
>eg.2
```matlab
for a = 1.0: -0.1: 0.0
   disp(a)
end
```
>eg.3
```matlab
for a = [24,18,17,23,28]
   disp(a)
end
```
## 循环控制语句
循环控制语句从其正常顺序更改执行。当执行离开范围时，在该范围内创建的所有自动对象都将被销毁。`MATLAB`支持以下控制语句。   

|控制语句|描述|
|----|----|
|break语句	|终止循环语句，并将执行转移到循环之后的语句。|
|continue语句|导致循环跳过主体的剩余部分，并在重申之前立即重新测试其状态。|

**说明**： 两个语句的使用和`C`语言的语法相同。   