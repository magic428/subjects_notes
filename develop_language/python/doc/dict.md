# python 字典元素排序   
## dict 基本概念
dict 用花括号｛｝表示，然后按照 key: value, 写出来即可。最后一个 key: value 的逗号可以省略。    
①、dict 的查找速度快，无论dict有10个元素还是10万个元素，查找速度都一样。而list的查找速度随着元素增加而逐渐下降。dict的缺点是占用内存大，还会浪费很多内容，list正好相反，占用内存小，但是查找速度慢.   
②、dict 通过key 来查找 value ,因此key 不能重复，而value可重复.  
③、dict储存的"key:value"是无序的，即不可用索引号切片等。  
Python的基本类型如字符串、整数、浮点数都是不可变的，都可以作为 key。但是list是可变的，就不能作为 key。  
用 dict 表示“名字”-“成绩”的查找表如下：   
```python
d = {
'Adam': 95, #key : value
'Lisa': 85,
'Bart': 59
} 
```
我们把名字称为key，对应的成绩称为value，dict就是通过 key 来查找 value。使用 d[key] 的形式来查找对应的 value，这和 list 很像，不同之处是，list 必须使用索引返回对应的元素，而dict使用key.   

我们知道 `Python` 的内置 `dictionary` 数据类型是无序的，通过 `key` 来获取对应的 `value`。可是有时我们需要对 `dictionary` 中的 `item` 进行排序输出，可能根据 `key`，也可能根据 `value` 来排。  
`python` 内置 `sorted` 函数的帮助文档：   
```python  
sorted(...)
sorted(iterable, cmp=None, key=None, reverse=False) --> new sorted list
```
## 1. 将dict按照value进行排序
1.1 将 `key` 和 `value` 调换位置后排序   
```python
def sort_by_value(d):
	items = dict.items()
	backitems = [[ v[1], v[0] ] for v in items]
	backitems.sort()
	return [ backitems[i][1] for i in range(0,len(backitems))]
```
把 `item` 的 `key` 和 `value` 交换位置放入一个 `list` 中，再根据 `list` 每个元素的第一个值，即原来的 `value` 值排序.    
上面的代码可用一行语句搞定：   
```python
[ v for v in sorted(dict.values())]
```
1.2 用lambda表达式来排序，更灵活   
```python
sorted(dict.items(), lambda x, y: cmp(x[1], y[1]))   # (cmp前加“-”表示降序排序)
# 降序
sorted(dict.items(), lambda x, y: cmp(x[1], y[1]), reverse=True)
```
1.3 用 `sorted` 函数的 `key = `参数排序   
```python
# 按照key进行排序
print sorted(dict1.items(), key=lambda d: d[0])
# 按照value进行排序 
print sorted(dict1.items(), key=lambda d: d[1])
```
1.4 抽取 `values` 进行排序   
```python
def sortedDictValues(adict):
	values = adict.values()
	values.sort()
	return [value for value in values]
```
使用第4种方法, 速度快,但是它只是截取了 `values` 部分进行处理,并未对 `keys` 也做相应处理.故应根据根据需求选择不同的排序方法.    
## 2. 按照key值排序
1.1 items.sort()   
```python
def sortedDictValues1(adict):
	items = adict.items()
	items.sort()
	return [value for key, value in items]
```
1.2 keys.sort()     
```python
def sortedDictValues2(adict):
	keys = adict.keys()
	keys.sort()
	return [dict[key] for key in keys]
```
1.3 keys.sort(), 返回 map   
```python
def sortedDictValues3(adict):
	keys = adict.keys()
	keys.sort()
	return map(adict.get, keys)
```
上面的代码可用一行语句搞定：   
```python
[(k, dict[k]) for k in sorted(dict.keys())]
```
把 `dictionary` 中的元素分离出来放到一个 `list` 中，对 `list` 排序，因为 `list` 有 `sort` 方法. 从而间接实现对 `dictionary` 的排序。可以对 `key`， `value` 或者 `item` “元素”分离。   
## 3. 判断 key 是否在 dict 中   
注意: 通过 key 访问 dict 的value，只要 key 存在，dict就返回对应的value。如果key不存在，会直接报错：KeyError。要避免 KeyError 发生，有两个办法：    
3.1 判断 key 是否存在，用 `in` 操作符    
```python
if 'Paul' in d:
	print d['Paul']
```
如果 `Paul` 不存在，`if语句`判断为 `False`，自然不会执行 `print d['Paul']`，从而避免了错误。
3.2 使用 `dict` 本身提供的一个 `get` 方法   
`dict.get(key, default=None)`，在 `key` 不存在的时候，返回默认值 `None`.   
```python
print d.get('Bart')
print d.get('Paul')
None
```
## 4. dict 操作  
4.1 更新 `dict`   
`dict` 是可变的，可以随时往 `dict` 中添加新的 `key-value` 。比如已有`dict`：
```python
d = {
'Adam': 95,
'Zart': 59
'Lisa': 85,
}
```
要把新同学'Paul'的成绩 72 加进去，用赋值语句：
```python
d['Paul'] = 72 
```
4.2 删除 dict 元素   
可使用 pop 方法 dict.pop(key[,default])，通过 key 值删除 dict 内元素，并返回被删除 key 对应的 value。若 key 不存在，且 default 值未设置，则返回 KeyError 异常
```python
>>> a
{1: 'abc', 2: 'efg', 3: 'hij'}
>>> a.pop(1)
'abc'
>>> a
{2: 'efg', 3: 'hij'}
>>> a.pop(1, False)
False
```
4.3 清空 `dict`   
可使用 `clear` 方法 `dict.clear()` 清空 `dict`.   
4.4 遍历/迭代 `dict`  
(1) for 循环遍历   
由于 `dict` 也是一个集合，所以遍历 dict 和遍历 list 类似，都可以通过 for 循环实现。   
```python
for key in d:
	print key,'-',d[key]
```
(2) values() 方法   
values()方法：返回dict 的value值, 把 dict 转换成了包含 value 的list.     
(3) itervalues() 方法
替代 values() 方法，迭代效果完全一样。不同的是 itervalues() 方法不会转换，它会在迭代过程中依次从 dict 中取出 value，所以 itervalues() 方法和 values() 方法相比节省了生成 list 所需的内存。   
(4) items() 方法   
items() 方法把 dict 对象转换成了包含 tuple 的 list，对这个 list 进行迭代，可以同时获得 key 和 value.   
```python
for key, value in d.items(): 
	print key, ':', value
```
和 values() 有一个 itervalues() 类似， items() 也有一个对应的 iteritems()，iteritems() 不把dict转换成list，而是在迭代过程中不断给出 tuple，所以， iteritems() 不占用额外的内存。    
## 5. **小结**
为什么dict查找速度这么快？因为dict的实现原理和查字典是一样的。假设字典包含了1万个汉字，我们要查某一个字，一个办法是把字典从第一页往后翻，直到找到我们想要的字为止，这种方法就是在list中查找元素的方法，list越大，查找越慢。   
第二种方法是先在字典的索引表里（比如部首表）查这个字对应的页码，然后直接翻到该页，找到这个字。无论找哪个字，这种查找速度都非常快，不会随着字典大小的增加而变慢。   
dict就是第二种实现方式，给定一个名字，比如'Michael'，dict在内部就可以直接计算出Michael对应的存放成绩的“页码”，也就是95这个数字存放的内存地址，直接取出来，所以速度非常快。   
你可以猜到，这种key-value存储方式，在放进去的时候，必须根据key算出value的存放位置，这样，取的时候才能根据key直接拿到value。   
dict可以用在需要高速查找的很多地方，在Python代码中几乎无处不在，正确使用dict非常重要，需要牢记的第一条就是dict的key必须是不可变对象。   
