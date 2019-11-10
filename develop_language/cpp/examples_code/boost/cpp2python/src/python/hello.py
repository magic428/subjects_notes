#coding-utf-8
# import sys
# sys.path.insert(0, "/home/klm/work/gitwork/work_note/dev_tools/cpp/code/boost/cpp2python/src/python")
import hello_ext
import numpy as np

print (hello_ext.greet())

# Three different initialization
planet = hello_ext.World() # 调用默认构造函数，产生类对象
planet = hello_ext.World(5, 6)
planet2 = hello_ext.World("constructor")

print (planet.sum_s())
print (planet2.greet())
planet.set("howdy")   # 调用对象的方法
print (planet.greet()) # 调用对象的方法

var = hello_ext.Var("hello_var")
var.value = 3.14
# var.name = 'hello' # error, variable name is readonly
print (var.name)

num = hello_ext.Num()
num.value = 10
print (num.rovalue) #  result: 10

derive = hello_ext.factory()
hello_ext.d(derive)

base = hello_ext.VBase()
# 定义派生类，继承C++类
class Derived(hello_ext.VBase):
    def f(self):
        return 42

derived = Derived()

print (base.f())
print (derived.f())

x = hello_ext.X() # create a new object
print (x.f(1))  # default int type
print (x.f(2, np.float64(3)))
print (x.f(4, np.float64(5), chr(6)))  # chr(6) convert * to char 
print (x.f(7,8,9))


hello_ext.foo(1)
hello_ext.foo(1, chr(2))
hello_ext.foo(1, chr(2), 3)  # 3对应的C++为unsigned int
hello_ext.foo(1, chr(2), 3, np.float64(4))

c = hello_ext.george()
c.wack_em(1)
c.wack_em(1,2)
c.wack_em(1,2,chr(3))

dic1 = {"whatever":1}
hello_ext.test2(dic1)
print(dic1)
hello_ext.test(dic1)
print(dic1)

arr = np.array([1,2,3], dtype = np.float32)
print (arr.dtype)
print (arr)
hello_ext.add_arr_1(arr,1,3)
print (arr)