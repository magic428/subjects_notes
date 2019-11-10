# 在 CMakeLists.txt 文件之间共享变量  

> 关键字: 共享变量.   

场景:  

项目目录结构如下图, 希望在 cmake 创建 Makefile 或 VS 工程时,  

- PROJECT_SRC_DIR/ 文件夹下的 CMakeLists.txt 中定义了 LIVE_VAR1 变量,  3rdparty/ 下的 CMakeLists.txt 需要共享 LIVE_VAR1 变量;   
- src/ 文件夹下的 CMakeLists.txt 中定义了 LIVE_VAR2 变量, 3rdparty/ 下的 CMakeLists.txt 需要共享 LIVE_VAR2 变量;   

```
|--- PROJECT_SRC_DIR/
            |--- src/
                  |- CMakeLists.txt
            |--- 3rdparty/
                  |- CMakeLists.txt
            CMakeLists.txt
|--- build/
```

## 方法 1 - set()  

一般用 set 命令定义的变量能从父目录传递到子目录, 但 src 和 3rdparty 在同级目录, 所以用 set 定义的变量无法共享, 要用 `set(variable value CACHE INTERNAL docstring)`, 这种方式定义的变量会把变量加入到 CMakeCache.txt, 然后各级目录共享会访问到这个变量.  

比如:  

在 src 下的 CMakeLists.txt 中定义一个变量 LIVE_VAR1:  

```cmake
#"variable sharing test" 是对变量的描述说明, 不能省略
set(LIVE_VAR1 "var 1 for test" CACHE INTERNAL "variable sharing test" )
```

在 3rdparty 下的 CMakeLists.txt 中读取这个变量:  

```cmake
MESSAGE(STATUS "ICD_LIBRARY :${ICD_LIBRARY}")
```

每次运行 cmake 都会更新这个变量, 你会在 CMakeCache.txt 中找到这个变量:  

```txt
//variable sharing test
LIVE_VAR1:INTERNAL=var 1 for test
```

## 方法 2 - set_property()/get_property()  

使用 set_property() 实现共享变量的方法, 不会将变量写入 CMakeCache.txt, 应该是内存中实现的.   

当用 set_property 定义的 property 时, 如果第一个指定作用域 (scope) 的参数设为 GLOBAL, 这个 property 在 cmake 运行期间作用域就是全局的. 然后其他目录下的CMakeLists.txt 可以用 get_property 来读取这个 property.  

比如:   

在 src 下的 CMakeLists.txt 中定义一个名为 LIVE_VAR2 的 global property.  
set(LIVE_VAR1 "var 1 for test" CACHE INTERNAL "variable sharing test" )

```cmake
set_property(GLOBAL PROPERTY LIVE_VAR2 "var 2 for test" )
```

在 3rdparty 下的 CMakeLists.txt 中读取这个 property:  

```cmake
// 先调用 get_property 将这个属性读取到变量 LIVE_VAR2_GET 中: 
get_property(LIVE_VAR2  GLOBAL PROPERTY LIVE_VAR2 ) 
// 显示 LIVE_VAR2  
MESSAGE(STATUS "LIVE_VAR2: ${LIVE_VAR2 }")
```

上面的例子可以看出这种方式相比方法一在使用变量时多了一步, 先要将先调用get_property 将这个 property 读取到一个变量中才能使用.  

## 3. 总结 

两种方法相比, 方法一的使用更加便利, 但方法一将变量保存在 CMakeCache.txt, 需要读写 CMakeCache.txt 文件, 目前没有发现别的副作用.  


## 参考资料  

[1] cmake:在各级子项目(目录)之间共享变量: https://blog.csdn.net/10km/article/details/50508184  
