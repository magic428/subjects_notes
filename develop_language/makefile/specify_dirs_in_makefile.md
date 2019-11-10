# Makefile 指定文件的生成目录  

> 原文地址: https://blog.csdn.net/javababy3/article/details/77918228

[上一篇博客](https://blog.csdn.net/javababy3/article/details/77875816) 虽然简单实现了自动处理依赖关系，但是生成的各种临时文件都混在一起，太乱了。  

假定我们的源文件放在 src 目录，头文件放在 inc 目录， .o 文件放在 obj 目录， .d 文件放在 dmk 目录，Makefile 和上述 4 个目录为同一级别。  

则定义如下变量：  

```makefile
D_SRC = src
D_INC = -I./inc
D_OBJ = obj
D_MK  = dmk
```

## 1. 自动遍历 src 目录下的所有 .c 文件

```makefile
# wildcard表示把 $(D_SRC) 目录下的 .c 文件遍历出来
SRC_C   = $(wildcard $(D_SRC)/*.c)
 
# foreach 表示遍历 $(D_SRC) 的所有子目录同时把子目录下的 .c 文件遍历出来
SRC_C   = $(foreach dir, $(D_SRC), $(wildcard $(dir)/*.c))
```

假设 src 目录中有 a.c,b.c，则 $(SRC_C) 表示 src/a.c src/b.c  

## 2. 生成所有的 .o 文件依赖和 .d 文件集合 

```makefile
OBJ_C   = $(addprefix $(D_OBJ)/,$(patsubst %.c,%.o,$(notdir $(SRC_C))))
SRC_MK  = $(addprefix $(D_MK)/, $(patsubst %.c,%.d,$(notdir $(SRC_C))))
```

notdir 表示去除目录，则 $(notdir $(SRC_C)) 表示 a.c b.c;   

patsubst 表示把 $(notdir $(SRC_C)) 中的.c替换成.o, 即 a.o b.o;   

addprefix 表示增加前缀 $(D_OBJ)/, 则OBJ_C 变量表示为 obj/a.o obj/b.o;   

## 3. 完整的 Makefile     

```makefile
D_SRC = src
D_INC = -I./inc
D_OBJ = obj
D_MK  = dmk
TATGET = hello  
 
SRC_C   = $(foreach dir, $(D_SRC), $(wildcard $(dir)/*.c))
OBJ_C   = $(addprefix $(D_OBJ)/,$(patsubst %.c,%.o,$(notdir $(SRC_C))))
SRC_MK  = $(addprefix $(D_MK)/, $(patsubst %.c,%.d,$(notdir $(SRC_C))))
 
$(TATGET):$(OBJ_C)
	gcc -o $@ $^
 
$(D_OBJ)/%.o:$(D_SRC)/%.c
	gcc -c -Wall $(D_INC) $< -o $@
 
$(D_MK)/%.d:$(D_SRC)/%.c
	@set -e; rm -f $@; \
	$(CC) -MM $(D_INC) $< > $@.$$$-$; \
	sed 's,\($*\)\.o[ :]*,$(D_OBJ)/\1.o $@ : ,g' < $@.$$$-$ > $@; \
	rm -f $@.$$$-$
 
include $(SRC_MK)
 
.PHONY: clean
clean:
	rm -f $(D_OBJ)/* $(TATGET) $(D_MK)/*
```

多出来的几行依赖关系其实就是一个字符串匹配的模式， clean 就不需要解释了。

## 4. vpath 自动变量的使用

vpath 第一个参数是查找的类型，第二个是查找的目录;  

```makefile
vpath %.c src # 查找依赖时如果遇到 %.c, 则自动到 src 目录下寻找
D_SRC = src
D_INC = -I./inc
D_OBJ = obj
D_MK  = dmk
TATGET = hello
 
SRC_C   = $(foreach dir, $(D_SRC), $(wildcard $(dir)/*.c))
OBJ_C   = $(addprefix $(D_OBJ)/,$(patsubst %.c,%.o,$(notdir $(SRC_C))))
SRC_MK  = $(addprefix $(D_MK)/, $(patsubst %.c,%.d,$(notdir $(SRC_C))))
 
$(TATGET):$(OBJ_C)
	gcc -o $@ $^
 
$(D_OBJ)/%.o:%.c  #自动去src目录下找.c结尾的文件。
	gcc -c -Wall $(D_INC) $< -o $@
 
$(D_MK)/%.d:%.c
	@set -e; rm -f $@; \
	$(CC) -MM $(D_INC) $< > $@.$$$-$; \
	sed 's,\($*\)\.o[ :]*,$(D_OBJ)/\1.o $@ : ,g' < $@.$$$-$ > $@; \
	rm -f $@.$$$-$
 
include $(SRC_MK)
 
.PHONY: clean
clean:
	rm -f $(D_OBJ)/* $(TATGET) $(D_MK)/*
```

查找多个目录用下面的写法:   

    vpath %.c src:src1:src2

大写 VPATH 只能指定依赖的查找目录，不能指定类型，所以也可以写成:   

```makefile
VPATH = src
D_SRC = src
D_INC = -I./inc
D_OBJ = obj
D_MK  = dmk
TATGET = hello
 
SRC_C   = $(foreach dir, $(D_SRC), $(wildcard $(dir)/*.c))
OBJ_C   = $(addprefix $(D_OBJ)/,$(patsubst %.c,%.o,$(notdir $(SRC_C))))
SRC_MK  = $(addprefix $(D_MK)/, $(patsubst %.c,%.d,$(notdir $(SRC_C))))
 
$(TATGET):$(OBJ_C)
	gcc -o $@ $^
 
$(D_OBJ)/%.o:%.c
	gcc -c -Wall $(D_INC) $< -o $@
 
$(D_MK)/%.d:%.c  #自动去VPATH指定的目录查找，指定多个路径 写成VPATH = src:src1:src2
	@set -e; rm -f $@; \
	$(CC) -MM $(D_INC) $< > $@.$$$-$; \
	sed 's,\($*\)\.o[ :]*,$(D_OBJ)/\1.o $@ : ,g' < $@.$$$-$ > $@; \
	rm -f $@.$$$-$
 
include $(SRC_MK)
 
.PHONY: clean
clean:
	rm -f $(D_OBJ)/* $(TATGET) $(D_MK)/*
```

> 最后 $@.$$$-$ 中的短横线要去掉，要不然排版有问题。  

换行记得用TAB键。    