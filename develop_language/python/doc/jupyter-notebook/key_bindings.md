# Jupyter Notebook 快捷键   

`Jupyter Notebook` 支持两种不同的键盘输入模式, 编辑模式和命令模式.  

- Edit Mode:　编辑模式下可以在 cell 中输入代码和文本, 此时 cell 边界为绿色;     
- Command Mode: 命令模式下将键盘绑定在 notebook 层的命令上, 此时 cell 边界为灰色且左侧有一个蓝色边;   

## Command Mode ( 按下 Esc 键键入命令模式)    

|快捷键|功能描述|快捷键|功能描述|
|:------|:-----|:-----------|:-----|
|F| find and replace|Ctrl-Shift-F / Ctrl-Shift-P / P|打开命令面板 |
|Enter| 进入编辑模式 |Shift-Enter| 运行当前 cell, 并选中下一个 cell |    
|Ctrl-Enter| 运行当前 cell|Alt-Enter| 运行当前 cell, 并新建下一个 cell |
|Y| 将 cell 类型改为 code |M| 将 cell 类型改为 markdown |
|R| 将 cell 类型改为 raw | 1| 将 cell 改为一级标题|
|2| 将 cell 改为二级标题|3| 将 cell 改为三级标题|
|4| 将 cell 改为四级标题|5| 将 cell 改为五级标题|
|6| 将 cell 改为六级标题|K / Up| 选中上一个 cell |
|J / Down| 选中下一个 cell|Shift-K / Shift-Up| 连续选中上一个 cell |
|Shift-J / Shift-Down| 连续选中下一个 cell |A| 在当前 cell 上方插入一个 cell |
|B| 在当前 cell 下方插入一个 cell |X| 剪切选中的 cell　|
|C| 拷贝选中的 cells　｜Shift-V| 在当前 cell 上面粘贴 cells | 
|V| 在当前 cell 下方粘贴 cells |Z| 撤销删除 cell(s) 操作 |
|D,D(按两下 D)| 删除选中的 cell(s) |Shift-M| 合并选中的 cell(s), 如果只有当前 cell 被选中, 那么就将当前 cell 和下一个 cell 合并|S / Ctrl-S| 保存 Checkpoint |
|L| 切换当前代码 cell 的行号开关 | Shift-L | 切换所有代码 cell 的行号开关 | 
|O| 切换选中 cell(s) 的输出开关 |Shift-O| 切换选中 cell(s) 的输出滚动开关 |
|H| 显示键盘快捷键帮助文档 |I,I | 中断 kernel 运行 |  
|0,0 | 重启 kernel |Esc / Q | close the pager|
|Shift-Space | 向上滚动笔记本 |Space | 向下滚动笔记本|

## Edit Mode ( 按下 Enter 键键入编辑模式)    

|快捷键|功能描述|快捷键|功能描述|
|:----|:-----|:-----|:-----|
|Tab| 代码补全或缩进|Shift-Tab|显示代码提示信息|
|Ctrl-]| 缩进 | Ctrl-[ | 取消缩进 |
|Ctrl-A| 全选 | Ctrl-Z | 撤销 |
|Ctrl-/| 注释 | Ctrl-D | 删除整行 |
|Ctrl-U| 取消选中| Insert | 切换文本覆盖开关 |
|Ctrl-Home | 回到起始 cell | Ctrl-End| 回到最后一个 cell |  
|Ctrl-Backspace| 删除单词的前半部分 |Ctrl-Delete | 删除单词的后半部分 |
|Ctrl-Y| redo | Alt-U | redo selection |
|Ctrl-Shift-Minus | 在光标位置切分 cell |Esc / Ctrl-M| 进入命令模式 |
