# jupyter 运行 python 虚拟环境 virtualenv

1) 切换进入虚拟环境   

```bash
source /path/to/virtualenv/bin/active 
```
2) 在虚拟环境内安装 ipykernel  

```bash
pip install ipykernel
```

3) 创建 kernel  

```bash
python -m ipykernel install --user --name=[kernel名]
```

例如： python -m ipykernel install --user --name=cv   

然后就 OK 了.   

