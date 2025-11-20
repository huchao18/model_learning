## 虚拟环境

查询所有虚拟环境：

```py
conda env list
```

创建虚拟环境：

```py
conda create -n 环境名 python=版本号
```

查询当前的源：

```py
conda config --show
#然后找到输出里面channels
```

激活虚拟环境：

```py
conda activate 环境名
```

退出虚拟环境：

```py
deactivate
```

# Docker

查询是否安装（版本）：

```py
docker --version
```
