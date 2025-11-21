# ros教程

## 事前准备

[Jetson Orin NX 亚博教程](https://www.yahboom.com/study/Orin-NX-SUPER)  提取码：**ntgy**

为了环境的隔离和便捷：拟采用docker进行ros整体拉取

# 电脑远程连接jetson

**用户名**：xavier   **密码**：123456

查询jetson的ip：

```bash
ifconfig
```

找到输出inet下面的（最后一段）：192.168.xxxx

这里采用windterm远程ssh连接

查询jetson版本：

```bash
lsb_release -a
```

该系统版本：Ubuntu 20.04 LTS（Focal Fossa）

# docker指令

[ROS镜像版本](https://github.com/sloretz/ros_oci_images?utm_source=chatgpt.com)

查询docker安装版本：

```bash
docker --version
```

查看是否在运行：

```bash
systemctl status docker
```

查看现有（所有包括停止）容器（去掉-a就是查询运行的容器）

```bash
#查看所有包括停止
sudo docker ps -a
#查看运行
sudo docker ps 
```

查看镜像:

```bash
sudo docker images
```

删除单个镜像（如果镜像被容器占用就要先删掉容器）

```bash
docker rmi <IMAGE_ID>
```

删除容器：

```bash
#删除停止的容器
sudo docker rm <CONTAINER_ID>

#强制删除容器
sudo docker rm -f <CONTAINER_ID>
```

启动容器：（不能用这个方式开启两个容器）

```bash
#ai后面容器名(带输入、输出并进入容器)
sudo docker start -ai ros2
```

进入**运行中容器**（伪终端）：

```bash
sudo docker exec -it <container> bash
```

停止容器：

```bash
sudo docker stop ros2
```

退出容器：

```bash
exit
```

# 新建容器

运行宿主机允许容器访问 X11           

```bash
xhost +local:docker
```

新建容器：(windterm中逐行发送)

```bash
sudo docker run -it \
  --name ros2 \                 # 容器名称：ros2（方便后续 start/stop）
  --env DISPLAY=$DISPLAY \      # 把宿主机的 DISPLAY 环境变量传入容器，用于 X11 图形界面显示
  --env QT_X11_NO_MITSHM=1 \    # 解决 Qt 应用与 X11 共享内存冲突的问题（常用于 ROS、RViz）
  -v /tmp/.X11-unix:/tmp/.X11-unix \  # 把宿主机的 X11 socket 挂载进去，让容器能访问 X server
  -v /home/xavier/hsc/ros2/:/root/ros2 \  # 挂载宿主机目录到容器 /root/ros2（共享代码、数据）
  --net=host \                  # 让容器使用宿主机网络（ROS 多节点通信推荐使用）
  ghcr.io/sloretz/ros:humble-desktop \   # 使用 ROS2 Humble 桌面版镜像
  bash                          # 启动容器后直接进入 bash 终端

```

# ros指令

查询ros版本：

```bash
printenv | grep ROS
```

# 代理

能行指令：[Clash在Linux下的使用 | 云端笔记](https://leux.net/doc/Clash%E5%9C%A8Linux%E4%B8%8B%E7%9A%84%E4%BD%BF%E7%94%A8.html)

订阅链接：+链接（要加引号）——遇到失败重新订阅一下

```bash
wget -O /home/xavier/mihomo/config.yaml 
```

运行：

```bash
sudo /usr/local/bin/clash -d /home/xavier/mihomo/
```

查看进程：

```bash
ps aux | grep mihomo
```

```bash
ps aux | grep 7890
```

终止进行：

```bash
kill PID
```

# windows远程显示

## windterm远程连接+图形化

<u>注意，在打开容器的时候，有时windterm输入xavier密码后卡住，ctr+c强制退出就可</u>

windterm远程连接[WindTerm - X11 转发](https://kingtoolbox.github.io/2020/07/21/x11_forwarding/?utm_source=chatgpt.com#X11-Forwarding)

1️⃣ Windows下载启动 VcXsrv 
2️⃣ WindTerm：开启 X11 Forwarding（外部显示）  
3️⃣ WindTerm SSH 登录到 Jetson  
4️⃣ WindTerm 里执行：

```bash
#测试wind输出localhost:10.0
echo $DISPLAY
#成功后会有一个小眼睛
xeyes
```

windows进行放行（设置display环境变量）。查询windows（终端）——ipconfigs——ipv4:192.168.31.163，windterm设置ipv4+端口0（一般）

```bash
export DISPLAY=192.168.31.163:0 #后面的ip根据查询填
xhost +
```

输出access control disabled, clients can connect from any host成功，然后进行配置宿主机允许容器访问x11

```bash
xhost +local:docker
```

新建容器，然后进入测试X11在容器里面是否成功

```bash
apt update
apt install -y x11-apps
xeyes
```

有眼睛在动就行（哈哈哈哈哈）

**加载 ROS2 的环境变量**，让当前 shell 能识别 ROS2 的命令和库。（当出现“<mark>bash: ros2: command not found</mark>”）⬇

```bash
source /opt/ros/humble/setup.bash
```

一个终端打开容器（sudo docker start -ai 容器名）测试小海龟：（但是不能通过键盘来移动小海龟）

```bash
ros2 run turtlesim turtlesim_node #打开界面
```

另一个终端打开容器（sudo docker exec -it 容器名 bash），要先**加载ros2环境变量**，方向键控制小海龟：（注意操作，让小海龟界面保持在最上面，然后输入下面该指令，有时候需要再点击一下该窗口）

```bash
ros2 run turtlesim turtle_teleop_key
```

## 采用VNC（也是靠VcXsrv转发显示）——优点是能显示jetson那边的界面

```
VNC连接就是设置好远程连接的ip，通过ifconfig查询ip，然后加上“：端口号”（默认下面的5901和5900，可以进行设置）
```

[VNC Viewer安装教程（保姆级安装）-CSDN博客](https://blog.csdn.net/yushaoyyds/article/details/133926519)

jetson安装VNC

```bash
sudo apt install -y tigervnc-standalone-server tigervnc-common
```

验证是否安装成功——输出/usr/bin/vncserver

```bash
which vncserver
```

是否能设置密码：would like....——选择n

```bash
vncpasswd
```

安装XFCE桌面来支持VNC：——安装过程光标选择“lightdm”再enter

```bash
sudo apt install xfce4 xfce4-goodies -y
```

生成 xstartup（每个用户一次）：

```bash
vncserver -kill :1
rm -rf ~/.vnc/xstartup
```

创建新的 xstartup:（vim编辑器）

![](./images/0edf0c91f2c89fcf2f2bf028df3b15b5498a518a.png)

```bash
vim ~/.vnc/xstartup
```

赋予可执行权限

```bash
chmod +x ~/.vnc/xstartup
```

启动：

```bash
vncserver :1 -geometry 1920x1080 -depth 24
```

停止：

```bash
vncserver -kill :1
```

确认VNC监听端口对不对：

```bash
sudo netstat -tlnp | grep 5901
```

正确输出：tcp        0      0 0.0.0.0:5901      0.0.0.0:*      LISTEN      <pid>/Xtigervnc

发现监听端口不对：

```bash
vim ~/.vnc/config
```

写入内容:localhost=0

## 强制启动：（以上指令已经配好）

```bash
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no
```

# 强制 VNC 显示 Jetson 本机桌面（这个只是复制jetson桌面）

```bash
sudo apt-get install x11vnc -y #安装
```

```bash
x11vnc -storepasswd #设置密码
```

```bash
x11vnc -display :0 -usepw #共享真实桌面5900
```

# 问题——主要是display显示环境变量的问题，进入容器测试默认为空，进行设置

首先测试环境变量问题：

```bash
echo $DISPLAY
```

DISPLAY环境变量问题导致显示的问题：

在 Linux（X11 系统）中，**所有 GUI 程序都必须通过 DISPLAY 才能知道在哪里显示窗口**。

例如：

| DISPLAY 值        | 含义             |
| ---------------- | -------------- |
| `:0`             | 本地物理显示器（你的桌面）  |
| `:1`             | 另一个本地 X 会话     |
| `localhost:10.0` | SSH 转发的远程显示    |
| 空的（没有值）          | GUI 完全无法显示，会报错 |

设置环境变量问题：

```bash
export DISPLAY=:0
```

```bash
export DISPLAY=localhost:10.0#ssh自动寻找
```
