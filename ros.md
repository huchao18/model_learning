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
docker rm <CONTAINER_ID>

#强制删除容器
docker rm -f <CONTAINER_ID>
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
  --name ros2 \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/xavier/hsc/ros2/:/root/ros2 \
  --net=host \
  ghcr.io/sloretz/ros:humble-desktop bash
```

# ros指令

查询ros版本：

```bash
printenv | grep ROS
```

# 代理

能行指令：[Clash在Linux下的使用 | 云端笔记](https://leux.net/doc/Clash%E5%9C%A8Linux%E4%B8%8B%E7%9A%84%E4%BD%BF%E7%94%A8.html)

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



windows进行放行

查询windows（终端）——ipconfigs——ipv4:192.168.31.163

windterm设置ipv4+端口0（一般）

```bash
export DISPLAY=192.168.31.163:0
xhost +
```

输出access control disabled, clients can connect from any host成功

然后进行配置宿主机允许容器访问x11

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



****加载 ROS2 的环境变量**，让当前 shell 能识别 ROS2 的命令和库。

```bash
source /opt/ros/humble/setup.bash
```

测试小海龟：（但是不能通过键盘来移动小海龟）

```bash
ros2 run turtlesim turtlesim_node #打开界面
```

方向键控制小海龟：

```bash
ros2 run turtlesim turtle_teleop_key
```



## 采用VNC

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

![](./images/2025-11-20-15-35-24-image.png)

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

强制启动：

```bash
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no
```
