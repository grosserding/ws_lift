# lift_comm 梯控通信模块

## 项目简介

梯控节点，接收导航模块的 ROS Service 请求，通过 RS485 (SMEC协议) 与电梯接口装置通信。

通信架构：

```
导航模块 ← ROS Service → lift_controller 节点 ← RS485/串口 → 电梯接口板
```

## 环境依赖

- ROS Noetic
- Python >= 3.10
- [uv](https://docs.astral.sh/uv/) 包管理器

## 新电脑首次部署

### 1. 安装 uv（如果没有）

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 2. 安装 Python 依赖

```bash
cd ~/hwm/EleControl/lift_comm
uv sync
```

> `uv sync` 会根据 `pyproject.toml` 自动创建虚拟环境并安装所有依赖（pyserial、empy==3.3.4、catkin_pkg、rospkg、netifaces）。
> 注意：empy 必须用 3.x 版本，4.x 与 ROS Noetic 不兼容，已在 pyproject.toml 中锁定。

### 3. 编译

```bash
source /opt/ros/noetic/setup.bash    # 加载 ROS 系统环境
rm -rf build devel                    # 清理旧编译缓存（路径变化时必须执行）
catkin_make
source devel/setup.bash               # 加载本工作空间生成的 srv 等文件
```

> **为什么每次新终端都要 source？**
> ROS 需要知道本工作空间生成的 srv 消息类型（LiftCall 等），不 source 的话 rosrun、roslaunch、rqt 都找不到这些自定义消息。

### 4. 自动 source（推荐）

```bash
echo "source ~/hwm/EleControl/lift_comm/devel/setup.bash" >> ~/.bashrc
```

> 写入后每次开新终端自动生效，不用手动 source。换了电脑路径不同需要重新执行。

### 5. 串口权限

```bash
sudo usermod -aG dialout $USER
```

> 将当前用户加入 dialout 组，否则访问 /dev/ttyUSB* 或 /dev/ttyACM* 会报 Permission denied。
> 执行后需要**注销重新登录**才生效。临时方案：`sudo chmod 666 /dev/ttyACM0`

## 修改代码后重新编译

以下情况需要重新 `catkin_make`：

| 修改内容 | 是否需要重新编译 |
|---------|:------------:|
| `srv/*.srv` 文件 | 是 |
| `CMakeLists.txt` | 是 |
| `package.xml` | 是 |
| Python 源码（`*.py`） | 否，直接生效 |

```bash
catkin_make
source devel/setup.bash
```

> Python 是解释型语言，改 .py 不需要编译。但 srv 文件会被 catkin 生成 Python/C++ 代码，所以改了 srv 必须重新 catkin_make。

## 启动方式

### 方式一：launch 文件（推荐）

```bash
roslaunch lift_comm lift_controller.launch
```

参数在 `src/lift_comm/launch/lift_controller.launch` 中修改：

```xml
<param name="port"     value="/dev/ttyACM0" />  <!-- 串口设备，ttyUSB0 或 ttyACM0 -->
<param name="baudrate" value="9600" />            <!-- 波特率 -->
<param name="robot_id" value="1" />               <!-- 机器人ID（0~255） -->
<param name="bnk"      value="0" />               <!-- 电梯群号，1号群=0 -->
<param name="nod"      value="0" />               <!-- 群内电梯号，1号梯=0 -->
```

### 方式二：rosrun

```bash
rosrun lift_comm lift_controller.py
```

可通过命令行传参覆盖默认值：

```bash
rosrun lift_comm lift_controller.py _port:=/dev/ttyACM0 _robot_id:=1
```

## 测试

### 测试 ROS Service 通信（不需要串口硬件）

```bash
# 终端1：启动测试服务端
rosrun lift_comm lift_servertest.py

# 终端2：手动调用服务
rosservice call /lift/call "target_floor: 3"
rosservice call /lift/state_inquiry "state_inquiry: true"
rosservice call /lift/hodor "hodor: true"
```

### 测试串口通信（需要串口硬件）

```bash
rosrun lift_comm elevator_control.py
```

> 直接运行底层模块，循环查询电梯状态并打印。需要串口已连接电梯接口板。

## ROS Service 说明

| 服务名 | srv 类型 | 调用方 | 功能 |
|--------|---------|-------|------|
| `/lift/call` | LiftCall | 导航 | 呼梯请求，登记目标楼层 |
| `/lift/state_inquiry` | StateInquiry | 导航 | 查询电梯当前状态（楼层、门、方向） |
| `/lift/hodor` | Hodor | 导航 | 按开门键，保持门打开15秒 |

## 串口通信参数

- 波特率：9600bps
- 数据位：8
- 校验位：偶校验 (Even)
- 停止位：1
- 协议：SMEC（详见 `机器人物联电梯接口装置通信协议.docx`）

## 文件结构

```
src/lift_comm/
├── src/
│   ├── elevator_control.py    # RS485 通信底层（SMEC协议）
│   ├── lift_controller.py     # 梯控主节点（ROS Service 服务端）
│   ├── lift_servertest.py     # 测试用服务端（不需要串口）
│   └── nav_clienttest.py      # 测试用客户端
├── srv/
│   ├── LiftCall.srv           # 呼梯 srv 定义
│   ├── StateInquiry.srv       # 状态查询 srv 定义
│   └── Hodor.srv              # 开门 srv 定义
├── launch/
│   └── lift_controller.launch # 启动文件（含参数配置）
├── CMakeLists.txt
└── package.xml
```

## G771 DTU 配置（USB转RS485透传）

使用 USR-G771 时，串口设备为 `/dev/ttyACM0`（数据口）。

G771 串口参数需配置为 9600 8E1，配置方式参考 [有人物联网官网](https://www.usr.cn/Product/296.html)。
