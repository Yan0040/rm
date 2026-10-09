# nav_lecture4小作业：Qos_debugger
这道题的目标就是让你快速上手、理解什么是ros。抛开复杂的概念，ros本质上完成的任务就是便利的进程间通信。比如，我有两个进程，一个进程发布雷达数据，另一个进程接收。使用ros就可以方便的完成通讯。你可以搜索以下，发送信息有哪些类型，分别有何特点。特别注意，不同的信息传输方式有不同的质量要求。你不会允许送的外卖没到你手上，但是一个电话过来，也许漏接了也无所谓，可能只是个诈骗。ros2也是这样。重点关注这一点会对这道题有所帮助

> 环境要求：ROS2 Humble

## 包结构

```
src/
  nav_hw_interfaces/     # 接口包：只放 .msg，无业务代码
    msg/SensorData.msg   # 可以打开.msg文件查看接口详细内容
  qos_debugger/          # 业务节点包
    src/qos_debugger_pub.cpp   # 发布 /SensorData
    src/qos_debugger_sub.cpp   # 订阅 /SensorData
```

## 编译

```bash
cd lecture4/homework
colcon build 
source install/setup.bash
```


## 任务一：实现pub和sub的通信

```bash
ros2 -h     //有忘记的命令就输入-h去查询用法
```

**现象**：启动pub和sub节点后sub节点订阅不到任何消息
提示：如果两个节点不能通过话题通信，我们应该如何区查看话题的详细信息（有没有相关的命令）
任务一仅修复qos_debugger_pub.cpp的一处或几处代码即可完成

任务一要写进作业里的内容可以是下面这一套。

**现象原因：**  
发布端默认 QoS 是 `best_effort`，订阅端默认是 `reliable`。ROS 2 要求订阅端 Reliable 时，发布端也必须 Reliable，否则匹配失败，sub 收不到任何消息。

**查看话题详细信息的命令：**

```bash
ros2 topic list
ros2 topic info /sensor_data -v
ros2 topic echo /sensor_data
```

`ros2 topic list` 确认话题在不在。  
`ros2 topic info /sensor_data -v` 能看到 Publisher / Subscription 的 Reliability、Durability、Depth。一边 `BEST_EFFORT`、一边 `RELIABLE`，就是对不上。  
`ros2 topic echo /sensor_data` 能看话题上有没有数据；匹配失败时这里也是空的。

相关命令还可以用：

```bash
ros2 node list
ros2 node info /sensor_publisher
ros2 node info /sensor_subscriber
```

**代码：** 只改 `qos_debugger_pub.cpp` 里这一处，把默认值改成 `reliable`：

```cpp
this->declare_parameter("reliability", "reliable");
```

改完重新 `colcon build`，停掉旧节点再启动。`ros2 topic info /sensor_data -v` 里两边 Reliability 都是 `RELIABLE`，sub 就能收到消息。
---

## 任务二：为什么收到的消息会丢包？/(ㄒoㄒ)/~~

第一问找到问题并修改代码后，记得重新
```colcon build```
```source install/setup.bash```
**现象**：sub会打印黄色的warning输出告诉你丢包的序列，每秒还会打印出丢包率

提示：
有没有什么命令可以查看节点的配置(ros2 param -h)
可以通过修复qos_debugger_sub.cpp中的一处或几处代码解决该问题（可能会有多种解决方法）
任务二可以这样写。

**原因：**  
发布端 100 Hz（约每 10 ms 一条）。订阅端参数 `callback_delay_ms` 默认是 **30**，回调里会 `sleep` 30 ms。处理速度慢于到达速度，队列 `depth` 只有 10，`KeepLast` 满了就会把旧消息挤掉，所以 seq 不连续，出现黄字丢包警告。

用参数确认：

```bash
ros2 param list
ros2 param list /sensor_subscriber
ros2 param get /sensor_subscriber callback_delay_ms
ros2 param get /sensor_subscriber depth
ros2 param describe /sensor_subscriber callback_delay_ms
```

会看到 `callback_delay_ms` 为 30、`depth` 为 10。运行时也可以改（代码里已经注册了参数回调）：

```bash
ros2 param set /sensor_subscriber callback_delay_ms 0
```

**改代码（任务要求改 `qos_debugger_sub.cpp`）：**  
把默认延迟改成 0：

```cpp
this->declare_parameter("callback_delay_ms", 0);
```

或删掉 / 注释回调里的 `sleep_for`。只加大 `depth` 只能多攒一会儿，处理永远慢于 100 Hz 时还是会丢，所以正道是把回调里的人为延迟去掉。

改完重新编译、source，再开 pub/sub。黄字丢包警告应消失，每秒打印的丢包率接近 0。

## 任务三：把收到的消息的帧率计算并打印出来（放在定时器回调函数中每秒打印一次即可）
补全qos_debugger_sub.cpp即可



在下面按顺序完成三个任务，要求把用到的命令放入代码块中并讲解命令，每一问最好加入自己的理解



