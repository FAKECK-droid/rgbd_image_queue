# RGB-D 图像队列管理器

ROS 2 Humble 图像队列包，只有一个自定义取图服务。队列节点订阅现有相机，不启动底盘、
导航或相机，也不修改现有 VLN 工具。所有图像保留原始编码、分辨率、时间戳和坐标系，不自动写磁盘。压缩输入先由官方插件解码，再缓存标准 Image；
缓存不会二次编码或缩放。彩色 JPEG 的有损压缩来自发布端，不能等同于原始像素。

## 两层缓存

- 历史队列：按匹配成功的 RGB-D 对计数，保存第 1、1+N、1+2N…对；
  长度超限时淘汰最旧的一对，超过寿命时自动淘汰。
- 独立最新缓存：每个匹配成功的图像对都更新，不受采样间隔、队列容量或历史清除影响。
  相机停止后仍保留最后一对。服务返回年龄和 exceeded_max_age，交由调用者决定是否使用。
- 时间匹配使用 message_filters ApproximateTime，容差内一对一消费，不重复使用同一输入帧。
  不强行修改时间戳；缺少任一图像流时不会拼接出不对应的图像对。
- 图像解码后在订阅、最新缓存和历史缓存之间使用共享引用；服务返回图像才发生必要的拷贝。
  服务和图像回调使用不同回调组，两个线程执行。
- 解码使用官方 image_transport 插件；本包不另外做图像内容、尺寸或坐标系检查，
  也没有自行加入重试、自动恢复逻辑。
  仅加入用户批准的三项参数范围检查，以及空队列等正常服务状态处理。

## 相机话题与传输格式

当前默认订阅真实的压缩话题：

- /camera/color/image_raw/compressed：sensor_msgs/CompressedImage，使用 compressed 插件。
- /camera/depth/image_raw/compressedDepth：sensor_msgs/CompressedImage，使用 compressedDepth 插件。

image_transport 负责 JPEG / compressedDepth PNG 解码，之后才进行时间匹配和缓存。
当前 Femto Bolt D2C 输出解码后均为 1280×720、camera_color_optical_frame。
彩色通常为 rgb8；深度为 16UC1，单位毫米，0 为无效深度。
compressedDepth 不是普通 JPEG；使用官方专用插件可保留 16 位深度与其格式头。

配置写实际话题全名，传输后缀与 color_transport/depth_transport 对应；
节点向 image_transport 提供去除传输后缀后的基话题。
如切换到原始图像，四项同时改为：

    color_topic: /camera/color/image_raw
    color_transport: raw
    depth_topic: /camera/depth/image_raw
    depth_transport: raw

输入压缩可以减少 ROS 传输数据量，但本包按要求解码后缓存，缓存内存仍按原始图像尺寸计算。
本包只做时间匹配，空间对齐由相机驱动负责。
如需相机内参，使用 /camera/color/camera_info 和 /camera/depth/camera_info。

## 编译和启动

    cd /home/yan/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --packages-select rgbd_image_queue --symlink-install
    source install/setup.bash
    ros2 launch rgbd_image_queue image_queue.launch.py

相机已经由导航或独立相机 launch 启动时，只运行这个队列 launch 即可。
本包不会重复打开设备；未启动相机时，服务仍可用，缓存保持为空。

使用其他配置文件：

    ros2 launch rgbd_image_queue image_queue.launch.py config:=/绝对路径/queue.yaml

namespace 和 use_sim_time 可通过 launch 参数设置。回放 bag 时使图像消息和节点使用同一
ROS 时间。队列年龄取“消息采集年龄”与“本机单调时钟接收后年龄”的较大者；
因此没有新图像、ROS 仿真时间暂停时，历史仍按实际经过时间淘汰。

## 全部参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| color_topic | /camera/color/image_raw/compressed | 彩色 Image 话题，启动时配置 |
| depth_topic | /camera/depth/image_raw/compressedDepth | 深度 Image 话题，启动时配置 |
| color_transport | compressed | 彩色输入传输方式，启动时配置 |
| depth_transport | compressedDepth | 深度输入传输方式，启动时配置 |
| service_name | ~/get_image | 默认解析为 /rgbd_image_queue/get_image |
| queue_length | 10 | 历史最多保留多少对；0 仅保留独立最新缓存；动态修改 |
| save_every_n_frames | 5 | 每 N 个匹配图像对保存一个到历史；必须 ≥1；动态修改 |
| max_image_age_sec | 5.0 | 历史图像最长寿命；0 不按年龄淘汰；动态修改 |
| sync_tolerance_sec | 0.04 | 两路 header.stamp 最大差值，秒；启动时配置 |
| sync_queue_size | 4 | 同步器每路缓冲长度，使用 ≥2；启动时配置 |
| qos_depth | 2 | 每路订阅 QoS 深度；best effort / volatile；启动时配置 |
| purge_period_sec | 0.2 | 历史主动清理周期，秒；正数；启动时配置 |

动态示例：

    ros2 param set /rgbd_image_queue queue_length 20
    ros2 param set /rgbd_image_queue save_every_n_frames 10
    ros2 param set /rgbd_image_queue max_image_age_sec 8.0

改变采样间隔后，下一对立即入队，之后按新间隔采样；缩小长度或寿命会立即裁剪。
三个范围分别为长度≥0、间隔≥1、寿命≥0，无效更新不改变现有配置。

1280×720 rgb8 + 16UC1 每对像素约 4.608 MB，默认 10 对历史加独立最新最多约
50.7 MB 像素数据。同步器、DDS 和正在返回的服务消息不计入此数字。
retained_image_bytes 只统计缓存持有的独立像素数据，同一对被两层引用时不会重复统计。

## 唯一自定义服务

名称：/rgbd_image_queue/get_image  
类型：rgbd_image_queue/srv/GetImage  
请求只包含 operation（uint8），定义了可在 C++/Python 中使用的命名常量：

| operation | 常量 | 返回内容 |
| --- | --- | --- |
| 0 | LATEST_COLOR | 独立最新缓存的彩色图 |
| 1 | LATEST_DEPTH | 独立最新缓存的深度图 |
| 2 | LATEST_PAIR | 独立最新缓存的 RGB-D 对 |
| 3 | QUEUE_COLOR | 历史队列最新一对中的彩色图 |
| 4 | QUEUE_DEPTH | 历史队列最新一对中的深度图 |
| 5 | QUEUE_PAIR | 历史队列最新图像对 |
| 6 | STATUS | 缓存状态，不返回像素数组 |
| 7 | CLEAR_QUEUE | 清空历史，保留独立最新图像对 |

取不到图像是正常状态：success=false，message 说明所选缓存没有匹配图像。
成功返回 has_color/has_depth 指示实际包含的图像；只取彩色时深度像素数组为空，反之亦然。
color_stamp、depth_stamp、stamp_delta_sec 来自同一对；pair_id 在进程内递增，
清空历史不会归零。paired_count 指同步器成功交给缓存的总对数，不是原始话题帧数。

状态还包含 queue_size、latest_available、latest_age_sec、enqueued_count、
expired_count、capacity_evicted_count 和 retained_image_bytes。

推荐使用摘要客户端，避免 ros2 service call 把数 MB 像素数组打印到终端：

    ros2 run rgbd_image_queue rgbd_queue_client status
    ros2 run rgbd_image_queue rgbd_queue_client queue_pair
    ros2 run rgbd_image_queue rgbd_queue_client latest_color
    ros2 run rgbd_image_queue rgbd_queue_client latest_depth
    ros2 run rgbd_image_queue rgbd_queue_client latest_pair
    ros2 run rgbd_image_queue rgbd_queue_client clear_queue

使用命名空间时，通过 --service /命名空间/rgbd_image_queue/get_image 指定服务。
客户端只显示图像摘要，但服务响应本身包含完整 sensor_msgs/Image。
客户端等待服务出现；Ctrl+C 可取消等待。

也可查看接口或只查询状态：

    ros2 interface show rgbd_image_queue/srv/GetImage
    ros2 service call /rgbd_image_queue/get_image rgbd_image_queue/srv/GetImage "{operation: 6}"

程序中直接使用：

```python
from rgbd_image_queue.srv import GetImage

client = node.create_client(GetImage, "/rgbd_image_queue/get_image")
request = GetImage.Request()
request.operation = GetImage.Request.QUEUE_PAIR
future = client.call_async(request)
# 在执行器中等待完成；future.result().color / .depth 即完整图像。
```

## 验证

test_pair_queue.cpp 验证采样、容量、寿命、最新缓存独立性、共享引用和三项参数范围。
test_ros_queue.py 在隔离 ROS 域发布合成图像，验证真实订阅、时间匹配、自定义服务、
动态参数和图像原样返回，不启动真实底盘或传感器。

    colcon test --packages-select rgbd_image_queue --event-handlers console_direct+
    colcon test-result --test-result-base build/rgbd_image_queue --verbose

## 已完成验证

- 11 项缓存单元测试 + 4 项真实 ROS 合成图像集成测试通过。
- 带命名空间的 launch 和配置加载通过；配置文件使用 /** 匹配节点命名空间。
- 原始 Image 话题实机测试通过：RGB-D 约 14.2 对/秒，节点单核 CPU 约 7.6%，
  RSS 约 102 MB。该值为短时实测量级，不是固定性能指标。
- 彩色/深度都为 1280×720，编码 rgb8 / 16UC1，坐标系均为
  camera_color_optical_frame；一对时间戳差约 24–25ms，小于 40ms 容差。
- 相机停止后历史清空，最新图像对仍保留并标记 exceeded_max_age。

实机验证记录（命令行显式使用原始话题）：

/home/yan/ros2_ws/build/rgbd_image_queue/qa/20261103_203616/raw_transport/live_report.json

### 当前压缩输入实机结果

使用 queue.yaml 当前默认的 compressed + compressedDepth，未覆盖话题：
1280×720 的 rgb8 / 16UC1 解码后配对约 14.7 对/秒，
队列进程约占单核 63%，RSS 约 141 MB（含解码插件）。
这是本机短时采样；压缩传输减少传输数据量，同时增加 JPEG/PNG 解码开销。
深度单位保持毫米，图像对时间差实测约 16–25ms。
历史寿命到期与最新缓存保留、单独取彩色/深度及图像对的服务均已实机验证。

/home/yan/ros2_ws/build/rgbd_image_queue/qa/20261103_203616/compressed_transport/live_report.json

## Person 持续跟随（2026-11-03 更新）

跟随启动文件只启动图像队列和 person 节点。底盘、雷达、相机、定位与 Nav2 由用户单独启动，
本文件不会包含 scout_mini_nav 的 bringup，也不覆盖其 CAN 端口。

### 启动

先在另一个终端启动你自己的硬件/导航 bringup，使用物体跟随导航配置，例如：

```bash
source /opt/ros/humble/setup.bash
source /home/yan/ros2_ws/install/setup.bash
ros2 launch scout_mini_nav bringup_launch.py \
  nav2_params_file:=/home/yan/ros2_ws/src/scout_mini_nav/params/nav2_params_object_follow.yaml
```

然后启动跟随端：

```bash
source /opt/ros/humble/setup.bash
source /home/yan/ros2_ws/install/setup.bash
ros2 launch rgbd_image_queue object_follow.launch.py \
  rate_hz:=3.0 imgsz:=416 half:=true visualize:=false clear_costmaps:=true
```

相机必须已将深度对齐到原始彩色图。跟随端默认使用 config/object_follow_queue.yaml：
原始 /camera/color/image_raw 和 /camera/depth/image_raw，raw 传输；
不保留历史队列，仅独立缓存最新配对。原有压缩配置 config/queue.yaml 保持不变。

### 性能改动

- JPEG/compressedDepth 编解码链路从跟随端移除；同机直接订阅原始 Image。
- 推理放入单独的一个后台工作线程，ROS 回调负责 TF、Action、服务和结果发布。
- 同时最多一组取图/推理任务，不累积历史工作。
- 默认 3 Hz、416 输入尺寸、CUDA FP16；PyTorch/OpenCV 限制 CPU 线程。
- launch 给检测进程设置 nice=5，BLAS/OpenMP 线程设为 1。
- 启动时每个硬件就绪话题只收一条，随后退订，避免持续反序列化点云和里程计。
- 相机内参只接收首次值；若相机模式改变，重启检测节点以重新读取内参。
- 深度只按行跨度、像素大小与字节序读取框中心单个像素，不转换整张深度图。
- 检测框和置信度一次从 GPU 拷回 CPU。

### 持续跟随和清理 costmap

首次选择最高置信度 person；之后选择与上一框 IoU 最大者，置信度打破平局。
不额外引入身份跟踪器。

初次等待 8 个 Nav2 生命周期节点 active、硬件就绪信号、图像服务、TF 和 Action Server。
就绪信号只确认首次输出，不再执行“最近 2 秒”等数据年龄检查。

检测中心按 CameraInfo 的 K/D 反投影，16UC1 深度毫米转米，32FC1 按米。
在线坐标变换直接使用最新可用 TF，不再查询图像采集时刻；发布时给 PoseStamped 填当前 ROS 时间。
这是在线跟随使用的近似变换，不保证图像曝光时刻与机器人姿态严格同步。
保留重复 pair_id 跳过、无 person/无中心深度/无 TF 时不发布的正常处理。

每次有效目标更新前，异步调用：

- /local_costmap/clear_entirely_local_costmap
- /global_costmap/clear_entirely_global_costmap

两项调用完成后向 /goal_update 发布最新 map 导航目标。清理期间的新检测覆盖待发布目标，
同一时间最多一组清理请求，不排队堆积。clear_costmaps:=false 可关闭该步骤。

当前没有在途 Action 时自动发起 NavigateToPose，并显式指定本包 config/person_follow.xml。
运行期间通过 /goal_update 更新目标，不为每帧重启 Action。
Action 被拒绝、失败/中止或正常结束后，清除在途状态、重新确认 Nav2 active，
默认间隔 0.3 秒后用最近目标再次发起；无需等待下一轮模型推理。
重试前也清理两张 costmap。实际重发时间仍取决于 Nav2 和清理服务是否就绪。
显式取消导航会暂停自动重发；重启跟随节点可恢复。

未新增目标丢失取消策略；没有新检测时，Nav2 和重试逻辑保留最后目标。

### 参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| rate_hz | 3.0 | 取图/推理频率上限，正数 |
| status_period_sec | 5.0 | 终端就绪、检测、目标发布和导航状态汇总周期，秒，正数 |
| follow_distance | 0.5 | base_link 到检测 person 位置的跟随间距，米；必须为正数 |
| imgsz | 416 | YOLO 推理尺寸 |
| half | true | CUDA FP16，device=cpu 时自动关闭 |
| cpu_threads | 1 | PyTorch CPU 线程数 |
| visualize | false | 显示所选框、中心像素、深度和 map 坐标 |
| retry_delay_sec | 0.3 | Action 结束后重发的最小间隔，秒 |
| clear_costmaps | true | 每次有效目标更新及重试前清理两张 costmap |
| behavior_tree | 安装目录 config/person_follow.xml | 跟随树 |
| confidence | 0.4 | person 置信度门限 |
| device | 0 | CUDA 设备号或 cpu |
| model_path | ~/ros2_ws/src/scout_mini_vln/rgbd_image_queue/test/yolo26n.pt | COCO 模型 |
| python_executable | ~/ros2_ws/.venvs/yolo-jetson/bin/python | 检测节点解释器 |
| service_name | /rgbd_image_queue/get_image | 队列和检测器共用服务名 |
| queue_config | 安装目录 config/object_follow_queue.yaml | 默认原始图像、无历史 |
| use_sim_time | false | 与外部硬件/Nav2 保持一致 |

单独启动脚本时先激活 YOLO venv，再使用 ros2 run rgbd_image_queue person_goal.py。
本轮按用户要求未运行测试、推理、launch 或实车验证，性能与导航行为由用户测试。

### 到达后的请求频率

config/person_follow.xml 在 FollowPath 成功到达后，通过内置 Delay 等待 1000 ms，
再允许 KeepRunningUntilFailure 发起下一次 FollowPath。这样到达跟随距离时，
新控制任务约为 1 Hz，避免“立即到达、立即重启”造成约 20 Hz 的日志和 Action 开销。
行驶中的 FollowPath 仍持续运行，controller_frequency 和图像处理 rate_hz 不受这个等待影响。
规划分支继续 1 Hz 更新目标路径；失败时仍交给脚本进行既有的自动重试。
修改 delay_msec 可以调整到达后的等待时间。本次仅构建安装，未运行测试。

### 2026-11-04 跟随距离只计算一次

过去先规划到 person，再把路径尾端截掉 1 米；NavFn 若选到附近的可达点，
又会额外拉大停车距离。现在脚本先计算离 person 0.5 米的导航停车点，
再发布到 /goal_update；行为树直接跟踪完整规划路径，不再使用 TruncatePath。

原始 person 的 map 观测独立发布到 /person_position，便于在 RViz 对照。
/goal_update 现在表达实际导航停车点，z=0，朝向 person。
当 base_link 已在 follow_distance 内时，目标设为当前机器人位置，不要求车辆后退。

两份导航 YAML 的 SimpleGoalChecker 与 DWB 位置容差统一为 0.10 米，
SimpleGoalChecker.stateful=false，保持每次到达判定都检查 XY 距离。
NavFn.tolerance 收紧为 0.05 米，避免把相距较远的可达点当作目标替代。
停车点不可达时应规划失败并重试，而非以远处替代终点报告到达。
这些参数需重启导航 bringup 加载。

启动跟随端示例：

    ros2 launch rgbd_image_queue object_follow.launch.py follow_distance:=0.5

原有到达后 1 秒等待、失败重试、异步清理 costmap 与后台推理仍适用。
本轮不运行测试或实车运动，距离效果由用户验证。


### 2026-11-04 终端就绪与检测日志

默认每 5 秒输出一轮状态；启动时也立即打印一次。启动参数独立于推理频率：

    ros2 launch rgbd_image_queue object_follow.launch.py status_period_sec:=2.0

- [未就绪]：逐项列出没有首帧的硬件话题、缺失 CameraInfo、Nav2 具体节点的
  lifecycle 状态/服务无响应、取图和 NavigateToPose 不可用、启用清图时的清理服务，
  以及缺少的 map <- 相机/map <- base_link TF 与查询错误。
- [全部就绪]：以上依赖均已确认；是否有有效 person 目标由检测日志另行报告。
- [检测]：尚未完成推理、未检测到 person、person 数量、所选框置信度与中心深度、
  深度无效/缺 TF、person 的 map 位置和距离；显示距最近结果多久及累计处理帧数。
- [图像]：取图失败的服务 message、重复 pair_id、等待取图或后台推理的时长。
- [导航]：没有有效目标、等待 action 接受、action 执行中、清理 costmap 进度、
  自动重试或手动取消暂停；显示 /goal_update 发布次数、最近目标坐标和发布时间间隔。

模型加载、硬件首帧、CameraInfo、Nav2 状态变更、action 接受/结束也有事件日志。
硬件就绪沿用启动首帧确认；没有新增持续点云订阅或按帧打印日志。
本轮只修改 Python/launch 并静态检查语法，不运行测试、推理或硬件。
当前安装的脚本及 launch 链接到源码，重启跟随 launch 即可使用。
