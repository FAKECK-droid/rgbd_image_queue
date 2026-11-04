# RGB-D 压缩图像队列管理器

直接订阅 sensor_msgs/CompressedImage，按时间戳配对、缓存并通过唯一自定义服务返回。
队列不解析压缩载荷，不编解码、不转码、不缩放，不自动写磁盘；不启动相机、底盘或导航。

## 数据与缓存

默认完整话题名：

- /camera/color/image_raw/compressed
- /camera/depth/image_raw/compressedDepth

若驱动使用 color_raw/compressed 等其他名称，修改 color_topic/depth_topic 即可。
两路 header、format、data 原样保留；已移除 image_transport 和编解码插件依赖。
message_filters ApproximateTime 在配置的时间差内一对一配对，任一帧最多使用一次。
空间对齐仍由相机驱动完成。

- 历史队列保存第 1、1+N、1+2N…个配对，超容量淘汰最旧者，超过寿命自动淘汰。
- 独立最新缓存每个配对都更新，不受历史采样、容量或清除影响；相机停止后仍保留最后一对。
- 两层共享压缩消息指针；服务只复制请求的彩色、深度或两者，复制在缓存锁外执行。
  取图不消耗队列元素。图像与服务使用不同回调组、两个执行线程。

年龄取“当前 ROS 时间减较早的采集时间戳”与“单调时钟接收后时间”的较大者，最小为 0。
因此 ROS 仿真时间暂停时历史也会淘汰；独立最新通过 image_age_sec / exceeded_max_age 表示陈旧程度。

仅保留已批准的三个参数范围检查，以及空缓存、索引越界、未知操作等服务正常状态处理。
未增加图像内容检查、重试或自动恢复。

## 编译和启动

开发机：

    cd /root/scout_mini_ws
    source /opt/ros/humble/setup.bash
    colcon build --packages-select rgbd_image_queue --symlink-install
    source install/setup.bash
    ros2 launch rgbd_image_queue image_queue.launch.py

部署到开发板后将工作空间换为 /home/yan/ros2_ws。
本次服务类型变更需要重新编译并重启所有客户端，旧 Image 响应接口不再兼容。

    ros2 launch rgbd_image_queue image_queue.launch.py config:=/绝对路径/queue.yaml
    ros2 launch rgbd_image_queue image_queue.launch.py namespace:=vision use_sim_time:=true

## 参数

下表为 config/queue.yaml 的启动值；color_transport / depth_transport 已取消。

| 参数 | 配置值 | 含义 |
| --- | --- | --- |
| color_topic | /camera/color/image_raw/compressed | 完整彩色压缩话题名 |
| depth_topic | /camera/depth/image_raw/compressedDepth | 完整深度压缩话题名 |
| service_name | ~/get_image | 默认 /rgbd_image_queue/get_image |
| queue_length | 10 | 历史容量；0 关闭历史 |
| save_every_n_frames | 10 | 每 N 个配对采样一次，首对入队 |
| max_image_age_sec | 10.0 | 历史最长寿命，秒；0 不按时间淘汰 |
| sync_tolerance_sec | 0.04 | 两路时间戳最大差值，秒 |
| sync_queue_size | 4 | 每路同步缓冲长度，使用 ≥2 |
| qos_depth | 2 | best effort / volatile 订阅深度 |
| purge_period_sec | 0.2 | 主动清理周期，正数，秒 |

三个缓存参数可动态修改，范围为长度≥0、间隔≥1、寿命≥0；其余参数在启动时指定。
直接运行节点而不加载 YAML 时，采样间隔和寿命分别为 5、5.0。
改变采样间隔后下一对立即入队，缩小容量或寿命会立即裁剪。

    ros2 param set /rgbd_image_queue queue_length 20
    ros2 param set /rgbd_image_queue save_every_n_frames 5
    ros2 param set /rgbd_image_queue max_image_age_sec 8.0

retained_image_bytes 统计缓存引用的独立压缩 data 字节数；两层共享一对时只计一次。
不包含同步器、DDS 和服务响应的内存。

## 服务

/rgbd_image_queue/get_image，类型 rgbd_image_queue/srv/GetImage。
请求为 uint8 operation + uint32 detail。

| operation | 常量 | detail 和返回内容 |
| --- | --- | --- |
| 0 | LATEST_COLOR | 独立最新彩色；忽略 detail |
| 1 | LATEST_DEPTH | 独立最新深度；忽略 detail |
| 2 | LATEST_PAIR | 独立最新图像对；忽略 detail |
| 3 | QUEUE_COLOR | 历史第 detail 个配对的彩色 |
| 4 | QUEUE_DEPTH | 历史第 detail 个配对的深度 |
| 5 | QUEUE_PAIR | 历史第 detail 个图像对 |
| 6 | STATUS | 只返回状态；忽略 detail |
| 7 | CLEAR_QUEUE | 只清空历史，保留独立最新；忽略 detail |
| 8 | QUEUE_COLOR_AT_AGE | 历史中最接近 detail 毫秒前的彩色 |
| 9 | QUEUE_DEPTH_AT_AGE | 历史中最接近 detail 毫秒前的深度 |
| 10 | QUEUE_PAIR_AT_AGE | 历史中最接近 detail 毫秒前的图像对 |

索引从新到旧：0 最新、1 次新，合法范围 [0, queue_size)。
处理请求时的索引随入队/淘汰变化；pair_id 在进程内递增，清空历史不会归零。

时间查询相对于服务处理时刻，使用上文的配对年龄，不是从最新帧往前推算。
只搜索未过期的历史；不搜索独立最新。误差相同选较新者。
超出历史时间范围时返回最近端点，不代表存在恰好该时刻的图像。

响应的 color / depth 均为完整 CompressedImage，has_color / has_depth 指明包含的内容。
只取一类时另一类 data 为空；无 width、height、encoding 等 Image 字段。
color_stamp、depth_stamp、stamp_delta_sec 来自同一配对。

- queue_index：实际命中的历史索引；独立最新、状态、清除或取不到图像时为 -1。
- image_age_sec：返回配对实际年龄；AT_AGE 的 age_error_sec = 实际年龄 - 请求年龄，
  正数表示返回帧较旧；其他操作该误差为 0。
- queue_size、latest_available、latest_age_sec、paired_count、enqueued_count、
  expired_count、capacity_evicted_count、retained_image_bytes：状态及累计计数。

空缓存或索引越界时 success=false，不替换成其他帧。STATUS / CLEAR_QUEUE 可对空缓存成功。
原操作 0–7 编号保留；原 QUEUE_* 请求 detail=0 时仍取历史最新。

## 调用示例

摘要客户端避免打印图像字节，实际服务响应包含完整压缩载荷：

    ros2 run rgbd_image_queue rgbd_queue_client latest_pair
    ros2 run rgbd_image_queue rgbd_queue_client queue_color 2
    ros2 run rgbd_image_queue rgbd_queue_client queue_pair 0
    ros2 run rgbd_image_queue rgbd_queue_client queue_pair_at_age 1500
    ros2 run rgbd_image_queue rgbd_queue_client queue_depth_at_age 500
    ros2 run rgbd_image_queue rgbd_queue_client status
    ros2 run rgbd_image_queue rgbd_queue_client clear_queue

queue_color 2 取从新到旧第 3 张历史彩色图。
通过 --service /vision/rgbd_image_queue/get_image 可指定命名空间下的服务。

    ros2 interface show rgbd_image_queue/srv/GetImage
    ros2 service call /rgbd_image_queue/get_image rgbd_image_queue/srv/GetImage "{operation: 6, detail: 0}"

Python：

    from rgbd_image_queue.srv import GetImage
    client = node.create_client(GetImage, "/rgbd_image_queue/get_image")
    request = GetImage.Request()
    request.operation = GetImage.Request.QUEUE_PAIR_AT_AGE
    request.detail = 1500
    future = client.call_async(request)
    # 在执行器中等待完成后，future.result().color / .depth 为 CompressedImage。

## 开发机验证

test_pair_queue.cpp 覆盖共享压缩数据、采样、容量、寿命、索引与时间选择。
test_ros_queue.py 使用非 JPEG/PNG 不透明载荷验证真实订阅、配对、服务逐字节返回及动态参数。
test_person_compressed.py 仅验证消费端深度读取，不加载 YOLO，不启动硬件或导航。

    colcon test --packages-select rgbd_image_queue --return-code-on-test-failure --event-handlers console_direct+
    colcon test-result --test-result-base build/rgbd_image_queue --verbose

2026-11-04 开发机编译通过，17 项缓存测试、6 项 ROS 集成测试、
4 项消费端格式测试及两份配置的命名空间 launch / 摘要客户端检查通过。
测试构建位于 build/rgbd_queue_local，安装位于 install/rgbd_queue_local；
使用现有测试安装可先 source /root/scout_mini_ws/install/rgbd_queue_local/setup.bash。

本机 WSL 默认网络下标准 ROS 示例也不能跨进程发现；本次 ROS 测试在临时网络命名空间执行，
未改变系统网络或生产代码。复现本次测试：

    sudo unshare --net bash -c 'ip link set lo up; source /opt/ros/humble/setup.bash; source /root/scout_mini_ws/install/rgbd_queue_local/setup.bash; cd /root/scout_mini_ws; colcon test --base-paths src/scout_mini_vln/rgbd_image_queue --packages-select rgbd_image_queue --build-base build/rgbd_queue_local --install-base install/rgbd_queue_local --return-code-on-test-failure'

验证日志：build/rgbd_image_queue_local_qa/test_netns.log、launch_smoke.log。
过去解码版实机 CPU/内存测量不适用于当前实现；本次未部署开发板。

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
/camera/color/image_raw/compressed 和 /camera/depth/image_raw/compressedDepth；
不保留历史队列，仅独立缓存最新配对。队列原样返回 CompressedImage；
person 节点在后台推理需要像素时解码 JPEG，检测到 person 后才解码深度 PNG。

### 性能改动

- 队列不做编解码；person 节点按推理频率解码，不按相机完整帧率解码。
- 推理放入单独的一个后台工作线程，ROS 回调负责 TF、Action、服务和结果发布。
- 同时最多一组取图/推理任务，不累积历史工作。
- 默认 3 Hz、416 输入尺寸、CUDA FP16；PyTorch/OpenCV 限制 CPU 线程。
- launch 给检测进程设置 nice=5，BLAS/OpenMP 线程设为 1。
- 启动时每个硬件就绪话题只收一条，随后退订，避免持续反序列化点云和里程计。
- 相机内参只接收首次值；若相机模式改变，重启检测节点以重新读取内参。
- 解开 compressedDepth PNG 后，只对框中心像素计算米制深度。
  16UC1 按毫米读取，32FC1 用头内的逆深度量化参数恢复米制值。
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
| queue_config | 安装目录 config/object_follow_queue.yaml | 默认压缩消息、无历史 |
| use_sim_time | false | 与外部硬件/Nav2 保持一致 |

单独启动脚本时先激活 YOLO venv，再使用 ros2 run rgbd_image_queue person_goal.py。
当前仅在开发机验证代码和压缩接口，未运行模型推理、跟随 launch 或实车导航。
深度消费者使用 ROS Humble compressedDepth PNG（不包含 RVL）；
格式头见 [官方 ConfigHeader](https://github.com/ros-perception/image_transport_plugins/blob/humble/compressed_depth_image_transport/include/compressed_depth_image_transport/compression_common.h)。

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
