# TaskCore

协程与等待器、GMB/DAQ 设备族能力、族 Collector 和 DAQ 处理线程契约。Windows / Linux，C++20，CMake ≥ 3.28。

## 边界与依赖

- Qt ≥ 6.8.3；Qt 由使用环境提供，不在仓库中打包。
- 上游：[Algorithm](https://github.com/Lateef-ZHOU/Algorithm), [DeviceCore](https://github.com/Lateef-ZHOU/DeviceCore)。已有同名 CMake 目标时复用；否则 FetchContent 拉取固定完整提交。
- 本仓库不依赖 LiDAR_DataACQ 源码、应用目录或设备型号；不包含第三方二进制。
- 核心只提供设备族的标准能力。保存格式、任务共用业务与族专用界面作为可选参考目标；三种具体编排、具体页面及型号处理仍由官邸实现。安装姿态等状态归设备实例。
- 默认仅 Qt Core/OpenMP；保留主线程取消、收包完成消费和线程 quit/wait 契约，不提供退出保存排空。

## 可选模块

| 开关（默认 OFF） | 目标 | 内容 |
|---|---|---|
| TASKCORE_WITH_DAQ_SAVING | Lidar::DAQSaving | LiDAR 功率谱、径向风、风廓线的现有文件格式和路径；Qt Core，无界面 |
| TASKCORE_WITH_TASK_SUPPORT | Lidar::TaskSupport | TaskBase，登记族对象、启动校验、状态和取消；自动带保存目标，无 QML |
| TASKCORE_WITH_QML | Lidar::TaskUI / Lidar.TaskUI | 可选族控件和任务参考页面；带 TaskSupport、UICore、ChartCore，要求 Qt 6.11.2 |

消费者必须在 FetchContent_MakeAvailable 前选择开关。核心目标不反向链接附加模块。界面消费者链接 LidarTaskUIplugin 并显式 `Q_IMPORT_QML_PLUGIN(LidarTaskUIPlugin)`（动态 Qt 下的静态模块也需要）；同时导入 UICore/ChartCore 插件。各模块输出目录的两级父目录用于 QML 工具 IMPORT_PATH。

TaskModule 接收 `list<GMBDeviceBase>` 和 `list<DAQDeviceBase>`，不认识具体设备页面。任务自己的速度、时间及保存设置在启动时形成快照，经族基 collectorFor 造请求；不读取设备页速度或累积范围。预览只调用族的 withinTravel/mounting。TaskBase 只登记族身份，retiring 同步取消并移除引用，排队收尾保留。

这些目标均是可选实现，不要求任务官邸继承 TaskBase 或 TaskModule。三个具体计划和协程不在本库，官邸可以只使用核心能力自行编排。附加保存继续使用独立线程，不反向访问日志对象，不增加退出排空。

## 组合与线程契约

2026-10-26 用户定：GMB/DAQ 族改为组合持有 Device。GMBDeviceBase 继承 QObject，DAQDeviceBase 继承 QThread，只有官邸实现处理线程 run。Device 独持 onStream/onConsume 回调，删除 onCompleted 虚转发；完成帧仍在主线程先提交消费、再通知等待器。DAQWorker、各型号 QObject worker、sink_、独立 QThread 成员和 moveToThread 全部删除。

每个 DAQ run 在工作线程创建局部 QObject 锚点及唯一的型号处理 lambda，排队消息只携只读 Collector 份额，不复制有状态的 lambda。风廓线环及写位置随该闭包在线程内构造、使用和销毁；闭包对官邸只发结果信号，不访问其主线程业务成员。QML 在主线程接收，跨线程 C++ 使用者也必须指定接收上下文。启动用一次 QSemaphore 交接，回调安装后才允许首帧提交，不维护运行/关闭状态位。

末端析构首句仍执行内部 PreDestruction。DAQ 先 stopWorker（quit/wait，随后清空消费回调）、再取消自有测试协程、最后销毁组合 Device；GMB 直接销毁 Device。Device 析构同步停止 Handle，然后 retiring、撤运行槽、释放锁。组合对象以 QObject 父子关系兜底构造失败，正常预析构显式销毁；删除期间指针保持有效，供 retiring 的同步撤单使用。外部只使用正常析构；线程源关闭只从所属主线程调用，不直接调用继承的 QThread start/quit/terminate。

族层的 device 属性借出既有 DeviceBase 机制面，无状态缓存和逐方法转发。QML 状态、名称、日志从 backend.device 读取；安装姿态、信号、型号规格与 objectName 仍归官邸。任务文件名及虚拟卡采样源标识取族对象 objectName，历史查询取族对象的 QML parent。锁、CAS、重试、取消、已完成帧保留、connectionChanged 延迟裁定和退出不排空政策均保持。makeOnStream/makeProcessor 等回调工厂不再强制类外定义。

## 独立构建和验证

安装对应 Qt 开发包，然后执行（将路径替换为本机 Qt 前缀）：

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/Qt/6.8.3/platform -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel 2
ctest --test-dir build -C Release --output-on-failure
```

Windows 在安装 MSVC 的环境运行以上命令，Qt 选择对应 MSVC 构建；Linux 使用 GCC 与 OpenMP 开发环境。
测试逐个独立编译公开头、拒绝意外加载 GUI/QML，并运行契约检查。processing_contract 通过实际 Handle/Device 连续交付三帧，检查处理闭包状态延续、结果信号排队返回所属线程，以及关闭后不再引用线程内锚点。GitHub Actions 在 Windows/Linux 使用 Qt 6.11.2，分别验证核心、无界面任务业务及可选 QML 组件；最低版本声明与最新验证版本是不同概念。

## 接入应用

```cmake
include(FetchContent)
FetchContent_Declare(lidar_taskcore
    GIT_REPOSITORY https://github.com/Lateef-ZHOU/TaskCore.git
    GIT_TAG <经过验证的完整提交号>)
FetchContent_MakeAvailable(lidar_taskcore)
target_link_libraries(my_app PRIVATE Lidar::TaskCore)
```

从仓库提交列表选择固定版本；示例占位值必须替换，不使用浮动分支保证可复现。多个模块共用已有上游目标，应用若需覆盖上游版本应先统一声明并创建它们。标准 `FETCHCONTENT_SOURCE_DIR_LIDAR_模块名`（模块名大写无分隔）可用于本地开发，发布验证应取消覆盖并实际拉取远端。

## 来源与许可

从 [LiDAR_DataACQ](https://github.com/Lateef-ZHOU/LiDAR_DataACQ) 的模块分离提交 `94ad6f8` 提取；初始分仓保持运行逻辑，独立整理构建、验证与发布入口。可选保存、任务业务和界面从应用 `f20e890` 的对应实现迁入；具体任务编排由各自独立官邸仓库提供。

自有代码采用 [MIT](LICENSE)，Copyright (c) 2026 周涛 (ZHOU TAO)。第三方依赖不随本仓库改为 MIT，参见 [依赖说明](THIRD_PARTY_NOTICES.md)。

## 可选网口表单适配

`TASKCORE_WITH_PCAP_FORMS=ON` 提供 `Lidar::PcapForms` 与 `<Device/PcapForm.h>`，共用枚举快照、连接参数校验，不保存第二份表单草稿。需要在首次配置 HandleCore 前开启 `HANDLECORE_WITH_PCAP`，原生依赖要求沿用该库。默认关闭，核心和仿真官邸无需 libpcap。

纯 QML `PcapCard` 属于可选 `Lidar.TaskUI`，由官邸传入后端、表单与固定配置；自身不链接抓包驱动。不创建新 UI 层级，不替官邸决定页面分区。两项从 LiDAR_DataACQ `5cc6d65` 原样迁入，用于多个网口官邸共同复用。

## 仿真参考页面

可选 TaskUI 另提供 `SimOnlyTaskModule` 和 `SimOnlyTaskRunSection`：接收单一 GMB/DAQ 族引用，持有累积/停稳设置，排列计划与运行分区。没有设备选择、速度、保存开关或目录草稿；具体官邸在启动时固定保存到当前工作目录。它与全功能 TaskModule 并列，不强制其他任务采用，也不把具体计划/协程放入核心。

FixedTask、CycleTask、SnakeTask 各提供自己的 simonly 末端页面，沿用各自既有计划和独立协程。设备退场和取消仍走原族引用接线；本次不改 C++ 核心、处理线程或保存退出策略。

simonly 页面的累积时间 `accum` 和停稳时间 `settle` 为创建时必填属性，不在页面完成后补写。具体任务官邸同时要求创建时提供角度计划，页面编辑和启动共用同一份值。运行分区只调用官邸无参 `start()` / `stop()`，不认识或转交计划参数；全功能 TaskModule 的接口保持原样。
