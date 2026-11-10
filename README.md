# TaskCore

2026-11-10 用户定（波长与抽稀）：激光波长移到族基，`DAQCollector(parameters, channels, wavelengthM)`，`complete()` 按 λ/2 换算后交各通道 `process(metersPerSecondPerMHz)`；`DAQChannel` 删除 `metersPerSecondPerMHz`。

2026-11-10 用户定（线性数组）：`DAQChannel::radialWind`、`cnr` 都是 `LinearArray<double>`（轴为斜距，型号在规格里给出同一轴，处理只填数值），删除 `rangeStartM`/`rangeStepM`；`radialReady(channel, LinearArray<double> speeds, LinearArray<double> cnr, calibrated)`；四束环以 `sameAxis` 判同一网格，反演为 `WindProfile::Calc(beams, at)`。频率轴暂仍为 `freqStartMHz`/`freqStepMHz`（功率谱是二维）。

2026-11-10 用户定（等间隔轴）：`DAQChannel` 以 `rangeStartM`/`rangeStepM`、`freqStartMHz`/`freqStepMHz` 取代逐点数组，径向风只要求两个步长为正；`radialReady(channel, speeds, cnr, calibrated)` 不再带斜距（几何由型号的绘图参数给出），`profileReady` 发均匀 `VerticalWindProfilePtr`，四束环按起点与门距判同一网格，反演用 `WindProfile::Calc(beams, start, step, at)`。帧 JSON 与风廓线 JSON 改写起点与步长。

2026-11-09 用户定（DAQ 第二步）：DAQ 请求改为按值参数 `DAQParameters{accumSec, context}`，删除 collectorFor、只读原型与 begin 中的验型；`DAQCollectorLike` 改为要求静态 `create(const DAQParameters&)` 交出完整可写自身，begin 直接 `create` 后交 Device。DAQDeviceBase::Request = DAQParameters。

2026-11-09 用户定（DAQ 族第一步）：删除 DAQ 处理线程，DAQDeviceBase 改为纯 C++ 旁支，`DAQDevice<HandleT, CollectorT>` 同时继承 Device 与族基（device() 返回 DeviceBase&，不用协变以避开 MSVC 对结构体模板实参的 C2062）。DAQChannel（名称、门数 × 频点数、每门距离、每频点频率、零风速参考、频移→风速系数及谱/风/CNR 数据）放在 DAQCollector 之前，DAQCollector 直接继承 std::vector<DAQChannel>；型号收齐末包调用 complete()，在收包线程按通道串行做标准处理后才判齐。主线程消费：按通道发 spectrumReady/radialReady/profileReady，Boost circular_buffer 保存最近 4 帧，同步位为真时逐通道反演风廓线。AcqSaver 并入核心（删除 TASKCORE_WITH_DAQ_SAVING 与 Lidar::DAQSaving），只保留 saveFrame/saveWindProfiles 两个入口：每帧一个 JSON、风廓线同构 JSON，英文键、6 位有效数字、缺测 null。TaskBase 的 addCard/start 收 QObject* 交叉转换验族，任务 QML 采集卡类型改为 QtObject。请求仍是只读原型（第二步见主程序待办）。

2026-11-09 用户定（云台瘦身）：删除 GMB 的 collectorFor 与原型路径。`GMBMoveParameters` 移入 GMBCollector.h，含目标位形、停稳与可选速度；官邸 `paramsFor(target, hold, speeds)` 做型号校验与量化（取代原 collectorFor 的预验职责），族口 `begin(const GMBMoveParameters&, done, lock)` 由型号 makeMoveCollector 直接造出完整可写的具体 Collector 交 Device，不再 dynamic_cast，启动不再 makeSelf。RunOrder 拆为 request 与 result 两槽，析构只对结果为空的项撤单，不再询问 settled；取代 10-25“提交时把原型移出账本”的裁定。TaskBase 启动时只解析速度表（speedsFromQml），协程逐束以目标、停稳、速度这些原始量调用 paramsFor 构造参数。DAQ 仍走只读原型，族基声明 Request 别名，改造见主程序待办。

2026-11-10 用户定（DAQ 参数）：DAQContext 并入 `DAQParameters{accumSec, pose, saveRoot, sessionDir, sync}`，形制对齐 GMBMoveParameters，Collector 只存 `parameters()`。官邸 `paramsFor(accumSec, pose, saveRoot, sessionDir, sync)` 做型号校验，累积时间越界记录原因并返回空；族口 `begin` 以同一判据复核后经 `create` 造完整 Collector；单卡测试与 TaskBase::start 发令前预验。

2026-11-09 用户定：GMBDeviceBase 改为纯 C++ 旁支，不继承 QObject；`GMBDevice<HandleT, CollectorT>` 同时继承 `Device<HandleT, CollectorT>` 与族基，`device()` 协变返回自身的 Device 基类，族基保留纯虚 `device()` 供编排取 DeviceBase。mountingChanged/anglesReady 改为纯虚契约，由末端以 Q_SIGNALS 覆写；删除 `device` Q_PROPERTY、`deviceObject()` 与 GMBDeviceBase 的 QML 登记。TaskBase 的 addGimbal/start 接收 QObject* 并交叉转换验族，非云台拒收；TaskModule/SimOnlyTaskModule 的云台类型改为 QtObject。DAQ 族本轮不变。task_ui_smoke 改为触发按钮实际绑定的 Action（原先只发 clicked，Action 不执行，测试此前已失败）。验证：Windows 全选项独立构建零警告、CTest 4/4。

协程与等待器、GMB/DAQ 设备族能力、族 Collector、DAQ 通道标准处理与统一落盘。Windows / Linux，C++20，CMake ≥ 3.28。

## 边界与依赖

- Qt ≥ 6.8.3；Qt 由使用环境提供，不在仓库中打包。
- 上游：[Algorithm](https://github.com/Lateef-ZHOU/Algorithm), [DeviceCore](https://github.com/Lateef-ZHOU/DeviceCore)。已有同名 CMake 目标时复用；否则 FetchContent 拉取固定完整提交。
- 本仓库不依赖 LiDAR_DataACQ 源码、应用目录或设备型号；不包含第三方二进制。
- 核心只提供设备族的标准能力。保存格式、任务共用业务与族专用界面作为可选参考目标；三种具体编排、具体页面及型号处理仍由官邸实现。安装姿态等状态归设备实例。
- 默认仅 Qt Core/OpenMP；保留主线程取消、收包完成消费和线程 quit/wait 契约，不提供退出保存排空。

## 可选模块

| 开关（默认 OFF） | 目标 | 内容 |
|---|---|---|
| TASKCORE_WITH_TASK_SUPPORT | Lidar::TaskSupport | TaskBase，登记族对象、启动校验、状态和取消；自动带保存目标，无 QML |
| TASKCORE_WITH_QML | Lidar::TaskUI / Lidar.TaskUI | 可选族控件和任务参考页面；带 TaskSupport、UICore、ChartCore，要求 Qt 6.11.2 |

消费者必须在 FetchContent_MakeAvailable 前选择开关。核心目标不反向链接附加模块。界面消费者链接 LidarTaskUIplugin 并显式 `Q_IMPORT_QML_PLUGIN(LidarTaskUIPlugin)`（动态 Qt 下的静态模块也需要）；同时导入 UICore/ChartCore 插件。各模块输出目录的两级父目录用于 QML 工具 IMPORT_PATH。

TaskModule 接收两个 `list<QtObject>`（云台与采集卡设备本身），不认识具体设备页面。任务自己的速度、时间及保存设置在启动时形成快照，经族基 collectorFor 造请求；不读取设备页速度或累积范围。预览只调用族的 withinTravel/mounting。TaskBase 只登记族身份，锁/完成链同步取消；QObject 销毁通知移除登记，排队收尾保留。

这些目标均是可选实现，不要求任务官邸继承 TaskBase 或 TaskModule。三个具体计划和协程不在本库，官邸可以只使用核心能力自行编排。附加保存继续使用独立线程，不反向访问日志对象，不增加退出排空。

### 采集卡数据的保存

`AcqSaver` 只有两个入口，均为 detached 线程逐段写 JSON，官邸不参与：`saveFrame` 每帧写 `<保存根>/<会话>/<帧时间>.json`，`saveWindProfiles` 在同步点写 `profile_<同步时间>.json`。每通道写名称、`rangeStartM`/`rangeStepM`、`freqStartMHz`/`freqStepMHz`、零风速参考、功率谱（行 = 门）、径向风与 CNR；风廓线写 `heightStartM`/`heightStepM` 与逐层水平风向、水平风速、垂直风速。非有限值写 `null`。


验证：`saving_contract` 检查帧 JSON 与风廓线 JSON 的字段、通道顺序、null 缺测和风向换算；`processing_contract` 检查收包线程完成标准处理、主线程按通道发信号与四束反演。

## 组合与线程契约

2026-11-03 用户定：Handle 上传改为纯虚 `onData(const QByteArray &, RxTime)`。`Device<HandleT, CollectorT>` 同时继承具体 Handle 和纯 C++ DeviceBase，直接重写上传；删除 Handle 的上传回调参数/成员与 Device 的 handle_、makeDispatch。唯一 QObject 来自 Handle，具体 Core 末端以信号实现 lockChanged 并提供原只读属性；族层和官邸仍组合拥有 Core，onStream/onConsume 仍是原业务回调。本轮不改协议、收包线程、CAS、重试、取消或消费次序。

族模板以最终 Core 为参数，并从 Core::CollectorType 约束 GMB/DAQ 的 Collector 族归属。DeviceBase 不再登记为 QML 类型；只读属性与锁信号在具体 Core 末端提供，无第二个 QObject 或信号转发。

2026-10-26 用户定：GMB/DAQ 族改为组合持有 Device。GMBDeviceBase 继承 QObject，DAQDeviceBase 继承 QThread，只有官邸实现处理线程 run。Device 独持 onStream/onConsume 回调，删除 onCompleted 虚转发；完成帧仍在主线程先提交消费、再通知等待器。DAQWorker、各型号 QObject worker、sink_、独立 QThread 成员和 moveToThread 全部删除。

每个 DAQ run 在工作线程创建局部 QObject 锚点及唯一的型号处理 lambda，排队消息只携只读 Collector 份额，不复制有状态的 lambda。风廓线环及写位置随该闭包在线程内构造、使用和销毁；闭包对官邸只发结果信号，不访问其主线程业务成员。QML 在主线程接收，跨线程 C++ 使用者也必须指定接收上下文。启动用一次 QSemaphore 交接，回调安装后才允许首帧提交，不维护运行/关闭状态位。

末端 Core 析构首句执行 PreDestruction：同步停止 Handle，撤运行槽、释放锁，再销毁派生状态。非 final 传输类和 Device 模板不在析构中调用预析构。DAQ 仍先 quit/wait 处理线程、取消单卡协程，再销毁 Core。任务由完成回调和锁取消链结束；删除 retiring，QObject::destroyed 仅负责移除登记与虚拟卡借用，此时族对象仍存活。

族层的 device 属性以 QObject* 借出具体 Core 的元对象机制面，无状态缓存和逐方法转发。QML 名称与日志由该机制面提供，状态在资源完成阶段回读；安装姿态、信号、型号规格与 objectName 仍归官邸。任务标识与历史查询边界保持。TaskBase 不订阅设备状态判断任务结束；具体协程退出时排队调用 refreshRunState，待帧退出后回读 done 并通知任务页，销毁通知仅移除登记引用。锁、CAS、重试、取消、已完成帧保留和退出不排空政策不变。

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

`TASKCORE_WITH_PCAP_FORMS=ON` 提供 `Lidar::PcapForms` 与 `<PcapForm.h>`，共用枚举快照、连接参数校验，不保存第二份表单草稿。`connectionParameters()` 无参，仅交网卡选项、提示及默认下标；固定线路参数由型号在连接时交给 `parseParameters`，不再通过此接口交给 QML。需要在首次配置 HandleCore 前开启 `HANDLECORE_WITH_PCAP`，原生依赖要求沿用该库。默认关闭，核心和仿真官邸无需 libpcap。

原可选 `PcapCard` 已无使用方并删除。普通串口/网口连接区使用 UICore 的 `ConnectionSection`，只收拢选择项和双按钮布局；枚举、选择值、设备命令及日志仍由官邸提供。`PcapForm` 的枚举、校验和构参职责保持。

## 仿真参考页面

可选 TaskUI 另提供 `SimOnlyTaskModule` 和 `SimOnlyTaskRunSection`：接收单一 GMB/DAQ 族引用，持有累积/停稳设置，排列计划与运行分区。没有设备选择、速度、保存开关或目录草稿；具体官邸在启动时固定保存到当前工作目录。它与全功能 TaskModule 并列，不强制其他任务采用，也不把具体计划/协程放入核心。

FixedTask、CycleTask、SnakeTask 各提供自己的 simonly 末端页面，沿用各自既有计划和独立协程。设备退场和取消仍走原族引用接线；本次不改 C++ 核心、处理线程或保存退出策略。

simonly 页面的累积时间 `accum` 和停稳时间 `settle` 为创建时必填属性，不在页面完成后补写。具体任务官邸同时要求创建时提供角度计划，页面编辑和启动共用同一份值。运行分区只调用官邸无参 `start()` / `stop()`，不认识或转交计划参数；全功能 TaskModule 的接口保持原样。

2026-11-02 验证：Windows 主程序 Release、生命周期及三任务等 CTest 7/7 通过；补三种任务因设备关闭而取消后主动发 runChanged 的检查，生命周期复跑通过。Linux Qt Core 6.8.3 开启 TaskSupport/保存，公开头、处理线程和 CTest 3/3 通过；未测 Linux QML。依赖固定到本轮本地提交，尚未推送，本地验证显式供入对应源目录。


2026-11-03 本机验证：Linux Qt Core 6.8.3 核心构建/公开头与测试 2/2；Windows Qt 6.11.2 生产装配、15 组 qmllint、生命周期与独立任务回归 7/7 通过。具体 Core 仍由官邸唯一持有，处理线程和收包线程的先后关闭顺序保持。依赖固定本地提交，未推送；联调使用相邻源码覆盖。

### 2026-11-03 回调类型声明整理

处理线程验证直接写连接循环的 std::function 类型，不再依赖 HandleCore 的 Sink 别名；生产线程与消费链未改。Linux/GCC/Qt Core 6.8.3 增量构建及 CTest 2/2 通过（virtual-upload-20261103/build-TaskCore/callback-alias-{build,test}.log）。

## 2026-11-03 网口参数边界

2026-11-03 用户定：网口连接区仅保留网卡选择和连接/断开，删除 IP、端口等固定参数展示及对应 QML 参数字段；实际连接配置、校验、ARP 和收发关闭行为保持。

Windows 验证：通过主程序本地源码覆盖增量构建，五个网口官邸 qmllint 目标成功；仍有 QObject 属性类型的既有静态提示。八秒离屏冒烟通过，仅既有字体提示；未进行硬件操作或 Linux 验证。证据：LiDAR_DataACQ/build/build-network-ui.log、smoke-network-ui.json。

## 源码目录（2026-11-03）

公开文件直接放在 `include/`，引用时使用文件名，不再携带原单仓库的业务目录前缀。CMake 目标名称、C++ 类型和 QML 模块名保持不变；使用方须同步更新头文件路径及固定依赖版本。
可选模块仍各自独立，各模块的文件平铺在自己的 `include/` 中，不混入核心目标。

目录整理验证（2026-11-03）：Windows Qt 6.11.2 / MSVC：核心及可选模块构建、公开头独立编译、CTest 4/4 通过。本轮通过本地源码覆盖验证，固定依赖同步更新到本地提交；尚未推送，不代表已验证远端拉取。

## 2026-11-04：设备日志直接接线（用户定）

GMB/DAQ 的 QML device 属性由模板层直接交出具体 Core 的 QObject 指针，不再通过 DeviceBase 取得日志源。GMB 族基只声明 mounting/setMounting 契约，唯一标定值及验锁写入在模板层，失败直接调用 Handle.logMessage；删除按字符串查找日志信号。Qt 属性、成功通知及标定行为保持。 本轮仅本地整理，未推送；Windows 集成验证记录见主程序维护操作。

## 2026-11-04：设备身份与占用查询（用户定）

设备身份直接读 DeviceBase 的 displayName / subtitle 常量成员，任务自身名称契约保持不动。

本轮仅本地整理；集成验证记录见主程序 docs/maintenance/validation.md，不推送。

## 2026-11-04：进料在消费端取值（用户定）

GMBCollector::onFeed 接收 optional<PanTiltModel> 的 const 引用，在实际读取角度时取值。空输入由 DeviceCore 的公共 feed 拦截；到位、驻留、重试和消费语义不变。

本轮仅本地整理，不推送；集成验证见主程序维护记录。

2026-11-05 表单统一：转台状态改用 UICore ReadoutRow/Readout，运行控制采用宽标签预设，采集控制的行列间距采用 FormStyle。保留官邸数据、定位操作和既定分组。主程序 Windows 增量构建与八秒离屏冒烟通过；固定 UICore 本地提交，本轮不推送。


### 2026-11-05 公共视觉与连接布局

说明与小标题使用 UICore HintText/Subheading；删除无调用方的 PcapCard。任务计划、启动参数与业务逻辑保持。 本轮仅本地整理，未推送；Windows 集成验证记录见主程序维护操作。

### 2026-11-05 任务预览文字统一

TaskPlanView 的普通行距采用 FormStyle；运行信息用 InfoRow，拿锁范围按值列宽度换行，状态圆点仍由任务页负责。五列计划表的 Cell 复用 FormLabel 的省略和悬停规则。表格结构、当前束高亮、双区布局与运行小标题前留白不变，预览计算和任务业务未改。

主程序 Windows 增量构建及八秒冒烟通过；本地固定配套 UICore，未推送。

2026-11-05 输入宽度整理：DAQControl 删除累积输入的局部 preferredWidth=80，由 FieldRow 分配空间；国耀独立“下发”按钮、输入校验与发令逻辑保持。同步 UICore 本地依赖；主程序 Windows 增量构建及八秒冒烟通过，未推送。
## 保存目录显示（2026-11-05）

单卡控制与任务落盘区只向 PathField 交 URL，不再把 URL 字符串绑定为显示文字；本机路径转换由 UICore 负责，业务目录值与提交行为不变。

验证：2026-11-05 Windows Qt 6.11.2 / MSVC 增量构建通过；UICore CTest 3/3、ChartCore nonuniform 1/1、主程序八秒桌面冒烟及四窗装配通过。装配检查覆盖三规格、三通道的18张初始风速/CNR图的内部及分段门位悬停。未连接硬件、未推送。

## 编辑完成提交（2026-11-05 用户定）

DAQControl 删除“下发”按钮；sendParameters 型号在每次合法编辑完成后提交累积时间，包括相同值再次提交，其余型号仅更新草稿。GimbalSection 的四个定位输入放在对应标题和实测读数下方；编辑完成提交同坐标系的两轴，世界角使用当前安装姿态转换。界面忽略返回值，不附加成功/失败处理；设备层原有检查、日志保持。回报只更新读数，不覆盖草稿或触发命令。安装标定和立即停止保留。

验证：Windows Release 增量构建、真实QML面板模拟后端验证、四窗口装配及八秒桌面冒烟通过。覆盖无初始化/回报误发送、四角提交与换算、同值累积重复提交和非下发型号。运行stderr为空；未连接硬件，未推送。


### 2026-11-05 连接间距、累积默认值与径向风页面整理

三个桌面任务共用的 accumDefault 改为1秒；显式启动快照、simonly 外部参数和独立协程保持。

主程序 Windows Release 增量构建、八秒桌面冒烟及四窗口装配检查通过；运行 stderr 为空，保留既有 CDL5/Qt 生成代码 C4702 编译警告。仅本地验证，未连接硬件、未推送。

## 读数与输入共列（2026-11-05 用户定）

GimbalSection 的四角定位复用 UICore ReadoutField，将标题、实测读数和输入草稿排为三行；删除本层对这一布局的重复实现。安装标定仍为仅设定项；编辑完成发令、坐标转换、读数和草稿独立、停止动作及组间距均保持。Windows主程序增量构建与模拟面板提交检查通过，未连接硬件、未推送。

## 表单分隔与提示统一（2026-11-05 用户定）

TaskPlanView 的状态行复用 UICore FieldRow，统一名称列与列间距，保留原标签颜色、状态圆点、阶段文字及运行分组留白；业务逻辑不变。

验证：Windows 主程序 Release 增量构建、四窗装配和八秒桌面冒烟通过，运行 stderr 为空；UICore CTest 3/3通过。未连接硬件，未推送。
