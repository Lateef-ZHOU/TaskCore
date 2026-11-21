# FamilyCore

设备族库：GMB 云台族与 DAQ 采集卡族。11-14 及更早的历史条目见[归档](docs/archive/README-2026-11-19.md)。

## 云台逐轴到位容差

云台收集器允许型号分别指定水平和俯仰容差，两轴都达标才累计原有停稳时间。沿用单值的型号仍对两轴使用同一容差。

## 云台世界角回显

实时回显通过 `worldForReadout` 选择世界角表示，默认复用 `worldOf`。铭辰保留越顶分支，任务读到的世界指向、标定和反演仍使用原换算。

## 2026-11-21：容器迁入绘图库（用户定）

映射改名 `LinearMapping`（线性）与 `NonLinearMapping`（非线性；逐点表只要求有限且单调，允许相等）；非单调的逐点值（如分段风速）用 `TableMapping`，不做单调校验，轴仍用 `NonLinearMapping`。二维谱、一维数组与带时间的剖面帧改由绘图库提供：`Grid2D<RowAxis, ColumnAxis>`、`Array1D<Axis>`（径向风与 CNR 合并为同一容器）、`TimedArray1D<Axis, T>`（墙钟时间 + 运行时缩放；剖面缩放为 sin 仰角）。`DAQChannel<Range, Frequency>` 分持原谱 `spectrum`（落盘）与去噪谱 `denoised`（绘图与风速/CNR 拟合，`displaySpectrum()` 取用）；DAQ 侧别名 `WindProfileFrame<Range> = TimedArray1D<Range, std::optional<LinAlg::Vec3>>`。FamilyCore 的 DAQ 头包含 ChartCore 的 `AxisMapping.h`/`DataArrays.h`，ChartCore 不再包含 `DAQFamily/`；库目标不编译 DAQ 头，测试经 `Lidar::ChartData` 接口目标取头。

## 2026-11-20：帧直接携带映射，删除 DAQParams（用户定）

距离轴与频率轴都是映射类型，帧模板直接以它们作参数；`DAQParams` 与帧上的 `params` 已删除。

- 均匀：`UniformMapping<Start, Step, Count>`，步长是权威值，覆盖终点 = Start + Step × Count。
- 非均匀：`NonUniformMapping<Edges>` 的模板实参是一个 constexpr 函数，表由它生成、映射内部静态持有 `std::array<double, N>`；类型名只带函数符号，421 点表也不会撑爆 QML 元类型签名（按值把 421 个边界写进类型曾触发 C2131）。
- 频率轴只要求是映射（均匀或非均匀）；风速换算由官邸交入的「下标 → 风速」映射承担，`RadialWindCalculation::Calc(spectrum, wind)` 按分数下标在相邻点间插值——族基不认识波长与零风速参考。
- `MappingLike` 直接按公共接口识别映射（`uniform`/`count`/`stop`/`at`，非均匀另带 `edges`），不设封闭名单、不前置声明；帧与算法直接写 `template<MappingLike Range, ...>`。轴的有效性（点数正、坐标有限且严格单调，递增或递减）由映射自己 `static_assert`；绘图等按下标递增定位的消费者另行要求递增。
- 帧：`Spectrum<Range, Frequency>`（两映射）、`RadialWind<Range>`、`CNR<Range>`、`VerticalWindProfileUnit<Range>`、`DAQChannel<Range, Frequency>`；数组尺寸与循环上界取自映射的 `count`。

风廓线高度 = 距离点坐标 × sin(运行时仰角)，按实际相邻层插值。

均匀通道的 JSON 字段保持。非均匀采集数据使用 `rangeM` 和 `rangeStopM`，非均匀风廓线使用 `heightM` 和 `heightStopM`。各坐标表分别对应当前文件的每个距离门或高度层，覆盖终点单独保存。Windows Release：FamilyCore CTest 16/16、ChartCore 4/4、四个采集卡官邸 CTest 通过，主程序整编零警告、8 秒离屏冒烟与基线一致。

## 2026-11-20：本机俯仰保留两个命令端点（用户定）

`PanTiltModel` 保留 −90° 和 270° 两个本机俯仰输入值，角距离仍按整圈计算。定位表单允许型号给出起步草稿，其他型号仍使用各自的下限。世界坐标的反解分支保持。

## 2026-11-20：采集保存根目录使用标准路径（用户定）

单卡测试与任务在界面入口把本地目录转换为 `std::filesystem::path`。采集请求的 `saveRoot` 使用 `std::optional<std::filesystem::path>`，`std::nullopt` 表示不落盘。保存层在拼接功率谱和风廓线文件路径时转换为 Qt 路径。

Windows Release CTest 16/16。落盘测试核对带中文、空格及特殊字符的根目录和会话目录，并读回功率谱及风廓线 JSON。

## 2026-11-20：采集会话目录使用标准字符串（用户定）

任务及单卡测试按原日期格式和设备标签生成 UTF-8 的 `std::string`。采集参数按值携带 `sessionDir`，保存时用同一个 Qt 字符串写文件路径和 JSON 的 `frame.session`。

## 2026-11-20：DAQ 收包链统一只读 span（用户定）

`DAQCollector::Feed` 为 `std::span<const unsigned char>`。DAQ 的 `onStream` 原样交给 `feed` 和型号 `assemble`，输入只在收包线程的同步调用中有效。CDL5、HFNL 和国耀把需跨包保留的字节复制进各自的收帧缓冲，仿真卡直接解析整帧。完成后上报的是解码后的结果份额。

## 2026-11-20：采集通道规格按值进入模板

官邸定义映射类型与帧别名，功率谱、径向风、CNR 和风廓线以映射类型实例化。网格约束仍在编译期检查，运行帧不重复保存轴配置。通道名由官邸提供，属于采集结果的元数据。风廓线保存按原通道下标取名称，跳过缺测通道时仍保留其余通道的正确名称。

## 2026-11-20：测试移除 LinearArray 依赖

family_algorithms 删除一维等间隔轴的行为测试；spectrum 删除对 `LinearArray2D` 的反例断言。Algorithm 的 `LinearArray.h` 已整体删除，功率谱轴配置由 ChartCore 的 `SurfaceGrid` 承担。

验证：FamilyCore build-family Release 零警告，CTest 16/16。

## 2026-11-19：GMB 解码链统一只读字节视图（用户定）

GMBDevice::onStream 不再构造 QByteArray，decodeStream 及铭辰上行、Pelco 成帧校验和仿真协议解码统一接收 std::span<const unsigned char>。Pelco 接收缓存改用 unsigned char 数组；视图仅在同步解码期间借用，排队到主线程的仍只有解析后的位形与 RxTime。发送侧协议词汇、DAQ 的 assemble 字节接口及 HandleCore、DeviceCore 不在本轮改动范围。

验证：FamilyCore CTest 16/16；主程序 CTest 8/8；simonly Release 构建零警告、CTest 11/11、QML lint 返回 0。仅本地提交，不推送，不更新远端固定版本。

## 2026-11-19：官邸直接实现收集器命令虚函数（用户定）

GMBCollector 改为 protected 继承 GMBParameters，官邸直接读取目标、停稳等待和两轴速度并实现 buildStart/buildStop/timeoutMs；族层删除 start_/stop_/timeoutMs_ 缓存及默认实现，只保留公共到位/驻留逻辑。GMBParameters 不再声明 final，以允许该受保护继承，但其受保护构造、只读字段和 paramsFor 唯一入口保持，仍不作为自由派生的多态参数类型。铭辰、亚安、仿真官邸和测试收集器同步直接实现三个虚函数。

验证：build-family Release 零警告，CTest 16/16；主程序原 build Release 零警告，CTest 8/8；simonly Release 构建零警告，CTest 11/11，QML lint 返回 0。仅本地提交，不推送，不更新远端固定版本。

## 2026-11-19：云台请求强制携带两轴速度（用户定）

删除 GMBParameters::Speeds 及 optional 速度状态，GMBParameters 直接以 const double 保存 panSpeed 和 tiltSpeed，构造时必须成组提供。GMBFamily::paramsFor 与三个任务协程同步改为分别接收两轴速度；铭辰、亚安和仿真官邸按新字段组包或保留快照，亚安协议不编码速度但请求仍携带速度。删除“速度可缺席”的测试分支，非有限速度仍按型号规则拒绝。

验证：build-family Release 零警告，CTest 16/16；主程序原 build Release 零警告，CTest 8/8，8 秒离屏冒烟与基线一致。simonly 使用相邻源码重新配置，Release 零警告，CTest 11/11，QML lint 返回 0；simonly 三个任务入口使用型号默认两轴速度，零速度不再作为“无速度”占位。仅本地提交，不推送，不更新远端固定版本。

## 2026-11-19：具体云台收集器归官邸（用户定）

删除 TypedGMBCollector 模板适配层，铭辰、亚安和仿真云台官邸直接定义各自的 final Collector 子类，按已校验的 GMBParameters 构造并交给 Device。FamilyCore 只保留 GMBCollector 的公共到位/驻留逻辑和 GMBCollectorLike 参数构造约束，测试收集器同步改为具体子类；启动/停止命令、容差、超时及到位判定行为不变。

验证：build-family Release 零警告，CTest 16/16；主程序原 build Release 零警告，CTest 8/8，8 秒离屏冒烟与基线一致。仅本地提交，不推送，不更新远端固定版本。

## 2026-11-19：无 run 停机命令归具体官邸（用户定）

GMBFamily 不增加停机接口，GMBDevice 不再经 CollectorT::stopOf() 代实现 stopCommands。铭辰、亚安和仿真云台官邸直接实现 DeviceCore 的 stopCommands；TypedGMBCollector 删除静态 stopOf，GMBCollectorLike 同步收紧为只要求参数构造。若有当前 run，停机仍使用该 Collector 实例的 buildStop；无 run 时由官邸提供本型号停机命令。

验证：build-family Release 零警告，CTest 16/16；主程序原 build Release 零警告，8 秒离屏冒烟与基线一致。主程序测试补齐 Qt、libserialport 和 Npcap 路径后全部通过。仅本地提交，不推送，不更新远端固定版本。

## 2026-11-19：GMB 请求值类型禁止派生（用户定）

GMBParameters 显式声明 final，明确它是协程与族层之间按值传递的最终请求类型，型号知识不通过参数派生类扩散。Collector<PanTiltModel, GMBParameters> 继续用于 DeviceCore 的 Feed/Params 契约，运行、验锁和收集行为不变。

验证：build-family Release 零警告，CTest 16/16；主程序原 build Release 零警告，覆盖实际云台消费链，8 秒离屏冒烟与基线一致。仅本地提交，不推送，不更新远端固定版本。

## 2026-11-19：静态轴向配置与作动值数组（用户定）

删除 Joint 类型及元组关节存储，GimbalModel 直接以私有 std::array<double, N> 保存运行时作动值；固定轴向由 std::array<double, 3> 值模板参数提供，不随模型实例复制。WorldModel 和 PanTiltModel 的高频复制仅携带 3 个或 2 个 double，角度窗口、正反解和类型约束保持。

验证：build-family Release 零警告，CTest 16/16；build-models CTest 1/1；主程序原 build Release 零警告，8 秒离屏冒烟与基线一致。仅本地提交，不推送，不更新远端固定版本。

## 2026-11-19：关节存储改为定长数组（用户定）

GimbalModel 按轴数模板化为 GimbalModel<N>，私有存储由 std::vector<Joint> 改为 std::array<Joint, N>；WorldModel 固定 3 轴，PanTiltModel 固定 2 轴。GimbalModelBase 仅作无数据类型标记，保留 GimbalModelLike 约束；轴向、角度窗口、正反解和私有只读轴访问不变。复制/移动按元素完成，不再使用 vector 的存储地址转移语义。

验证：build-family Release 零警告，CTest 16/16；build-models CTest 1/1；主程序原 build Release 零警告，8 秒离屏冒烟与基线一致。仅本地提交，不推送，不更新远端固定版本。

## 2026-11-19：高度与俯仰采用连续越顶窗口（用户定）

WorldModel 的 elevation 与 PanTiltModel 的 tilt 统一以 Wrap270 归一化到 [-90°, 270°)，保留水平以下及越过天顶的连续表示；roll 继续以 Wrap180 归一化到 [-180°, 180°)。正逆解公式、轴向配置与代表解策略不变，190° 保留原值，270° 折回 −90°。

验证：build-family Release 零警告，CTest 16/16；build-models CTest 1/1；主程序原 build Release 零警告，8 秒离屏冒烟与基线一致。仅本地提交，不推送，不更新远端固定版本。

## 2026-11-19：Joint 与模型角度窗口分离（用户定）

Joint 不再接收或保存 offset，只描述轴向与当前作动值；WorldModel 以 Wrap360 归一化方位、以 Wrap180 归一化高度和翻滚，PanTiltModel 同样自行归一化两轴。正解直接使用周期角，反解继续通过 jointBasis 处理既有轴向，不增加机械零偏。两种模型的零位正解均为单位朝向，即零俯仰、零翻滚对应水平。

验证：build-family Release 零警告，CTest 16/16；build-models CTest 1/1；主程序原 build Release 零警告，8 秒离屏冒烟与基线一致。仅本地提交，不推送，不更新远端固定版本。

## 2026-11-19：FamilyBase 移除 Qt 宏依赖（用户定）

FamilyBase 删除未使用的 QString 包含，复制与移动禁用改用 Algorithm/ObjectPolicy.h 的 DISABLE_COPY_MOVE，语义与 Q_DISABLE_COPY_MOVE 一致；族接口、begin/stop 和运行行为不变。验证：build-family Release 零警告，CTest 16/16；主程序原 build Release 零警告，8 秒离屏冒烟与基线一致。仅本地提交，不推送，不更新远端固定版本。

## 2026-11-17：标准字符串锁请求（用户定）

DAQ 测试采集直接以 UTF-8 名称构造 LockRequest，TaskBase 在创建请求时将现有任务显示名转换为 std::string；设备返回 optional<std::string> 占用者名称，Qt 属性适配仍由既有 qmlHolder 负责。任务名称接口、锁身份与取消链保持。验证见主程序维护操作；仅本地，不推送。

## 2026-11-17：onStream 接收只读字节视图（用户定）

Device::onStream 的输入统一为 std::span<const unsigned char>，与 onData 一致；onData 直接转交原视图，删除 Device 层的 QByteArray 适配及包含。返回类型仍为 optional<CollectorT::Feed>，判空、同步进料、完成 CAS 和主线程消费顺序保持。

DAQ 的 Collector 进料与 GMB 的 decodeStream 尚使用 Qt 字节接口，仅在对应族入口以 fromRawData 同步借用，不复制字节、不延长输入寿命；型号跨包留存仍须写入自有收集区，GMB 排队只携带解析后的位形和时刻。本轮不迁移 Collector::Feed、assemble 或 decodeStream 的签名，不新增缓存或转发。验证见主程序 docs/maintenance/validation.md；仅本地修改，不推送。

## 2026-11-17：GMB 命令集合接入 vector（用户定）

GMBCollector 的构造参数、原有 start_/stop_ 缓存和 buildStart/buildStop、型号 startOf/stopOf 统一为 std::vector<std::vector<unsigned char>>；GMBDevice::stopCommands 与 Collector 类型约束同步适配。缓存保持不可变，构造时移动接收，值返回复制缓存；调用方修改返回集合不会影响后续启动、重试或撤单。未增加状态，也未改变运动判齐、驻留、重试或取消。

验证见主程序 docs/maintenance/validation.md；本轮使用相邻源码覆盖，不改远端依赖锁，不推送。

## 2026-11-17：网口表单校验范围（用户定）

PcapForm 仅检查网卡下标、地址文本和端口，本层解析失败不会触碰当前连接或重连种子。读取超时和内核缓冲大小由 PcapHandle::connOpen 在打开资源前统一校验，表单不重复检查；后续异步打开失败不属于本层解析失败。本轮只纠正注释，构参与业务行为保持；验证见主程序维护操作，暂不推送。

## 2026-11-17：网口连接参数收紧（用户定）

PcapForm 从同一枚举项提取 pcapDeviceName 与 hostMAC 构造 PcapParams，不再传入整个 NetworkInterfaceInfo。名称/描述继续供 QML 选项使用，枚举缓存、下标/地址/端口校验、型号读取策略和先验参后连接保持。验证见主程序维护操作；相邻源码覆盖，远端锁定版本不变，不推送。

## 2026-11-17：onTX 接入 vector 只读引用（用户定）

处理测试的 onTX 输入改为 const std::vector<unsigned char>&。仅适配 DAQ 处理测试的传输替身；生产族接口、算法和共享份额传输不变。 不新增成员状态；使用相邻本地 HandleCore 源码覆盖，远端锁定版本暂不变，不推送。Windows build-family Release 与 CTest 16/16 通过，包含实际 DAQ 处理链。 证据见主程序 docs/maintenance/validation.md 本轮记录。

## 2026-11-17：网口表单接入标准地址值（用户定）

PcapForm 改用 PacketProtocol 的 MAC/IPv4 地址和 optional 解析结果，QString 只在表单边界转为 std::string。网卡快照、下标/端口校验、先验参后连接保持；不再接受 IPv4 前导零/空白或混用 MAC 分隔符等旧宽松写法。未新增设置副本或日志转发。

主程序实际消费者的 Release 构建及 8 秒离屏冒烟通过，仍使用本地源码覆盖，未变更远端锁定提交、不推送。证据见主程序 docs/maintenance/validation.md 本轮记录。

## 2026-11-15：网卡列表接入 vector（用户定）

PcapForm 的枚举缓存改为 const std::vector<NetworkInterfaceInfo>，空列表仍给 QML 默认下标 -1；先拒绝负下标，再按 size_t 比较和索引。UI 选项仍只在边界转换，保持首次枚举、下标与线路参数校验。

验证：PcapForm 的实际消费方随主程序 Release 增量构建通过，8 秒离屏冒烟与既有 stderr 基线一致。证据见主程序 build/pcap-vector-LiDAR_DataACQ-build.log、pcap-vector-smoke.json；不推送。


## 2026-11-15：网卡表单字符串边界（用户定）

PcapForm 消费 NetworkInterfaceInfo 的 std::string 名称与描述，只在构造 QML 选项时转为 QString。网卡标识与连接参数快照保留标准字符串；一次枚举、下标校验和线路参数校验保持。

验证：共享 PcapForm 的实际消费方随主程序 Release 增量构建通过；主程序及激光、温控三个配置的 qmllint 目标返回 0，8 秒离屏冒烟与原 stderr 基线一致。本轮未重跑与表单无关的族算法专项。证据见主程序 build/pcap-string-*。


---

## 边界与依赖

- Qt ≥ 6.8.3；Qt 由使用环境提供，不在仓库中打包。
- 上游：[Algorithm](https://github.com/Lateef-ZHOU/Algorithm), [DeviceCore](https://github.com/Lateef-ZHOU/DeviceCore)。已有同名 CMake 目标时复用；否则 FetchContent 拉取固定完整提交。
- 本仓库不依赖 LiDAR_DataACQ 源码、应用目录或设备型号；不包含第三方二进制。
- 核心提供设备族标准能力、通道处理及 AcqSaver 统一保存；协程由 DeviceCore 提供；任务共用业务与族专用界面作为可选参考目标。三种具体编排、具体页面及协议凑帧由官邸实现。安装姿态等状态归设备实例。
- 默认仅 Qt Core/OpenMP；保留主线程取消、收包完成消费和 Handle 同步关闭契约，不提供退出保存排空。

## 可选模块

| 开关（默认 OFF） | 目标 | 内容 |
|---|---|---|
| FAMILYCORE_WITH_TASK_SUPPORT | Lidar::TaskSupport | TaskBase，登记族对象、启动校验、状态和取消，无 QML |
| FAMILYCORE_WITH_QML | Lidar::TaskUI / Lidar.TaskUI | 可选族控件和任务参考页面；带 TaskSupport、UICore、ChartCore，要求 Qt 6.11.2 |

消费者必须在 FetchContent_MakeAvailable 前选择开关。核心目标不反向链接附加模块。界面消费者链接 LidarTaskUIplugin 并显式 `Q_IMPORT_QML_PLUGIN(LidarTaskUIPlugin)`（动态 Qt 下的静态模块也需要）；同时导入 UICore/ChartCore 插件。各模块输出目录的两级父目录用于 QML 工具 IMPORT_PATH。

TaskModule 接收两个 `list<QtObject>`（云台与采集卡设备本身），不认识具体设备页面。任务自己的速度、时间及保存设置在启动时形成快照，经官邸实现的族接口 paramsFor 造请求；不读取设备页速度或累积范围。预览只调用族的 withinTravel/mounting。任一参与设备被外部促停，整项协程通过锁/完成链取消；TaskBase 的 QObject 销毁通知同样取消任务并移除登记，排队收尾保留。

这些目标均是可选实现，不要求任务官邸继承 TaskBase 或 TaskModule。三个具体计划和协程不在本库，官邸可以只使用核心能力自行编排。核心中的保存继续使用独立线程，不反向访问日志对象，不增加退出排空。

### 采集卡数据的保存

`AcqSaver` 只有两个入口，均为 detached 线程逐段写 JSON，官邸不参与：`saveFrame` 每帧写 `<保存根>/<会话>/<帧时间>.json`，`saveWindProfiles` 在同步点写 `profile_<同步时间>.json`。每通道写名称、`startRange`/`rangeStep`、`startFreq`/`freqStep`、`startWind`/`windStep`（非线性映射改写 `ranges`/`freqs`/`winds` 逐点数组）、功率谱（行 = 门）、径向风与 CNR；风廓线写 `startHeight`/`heightStep`（非线性写 `heights`）与逐层水平风向、水平风速、垂直风速。非有限值写 `null`。


`saveFrame` 只接收 `std::shared_ptr<const C>`，C 满足 DAQCollectorLike；完成帧从消费链到保存线程始终使用只读份额，不再兼容可写份额的模板推导或在入口转换类型。DAQDevice 直接提交帧与风廓线，由 AcqSaver 的两个入口统一检查 saveRoot；为空时返回，不启动写盘线程。保存格式、独立线程及退出不排空策略不变。

验证：`saving_contract` 编译期检查只读入口，运行时检查帧 JSON 与风廓线 JSON 的字段、通道顺序、null 缺测和风向换算；`processing_contract` 检查收包线程完成标准处理、主线程按通道发信号与四束反演。

## 继承、请求与线程契约

GMBFamily / DAQFamily 是纯 C++ 族接口，继承 FamilyBase<Request, CollectorBase>。FamilyDevice 同时继承具体 Device 与族接口，统一实现 device() / begin() / stop()；GMBDevice / DAQDevice 实现各族业务，具体官邸为 final Qt 末端。唯一 QObject 来自 Handle，不再另持 Core，也没有 onStream/onConsume 回调成员。

官邸 paramsFor 校验并返回不可变的 GMBParameters / DAQParameters；协程按设备保存请求，经 begin 交 Device::start。Device 首次启动与看门狗重试都通过 const Params& 构造本型号 Collector，不保留轻量原型或 makeSelf 路径。完成结果以只读族基份额交等待器，具体消费仍保留型号类型。

RunAwaiter 成功汇总时，将设备引用、按值请求参数和只读 Collector 份额复制到结果槽，再移动交给协程；不复制 Collector 内的采集数组。DAQDevice 显式包含族接口与共同实现所在的 FamilyBase.h，不借用等待器的间接包含。

DAQCollector 只提供参数、帧时刻及收集契约；TypedDAQCollector<Channels...> 持有 optional<tuple<Channels...>>。型号 assemble 在收包线程凑齐字节并解析通道，标准 DAQChannel<Spec> 构造时计算径向风/CNR，处理完成后才判齐及 CAS 摘槽。主线程 onConsume 按通道发信号、提交 detached 保存、维护最近四束环，并在同步点反演风廓线；没有独立 DAQ 处理线程、锚点或启动交接。

末端析构首句执行 PreDestruction；DAQ 先取消本层单卡协程，再接续 Device 同步关闭 Handle、撤运行槽及锁，保护尚在的派生成员。中间层析构不重复调用预析构。处理期间占住收包循环，耗时计入看门狗等待，关闭也要等当前处理结束；大帧、多卡场景的耗时余量尚需实测。

TaskBase 登记族身份，任务结束依靠完成回调与锁取消链，QObject::destroyed 还会取消整项任务并移除登记；不在设备退出后切换到另一设备继续编排。QML 借用末端设备本身的身份、日志和状态；任务状态仍在协程结束后的排队收尾中回读。普通取消保留已完成帧，退出不新增保存排空。

单卡 startTest 在 paramsFor 拒绝无效输入后，向私有 runTest 协程按值交入 DAQParameters；协程内不再保存必定有效的 optional。GMBCollector 的目标直接读取 parameters.target，measured() 只返回实测位形。连续驻留的首末时刻用一个 optional<pair> 成组保存：未进圈或出圈即为空，进圈同时创建首末时刻，后续圈内样本推进末时刻；停止回报不会自行完成，重试按参数新建实例。

GMBCollector 的容差、启动命令、停止命令和看门狗时限均为 private const 配置，型号只在构造时交入，不直接访问成员。驻留时长直接读取 parameters.hold，settled 以 max(parameters.hold, kMinHold) 保留最小驻留约束，不另存 hold_。公共收集接口和到位、驻留、重试语义保持；`gmb_collector_contract` 验证这些行为。

`measured()` 仅允许普通或 const 左值调用，返回 `const std::optional<PanTiltModel>&`；两种右值调用均拒绝。借用不保活 Collector，调用方须在整个读取期间保有对象或其共享份额；收集期间不得与 feed 并发读取，完成交付后结果冻结。任务在结果份额保活范围内以 const 引用立即换算世界朝向，不复制关节数组。测试检查返回类型、左右值调用约束、原观察槽借用及完成后的冻结行为。

## 独立构建和验证

安装对应 Qt 开发包，然后执行（将路径替换为本机 Qt 前缀）：

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/Qt/6.8.3/platform -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel 2
ctest --test-dir build -C Release --output-on-failure
```

Windows 在安装 MSVC 的环境运行以上命令，Qt 选择对应 MSVC 构建；Linux 使用 GCC 与 OpenMP 开发环境。
测试逐个独立编译公开头、拒绝意外加载 GUI/QML，并运行契约检查。processing_contract 通过实际 Handle/Device 交付四个方位的四帧，检查收包线程处理、主线程发信号、同步反演及单卡参数拒绝/连续采集/取消。gmb_collector_contract 使用当前参数构造接口与真实单调钟（约 4.2 秒），覆盖新重试、出圈再入圈、停止回报、驻留下限和完成冻结；从主程序旧 Collector 测试迁入，不保留原型工厂接口。GitHub Actions 在 Windows/Linux 使用 Qt 6.11.2，分别验证核心、无界面任务业务及可选 QML 组件；最低版本声明与最新验证版本是不同概念。

## 接入应用

```cmake
include(FetchContent)
FetchContent_Declare(lidar_familycore
    GIT_REPOSITORY https://github.com/Lateef-ZHOU/TaskCore.git
    GIT_TAG <经过验证的完整提交号>)
FetchContent_MakeAvailable(lidar_familycore)
target_link_libraries(my_app PRIVATE Lidar::FamilyCore)
```

从仓库提交列表选择固定版本；示例占位值必须替换，不使用浮动分支保证可复现。多个模块共用已有上游目标，应用若需覆盖上游版本应先统一声明并创建它们。标准 `FETCHCONTENT_SOURCE_DIR_LIDAR_模块名`（模块名大写无分隔）可用于本地开发，发布验证应取消覆盖并实际拉取远端。

## 来源与许可

从 [LiDAR_DataACQ](https://github.com/Lateef-ZHOU/LiDAR_DataACQ) 的模块分离提交 `94ad6f8` 提取；初始分仓保持运行逻辑，独立整理构建、验证与发布入口。可选保存、任务业务和界面从应用 `f20e890` 的对应实现迁入；具体任务编排由各自独立官邸仓库提供。

自有代码采用 [MIT](LICENSE)，Copyright (c) 2026 周涛 (ZHOU TAO)。第三方依赖不随本仓库改为 MIT，参见 [依赖说明](THIRD_PARTY_NOTICES.md)。

