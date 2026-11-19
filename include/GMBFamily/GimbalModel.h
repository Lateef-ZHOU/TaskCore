#pragma once
#include "ObjectPolicy.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <concepts>
#include <numbers>
#include <tuple>
#include <utility>
#include <vector>
#include <memory>

#include "SO3.h" // 朝向对象 + 流形工具；角量单位契约也写在那里

// GimbalModel.h — 云台的**位形模型**一族：Joint（关节）→ GimbalModel（关节链 + 正解）→
// WorldModel{az,el,roll} / PanTiltModel{pan,tilt}（各自的反解）。
// SO3 是"朝向本身"，本文件是"哪种装配、作动到哪儿会得到那个朝向"——两层，上层包含下层。
// 模型实例即一个位形（谈位形不谈指向）；角量一律是度。

// 角度回绕到 [0, 360)（度）：供模型归一化及反解收尾使用。
[[nodiscard]] inline double Wrap360(double deg) noexcept
{
    deg = std::fmod(deg, 360.0);
    return deg < 0.0 ? deg + 360.0 : deg;
}

// 角度回绕到 [-180, 180)（度）：供以水平为零的模型角使用。
[[nodiscard]] inline double Wrap180(double deg) noexcept
{
    return Wrap360(deg + 180.0) - 180.0;
}

// 角度回绕到 [-90, 270)（度）：供世界模型的高度角使用，连续范围覆盖水平以下及越过天顶的分支。
[[nodiscard]] inline double Wrap270(double deg) noexcept
{
    return Wrap360(deg + 90.0) - 90.0;
}

// 最短无向角距离（度），范围 [0, 180]。
[[nodiscard]] inline double AngularDistance(double a, double b) noexcept
{
    const double d = std::fmod(std::abs(a - b), 360.0);
    return d > 180.0 ? 360.0 - d : d;
}

// 固定的关节轴向类型标签：只提供编译期常量，模型实例不再保存轴向或指针。
namespace GimbalModelAxes
{
struct MinusZ { static constexpr LinAlg::Vec3 value{0.0, 0.0, -1.0}; };
struct X { static constexpr LinAlg::Vec3 value{1.0, 0.0, 0.0}; };
struct MinusY { static constexpr LinAlg::Vec3 value{0.0, -1.0, 0.0}; };
} // namespace GimbalModelAxes

// Joint — 一根关节：轴向由类型标签指定，实体只保存当前作动值（°）。
// 关节只描述轴向和当前值，不拥有角度窗口或机械零偏；作动值是周期量，正解直接使用。
// 读数窗口由具体模型在构造时统一归一化，设备行程由官邸按协议校验。
template <class AxisTag>
class Joint
{
public:
    // 作动值（任意实数）。轴向由模板参数给出，不在对象内另存。
    constexpr explicit Joint(const double deg) noexcept : value_(deg) {}

    [[nodiscard]] static constexpr const LinAlg::Vec3 &axis() noexcept { return AxisTag::value; }
    // 本轴此刻的作动值；Joint 本身不做窗口折返。
    [[nodiscard]] constexpr double value() const noexcept { return value_; }

private:
    double value_;
};

// 无数据的模型标记，仅供 GimbalModelLike 识别本族；不公开基类析构或容器入口。
class GimbalModelBase
{
public:
    GimbalModelBase() = default;
    GimbalModelBase(const GimbalModelBase &) = default;
    GimbalModelBase &operator=(const GimbalModelBase &) = default;
    GimbalModelBase(GimbalModelBase &&) = default;
    GimbalModelBase &operator=(GimbalModelBase &&) = default;
protected:
    ~GimbalModelBase() = default;
};

// 完整模型接口：反解返回自身类型；不证明有解、唯一或正反解互逆。
// 仅在完整类型的使用处检查，不能在派生类定义完成前求值。
template <class T>
concept GimbalModelLike = std::derived_from<T, GimbalModelBase> && requires(const SO3 &orientation)
{
    { T::fromOrientation(orientation) } -> std::same_as<T>;
};

// ============================================================
// GimbalModel — 一台云台的**一个位形**（关节列表，每根关节自带当前值）：位形 ↔ 朝向
//
// 定朝向需「轴序 / 内外旋 / 符号」约定——本质是把每根关节的（带符号）单位轴向按复合顺序排成一串。
// 关节列表就是模型的核心数据，轴数即表长；每个 Joint 保存当前作动值，具体模型在构造时归一化。指数积(PoE)/旋量：
//     R(θ₁,…,θ_N) = ∏ exp(θ_i[û_i]×)   （内旋=动轴=正序连乘）
//
// 正向 toOrientation 通用且简单，收在基类。**反向（朝向 → 位形）是真逆运动学**，通用解复杂度极高、
// 可能多解 / 无解（存在性看雅可比的秩、唯一性看冗余自由度），故基类不提供，由每个具体模型自写
// 静态口 `fromOrientation`：WorldModel 定义三轴反解，PanTiltModel 因前两轴相同而复用其结果。
// 关节列表构造与析构为 protected，复制和移动构造为 public；外部不能直接创建并销毁基类值。
// 一袋任意关节没有已知反解，不算一台完整模型。
//
// 私有继承 std::tuple 仅用于保存各轴作动值；轴向由模板给出，不随模型实例复制。
// axis() 仅供派生模型只读访问；整体赋值使用具体模型类型。
// 调用方不得经 GimbalModel 引用或指针对基类子对象赋值，以免破坏具体模型的轴数、轴序与固定轴配置。
// 轴数由模板实参固定；移动按元素进行，移出后的对象仅用于析构或重新赋值，恢复前不读角度、不做正解或距离计算。
//
// 「两个模型间换算」（世界 ↔ 云台）**不做成成员**——它是两台模型 + 一个安装朝向的关系，不属于
// 任一模型；就是几行 SO3 复合（`toOrientation` / `inverseRotation` / `fromOrientation`），住在持有安装标定的
// 云台族基（GMBFamily::gimbalOf / worldOf）。
// ============================================================
template <class... AxisTags>
class GimbalModel : public GimbalModelBase
{
    using Joints = std::tuple<Joint<AxisTags>...>;
    Joints joints_;
public:
    GimbalModel(const GimbalModel &) = default;
    GimbalModel &operator=(const GimbalModel &) = default;
    GimbalModel(GimbalModel &&) noexcept = default;
    GimbalModel &operator=(GimbalModel &&) noexcept = default;

    // 两个位形之间的**距离**（°）：逐轴取圆周差，取最大者——量的是"每根关节离目标值多远"，不是两个朝向差多远
    //（后者是 SO(3) 上的测地角；判到位不用它，云台的每根关节都得到位，用户定）。
    //   · **圆周差**：每根关节都是一圈，0° 与 360° 是同一处，直接相减在跨零时会得出近一整圈的假距离——实机：
    //     命令 0°、停在 359.97°，差 359.97 实为 0.03。算法（用户提出）：两角差的绝对值超过半周，就取 360 − |差|。
    //   · **取最大者**：保持"每轴都在容差内"的含义，族级收集器的容差照旧按单轴度数给。
    // 仅接收两个同型的完整模型（WorldModel 与 PanTiltModel 比不了）；调用面写具体型（PanTiltModel::distance）。
    // 同型实例须保持相同轴数、轴序和轴向；concept 只检查接口，不验证这些配置。
    template <GimbalModelLike Model>
    [[nodiscard]] static double distance(const Model &a, const Model &b)
    {
        double worst = 0.0;
        [&]<std::size_t... I>(std::index_sequence<I...>) {
            ((worst = std::max(worst, AngularDistance(std::get<I>(a.joints_).value(), std::get<I>(b.joints_).value()))), ...);
        }(std::make_index_sequence<sizeof...(AxisTags)>{});
        return worst;
    }

    // **本位形**的朝向（PoE 正向，通用）：R = ∏ exp(θ_i·û_i)，θ_i 取自各关节自己的当前值。
    // **无参**：位形就在自己身上。两个位形就是两个对象，换算写成
    //     mounting.toOrientation().inverseRotation() * worldTarget.toOrientation()
    [[nodiscard]] SO3 toOrientation() const noexcept
    {
        constexpr double D2R = std::numbers::pi / 180.0;
        SO3 R = SO3::Identity();
        std::apply([&](const auto &...joint) {
            ((R = R * SO3::exp(joint.axis() * (joint.value() * D2R))), ...);
        }, joints_);
        return R;
    }

protected:
    // 派生模型按自身固定轴数访问；不提供分量写入口。
    template <std::size_t I>
    [[nodiscard]] constexpr const auto &axis() const noexcept { return std::get<I>(joints_); }

    explicit GimbalModel(Joint<AxisTags>... joints) noexcept : joints_(std::make_tuple(std::move(joints)...)) {}
    // **非虚 + protected**：本类一个虚函数都没有，而位形从不经基类指针删除（它们是被按值造、
    // 按值传的东西）。protected 禁止外部通过 GimbalModel 基类指针删除对象。
    ~GimbalModel() = default;
};

// ============================================================
// WorldModel — 世界系 ENU，本质是一台「三正交轴、内旋」的虚拟云台
//
// 关键洞见：世界系不是独立概念，它就是个 GimbalModel——az/el/roll 即这台三轴虚拟云台的作动量。
// 三轴取 û₁=-ẑ(方位,真北顺时针)、û₂=x̂(高度,抬头为正)、û₃=û₁×û₂=-ŷ(翻滚)，共同构成 orthoBasis 的正交基。
// 反解：三正交轴直接 XYZ 提取。
//
// **一个实例就是一个世界朝向**：安装标定是一个，本次目标是另一个，两者同类不同值。
// 构造收三个角（任意实数，各折进自己的窗、不坍缩）；裸构造给出归零位，即水平正北。
// **翻滚有默认值 0**：指向是 S² 上的一点、朝向是 SO(3) 上的一点，同一指向的全部朝向构成陪集
// R·SO(2)，翻滚是那根纤维的坐标。对"我要指哪"而言它不关心，给个代表元即可；对"基座架成什么样"
// 而言它是实打实的物理，那一处必须显式给。
// ============================================================
class WorldModel : public GimbalModel<GimbalModelAxes::MinusZ, GimbalModelAxes::X, GimbalModelAxes::MinusY>
{
public:
    // 摆到这个世界朝向：三个角任意实数，各折进自己的窗、不坍缩。翻滚默认 0（见类注）。
    // 本模型构造时归一化：方位 [0°, 360°)（罗盘方向），高度 [−90°, 270°)（连续覆盖越顶分支），翻滚 [−180°, 180°)；读口与界面用同一张窗。
    WorldModel(const double azimuth, const double elevation, const double roll = 0.0)
        : GimbalModel(Joint<GimbalModelAxes::MinusZ>(Wrap360(azimuth)),
                      Joint<GimbalModelAxes::X>(Wrap270(elevation)),
                      Joint<GimbalModelAxes::MinusY>(Wrap180(roll)))
    {
    }
    WorldModel() : WorldModel(0.0, 0.0, 0.0) {} // 归零位：水平正北

    [[nodiscard]] constexpr double azimuth() const noexcept { return axis<0>().value(); }
    [[nodiscard]] constexpr double elevation() const noexcept { return axis<1>().value(); }
    [[nodiscard]] constexpr double roll() const noexcept { return axis<2>().value(); }

    // E = UᵀRU 转到本模型关节基，按标准内旋 XYZ 提取三个角（度）。
    // 方位归 [0, 360)，高度取 [-90, 90]，翻滚由构造函数归 [-180, 180)。
    // 极点带内第三角取零，其影响折入第一角；带内代表解不承诺精确还原原朝向。
    [[nodiscard]] static WorldModel fromOrientation(const SO3 &R)
    {
        constexpr double R2D = 180.0 / std::numbers::pi;
        // 极点带宽与代表解沿用本模型约定。
        const LinAlg::Mat3 E = jointBasis(R);
        const double e02 = std::clamp(E(0, 2), -1.0, 1.0); // sin θ₂
        const double c2 = std::hypot(E(0, 0), E(0, 1));                             // |cos θ₂|
        if (c2 <= kPoleCos)
            return {Wrap360(std::atan2(e02 * E(1, 0), E(1, 1)) * R2D), // θ₁（θ₃ 已折进来）
                    std::asin(e02) * R2D,                                   // θ₂ 保留原提取值
                    0.0};                                                   // θ₃ 取 0 作代表
        return {Wrap360(std::atan2(-E(1, 2), E(2, 2)) * R2D), // θ₁
                std::asin(e02) * R2D,                              // θ₂
                std::atan2(-E(0, 1), E(0, 0)) * R2D};              // θ₃
    }

private:
    // 构造与反解换基共用同一份固定轴向，反解无需构造归零位形。
    static constexpr std::array<LinAlg::Vec3, 3> kAxes{
        GimbalModelAxes::MinusZ::value, GimbalModelAxes::X::value, GimbalModelAxes::MinusY::value};

    // 极点带宽：|cos θ₂| ≤ kPoleCos 按"光指着极点"收（本模型反解使用，双轴模型复用结果）。
    // **带宽取 ±0.1°，不是"正好 90°"**（用户定）：严格的 90° 谁也到不了——浮点会丢精度，设备更有误差。
    // 落在带内就按极点收，换来的是读数**稳定**：带外 0.05° 处俯仰动一点方位就甩几十度。
    // 代价明码：带内报出的三角是**代表**而非精确分解，重新合成会与原朝向差一点（量级 ≈ |θ₃|·sin(带宽)；
    // 指向误差不超过 2·sin(带宽) ≈ 0.2°，且只发生在"光已经指着天顶"的时候）。
    // **带宽该盖住"水平电机误差投到光束上的那个量"**（实机定标）：铭辰下单天顶时水平差 0.11°，
    // 光束离云台自身极点 28°，投影后 0.11°·sin(28°) ≈ 0.05° —— 光就停在距天顶 0.05° 处。带宽取 0.05°
    // 恰好卡在边界上、时灵时不灵，故取 0.1° 留一倍余量。换设备（水平精度不同）就该重算这个数。
    static constexpr double kPoleCos = 1.7453284e-3; // = sin(0.1°) = cos(89.9°)

    // 本模型固定关节基下的朝向；不向任意关节链提供这套反解假设。
    [[nodiscard]] static constexpr LinAlg::Mat3 jointBasis(const SO3 &R) noexcept
    {
        const LinAlg::Mat3 U = orthoBasis();
        return U.transpose() * R * U;
    }

    [[nodiscard]] static constexpr LinAlg::Mat3 orthoBasis() noexcept
    {
        const auto &a0 = kAxes[0], &a1 = kAxes[1], &a2 = kAxes[2];
        return {a0.x(), a1.x(), a2.x(),
                a0.y(), a1.y(), a2.y(),
                a0.z(), a1.z(), a2.z()};
    }
};

// ============================================================
// PanTiltModel — 双轴 pan(绕 -ẑ) + tilt(绕 x̂)，内旋，ENU。R = Rz(-pan)·Rx(tilt)。
//
// **本类是位形，不是指向**（用户定；算法层只谈位形，拒绝拿"指向"混淆概念）：
//   · **位形空间** = 两根转轴各一圈 = 环面 T²。构造收任意实数、各折进自己的窗（水平 [0°,360°)、俯仰
//     [−180°,180°)）、不坍缩。它是双轴云台这一数学对象的全部位形，不是哪台设备的行程。
//   · **正解是单射**：Rz(−p)·Rx(t) = Rz(−p')·Rx(t') ⇒ Rz(p'−p) = Rx(t'−t) ⇒ 两边都是单位阵。环面上
//     每个位形各得一个 SO(3) 朝向，**两极也不例外**——"两极处水平角任意"说的是指向，不是位形。
//   · **像是 SO(3) 里的二维子集**：双轴表达不了一般姿态。它恰是世界模型（前两轴相同）把第三轴
//     （翻滚，−ŷ）钉在 0 的那部分。
//
// ⚠ **别把位形空间收窄**（俯仰收到 [0°,90°] 或 [−90°,90°]）：理由听上去是"(p,t) 与 (p+180°,180°−t) 指向相同、
// 二义"，但那是把指向当成了位形——两者**朝向不同**（翻滚差 180°），在位形空间里本就是两个点。收窄的
// 后果是半个环面装不进来（实测俯仰 92° 换不出世界角）。**反解取 |俯仰| ≤ 90° 那支是另一回事**：那是
// "目标翻滚被忽略"之后对指向的唯一代表，位形空间本身仍是整个环面。
//
// **设备行程不在这儿**：某台云台够不够得着由**官邸**按协议定、造斗 / 组包时查（McztVocab::TiltAngleMin/Max、
// SqafConfig::tiltMin/Max），与本类的范围互不抄数（用户定："小解耦"）。发给电机的最后一道闸仍是
// `McztVocab::*2Code` 里的钳位。
//
// **反解 `fromOrientation`：朝向 → 位形**：目标的翻滚直接忽略（用户定：双轴对翻滚无计可施，需求只要指向），
// 前两轴与 WorldModel 一致，复用其反解的前两个分量——水平 [0°,360°)、俯仰 [−90°,90°]，两极折进水平角，与 WorldModel 同一套。
// ============================================================
class PanTiltModel : public GimbalModel<GimbalModelAxes::MinusZ, GimbalModelAxes::X>
{
public:
    // 摆到这个位形：两个角任意实数，各折进自己的窗、不坍缩；设备行程不在此判（归官邸）。
    // 本模型构造时归一化：水平 [0°, 360°)，俯仰 [−90°, 270°)（连续覆盖越顶分支）；读口与界面用同一张窗。
    PanTiltModel(const double pan, const double tilt)
        : GimbalModel(Joint<GimbalModelAxes::MinusZ>(Wrap360(pan)), // pan：绕 −ẑ
                      Joint<GimbalModelAxes::X>(Wrap270(tilt)))     // tilt：绕 x̂
    {
    }
    PanTiltModel() : PanTiltModel(0.0, 0.0) {} // 归零位，一个真实合法的位形

    [[nodiscard]] constexpr double pan() const noexcept { return axis<0>().value(); }
    [[nodiscard]] constexpr double tilt() const noexcept { return axis<1>().value(); }

    // 朝向 → 位形：目标翻滚忽略，取 WorldModel 反解的前两个分量（见类注）；设备行程由官邸造斗时查。
    [[nodiscard]] static PanTiltModel fromOrientation(const SO3 &R)
    {
        const WorldModel world = WorldModel::fromOrientation(R);
        return {world.azimuth(), world.elevation()};
    }
};

static_assert(GimbalModelLike<WorldModel>);
static_assert(GimbalModelLike<PanTiltModel>);

// 按到料时刻排列的世界指向；采样者按前值保持解释相邻回报之间的区间。
using WorldPoseHistory = std::vector<std::pair<std::chrono::system_clock::time_point, WorldModel>>;
// 窗口快照一次构建，跨 QML 和采样线程只传递只读共享份额。
using WorldPoseHistoryPtr = std::shared_ptr<const WorldPoseHistory>;

static_assert(ValueObject<Joint<GimbalModelAxes::MinusZ>>);
static_assert(ValueObject<WorldModel>);
static_assert(ValueObject<PanTiltModel>);
