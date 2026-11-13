#pragma once
#include "ObjectPolicy.h"
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <span>

#include "Matrix.h"
#include "SO3.h"

// ============================================================
// SiteCalibration — 站点标定（手眼标定 AX=XB，分离地基与手机偏差）
//
// 物理模型（全程旋转、roll 全在场）：手机贴云台顶，第 i 点已知云台姿态 gimbal_i、手机实测世界系
// 全朝向 measured_i，满足
//     R_meas_i = R_base · R_gimbal_i · R_offset
//   R_base  ：地基安装姿态（= 零位三元组，待求）
//   R_offset：手机相对云台顶的固定偏差（待求）
//
// 解法（两步，皆为旋转，不假设 pan 等分、不丢 roll）：
//   ① 手眼：相邻对的相对运动 A=R_meas_iᵀR_meas_j、B=R_gimbal_iᵀR_gimbal_j 满足 B=R_off·A·R_offᵀ，
//      即旋转向量 log(B)=R_off·log(A)——问题变成「找一个旋转，把一组向量 a 转到 b」（Wahba 问题）：
//      累加 M=Σ log(B)·log(A)ᵀ，离它最近的旋转即 R_offset。
//      **需 ≥2 个不平行的相对转轴** → 标定时云台除 pan 还须 tilt（俯仰激励）；转轴全共线（只转了
//      水平）时 R_offset 与 R_base 绕 pan 轴的分量不可分（规范自由度），此时**退化为 R_offset=I**、
//      只解 R_base——手机绕水平轴贴歪的那几度会原样算进地基的方位。
//      **补秩**：两个不平行的方向本就足以定出一个旋转，但那时 M 的秩只有 2，求最近旋转的迭代（要对
//      M 求逆）起不了头。「四点、隔点抬俯仰」恰是这种情形：第 1→2 段与第 3→4 段的相对转动相同，
//      独立方向只有两个。故先把 M 加上自己的余因子阵（按 ‖M‖ 归一）：
//          cof(Σ b·aᵀ) = Σ_{i<j} (b_i×b_j)·(a_i×a_j)ᵀ          （Cauchy–Binet）
//      即把每两对观测的**叉积**也当成一对观测补进去——旋转保叉积（b=R·a ⇒ b_i×b_j=R·(a_i×a_j)），
//      自洽的两轴观测由此补出第三个方向；不保证任意含噪矩阵的带符号奇异值都变为正。
//      M 本来满秩且自洽时结果不变；秩 1（转轴全共线）时
//      余因子阵为零、补不出来，照旧退化。
//   ② 回代：R_base = 平均_i( R_meas_i · (R_gimbal_i·R_offset)ᵀ )，求和后投影回 SO(3)。
//
// ⚠ **本类不判「激励够不够」**：俯仰只抬几度时第二个方向淹在读数噪声里，R_offset 绕主轴那一圈由
// 噪声说了算，而残差照样很小。激励质量须由调用流程保证；族基的激励角常量本身不构成校验。
// 量级（合成数据：手机贴歪约 6°，读数噪声方位 0.5°、倾角 0.1°，四点隔点抬俯仰）——地基姿态的误差
// 均值：抬 40° 约 0.26°、抬 15° 约 0.45°、抬 5° 约 1.2°；只转水平约 5°（即手机绕水平轴贴歪的量）。
// 方位读数的噪声到 3°（手机罗盘的实际水平）时，抬 40° 四点约 1.3°——瓶颈在罗盘，不在算法。
//
// 样本与结果一律用**朝向 SO3**（旋转矩阵本体），本类自始至终不碰欧拉角、不知装配约定——角 ↔ 矩阵
// 的转换由调用方按云台模型（PanTiltModel / WorldModel 的 toOrientation）先做好再喂进来。
// **measured 必须是全朝向**（方位 / 高度 / 翻滚三个角都要）：算法吃的是相邻两点之间的相对转动，
// 只有指向的两个角拼不出来。样本按运动顺序传入（相邻相对旋转避免接近 180°，否则旋转向量轴向
// 不可靠）。成员函数在类内定义（header-only，单文件）。
// 当前由算法测试调用，生产标定页尚未接线。
// ============================================================
class SiteCalibration
{
public:
    struct Sample
    {
        SO3 gimbal;   ///< 已知云台姿态（朝向）
        SO3 measured; ///< 手机实测世界系朝向
    };

    struct Estimate
    {
        SO3 base;   ///< 地基零位姿态 R_base
        SO3 offset; ///< 手机相对云台顶偏差 R_offset；数据仅单轴时退化为单位阵
        bool offsetResolved = false; ///< true=偏差候选通过现有行列式判据；false=退回 I；不代表激励质量
        double residualDeg = 0.0;  ///< 二次投影前按 logRotation 计算的 RMS 残差 (°)，不单独证明标定可信
    };

    // 借用样本，不留存视图；调用期间底层存储须有效且不被改写。
    // N<3 或地基投影行列式 <0.5 时返回 nullopt；偏差退回 I 仍可返回有效结果。
    [[nodiscard]] static std::optional<Estimate> estimateFromSamples(const std::span<const Sample> &samples) noexcept
    {
        using namespace LinAlg;
        const std::size_t N = samples.size();
        if (N < 3) return std::nullopt;

        // ① 手眼解 R_offset：相邻对的相对旋转向量 b = R_off·a，累加 Wahba 阵 M = Σ b·aᵀ。
        Mat3 M = Mat3::Zeros();
        for (std::size_t i = 0; i + 1 < N; ++i)
        {
            const Vec3 a = SO3::logRotation(samples[i].measured.transpose() * samples[i + 1].measured);
            const Vec3 b = SO3::logRotation(samples[i].gimbal.transpose() * samples[i + 1].gimbal);
            M = M + Mat3{b.x() * a.x(), b.x() * a.y(), b.x() * a.z(),
                         b.y() * a.x(), b.y() * a.y(), b.y() * a.z(),
                         b.z() * a.x(), b.z() * a.y(), b.z() * a.z()}; // 外积累加
        }
        // 补秩（道理见文件头 ①）：M += cof(M) / ‖M‖。余因子阵的第 i 行 = M 另外两行的叉积（轮换序）。
        {
            const Vec3 r0{M(0, 0), M(0, 1), M(0, 2)}, r1{M(1, 0), M(1, 1), M(1, 2)}, r2{M(2, 0), M(2, 1), M(2, 2)};
            const Vec3 c0 = Vec3::cross(r1, r2), c1 = Vec3::cross(r2, r0), c2 = Vec3::cross(r0, r1);
            const Mat3 cof{c0.x(), c0.y(), c0.z(),
                           c1.x(), c1.y(), c1.z(),
                           c2.x(), c2.y(), c2.z()};
            const double scale = std::sqrt(r0.norm2() + r1.norm2() + r2.norm2()); // ‖M‖（Frobenius 范数）
            // 整个余因子矩阵共用除数与拒绝阈值；拒绝时 M 保持不变。
            if (const auto normalized = cof / scale)
                M = M + *normalized;
        }
        const Mat3 cand = SO3::nearestRotation(M);
        const bool offsetResolved = cand.determinant() > 0.5; // 既有投影判据，不额外判定激励质量
        const Mat3 Roff = offsetResolved ? cand : Mat3::Identity();

        // ② 回代旋转平均求 R_base：S = Σ R_meas_i·(R_gimbal_i·R_off)ᵀ，投影回 SO(3)。
        Mat3 S = Mat3::Zeros();
        for (std::size_t i = 0; i < N; ++i)
        {
            const Mat3 Rbi = samples[i].measured * (samples[i].gimbal * Roff).transpose();
            S = S + Rbi;
        }
        const Mat3 Rbase = SO3::nearestRotation(S);
        if (Rbase.determinant() < 0.5) return std::nullopt; // 求和退化（朝向自相矛盾）→ 失败

        // ③ 残差：predicted = R_base·R_gimbal_i·R_off，与实测的失配旋转取角，RMS。
        // 使用二次投影前的 Rbase/Roff；不在最终 fromApprox 后重算，因此不保证等于返回旋转的重算残差。
        double sse = 0.0;
        for (std::size_t i = 0; i < N; ++i)
        {
            const Mat3 err = (Rbase * samples[i].gimbal * Roff).transpose() * samples[i].measured;
            const double ang = SO3::logRotation(err).norm(); // 失配角（弧度）
            sse += ang * ang;
        }

        // 保留既有第二次投影；第一次迭代可能已达上限，不将重复投影视为严格幂等。
        return Estimate{SO3::fromApprox(Rbase), SO3::fromApprox(Roff), offsetResolved,
                        std::sqrt(sse / static_cast<double>(N)) * (180.0 / std::numbers::pi)};
    }
};

static_assert(ValueObject<SiteCalibration>);
static_assert(ValueObject<SiteCalibration::Sample>);
static_assert(ValueObject<SiteCalibration::Estimate>);
