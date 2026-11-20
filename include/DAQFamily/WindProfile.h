#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <vector>
#include <memory>
#include <chrono>

#include "DAQFamily/DAQChannelSpec.h"
#include "Matrix.h"
#include "GMBFamily/GimbalModel.h"

// ============================================================
// WindProfile — 多波束径向风反演风廓线
//
// 一轮 ≥4 个不同方位的波束（各自世界系方位角/高度角 + 一列径向风速），最小二乘解出
// 固定时间与平均高度角后，直接在最终帧中计算各高度门的 ENU 向量 (u,v,w)。
//
// Input 按值持有朝向及径向风的只读共享份额；Calc 借用调用方已准备的波束列表，不另存历史。
// ============================================================
namespace WindProfile
{
    // 当前业务采用四束：Calc 的接收下限、DAQ 最近完成帧历史容量和四轴计划束数须一致。
    // 这是业务约束，不是解出三个风分量的数学最小束数；改变扫描束数还须同步修改计划页面与编排。
    inline constexpr int kBeams = 4;

    template<class Range> requires Mapping<Range>
    struct Input
    {
        /// 该波束的**世界朝向**（只读方位与高度；翻滚与光束无关）。**空 = 朝向未知**。
        /// **参考系由类型钉住**：只收 `WorldModel`，由编排层经云台族口 `worldOf` 换好再交——
        /// 收一袋裸数的话，喂进来的若是云台角，基座一歪就指错且不响。
        std::optional<WorldModel> dir;
        std::shared_ptr<const RadialWind<Range>> ws; ///< 逐点径向风速；距离轴由类型确定
    };

    // 正仰角对应递增高度网格；距离轴顺序在编译期校验。
    template<class Range> requires Mapping<Range>
    [[nodiscard]] bool hasValidGrid(const VerticalWindProfileUnit<Range> &frame) noexcept
    {
        if constexpr (Range::uniform)
            return frame.heightStepM() > 0.0 && std::isfinite(frame.heightStartM())
                && std::isfinite(frame.heightAt(frame.size() - 1));
        else
            return frame.heightScale() > 0.0 && std::isfinite(frame.heightStartM())
                && std::isfinite(frame.heightAt(frame.size() - 1));
    }

    template<class Range> requires Mapping<Range>
    [[nodiscard]] std::optional<LinAlg::Vec3> windAt(const VerticalWindProfileUnit<Range> &frame,
                                                   const double heightM) noexcept
    {
        if (!hasValidGrid(frame) || !std::isfinite(heightM)) return std::nullopt;
        const double start = frame.heightStartM();
        if (heightM <= start || frame.size() == 1) return frame.front();
        if (heightM >= frame.heightAt(frame.size() - 1)) return frame.back();
        std::size_t low;
        double fraction;
        if constexpr (Range::uniform)
        {
            const double step = frame.heightStepM();
            const double position = (heightM - start) / step;
            low = static_cast<std::size_t>(position);
            if (low >= frame.size() - 1) return frame.back();
            if (heightM == frame.heightAt(low)) return frame[low];
            if (heightM == frame.heightAt(low + 1)) return frame[low + 1];
            fraction = position - static_cast<double>(low);
        }
        else
        {
            const auto gates = std::views::iota(std::size_t{0}, frame.size());
            const auto high = *std::ranges::lower_bound(gates, heightM, {}, [&frame](std::size_t gate) {
                return frame.heightAt(gate);
            });
            const auto upper = frame.heightAt(high);
            if (heightM == upper) return frame[high];
            low = high - 1;
            const auto lower = frame.heightAt(low);
            fraction = (heightM - lower) / (upper - lower);
        }
        if (!frame[low] || !frame[low + 1]) return std::nullopt;
        return *frame[low] * (1.0 - fraction) + *frame[low + 1] * fraction;
    }

    // 反演直接生成带到料时刻的只读整帧；层数组属于该帧，失败返回空份额。
    template<class Range> requires Mapping<Range>
    [[nodiscard]] std::shared_ptr<const VerticalWindProfileUnit<Range>> Calc(std::span<const Input<Range>> beams,
        std::chrono::system_clock::time_point at)
    {
        const int N = static_cast<int>(beams.size());
        if (N < kBeams) return {};
        for (const auto &b : beams)
            if (!b.dir || !b.ws) return {}; // 任一束朝向未知（无云台 / 到位斗上还没有实测位形）→ 整组不反演

        constexpr double DEG_TO_RAD = std::numbers::pi / 180.0;

        // 设计矩阵每行 = 波束世界指向的单位矢量（ENU 球面→笛卡尔，方位/高度，本就是风反演的内核）。
        std::vector<double> A(N * 3);
        for (int i = 0; i < N; ++i)
        {
            const double h = beams[i].dir->azimuth() * DEG_TO_RAD;
            const double v = beams[i].dir->elevation() * DEG_TO_RAD;
            const double cv = std::cos(v);
            A[i * 3 + 0] = std::sin(h) * cv;
            A[i * 3 + 1] = std::cos(h) * cv;
            A[i * 3 + 2] = std::sin(v);
        }

        LinAlg::Mat3 AtA = LinAlg::Mat3::Zeros();
        for (int i = 0; i < N; ++i)
        {
            const double x = A[i * 3], y = A[i * 3 + 1], z = A[i * 3 + 2];
            AtA = AtA + LinAlg::Mat3{x * x, x * y, x * z,
                                    y * x, y * y, y * z,
                                    z * x, z * y, z * z};
        }

        const auto AtA_inv = AtA.inverse();
        if (!AtA_inv) return {}; // 伪逆 (AᵀA)⁻¹ 奇异 → 几何退化

        std::vector<double> Pinv(3 * N, 0.0);
        for (int r = 0; r < 3; ++r)
            for (int i = 0; i < N; ++i)
            {
                double val = 0.0;
                for (int c = 0; c < 3; ++c)
                    val += (*AtA_inv)(r, c) * A[i * 3 + c];
                Pinv[r * N + i] = val;
            }

        // 同一距离轴在类型上保证各束的层数和距离网格一致。
        constexpr int nGates = Range::count;

        // 高度 = 斜距 × **sin**(高度角)。高度角自水平面起、抬头为正（WorldModel 的 el），
        // 与上面设计矩阵的竖直分量 sin(v) 是同一个口径。
        // ⚠ 别写成 cos：高度 = 斜距 × cos 是**天顶角**口径的式子，而喂进来的是高度角。写反了不会
        // 报错——60° 高度角下算出的高度只有正确值的 58%（cos60° / sin60°），还随风廓线一起落盘；
        // 只有 45° 时两者碰巧相等。
        double meanEl = 0.0;
        for (int i = 0; i < N; ++i) meanEl += beams[i].dir->elevation() * DEG_TO_RAD;
        meanEl /= N;
        const double sinEl = std::sin(meanEl);
        if (!(sinEl > 0.0)) return {};
        auto result = std::make_shared<VerticalWindProfileUnit<Range>>(at, meanEl / DEG_TO_RAD);
        const double *pinvU = Pinv.data() + 0 * N;
        const double *pinvV = Pinv.data() + 1 * N;
        const double *pinvW = Pinv.data() + 2 * N;

#pragma omp parallel for
        for (int k = 0; k < nGates; ++k)
        {
            double uk = 0.0, vk = 0.0, wk = 0.0;
            bool valid = true;
            for (int i = 0; i < N; ++i)
            {
                const double vr = (*beams[i].ws)[k]; // const 元素直读
                if (!std::isfinite(vr)) { valid = false; break; }
                uk += pinvU[i] * vr;
                vk += pinvV[i] * vr;
                wk += pinvW[i] * vr;
            }
            (*result)[k] = valid ? std::optional<LinAlg::Vec3>(std::in_place, uk, vk, wk) : std::nullopt;
        }
        return result; // 转交 const 份额，不搬运数组。
    }
} // namespace WindProfile
