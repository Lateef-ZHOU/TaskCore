#pragma once
#include <concepts>
#include <memory>
#include <optional>
#include <utility>
#include <QByteArray>
#include <QDir>
#include <QString>

#include "Algorithm/Geometry/GimbalModel.h"
#include "Device/Base/RunBase.h" // Collector 契约基类
#include "Device/Base/RxTime.h"

// 一次采集的不可变消费上下文；重试照抄，完成帧自带处理与保存所需信息。
struct DAQContext
{
    std::optional<WorldModel> pose;
    std::optional<QDir> saveRoot;
    QString sessionDir;
    bool sync = false;
};

// 族基提供字节进料契约、时刻与消费上下文；谱的存储和 constFrame 由具体型号提供。
class DAQCollector : public Collector<QByteArray> // 喂料是字节：分片 / 包原样喂
{
public:
    // 当前帧的到料时刻由型号按协议确定；完成后供落盘文件名和风廓线时间轴使用。
    [[nodiscard]] RxTime frameTime() const noexcept { return frameTime_; }

    [[nodiscard]] const DAQContext &context() const noexcept { return context_; }

protected:
    // 派生构造必须显式交入上下文；确实不需要时传 {}。
    explicit DAQCollector(DAQContext context) : context_(std::move(context)) {}
    // 非虚 + protected：与 Collector / GMBCollector 同判例——销毁恒经控制块记下的具体删除器，
    // 从不经本类指针 delete；protected 顺带把裸删门封死。
    ~DAQCollector() = default;

    // 仅型号 onFeed 写：未完成时可随协议换帧更新，完成后公共 feed 不再改写。
    // makeSelf 只保留不可变配置，另建初始帧时刻，不继承旧尝试的观测。
    RxTime frameTime_{};

private:
    const DAQContext context_;
};

// 具体类型须属于本族、沿用 feed；静态工厂交只读自身，makeSelf 交可写自身。
template <typename C>
concept DAQCollectorLike = std::derived_from<C, DAQCollector> && CollectorLike<C>
    && requires(int accumSec, DAQContext context) {
        { C::createPrototype(accumSec, std::move(context)) } -> std::same_as<std::shared_ptr<const C>>;
    };
