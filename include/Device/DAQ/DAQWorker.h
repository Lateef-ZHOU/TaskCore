#pragma once
#include <concepts>
#include <memory>
#include <QObject>

#include "Device/DAQ/DAQCollector.h" // 采集帧族基与消费上下文

// 型号处理器兼 Qt 队列接收上下文；只接具体完成帧，消费上下文从帧读取。
// 接收投递由设备的 onConsume 信号连接完成，worker 不持设备引用。
// 消费端只约束族归属；进料与工厂约束由设备启动层检查。
template <std::derived_from<DAQCollector> C>
class DAQWorker : public QObject
{
public:
    // [工作线程] 完整帧与其不可变上下文；只读 double 成品，执行反演、出图和保存。
    virtual void operator()(const std::shared_ptr<const C> &self) = 0;

    // DAQDevice 以 unique_ptr<DAQWorker<C>> 独占，经基类删除具体处理器，故保持公开虚析构。
    // 设备在释放处理器之前同步停止处理线程，排队消息只共享只读 Collector。
    ~DAQWorker() override = default;

protected:
    DAQWorker() = default;
};
