#pragma once
#include <QtQml/qqmlregistration.h>
#include "DeviceBase.h"
#include "GMBDevice.h"
#include "DAQDevice.h"
#include "ChartTypes.h"
#include "TaskBase.h"

// 可选界面登记：核心及任务业务目标不依赖 Qt Qml。
// 云台族基不是 QObject，不登记 QML 类型；页面以 QtObject 持有云台设备本身。
struct DAQDeviceBaseQml
{
    Q_GADGET
    QML_FOREIGN(DAQDeviceBase)
    QML_NAMED_ELEMENT(DAQDeviceBase)
    QML_UNCREATABLE("设备族公共接口")
};

struct TaskBaseQml
{
    Q_GADGET
    QML_FOREIGN(TaskBase)
    QML_NAMED_ELEMENT(TaskBase)
    QML_UNCREATABLE("可选的任务公共业务")
};
