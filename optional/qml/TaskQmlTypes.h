#pragma once
#include <QtQml/qqmlregistration.h>
#include "Device/Base/DeviceBase.h"
#include "Device/GMB/GMBDevice.h"
#include "Device/DAQ/DAQDevice.h"
#include "UI/Chart/ChartTypes.h"
#include "Task/TaskBase.h"

// 可选界面登记：核心及任务业务目标不依赖 Qt Qml。
struct DeviceBaseQml
{
    Q_GADGET
    QML_FOREIGN(DeviceBase)
    QML_NAMED_ELEMENT(DeviceBase)
    QML_UNCREATABLE("设备公共接口")
};

struct GMBDeviceBaseQml
{
    Q_GADGET
    QML_FOREIGN(GMBDeviceBase)
    QML_NAMED_ELEMENT(GMBDeviceBase)
    QML_UNCREATABLE("设备族公共接口")
};
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
