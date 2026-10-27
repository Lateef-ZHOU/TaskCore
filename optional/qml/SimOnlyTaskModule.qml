pragma ComponentBehavior: Bound
import QtQuick
import Lidar.UI

// 仿真参考页面：固定一台云台、一张卡；启动时始终保存到当前工作目录。
ModulePage {
    id: root
    readonly property string kind: "task"
    required property var settingsParameters
    required property Section planPage
    required property GMBDeviceBase gimbal
    required property DAQDeviceBase card
    required property int accum
    required property int settle
    readonly property int accumMin: settingsParameters.accumMin
    readonly property int accumMax: settingsParameters.accumMax
    mainMinWidth: 560
    mainMargin: 12
    sections: [planPage, run]
    SimOnlyTaskRunSection { id: run; settings: root }
    function registerDevices(task): void {
        task.addGimbal(gimbal)
        task.addCard(card)
    }
    function forgetDevice(device): void {
        if (gimbal === device) gimbal = null
        if (card === device) card = null
    }
}
