// 任务族页面：公共草稿与业务分区。官邸提供唯一计划、预览及具体后端的末端接线。
pragma ComponentBehavior: Bound
import QtQuick
import Lidar.UI

ModulePage {
    id: root
    readonly property string kind: "task"
    required property var settingsParameters
    required property Section planPage
    mainMinWidth: 560
    mainMargin: 12

    required property list<GMBDeviceBase> gimbals
    required property list<DAQDeviceBase> cards
    property GMBDeviceBase gimbal: gimbals.length ? gimbals[0] : null
    property list<DAQDeviceBase> pickedCards: []
    property int accum: settingsParameters.accumDefault
    property int settle: settingsParameters.settleDefault
    property bool save: false
    property url saveDir: settingsParameters.saveDirDefault
    property real panSpeed: settingsParameters.speedDefault
    property real tiltSpeed: settingsParameters.speedDefault
    readonly property int accumMin: settingsParameters.accumMin
    readonly property int accumMax: settingsParameters.accumMax

    sections: [selection, planPage, saving, run]
    TaskSelectionSection { id: selection; settings: root }
    TaskSaveSection { id: saving; settings: root }
    TaskRunSection { id: run; settings: root; plan: root.planPage }

    // 末端把自己的业务对象交入登记操作；族层不保存或公开该对象。
    function registerDevices(task): void {
        for (const device of gimbals) task.addGimbal(device)
        for (const device of cards) task.addCard(device)
    }

    function forgetDevice(device): void {
        gimbals = gimbals.filter(item => item !== null && item !== device)
        cards = cards.filter(item => item !== null && item !== device)
        pickedCards = pickedCards.filter(item => item !== null && item !== device)
        if (!gimbal || gimbal === device) gimbal = gimbals.length ? gimbals[0] : null
    }
}
