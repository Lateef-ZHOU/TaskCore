// 任务族业务分区：官邸持保存草稿，启动边界校验并转换目录。
pragma ComponentBehavior: Bound
import QtQuick
import Lidar.UI
import QtQuick.Controls
import QtQuick.Layouts

Section {
    id: root
    required property var settings
    title: qsTr("落盘")

    BoolField { text: qsTr("保存数据到本地"); value: root.settings.save
                onCommitted: function (v) {
                    if (v === root.settings.save) return
                    root.settings.save = v
                    root.settings.logMessage(qsTr("保存数据到本地：") + (v ? qsTr("已勾选") : qsTr("未勾选")))
                } }
    PathField { title: qsTr("选择保存目录")
                value: root.settings.saveDir; text: String(root.settings.saveDir)
                onCommitted: function (dir) { root.settings.saveDir = dir } }

    Label {
        Layout.fillWidth: true
        text: qsTr("会话目录下还会按设备标识再分一层，多台设备同时采集不撞名。")
        wrapMode: Text.Wrap
        Layout.preferredWidth: 1
        color: palette.placeholderText
    }
}
