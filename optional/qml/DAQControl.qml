// 采集卡族公共控制；型号只补充内容和是否需要单独下发累积时间。
import QtQuick
import Lidar.UI
import QtQuick.Controls
import QtQuick.Layouts

Section {
    id: root
    required property var device
    required property var acquisition
    property bool sendParameters: false
    property string accumulationHint: ""
    default property alias modelContent: modelFields.data
    title: qsTr("设备控制")

    ColumnLayout { id: modelFields; Layout.fillWidth: true; spacing: 8 }
    RowLayout {
        Layout.fillWidth: true
        spacing: 8
        FieldRow {
            Layout.fillWidth: true
            label: qsTr("累积时间（s）")
            IntField { Layout.fillWidth: true; Layout.preferredWidth: 80
                       minimum: root.acquisition.acquisitionParameters.accumMin
                       maximum: root.acquisition.acquisitionParameters.accumMax
                       value: root.acquisition.accum
                       onCommitted: function (v) {
                           if (v === root.acquisition.accum) return
                           root.acquisition.accum = v
                           root.acquisition.logMessage(qsTr("累积时间（s）：%1").arg(v))
                       } }
        }
        Button {
            visible: root.sendParameters; text: qsTr("下发")
            onClicked: {
                if (root.device.sendParams(root.acquisition.accum))
                    root.acquisition.logMessage(qsTr("已下发累积时间 %1 秒").arg(root.acquisition.accum))
            }
        }
    }
    Label {
        visible: text.length > 0
        text: root.accumulationHint
        color: palette.placeholderText
        font.pointSize: 9
        wrapMode: Text.Wrap
        Layout.preferredWidth: 1
        Layout.fillWidth: true
    }
    BoolField { text: qsTr("保存数据"); value: root.acquisition.save
                onCommitted: function (v) {
                    root.acquisition.save = v
                    root.acquisition.logMessage(qsTr("保存数据：") + (v ? qsTr("已勾选") : qsTr("未勾选")))
                } }
    PathField { title: qsTr("选择保存目录")
                value: root.acquisition.saveDir; text: root.acquisition.saveDir.toString()
                onCommitted: function (dir) { root.acquisition.saveDir = dir } }

    RowLayout {
        Layout.fillWidth: true
        Layout.topMargin: 4
        spacing: 8
        Button { Layout.fillWidth: true; text: qsTr("开始采集")
                 onClicked: root.device.startTest(root.acquisition.accum, root.acquisition.save, root.acquisition.saveDir) }
        Button { Layout.fillWidth: true; text: qsTr("停止采集")
                 onClicked: {
                     root.device.cancelTest()
                     root.acquisition.logMessage(qsTr("测试已停止"))
                 } }
    }
}
