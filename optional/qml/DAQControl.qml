// 采集卡族公共控制；型号只补充内容和是否在编辑完成时下发累积时间。
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

    ColumnLayout {
        id: modelFields
        Layout.fillWidth: true
        spacing: FormStyle.rowSpacing
    }
    FieldRow {
        label: qsTr("累积时间（s）")
        IntField {
            Layout.fillWidth: true
            minimum: root.acquisition.acquisitionParameters.accumMin
            maximum: root.acquisition.acquisitionParameters.accumMax
            value: root.acquisition.accum
            onCommitted: function (v) {
                if (v !== root.acquisition.accum) {
                    root.acquisition.accum = v;
                    root.acquisition.logMessage(qsTr("累积时间（s）：%1").arg(v));
                }
                if (root.sendParameters)
                    root.device.sendParams(v);
            }
        }
    }
    HintText {
        visible: text.length > 0
        text: root.accumulationHint
    }
    BoolField {
        text: qsTr("保存数据")
        value: root.acquisition.save
        onCommitted: function (v) {
            root.acquisition.save = v;
            root.acquisition.logMessage(qsTr("保存数据：") + (v ? qsTr("已勾选") : qsTr("未勾选")));
        }
    }
    PathField {
        title: qsTr("选择保存目录")
        value: root.acquisition.saveDir
        onCommitted: function (dir) {
            root.acquisition.saveDir = dir;
        }
    }

    DoubleButtonRow {
        Layout.topMargin: 4
        firstAction: Action {
            text: qsTr("开始采集")
            onTriggered: root.device.startTest(root.acquisition.accum, root.acquisition.save, root.acquisition.saveDir)
        }
        secondAction: Action {
            text: qsTr("停止采集")
            onTriggered: {
                root.device.cancelTest();
                root.acquisition.logMessage(qsTr("测试已停止"));
            }
        }
    }
}
