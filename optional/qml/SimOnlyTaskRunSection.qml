// 时间输入直接编辑官邸；开始按钮与自动启动共用官邸的无参 start()。
pragma ComponentBehavior: Bound
import QtQuick
import Lidar.UI
import QtQuick.Controls
import QtQuick.Layouts

Section {
    id: root
    required property var settings
    title: qsTr("运行控制")

    FieldRow {
        wideLabel: true
        label: qsTr("停稳等待（s）")
        IntField {
            Layout.fillWidth: true
            minimum: root.settings.settingsParameters.settleMin
            maximum: root.settings.settingsParameters.settleMax
            value: root.settings.settle
            onCommitted: function (v) {
                if (v === root.settings.settle)
                    return;
                root.settings.settle = v;
                root.settings.logMessage(qsTr("停稳等待：%1 s").arg(v));
            }
        }
    }
    FieldRow {
        wideLabel: true
        label: qsTr("累积时间（s，各卡）")
        IntField {
            Layout.fillWidth: true
            minimum: root.settings.accumMin
            maximum: root.settings.accumMax
            value: root.settings.accum
            onCommitted: function (v) {
                if (v === root.settings.accum)
                    return;
                root.settings.accum = v;
                root.settings.logMessage(qsTr("累积时间：%1 s").arg(v));
            }
        }
    }

    HintText {
        text: qsTr("开始时使用当前设置；运行中修改设置将在下次启动时生效。")
    }

    DoubleButtonRow {
        firstAction: Action {
            text: qsTr("开始")
            onTriggered: root.settings.start()
        }
        secondAction: Action {
            text: qsTr("停止")
            onTriggered: root.settings.stop()
        }
    }

    HintText {
        text: qsTr("停止即立即停止：取消当前等待并放锁。")
    }
}
