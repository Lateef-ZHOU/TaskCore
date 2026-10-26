// 任务族业务分区：编辑官邸的时间设置，交同一个 plan 给官邸 start 组装快照。
pragma ComponentBehavior: Bound
import QtQuick
import Lidar.UI
import QtQuick.Controls
import QtQuick.Layouts

Section {
    id: root
    required property var settings
    required property var plan
    title: qsTr("运行控制")

    FieldRow {
        labelWidth: 160
        label: qsTr("停稳等待（s）")
        IntField {
            Layout.fillWidth: true
            minimum: root.settings.settingsParameters.settleMin; maximum: root.settings.settingsParameters.settleMax
            value: root.settings.settle
            onCommitted: function (v) {
                if (v === root.settings.settle) return
                root.settings.settle = v
                root.settings.logMessage(qsTr("停稳等待：%1 s").arg(v))
            }
        }
    }
    FieldRow {
        labelWidth: 160
        label: qsTr("累积时间（s，各卡）")
        IntField {
            Layout.fillWidth: true
            minimum: root.settings.accumMin; maximum: root.settings.accumMax
            value: root.settings.accum
            onCommitted: function (v) {
                if (v === root.settings.accum) return
                root.settings.accum = v
                root.settings.logMessage(qsTr("累积时间：%1 s").arg(v))
            }
        }
    }

    FieldRow {
        labelWidth: 160
        label: qsTr("任务水平速度（°/s）")
        RealField {
            Layout.fillWidth: true
            minimum: root.settings.settingsParameters.speedMin; maximum: root.settings.settingsParameters.speedMax
            value: root.settings.panSpeed
            onCommitted: function(v) { root.settings.panSpeed = v }
        }
    }
    FieldRow {
        labelWidth: 160
        label: qsTr("任务俯仰速度（°/s）")
        RealField {
            Layout.fillWidth: true
            minimum: root.settings.settingsParameters.speedMin; maximum: root.settings.settingsParameters.speedMax
            value: root.settings.tiltSpeed
            onCommitted: function(v) { root.settings.tiltSpeed = v }
        }
    }
    Label {
        Layout.fillWidth: true
        text: qsTr("任务速度独立设置，仅用于支持指定定位速度的云台。")
        wrapMode: Text.Wrap
        Layout.preferredWidth: 1
        color: palette.placeholderText
    }
    Label {
        Layout.fillWidth: true
        text: qsTr("开始时使用当前设置；运行中修改设置将在下次启动时生效。")
        wrapMode: Text.Wrap
        Layout.preferredWidth: 1
        color: palette.placeholderText
    }

    RowLayout {
        Layout.fillWidth: true
        spacing: 8
        Button {
            Layout.fillWidth: true; text: qsTr("开始")
            enabled: !!root.plan
            onClicked: root.settings.start(root.plan.azimuths, root.plan.elevation)
        }
        Button { Layout.fillWidth: true; text: qsTr("停止"); onClicked: root.settings.stop() }
    }

    Label {
        Layout.fillWidth: true
        text: qsTr("停止即立即停止：取消当前等待并放锁。")
        wrapMode: Text.Wrap
        Layout.preferredWidth: 1 // 换行文字的 implicitWidth 是「整段不折行」的宽度，会顶爆列宽
        color: palette.placeholderText
    }
}
