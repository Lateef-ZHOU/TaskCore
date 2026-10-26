// 任务族业务分区：设备选取与提交，选择仅由 QML 官邸持有。
pragma ComponentBehavior: Bound
import QtQuick
import Lidar.UI
import QtQuick.Controls
import QtQuick.Layouts

Section {
    id: root
    required property var settings
    title: qsTr("任务设置")

    // 使用设备族对象身份；列表变化不会把旧下标解释为另一台设备。
    Label { text: qsTr("云台（单选）"); font.bold: true; Layout.topMargin: 4 }

    Repeater {
        model: root.settings.gimbals
        delegate: RadioButton {
            readonly property bool selected: root.settings.gimbal === modelData
            autoExclusive: false
            required property var modelData
            Layout.fillWidth: true
            text: modelData ? modelData.device.displayName + " · " + modelData.device.subtitle : ""
            onSelectedChanged: checked = selected
            Component.onCompleted: checked = selected
            onClicked: {
                root.settings.gimbal = modelData
                checked = selected
            }
        }
    }

    Label { text: qsTr("采集卡（可多选）"); font.bold: true; Layout.topMargin: 4 }

    Repeater {
        model: root.settings.cards
        delegate: BoolField {
            required property var modelData
            Layout.fillWidth: true
            text: modelData ? modelData.device.displayName + " · " + modelData.device.subtitle : ""
            value: root.settings.pickedCards.includes(modelData)
            onCommitted: function (selected) {
                const others = root.settings.pickedCards.filter(card => card !== modelData)
                root.settings.pickedCards = selected ? others.concat([modelData]) : others
            }
        }
    }
}
