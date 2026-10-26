// 四台网口设备共用连接分区；表单在各自 QML 官邸，重建分区不丢设置。
import QtQuick
import Lidar.UI
import QtQuick.Controls
import QtQuick.Layouts

Section {
    id: root
    required property var device
    required property var settings
    required property var parameters
    Layout.fillWidth: true
    title: qsTr("设备连接")

    function commit(name, value, label): void {
        if (root.settings[name] === value) return
        root.settings[name] = value
        root.settings.logMessage(label + "：" + (name === "networkInterfaceIndex"
                                 ? root.parameters.networkInterfaceOptions[value] : value))
    }

    FieldRow {
        label: qsTr("网卡")
        ChoiceField { Layout.fillWidth: true
                      model: root.parameters.networkInterfaceOptions
                      tips: root.parameters.networkInterfaceTips
                      value: root.settings.networkInterfaceIndex
                      onCommitted: function (i) { root.commit("networkInterfaceIndex", i, qsTr("网卡")) } }
    }
    FieldRow {
        label: qsTr("本机 IPv4")
        IPv4Field { Layout.fillWidth: true; Layout.preferredWidth: 120
                    value: root.settings.hostIPv4
                    onCommitted: function (v) { root.commit("hostIPv4", v, qsTr("本机 IPv4")) } }
    }
    FieldRow {
        label: qsTr("本机端口")
        IntField { Layout.fillWidth: true; Layout.preferredWidth: 80
                   minimum: root.parameters.portMin; maximum: root.parameters.portMax
                   value: root.settings.hostPort
                   onCommitted: function (v) { root.commit("hostPort", v, qsTr("本机端口")) } }
    }
    FieldRow {
        label: qsTr("设备 MAC")
        visible: root.parameters.hasDeviceMAC
        MacField { Layout.fillWidth: true; Layout.preferredWidth: 140
                   value: root.settings.deviceMAC
                   onCommitted: function (v) { root.commit("deviceMAC", v, qsTr("设备 MAC")) } }
    }
    FieldRow {
        label: qsTr("设备 IPv4")
        IPv4Field { Layout.fillWidth: true; Layout.preferredWidth: 120
                    value: root.settings.deviceIPv4
                    onCommitted: function (v) { root.commit("deviceIPv4", v, qsTr("设备 IPv4")) } }
    }
    FieldRow {
        label: qsTr("设备端口")
        IntField { Layout.fillWidth: true; Layout.preferredWidth: 80
                   minimum: root.parameters.portMin; maximum: root.parameters.portMax
                   value: root.settings.devicePort
                   onCommitted: function (v) { root.commit("devicePort", v, qsTr("设备端口")) } }
    }

    RowLayout {
        Layout.fillWidth: true
        Layout.topMargin: 4
        spacing: 8
        Button {
            Layout.fillWidth: true
            text: qsTr("连接")
            onClicked: {
                const accepted = root.parameters.hasDeviceMAC
                    ? root.device.open(root.settings.networkInterfaceIndex, root.settings.hostIPv4, root.settings.hostPort,
                                       root.settings.deviceIPv4, root.settings.devicePort, root.settings.deviceMAC)
                    : root.device.open(root.settings.networkInterfaceIndex, root.settings.hostIPv4, root.settings.hostPort,
                                       root.settings.deviceIPv4, root.settings.devicePort)
                if (accepted) root.settings.logMessage(qsTr("连接请求已提交"))
            }
        }
        Button { Layout.fillWidth: true; text: qsTr("断开"); onClicked: root.device.close() }
    }
}
