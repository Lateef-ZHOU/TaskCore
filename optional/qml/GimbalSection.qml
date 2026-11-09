// GimbalSection —— 转台族共享的「状态与定位」内容；官邸随模板交入设备和固定行程。
//
// 可选参考分区：实时角度、安装标定、两种坐标系定位与停止；官邸可以自行提供其他界面。
// 具体型号动作由官邸单独组织，不要求继承固定的设备族页面。
//
// 数据来自设备的主线程 anglesReady 信号，与角度图各自订阅；读数不依赖图表存在。
// **动作一律直发**（不起 run、不拿锁、不等到位）：验锁在 Device::send 里统一做，被任务占着就拒并记一条。
// 界面不做门控。
import QtQuick
import Lidar.UI
import QtQuick.Controls
import QtQuick.Layouts

Section {
    id: root
    required property var device // 分区模板的创建参数传入本台设备
    required property var settings
    required property real panMin
    required property real panMax
    required property real tiltMin
    required property real tiltMax
    title: qsTr("状态与定位")

    function fmt(v) {
        return (v === undefined || isNaN(v)) ? "—" : v.toFixed(2) + "°";
    }

    // 每列依次为标题、实测读数、目标草稿；仅编辑完成提交同坐标系的两轴。
    ReadoutRow {
        ReadoutField {
            id: pan
            caption: qsTr("转台水平")
            RealField {
                id: tPan
                Layout.fillWidth: true
                minimum: root.panMin
                maximum: root.panMax
                value: root.panMin
                onCommitted: function (v) {
                    tPan.value = v;
                    root.device.issueGoTo(tPan.value, tTilt.value, root.settings.moveParameters());
                }
            }
        }
        ReadoutField {
            id: tilt
            caption: qsTr("转台俯仰")
            RealField {
                id: tTilt
                Layout.fillWidth: true
                minimum: root.tiltMin
                maximum: root.tiltMax
                value: root.tiltMin
                onCommitted: function (v) {
                    tTilt.value = v;
                    root.device.issueGoTo(tPan.value, tTilt.value, root.settings.moveParameters());
                }
            }
        }
        ReadoutField {
            id: azimuth
            caption: qsTr("世界方位")
            RealField {
                id: tAz
                Layout.fillWidth: true
                minimum: 0
                maximum: 360
                value: 0
                onCommitted: function (v) {
                    tAz.value = v;
                    const pose = root.device.mounting.gimbalAngles(tAz.value, tEl.value);
                    root.device.issueGoTo(pose.pan, pose.tilt, root.settings.moveParameters());
                }
            }
        }
        ReadoutField {
            id: elevation
            caption: qsTr("世界高度")
            RealField {
                id: tEl
                Layout.fillWidth: true
                minimum: -180
                maximum: 180
                value: 0
                onCommitted: function (v) {
                    tEl.value = v;
                    const pose = root.device.mounting.gimbalAngles(tAz.value, tEl.value);
                    root.device.issueGoTo(pose.pan, pose.tilt, root.settings.moveParameters());
                }
            }
        }
    }

    Connections {
        target: root.device
        function onMountingChanged(pose) {
            mount.load(pose);
        }
        function onAnglesReady(at, p, ti, az, el) {
            pan.text = root.fmt(p);
            tilt.text = root.fmt(ti);
            azimuth.text = root.fmt(az);
            elevation.text = root.fmt(el);
        }
    }

    // ── 安装标定：**改动即设定，没有「设定」按钮**（用户定：有按钮的话实际用起来老是忘了按）。
    // 拒绝原因由 setMounting 记录；界面仅在写入成功后记录操作日志。
    Subheading {
        text: qsTr("安装标定")
        Layout.topMargin: 8
    }
    // 标定三个角一次提交，避免把一份安装姿态拆成三次写入。
    // 创建时读完整姿态；后续直接使用信号载荷，不在收到通知后回读设备。
    // 上下限写死：三个角都收整圈（方位 0~360，高度 / 横滚 ±180，与读回的窗一致），不从 C++ 问（用户定）。
    FieldStrip {
        id: mount
        Component.onCompleted: load(root.device.mounting)
        function load(pose) {
            az.value = pose.azimuth;
            el.value = pose.elevation;
            roll.value = pose.roll;
        }
        function apply() {
            if (root.device.setMounting(az.value, el.value, roll.value))
                root.device.logMessage(qsTr("安装标定已设定：方位 %1°，高度 %2°，横滚 %3°").arg(az.value.toFixed(2)).arg(el.value.toFixed(2)).arg(roll.value.toFixed(2)));
        }
        FieldStrip.Col {
            label: qsTr("零位方位角（°）")
            RealField {
                id: az
                Layout.fillWidth: true
                minimum: 0
                maximum: 360
                value: 0
                onCommitted: function (v) {
                    az.value = v;
                    mount.apply();
                }
            }
        }
        FieldStrip.Col {
            label: qsTr("零位高度角（°）")
            RealField {
                id: el
                Layout.fillWidth: true
                minimum: -180
                maximum: 180
                value: 0
                onCommitted: function (v) {
                    el.value = v;
                    mount.apply();
                }
            }
        }
        FieldStrip.Col {
            label: qsTr("零位横滚角（°）")
            RealField {
                id: roll
                Layout.fillWidth: true
                minimum: -180
                maximum: 180
                value: 0
                onCommitted: function (v) {
                    roll.value = v;
                    mount.apply();
                }
            }
        }
    }
    HintText {
        text: qsTr("改动即生效，不必另按设定。")
    }

    Button {
        Layout.fillWidth: true
        Layout.topMargin: 8
        text: qsTr("立即停止")
        onClicked: {
            if (root.device.issueStop())
                root.device.logMessage(qsTr("已下发：停止"));
        }
    }
}
