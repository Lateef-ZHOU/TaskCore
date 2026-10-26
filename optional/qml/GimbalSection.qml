// GimbalSection —— 云台族共享的「状态与定位」内容；官邸随模板交入设备和固定行程。
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
        return (v === undefined || isNaN(v)) ? "—" : v.toFixed(2) + "°"
    }

    // ── 实时角度：**四个读数并排一行**（用户定：右栏够宽，别占两行）──
    // 四列等宽，每列「上名目 + 下读数」。
    // 纯数字读数使用等宽字体，避免数值变化时字符位置抖动。
    component Readout: Label {
        color: palette.placeholderText
        Layout.fillWidth: true
        Layout.preferredWidth: 64 // 等宽平分，别虚报（理由见 FieldStrip.qml 头注）
        elide: Text.ElideRight
    }
    component ReadoutValue: Label {
        text: "—"
        font.family: "Consolas"
        font.pointSize: 13
        Layout.fillWidth: true
        Layout.preferredWidth: 64 // 同上，与名目列同宽
    }

    GridLayout {
        Layout.fillWidth: true
        columns: 4
        columnSpacing: 10
        rowSpacing: 2

        Readout { text: qsTr("云台水平") }
        Readout { text: qsTr("云台俯仰") }
        Readout { text: qsTr("世界方位") }
        Readout { text: qsTr("世界高度") }
        ReadoutValue { id: pan }
        ReadoutValue { id: tilt }
        ReadoutValue { id: azimuth }
        ReadoutValue { id: elevation }
    }

    Connections {
        target: root.device
        function onMountingChanged(pose) { mount.load(pose) }
        function onAnglesReady(at, p, ti, az, el) {
            pan.text = root.fmt(p)
            tilt.text = root.fmt(ti)
            azimuth.text = root.fmt(az)
            elevation.text = root.fmt(el)
        }
    }

    // ── 安装标定：**改动即设定，没有「设定」按钮**（用户定：有按钮的话实际用起来老是忘了按）。
    // 拒绝原因由 setMounting 记录；界面仅在写入成功后记录操作日志。
    Label { text: qsTr("安装标定"); font.bold: true; Layout.topMargin: 8 }
    // 标定三个角一次提交，避免把一份安装姿态拆成三次写入。
    // 创建时读完整姿态；后续直接使用信号载荷，不在收到通知后回读设备。
    // 上下限写死：三个角都收整圈（方位 0~360，高度 / 横滚 ±180，与读回的窗一致），不从 C++ 问（用户定）。
    FieldStrip {
        id: mount
        Component.onCompleted: load(root.device.mounting)
        function load(pose) {
            az.value = pose.azimuth
            el.value = pose.elevation
            roll.value = pose.roll
        }
        function apply() {
            if (root.device.setMounting(az.value, el.value, roll.value))
                root.device.device.logSource.logMessage(qsTr("安装标定已设定：方位 %1°，高度 %2°，横滚 %3°")
                    .arg(az.value.toFixed(2)).arg(el.value.toFixed(2)).arg(roll.value.toFixed(2)))
        }
        FieldStrip.Col {
            label: qsTr("零位方位角（°）")
            RealField { id: az; Layout.fillWidth: true
                        minimum: 0; maximum: 360
                        value: 0
                        onCommitted: function (v) { az.value = v; mount.apply() } }
        }
        FieldStrip.Col {
            label: qsTr("零位高度角（°）")
            RealField { id: el; Layout.fillWidth: true
                        minimum: -180; maximum: 180
                        value: 0
                        onCommitted: function (v) { el.value = v; mount.apply() } }
        }
        FieldStrip.Col {
            label: qsTr("零位横滚角（°）")
            RealField { id: roll; Layout.fillWidth: true
                        minimum: -180; maximum: 180
                        value: 0
                        onCommitted: function (v) { roll.value = v; mount.apply() } }
        }
    }
    Label {
        text: qsTr("改动即生效，不必另按设定。")
        color: palette.placeholderText
        font.pointSize: 9
        wrapMode: Text.Wrap
        Layout.fillWidth: true
        Layout.preferredWidth: 1 // 换行文字的 implicitWidth 是「整段不折行」的宽度，会顶爆列宽
    }

    // ── 前往云台系：两轴一起 ──
    Label { text: qsTr("前往云台系"); font.bold: true; Layout.topMargin: 8 }
    // **C++ 侧零存储**：这两个是一次性动作的入参，按「页面重建后该不该还在 → 不该」的判据就住界面，
    // 点按钮时当实参递下去。
    // **谁持值谁回写**：committed 只是"新的合法值出现了"这条通知，它不替人改 value。
    // 初值走 value 绑定而不是 Component.onCompleted —— 后者在实例上再写一份会**覆盖**
    // ValueField 自带的那份（同一个附加属性、同一个处理槽）；绑定在首次提交时被赋值打断，
    // 那正是"此后归界面自己管"的意思。
    FieldStrip {
        FieldStrip.Col {
            label: qsTr("水平角（°）")
            RealField { id: tPan; Layout.fillWidth: true
                        minimum: root.panMin; maximum: root.panMax
                        value: root.panMin
                        onCommitted: function (v) { tPan.value = v } }
        }
        FieldStrip.Col {
            label: qsTr("俯仰角（°）")
            RealField { id: tTilt; Layout.fillWidth: true
                        minimum: root.tiltMin; maximum: root.tiltMax
                        value: root.tiltMin
                        onCommitted: function (v) { tTilt.value = v } }
        }
    }
    Button {
        Layout.fillWidth: true
        text: qsTr("前往云台角")
        onClicked: {
            if (root.device.issueGoTo(tPan.value, tTilt.value, root.settings.moveParameters()))
                root.device.device.logSource.logMessage(qsTr("已下发：前往云台角（水平 %1°，俯仰 %2°）")
                    .arg(tPan.value.toFixed(2)).arg(tTilt.value.toFixed(2)))
        }
    }

    // ── 前往世界系：经族口 gimbalOf 减安装标定 + 逆运动学；两个角都收整圈，上下限写死，初值水平正北 ──
    Label { text: qsTr("前往世界系"); font.bold: true; Layout.topMargin: 8 }
    FieldStrip {
        FieldStrip.Col {
            label: qsTr("世界方位（°）")
            RealField { id: tAz; Layout.fillWidth: true
                        minimum: 0; maximum: 360
                        value: 0
                        onCommitted: function (v) { tAz.value = v } }
        }
        FieldStrip.Col {
            label: qsTr("世界高度（°）")
            RealField { id: tEl; Layout.fillWidth: true
                        minimum: -180; maximum: 180
                        value: 0
                        onCommitted: function (v) { tEl.value = v } }
        }
    }
    Button {
        Layout.fillWidth: true
        text: qsTr("前往世界角")
        onClicked: {
            const pose = root.device.mounting.gimbalAngles(tAz.value, tEl.value)
            if (root.device.issueGoTo(pose.pan, pose.tilt, root.settings.moveParameters()))
                root.device.device.logSource.logMessage(qsTr("已下发：前往世界角（方位 %1°，高度 %2°） → 云台（水平 %3°，俯仰 %4°）")
                    .arg(tAz.value.toFixed(2)).arg(tEl.value.toFixed(2))
                    .arg(pose.pan.toFixed(2)).arg(pose.tilt.toFixed(2)))
        }
    }

    Button {
        Layout.fillWidth: true
        Layout.topMargin: 8
        text: qsTr("立即停止")
        onClicked: {
            if (root.device.issueStop())
                root.device.device.logSource.logMessage(qsTr("已下发：停止"))
        }
    }
}
