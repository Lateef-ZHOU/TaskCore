// 三种波束任务共用的预览与进度布局；路径画法由官邸直接指定。
pragma ComponentBehavior: Bound
import QtQuick
import Lidar.UI
import QtQuick.Controls
import QtQuick.Layouts

RowLayout {
    id: page
    required property var task
    required property var plan
    required property var settings
    readonly property var gimbal: settings.gimbal
    property var preview: ({ reachable: [], order: [], rows: [], hint: "", slipRing: false })

    function refreshPreview(mounting): void {
        const reachable = [], order = [], rows = []
        let valid = true
        const slipRing = gimbal ? gimbal.slipRing : false
        if (gimbal) {
            for (let i = 0; i < plan.azimuths.length; ++i) {
                const az = plan.azimuths[i], el = plan.elevation
                const pose = mounting.gimbalAngles(az, el)
                // 预览只查询型号判据，Collector 由运行协程请求。
                const reach = Number.isFinite(az) && Number.isFinite(el)
                    && gimbal.withinTravel(pose.pan, pose.tilt)
                reachable.push(reach)
                valid = valid && reach
                rows.push({ step: "", worldAz: az.toFixed(2) + "°", worldEl: el.toFixed(2) + "°",
                    pan: pose.pan.toFixed(2) + "°" + (reach ? "" : qsTr("（超行程）")),
                    tilt: pose.tilt.toFixed(2) + "°" })
            }
            for (let i = 0; i < rows.length; ++i) {
                rows[i].step = valid ? String(i + 1) : "—"
                if (valid) order.push(i)
            }
        }
        const hint = !gimbal ? qsTr("没有可选的云台")
            : !valid ? qsTr("有束够不着（×）：计划无效，开始会被拒绝。")
            : slipRing ? qsTr("数字 = 行进顺序（即输入顺序）。本云台有滑环，可一直同向转，轮末跨零续转即可。")
            : qsTr("数字 = 行进顺序（即输入顺序）。本云台无滑环，请参照表中云台水平角人工排成单调。")
        preview = { reachable: reachable, order: order, rows: rows, hint: hint, slipRing: slipRing }
    }
    onGimbalChanged: refreshPreview(gimbal ? gimbal.mounting : null)
    Component.onCompleted: refreshPreview(gimbal ? gimbal.mounting : null)
    Connections {
        target: page.plan
        function onAzimuthsChanged(): void { page.refreshPreview(page.gimbal ? page.gimbal.mounting : null) }
        function onElevationChanged(): void { page.refreshPreview(page.gimbal ? page.gimbal.mounting : null) }
    }
    Connections {
        target: page.gimbal
        function onMountingChanged(mounting): void { page.refreshPreview(mounting) }
    }

    required property int pathStyle
    Layout.fillWidth: true
    Layout.fillHeight: true
    spacing: 16

    ColumnLayout {
        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.horizontalStretchFactor: 3
        spacing: 6

        Label { text: qsTr("波束罗盘"); font.bold: true }

        // **颜色从这儿绑进去**：QQuickItem 没有 QPalette，主题一变绑定自己会跟；
        // 任务色与"够不着"的红不随主题走，从 Theme 单例取（全仓唯一出处）。
        BeamCompass {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 300

            azimuths: page.plan.azimuths
            elevation: page.plan.elevation
            reachable: page.preview.reachable
            order: page.preview.order
            pathStyle: page.pathStyle
            slipRing: page.preview.slipRing
            current: page.task.currentBeam

            gridColor: palette.midlight
            rimColor: palette.mid
            dimColor: palette.placeholderText
            inkColor: palette.windowText
            baseColor: palette.base
            taskColor: Theme.task
            badColor: Theme.unreachable
        }

        Label {
            Layout.fillWidth: true
            text: page.preview.hint
            wrapMode: Text.Wrap
            Layout.preferredWidth: 1 // 换行文字的 implicitWidth 是「整段不折行」的宽度，会顶爆列宽
            color: palette.placeholderText
        }
    }

    ColumnLayout {
        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.horizontalStretchFactor: 2
        spacing: 6

        Label { text: qsTr("计划表"); font.bold: true }

        // 计划表：一行表头 + 每束一行。**五列等宽**，文字由本页按当前世界角和安装标定生成。
        // 四行的表用不上 TableView 那套（列宽同步、滚动、选择模型全用不着）。
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: palette.base
            border.color: palette.mid
            border.width: 1
            radius: 4

            component Cell: Label {
                Layout.fillWidth: true
                Layout.preferredWidth: 60 // 等宽平分，别虚报（理由见 FieldStrip.qml 头注）
                horizontalAlignment: Text.AlignHCenter
                elide: Text.ElideRight
            }

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 1
                spacing: 0

                Rectangle {
                    Layout.fillWidth: true
                    implicitHeight: head.implicitHeight + 10
                    color: palette.alternateBase
                    RowLayout {
                        id: head
                        anchors.fill: parent
                        anchors.margins: 5
                        spacing: 4
                        Cell { text: qsTr("行进"); font.bold: true }
                        Cell { text: qsTr("世界方位"); font.bold: true }
                        Cell { text: qsTr("世界高度"); font.bold: true }
                        Cell { text: qsTr("云台水平"); font.bold: true }
                        Cell { text: qsTr("云台俯仰"); font.bold: true }
                    }
                }

                Repeater {
                    model: page.preview.rows
                    delegate: Rectangle {
                        id: beamRow
                        required property int index
                        required property var modelData
                        Layout.fillWidth: true
                        implicitHeight: row.implicitHeight + 10
                        // 当前束点一层任务色的淡底（与罗盘同一支色）
                        color: index === page.task.currentBeam ? Qt.alpha(Theme.task, 0.16) : "transparent"
                        RowLayout {
                            id: row
                            anchors.fill: parent
                            anchors.margins: 5
                            spacing: 4
                            Cell { text: beamRow.modelData.step }
                            Cell { text: beamRow.modelData.worldAz }
                            Cell { text: beamRow.modelData.worldEl }
                            Cell { text: beamRow.modelData.pan }
                            Cell { text: beamRow.modelData.tilt }
                        }
                    }
                }

                Item { Layout.fillHeight: true } // 把行顶到上方
            }
        }

        Label { text: qsTr("运行"); font.bold: true; Layout.topMargin: 6 }

        GridLayout {
            Layout.fillWidth: true
            columns: 2
            columnSpacing: 12
            rowSpacing: 4

            component Name: Label {
                color: palette.placeholderText
                Layout.preferredWidth: 70
            }

            Name { text: qsTr("状态") }
            RowLayout {
                spacing: 6
                Layout.fillWidth: true
                // ● 运行中是任务色、空闲是灰
                Label {
                    text: "●"
                    color: page.task.running ? Theme.task : palette.placeholderText
                }
                Label {
                    Layout.fillWidth: true
                    elide: Text.ElideRight
                    text: page.task.beamText ? page.task.beamText + " · " + page.task.phaseText
                                             : page.task.phaseText
                }
            }

            Name { text: qsTr("已采集") }
            Label { Layout.fillWidth: true; text: page.task.deliveredText }

            Name { text: qsTr("拿锁范围") }
            Label {
                Layout.fillWidth: true
                text: page.task.scopeText
                wrapMode: Text.Wrap
                Layout.preferredWidth: 1 // 换行文字的 implicitWidth 是「整段不折行」的宽度，会顶爆列宽
            }

            Name { text: qsTr("开始于") }
            Label { Layout.fillWidth: true; text: page.task.sinceText }
        }
    }
}
