import QtQuick
import Lidar.Charts

AngleChartItem {
    id: root
    required property var device
    // configuration 是绘图器的 required 创建属性，由官邸随模板一次交入。
    Connections {
        target: root.device
        function onAnglesReady(at, pan, tilt, azimuth, elevation) {
            root.append(at, pan, tilt, azimuth, elevation)
        }
    }
}
