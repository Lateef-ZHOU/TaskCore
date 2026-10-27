#include <QGuiApplication>
#include <QQmlComponent>
#include <QQmlEngine>
#include <QQmlListReference>
#include <QtQml/QQmlExtensionPlugin>
#include <QDebug>
#include <memory>
#include "Task/TaskBase.h"
Q_IMPORT_QML_PLUGIN(LidarUIPlugin)
Q_IMPORT_QML_PLUGIN(LidarChartsPlugin)
Q_IMPORT_QML_PLUGIN(LidarTaskUIPlugin)

int main(int argc, char **argv)
{
    QGuiApplication app(argc, argv);
    QQmlEngine engine;
    QQmlComponent component(&engine);
    bool warned = false;
    QObject::connect(&engine, &QQmlEngine::warnings, &engine, [&](const auto &) { warned = true; });
    component.setData(R"(import QtQuick
import Lidar.UI
import Lidar.TaskUI
TaskModule {
    gimbals: []
    cards: []
    planPage: Section { property list<real> azimuths: [0]; property real elevation: 60 }
    function start(azimuths, elevation): bool { return false }
    function stop(): void {}
})", QUrl());
    std::unique_ptr<QObject> page(component.createWithInitialProperties(
        {{"settingsParameters", TaskBase::settingsParameters()}}));
    if (!page) { qCritical() << component.errors(); return 1; }
    const QQmlListReference sections(page.get(), "sections");
    if (warned || sections.count() != 4 || !sections.at(1)
        || sections.at(3)->property("plan").value<QObject *>() != sections.at(1)) return 2;
    QQmlComponent sim(&engine);
    sim.setData(R"(import QtQuick
import Lidar.UI
import Lidar.TaskUI
SimOnlyTaskModule {
    gimbal: null
    card: null
    accum: 7
    settle: 4
    planPage: Section {}
    property int starts: 0
    property int stops: 0
    property int argumentCount: -1
    function start(): bool { argumentCount = arguments.length; ++starts; return true }
    function stop(): void { argumentCount = arguments.length; ++stops }
})", QUrl());
    std::unique_ptr<QObject> simPage(sim.createWithInitialProperties(
        {{"settingsParameters", TaskBase::settingsParameters()}}));
    if (!simPage) { qCritical() << sim.errors(); return 3; }
    // 真正触发运行分区的按钮，确认只发无参命令、不再反摸计划。
    for (const auto &text : {QStringLiteral("开始"), QStringLiteral("停止")}) {
        QObject *button = nullptr;
        for (auto *child : simPage->findChildren<QObject *>())
            if (child->property("text").toString() == text) { button = child; break; }
        if (!button || !QMetaObject::invokeMethod(button, "clicked")) return 4;
        if (simPage->property("argumentCount").toInt() != 0) return 5;
    }
    if (simPage->property("starts").toInt() != 1 || simPage->property("stops").toInt() != 1 || warned) return 6;
    return 0;
}
