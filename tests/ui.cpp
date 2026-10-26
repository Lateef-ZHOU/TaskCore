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
    return 0;
}
