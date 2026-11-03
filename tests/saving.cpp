#include "AcqSaver.h"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <initializer_list>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QList>
#include <QTemporaryDir>
#include <QThread>

namespace {
int checks = 0;
void check(bool ok, const char *message)
{
    ++checks;
    if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}

// 只等待测试自己提交的单个文件成为完整 JSON，不给生产保存器增加排空/回调接口。
QJsonObject readCompleted(const QString &path)
{
    QElapsedTimer elapsed;
    elapsed.start();
    while (elapsed.elapsed() < 5000) {
        QFile file(path);
        if (file.open(QIODevice::ReadOnly)) {
            const auto document = QJsonDocument::fromJson(file.readAll());
            if (document.isObject()) return document.object();
        }
        QThread::msleep(5);
    }
    check(false, "saved JSON did not complete");
    return {};
}

void arrayEquals(const QJsonObject &json, const char *key, std::initializer_list<double> expected)
{
    const auto values = json.value(QLatin1String(key)).toArray();
    check(values.size() == static_cast<qsizetype>(expected.size()), key);
    qsizetype index = 0;
    for (const auto value : expected) check(std::abs(values[index++].toDouble() - value) < 1e-6, key);
}
}

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    QTemporaryDir temporary;
    check(temporary.isValid(), "temporary output directory");
    const QDir root(temporary.path());
    const QString session = QStringLiteral("session");
    const auto at = std::chrono::system_clock::time_point(std::chrono::seconds(100000));

    // 旧聚合调用仍可直接使用，且不凭空补充均匀距离数据。
    AcqSaver::saveRadialWind(root, session, "uniform", "frame-time", {{"CDL", {{1, 2}, {3, 4}}}});
    const auto uniform = readCompleted(root.filePath("Radialwind/session/uniform.txt"));
    arrayEquals(uniform, "Vrcdl", {1, 2});
    arrayEquals(uniform, "CNRcdl", {3, 4});
    check(uniform.size() == 3 && uniform["date"] == "frame-time", "uniform radial format unchanged");

    {
        QVector<AcqSaver::NonuniformRadialChannel> channels{
            {"ADC0", {12.5, 25, 61.75}, {{1, 2, 3}, {-10, -11, -12}}},
            {"ADC1", {3, 80}, {{4, 5}, {6, 7}}}
        };
        AcqSaver::saveRadialWind(root, session, "irregular", "source-time", channels);
        // 调用方覆盖和销毁输入，线程仍应持有原份额；各通道可用不同网格。
        channels[0].ranges[0] = 999;
        channels[0].data.ws[0] = 999;
    }
    const auto radial = readCompleted(root.filePath("Radialwind/session/irregular.txt"));
    arrayEquals(radial, "Rangeadc0", {12.5, 25, 61.75});
    arrayEquals(radial, "Vradc0", {1, 2, 3});
    arrayEquals(radial, "CNRadc0", {-10, -11, -12});
    arrayEquals(radial, "Rangeadc1", {3, 80});
    arrayEquals(radial, "Vradc1", {4, 5});
    arrayEquals(radial, "CNRadc1", {6, 7});
    check(radial["date"] == "source-time", "radial timestamp preserved");

    {
        auto frame = std::make_shared<NonuniformVerticalWindProfileUnit>();
        frame->at = at;
        frame->heights = {50.5, 80, 220.25};
        frame->winds = {{3, 4, 1}, {0, -2, -3}, {-4, 0, 2}};
        AcqSaver::saveWindProfile(NonuniformVerticalWindProfilePtr(std::move(frame)), root, session, "ADC0");
    }
    const auto profile = readCompleted(root.filePath("Windprofile/ADC0_session.txt"));
    arrayEquals(profile, "High", {50.5, 80, 220.25});
    arrayEquals(profile, "HSpeed", {5, 2, 4});
    arrayEquals(profile, "VSpeed", {1, -3, 2});
    arrayEquals(profile, "HDir", {216.8699, 0, 90});
    const auto expectedTime = QDateTime::fromSecsSinceEpoch(100000).toString("yyyy-MM-dd HH:mm:ss");
    check(profile["Date"] == expectedTime, "profile uses acquisition time");

    AcqSaver::saveWindProfile(std::make_shared<const VerticalWindProfileUnit>(
        VerticalWindProfileUnit{at, 50, 30, {{1, 0, 0}, {2, 0, 0}, {3, 0, 0}}}), root, session, "SIM");
    arrayEquals(readCompleted(root.filePath("Windprofile/SIM_session.txt")), "High", {50, 80, 110});

    // 不等长、重复、逆序、非有限位置在入口拒绝，不能覆盖已有合法文件。
    for (const auto &ranges : QList<QVector<double>>{{1, 2}, {1, 1, 2}, {2, 1, 3},
            {1, std::numeric_limits<double>::quiet_NaN(), 3}, {1, 2, std::numeric_limits<double>::infinity()}}) {
        AcqSaver::saveRadialWind(root, session, "irregular", "invalid",
            QVector<AcqSaver::NonuniformRadialChannel>{{"ADC0", ranges, {{1, 2, 3}, {4, 5, 6}}}});
        auto frame = std::make_shared<NonuniformVerticalWindProfileUnit>();
        frame->at = at;
        frame->heights.assign(ranges.cbegin(), ranges.cend());
        frame->winds = {{1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
        AcqSaver::saveWindProfile(NonuniformVerticalWindProfilePtr(std::move(frame)), root, session, "ADC0");
    }
    AcqSaver::saveWindProfile(NonuniformVerticalWindProfilePtr{}, root, session, "ADC0");
    AcqSaver::saveRadialWind(root, session, "invalid", "invalid",
        QVector<AcqSaver::NonuniformRadialChannel>{{"ok", {1}, {{2}, {3}}}, {"bad", {1}, {{2}, {}}}});
    check(!QFile::exists(root.filePath("Radialwind/session/invalid.txt")), "whole group rejected before file creation");
    check(readCompleted(root.filePath("Radialwind/session/irregular.txt")) == radial, "bad radial cannot overwrite");
    check(readCompleted(root.filePath("Windprofile/ADC0_session.txt")) == profile, "bad profile cannot overwrite");

    // 单层无需除以 N-1；空数组也保持等长，不制造虚假坐标。
    AcqSaver::saveWindProfile(std::make_shared<const NonuniformVerticalWindProfileUnit>(
        NonuniformVerticalWindProfileUnit{at, {99}, {{1, 2, 3}}}), root, session, "single");
    arrayEquals(readCompleted(root.filePath("Windprofile/single_session.txt")), "High", {99});
    AcqSaver::saveRadialWind(root, session, "empty", "empty",
        QVector<AcqSaver::NonuniformRadialChannel>{{"ADC0", {}, {{}, {}}}});
    const auto empty = readCompleted(root.filePath("Radialwind/session/empty.txt"));
    arrayEquals(empty, "Rangeadc0", {});
    arrayEquals(empty, "Vradc0", {});
    arrayEquals(empty, "CNRadc0", {});
    std::cout << checks << " saving checks passed\n";
}
