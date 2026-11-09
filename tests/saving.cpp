#include "AcqSaver.h"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
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

// 两路：A 已标定（零风速参考在频带内），B 未标定（风速 NaN）。
class SavedFrame final : public DAQCollector
{
public:
    static std::shared_ptr<SavedFrame> create(const DAQParameters &p)
    { return std::shared_ptr<SavedFrame>(new SavedFrame(p.accumSec, p.context)); }
    std::shared_ptr<SavedFrame> makeSelf() const { return create({accumSec(), context()}); }
    QList<QByteArray> buildStart() const override { return {}; }
    int timeoutMs() const override { return 0; }
private:
    SavedFrame(int seconds, DAQContext context)
        : DAQCollector(std::move(context), seconds, {channel("A", 1.5), channel("B", 99.0)}) {}
    static DAQChannel channel(const char *name, double zero)
    {
        DAQChannel c;
        c.name = QString::fromLatin1(name);
        c.rows = 2;
        c.columns = 4;
        c.rangeM = {0, 15};
        c.freqMHz = {0, 1, 2, 3};
        c.zeroWindMHz = zero;
        c.metersPerSecondPerMHz = 0.775;
        return c;
    }
    void onFeed(const std::optional<QByteArray> &, RxTime at) override
    {
        for (auto &c : *this)
            c.spectrum = {1, 2, 9, 2, 1, 3, 8, 3};
        frameTime_ = at;
        complete();
    }
};
static_assert(DAQCollectorLike<SavedFrame>);
}

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    QTemporaryDir temp;
    check(temp.isValid(), "temporary directory unavailable");
    const QDir root(temp.path());
    const auto at = std::chrono::system_clock::now();

    // 无朝向帧：朝向写 null，未标定通道的风速写 null，CNR 照常。
    auto frame = SavedFrame::create({3, {std::nullopt, root, QStringLiteral("session"), false}});
    check(frame->feed(QByteArray("x"), at), "frame did not complete");
    AcqSaver::saveFrame(frame, QStringLiteral("Card"));
    const auto saved = readCompleted(root.filePath("session/" + AcqSaver::fileStem(at) + ".json"));
    const auto info = saved.value("frame").toObject();
    check(info.value("session").toString() == "session" && info.value("frameTime").toString() == AcqSaver::timeStem(at)
          && info.value("azimuthDeg").isNull() && info.value("elevationDeg").isNull()
          && !info.value("sync").toBool() && info.value("accumSec").toInt() == 3 && info.value("card").toString() == "Card",
          "frame information differs");
    const auto channels = saved.value("channels").toArray();
    check(channels.size() == 2, "channel count differs");
    const auto a = channels[0].toObject(), b = channels[1].toObject();
    check(a.value("name").toString() == "A" && b.value("name").toString() == "B", "channel order or names differ");
    check(a.value("rangeM").toArray().size() == 2 && a.value("freqMHz").toArray().size() == 4
          && a.value("zeroWindMHz").toDouble() == 1.5, "channel axes differ");
    const auto spectrum = a.value("spectrum").toArray();
    check(spectrum.size() == 2 && spectrum[0].toArray().size() == 4 && spectrum[1].toArray()[2].toDouble() == 8,
          "spectrum rows differ");
    check(a.value("radialWind").toArray().size() == 2 && !a.value("radialWind").toArray()[0].isNull(), "calibrated wind missing");
    check(b.value("radialWind").toArray()[0].isNull() && !b.value("cnr").toArray()[0].isNull(), "uncalibrated wind not null");

    // 风廓线：东风 5 m/s、上升 0.5 m/s → 来向 90°。
    auto unit = std::make_shared<NonuniformVerticalWindProfileUnit>();
    unit->at = at;
    unit->heights = {10, 20};
    unit->winds = {LinAlg::Vec3(-5, 0, 0.5), LinAlg::Vec3(0, -3, 0)};
    AcqSaver::saveWindProfiles(frame, {{QStringLiteral("A"), unit}});
    const auto profile = readCompleted(root.filePath("session/profile_" + AcqSaver::fileStem(at) + ".json"));
    const auto first = profile.value("channels").toArray()[0].toObject();
    check(profile.value("frame").toObject().value("session").toString() == "session"
          && first.value("name").toString() == "A" && first.value("heightM").toArray().size() == 2, "profile layout differs");
    check(std::abs(first.value("horizontalDirectionDeg").toArray()[0].toDouble() - 90.0) < 1e-3
          && std::abs(first.value("horizontalSpeed").toArray()[0].toDouble() - 5.0) < 1e-3
          && std::abs(first.value("verticalSpeed").toArray()[0].toDouble() - 0.5) < 1e-3
          && std::abs(first.value("horizontalDirectionDeg").toArray()[1].toDouble() - 0.0) < 1e-3,
          "profile direction or speeds differ");

    // 不落盘：saveRoot 为空时入口直接返回。
    AcqSaver::saveFrame(SavedFrame::create({1, {}}), QStringLiteral("Card"));
    std::cout << "saving checks passed: " << checks << '\n';
    return 0;
}
