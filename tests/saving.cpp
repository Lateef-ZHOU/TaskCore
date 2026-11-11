#include <limits>
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
    explicit SavedFrame(const DAQParameters &p) : DAQCollector(p, 1.55e-6) {}
    QList<QByteArray> buildStart() const override { return {}; }
    int timeoutMs() const override { return 0; }
private:
    std::optional<std::vector<DAQChannel>> assemble(QByteArrayView, RxTime at) override
    {
        frameTime_ = at;
        std::vector<DAQChannel> channels;
        // B 以 NaN 参考造出缺测风速，检查缺测写成 null。
        for (const auto &[name, zero] : {std::pair{"A", 1.5}, std::pair{"B", std::numeric_limits<double>::quiet_NaN()}}) {
            LinearArray2D<double> spectrum(2, 4, 0, 15, 0, 1);
            static_cast<std::vector<double> &>(spectrum) = {1, 2, 9, 2, 1, 3, 8, 3};
            channels.emplace_back(QString::fromLatin1(name), std::move(spectrum), zero, wavelengthM());
        }
        return channels;
    }
};
static_assert(DAQCollectorLike<SavedFrame>);

// 存盘测试不经设备：自行派生通行证造参数（通行证只防误用，不防刻意派生）。
struct TestKey : DAQParamsKey { TestKey() {} };
}

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    QTemporaryDir temp;
    check(temp.isValid(), "temporary directory unavailable");
    const QDir root(temp.path());
    const auto at = std::chrono::system_clock::now();

    // 无朝向帧：朝向写 null，未标定通道的风速写 null，CNR 照常。
    auto frame = std::make_shared<SavedFrame>(DAQParameters(TestKey(), 3, std::nullopt, root, QStringLiteral("session"), false));
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
    check(a.value("rangeStartM").toDouble() == 0 && a.value("rangeStepM").toDouble() == 15
          && a.value("freqStartMHz").toDouble() == 0 && a.value("freqStepMHz").toDouble() == 1
          && a.value("zeroWindMHz").toDouble() == 1.5, "channel axes differ");
    const auto spectrum = a.value("spectrum").toArray();
    check(spectrum.size() == 2 && spectrum[0].toArray().size() == 4 && spectrum[1].toArray()[2].toDouble() == 8,
          "spectrum rows differ");
    check(a.value("radialWind").toArray().size() == 2 && !a.value("radialWind").toArray()[0].isNull(), "wind missing");
    check(b.value("radialWind").toArray()[0].isNull() && !b.value("cnr").toArray()[0].isNull(), "missing wind not null");

    // 风廓线：东风 5 m/s、上升 0.5 m/s → 来向 90°。
    auto unit = std::make_shared<VerticalWindProfileUnit>(VerticalWindProfileUnit{at,
        LinearArray<LinAlg::Vec3>(10.0, 10.0, {LinAlg::Vec3(-5, 0, 0.5), LinAlg::Vec3(0, -3, 0)})});
    AcqSaver::saveWindProfiles(frame, {{QStringLiteral("A"), unit}});
    const auto profile = readCompleted(root.filePath("session/profile_" + AcqSaver::fileStem(at) + ".json"));
    const auto first = profile.value("channels").toArray()[0].toObject();
    check(profile.value("frame").toObject().value("session").toString() == "session"
          && first.value("name").toString() == "A" && first.value("heightStartM").toDouble() == 10
          && first.value("heightStepM").toDouble() == 10 && first.value("horizontalSpeed").toArray().size() == 2, "profile layout differs");
    check(std::abs(first.value("horizontalDirectionDeg").toArray()[0].toDouble() - 90.0) < 1e-3
          && std::abs(first.value("horizontalSpeed").toArray()[0].toDouble() - 5.0) < 1e-3
          && std::abs(first.value("verticalSpeed").toArray()[0].toDouble() - 0.5) < 1e-3
          && std::abs(first.value("horizontalDirectionDeg").toArray()[1].toDouble() - 0.0) < 1e-3,
          "profile direction or speeds differ");

    // 不落盘：saveRoot 为空时入口直接返回。
    AcqSaver::saveFrame(std::make_shared<SavedFrame>(DAQParameters(TestKey(), 1, std::nullopt, std::nullopt, {}, false)),
                        QStringLiteral("Card"));
    std::cout << "saving checks passed: " << checks << '\n';
    return 0;
}
