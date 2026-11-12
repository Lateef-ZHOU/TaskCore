#include "AcqSaver.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include <QByteArray>
#include <QByteArrayView>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QIODevice>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QList>
#include <QString>
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

// 两路：A 正常换算；B 以 NaN 参考造出缺测风速，检查缺测写成 null。
struct SavedA
{
    static constexpr const char *name = "A";
    static constexpr double wavelengthM = 1.55e-6, zeroWindMHz = 1.5;
    static constexpr int rows = 2, columns = 4;
    static constexpr double rangeStartM = 0, rangeStepM = 15, frequencyStartMHz = 0, frequencyStepMHz = 1;
};
struct SavedB : SavedA
{
    static constexpr const char *name = "B";
    static constexpr double zeroWindMHz = std::numeric_limits<double>::quiet_NaN();
};
class SavedFrame final : public TypedDAQCollector<DAQChannel<SavedA>, DAQChannel<SavedB>>
{
public:
    explicit SavedFrame(const DAQParameters &p) : TypedDAQCollector(p) {}
    QList<QByteArray> buildStart() const override { return {}; }
    int timeoutMs() const override { return 0; }
private:
    std::optional<Frame> assemble(QByteArrayView, RxTime at) override
    {
        frameTime_ = at;
        const std::vector<double> values{1, 2, 9, 2, 1, 3, 8, 3};
        return Frame{DAQChannel<SavedA>(values), DAQChannel<SavedB>(values)};
    }
};
static_assert(DAQCollectorLike<SavedFrame>);

template<class T>
concept SaveFrameAccepted = requires(std::shared_ptr<T> frame) {
    AcqSaver::saveFrame(std::move(frame), QString{});
};
static_assert(SaveFrameAccepted<const SavedFrame>);
static_assert(!SaveFrameAccepted<SavedFrame>);

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
    auto collecting = std::make_shared<SavedFrame>(DAQParameters(TestKey(), 3, std::nullopt, root, QStringLiteral("session"), false));
    check(collecting->feed(QByteArray("x"), at), "frame did not complete");
    const std::shared_ptr<const SavedFrame> frame = std::move(collecting);
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
    AcqSaver::saveFrame(std::make_shared<const SavedFrame>(DAQParameters(TestKey(), 1, std::nullopt, std::nullopt, {}, false)),
                        QStringLiteral("Card"));
    std::cout << "saving checks passed: " << checks << '\n';
    return 0;
}
