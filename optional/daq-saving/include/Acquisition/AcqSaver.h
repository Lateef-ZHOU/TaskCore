#pragma once
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <thread>
#include <utility>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QIODevice>
#include <QString>
#include <QTextStream>
#include <QVector>

#include "Algorithm/Wind/RadialWind.h"  // RadialWind::Result（一帧的 ws/cnr 两列——落盘直接收它，不另立平行类型）
#include "Algorithm/Wind/VerticalWindProfile.h"
#include "Device/Base/RxTime.h"

// ============================================================
// AcqSaver.h — 本应用的文件格式与路径政策，由具体处理器和任务官邸共用。
// TaskCore 不依赖该工具；启动单卡测试时由官邸交入会话名。
//
// 进本文件的判据是"数数族里第二个成员用不用它"：
//   · saveSpec        —— double 列表 + 通道 tag；直接写小端 binary64 无头文件；
//   · saveWindProfile —— 收 VerticalWindProfilePtr + sessionDir + tag，反演产物是族级算法的产物；
//   · saveRadialWind  —— 收**通道名单**（每路一份 tag + ws/cnr），JSON 键由 tag 现拼。真正的型号
//                        知识只有**通道名**，而通道名是调用点给的参数：国实卡两路（CDL/HOM），
//                        国耀卡一路（CDL）、虚拟卡一路（SIM）。型号夹里没有落盘代码。
//
// **不存原始帧**（用户权衡后的取舍）：原始字节与谱是同一份信息的两个阶段——原始帧要解包 + 运算
// 一次才成谱，留后者即可。代价是**不能从原始字节重算谱**（抽谱若有 bug 无从回溯）。
//
// 形制：public save* 是入口（起 detached 线程，立即返回）；private doSave* 是真正在线程里跑
// 的 IO 体。数据份额按值进线程（谱/径向风为 QVector 隐式共享，风廓线为只读整帧 shared_ptr），与来源完全解耦
// ——来源可在入口返回后即亡，实际磁盘 IO 不占用计算 worker。
// detached 线程不 join，进程退出时若仍在写可能截断。
//
// **本层只管 IO**：
//   ① **根目录是目录对象（QDir），底下几级名字是算好的串**。root 由界面的「选文件夹」对话框经持值方
//      一路递下来，全程是 QDir——字符串只保证自己是字符串，不保证是路径（用户定）；往下拼路径一律
//      `QDir::filePath`，不手写分隔符。名字的两半来自两个不同的东西：sessionDir 是 run 级的（编排层
//      算好经 Collector 上下文递下），fileStem / timeStem 由 worker 按帧时刻生成；风廓线的 Date
//      在保存时取整帧自带时刻。日历时间与格式转换均归本类。
//   ② **存不存由调用方判**（saveRoot 空 = 不落盘，worker 判）：本层"被调用即落盘"。
// ============================================================
class AcqSaver
{
public:
    // ── 会话目录及帧级文件名、时间字段格式化 ────────────
    // 分成两组不是随意切的：sessionDir 只吃 **run 级**参数（一轮采集共用一个目录），
    // 另两个只吃 **帧级**产物（一帧一个文件名）。sessionDir 由编排层算好经 Collector 上下文递下，
    // 另两个由 worker 从斗的 frameTime 现算，本层不必认识会话或斗。
    [[nodiscard]] static QString sessionDir(const QDateTime &sessionTime, const QString &tag);
    [[nodiscard]] static QString fileStem(RxTime frameTime);
    [[nodiscard]] static QString timeStem(RxTime frameTime);

    // 一路通道的径向风产物：通道名 + 该路的 ws/cnr 两列。data 直接复用 RadialWind::Result
    // ——它恰是这两列，另立一个平行结构只会多一个要同步的类型。
    struct RadialChannel
    {
        QString tag;             // 通道名（"HOM" / "CDL"）——JSON 键由它现拼，见 doSaveRadialWind
        RadialWind::Result data; // 该路径向风速 + CNR
    };

    // 三条落盘：**被调用即落盘**（存不存归调用方判，见类注②）。路径串由调用方用上面
    // 三个格式化口算好——sessionDir 是目录名（不含前缀），fileStem/timeStem 是帧级串。
    // root = 落盘根目录（调用方给；三类产物各在 root 下的 Rawspec / Radialwind / Windprofile）。
    static void saveSpec(QDir root, QString sessionDir, QString fileStem,
                         QVector<double> spec, QString tag);
    // 调用方仅提交非空帧；线程独立持有只读份额，Date 取帧的到料时刻。
    static void saveWindProfile(VerticalWindProfilePtr result, QDir root, QString sessionDir,
                                QString tag);
    static void saveRadialWind(QDir root, QString sessionDir, QString fileStem, QString timeStem,
                               QVector<RadialChannel> channels);

private:
    [[nodiscard]] static QDateTime calendarTime(const RxTime at)
    {
        return QDateTime::fromStdTimePoint(std::chrono::time_point_cast<std::chrono::milliseconds>(at)).toLocalTime();
    }
    static void doSaveSpec(QDir root, QString sessionDir, QString fileStem,
                           const QVector<double> spec, QString tag);
    static void doSaveWindProfile(VerticalWindProfilePtr result, QDir root, QString sessionDir,
                                  QString tag);
    static void doSaveRadialWind(QDir root, QString sessionDir, QString fileStem, QString timeStem,
                                 const QVector<RadialChannel> channels);
};

// ============================================================
// AcqSaver — inline 实现
// ============================================================


// ── 会话目录及帧级时间格式化 ──────
inline QString AcqSaver::sessionDir(const QDateTime &sessionTime, const QString &tag)
{
    return sessionTime.toString("yyyyMMdd_HHmmss") + "_" + tag;
}
inline QString AcqSaver::fileStem(RxTime frameTime)
{
    return calendarTime(frameTime).toString("yyyyMMdd_HHmmss_zzz");
}
inline QString AcqSaver::timeStem(RxTime frameTime)
{
    return calendarTime(frameTime).toString("yyyy-MM-dd HH:mm:ss.zzz");
}

// ── 入口：起线程即返回（存不存归调用方判）──────

inline void AcqSaver::saveSpec(QDir root, QString sessionDir, QString fileStem,
                               QVector<double> spec, QString tag)
{
    std::thread(&AcqSaver::doSaveSpec, std::move(root), std::move(sessionDir), std::move(fileStem), std::move(spec),
                std::move(tag))
        .detach();
}

// 一轮一份风廓线：独立线程持只读整帧，时间取末束到料时刻；层数组不再拷贝。
inline void AcqSaver::saveWindProfile(VerticalWindProfilePtr result, QDir root, QString sessionDir,
                                     QString tag)
{
    std::thread(&AcqSaver::doSaveWindProfile, std::move(result), std::move(root), std::move(sessionDir),
                std::move(tag))
        .detach();
}

// [独立线程] 功率谱 → <root>/Rawspec/<sessionDir>/<tag>_<fileStem>.bin
inline void AcqSaver::doSaveSpec(QDir root, QString sessionDir, QString fileStem,
                                 const QVector<double> spec, QString tag)
{
    // 本项目的平台直接写入内存表示；编译期保证它就是约定的小端 IEEE 754 binary64。
    static_assert(std::endian::native == std::endian::little);
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559
                  && std::numeric_limits<double>::digits == 53);
    const QDir dir(root.filePath("Rawspec/" + sessionDir));
    const QString path = dir.filePath(tag + "_" + fileStem + ".bin");
    // const 遍历不触发共享数组分离；允许小数和大于 2^53 的有限功率，保存原谱而非显示钳值。
    for (double value : spec) {
        if (!std::isfinite(value) || value < 0) {
            qWarning().noquote() << "Cannot save spectrum: non-finite or negative power:" << path;
            return;
        }
    }
    if (!dir.mkpath(QStringLiteral("."))) {
        qWarning().noquote() << "Cannot create spectrum directory:" << dir.path();
        return;
    }
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        qWarning().noquote() << "Cannot open spectrum file:" << path << f.errorString();
        return;
    }
    const qint64 bytes = static_cast<qint64>(spec.size()) * static_cast<qint64>(sizeof(double));
    const qint64 written = f.write(reinterpret_cast<const char *>(spec.constData()), bytes);
    if (written != bytes) {
        qWarning().noquote() << "Incomplete spectrum write:" << path << "expected" << bytes
                            << "wrote" << written << f.errorString();
        return;
    }
    // flush 检查 Qt 缓冲的提交结果；不代表断电持久化，也不改变 detached 线程的退出边界。
    if (!f.flush())
        qWarning().noquote() << "Cannot flush spectrum file:" << path << f.errorString();
}

// 一帧一份径向风：入口按值接收共享份额，再移交线程；写入端始终只读。
inline void AcqSaver::saveRadialWind(QDir root, QString sessionDir, QString fileStem, QString timeStem,
                                     QVector<RadialChannel> channels)
{
    std::thread(&AcqSaver::doSaveRadialWind, std::move(root), std::move(sessionDir), std::move(fileStem),
                std::move(timeStem), std::move(channels))
        .detach();
}

// [独立线程] 径向风速 / CNR → <root>/Radialwind/<sessionDir>/<fileStem>.txt
// **一帧一个文件，各路并列在同一份 JSON 里**。键名由通道名现拼：`Vr<小写tag>` / `CNR<小写tag>`
// ——国实卡两路得 Vrcdl/CNRcdl/Vrhom/CNRhom，国耀卡一路得 Vrcdl/CNRcdl。
// ⚠ 键名、顺序与 6 位小数是下游解析脚本认的格式，改了要同步改脚本。
inline void AcqSaver::doSaveRadialWind(QDir root, QString sessionDir, QString fileStem, QString timeStem,
                                      const QVector<RadialChannel> channels)
{
    const QDir dir(root.filePath("Radialwind/" + sessionDir));
    const QString path = dir.filePath(fileStem + ".txt");
    if (!dir.mkpath(QStringLiteral("."))) {
        qWarning().noquote() << "Cannot create radial wind directory:" << dir.path();
        return;
    }
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning().noquote() << "Cannot open radial wind file:" << path << f.errorString();
        return;
    }
    QTextStream ts(&f);
    const auto writeArray = [&](const QVector<double> &vec) {
        ts << "[";
        for (int i = 0; i < vec.size(); ++i)
        {
            if (i > 0)
                ts << ",";
            ts << (std::isnan(vec[i]) ? "NaN" : QString::number(vec[i], 'f', 6));
        }
        ts << "]";
    };
    ts << "{\n";
    for (const RadialChannel &ch : channels)
    {
        const QString key = ch.tag.toLower();
        ts << "  \"Vr"  << key << "\":"; writeArray(ch.data.ws);  ts << ",\n";
        ts << "  \"CNR" << key << "\":"; writeArray(ch.data.cnr); ts << ",\n";
    }
    ts << "  \"date\":\"" << timeStem << "\"\n";
    ts << "}\n";
    ts.flush();
    if (ts.status() != QTextStream::Ok) {
        qWarning().noquote() << "Cannot write radial wind file:" << path << f.errorString();
        return;
    }
    if (!f.flush())
        qWarning().noquote() << "Cannot flush radial wind file:" << path << f.errorString();
}

// [独立线程] 风廓线快照 → <root>/Windprofile/<tag>_<sessionDir>.txt（一轮一份，分通道）。
inline void AcqSaver::doSaveWindProfile(const VerticalWindProfilePtr result, QDir root, QString sessionDir,
                                        QString tag)
{
    const QDir dir(root.filePath(QStringLiteral("Windprofile")));
    const QString path = dir.filePath(tag + "_" + sessionDir + ".txt");
    if (!dir.mkpath(QStringLiteral("."))) {
        qWarning().noquote() << "Cannot create wind profile directory:" << dir.path();
        return;
    }
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning().noquote() << "Cannot open wind profile file:" << path << file.errorString();
        return;
    }
    QTextStream ts(&file);
    auto writeArray = [&](auto valueAt)
    {
        ts << "[";
        for (std::size_t i = 0; i < result->winds.size(); ++i)
        {
            if (i > 0) ts << ",";
            const double value = valueAt(i);
            ts << (std::isnan(value) ? "NaN" : QString::number(value, 'f', 4));
        }
        ts << "]";
    };
    ts << "{\"HSpeed\":";
    writeArray([&](std::size_t i) {
        const auto &wind = result->winds[i];
        return wind ? std::hypot(wind.x(), wind.y()) : std::numeric_limits<double>::quiet_NaN();
    });
    ts << ",\n";
    ts << "\"HDir\":";
    writeArray([&](std::size_t i) {
        const auto &wind = result->winds[i];
        if (!wind) return std::numeric_limits<double>::quiet_NaN();
        double direction = std::atan2(-wind.x(), -wind.y()) * 180.0 / std::numbers::pi;
        if (direction < 0.0) direction += 360.0;
        if (direction >= 360.0) direction -= 360.0;
        return direction;
    });
    ts << ",\n";
    ts << "\"VSpeed\":";
    writeArray([&](std::size_t i) {
        const auto &wind = result->winds[i];
        return wind ? wind.z() : std::numeric_limits<double>::quiet_NaN();
    });
    ts << ",\n";
    ts << "\"High\":";
    writeArray([&](std::size_t i) { return result->heightAt(i); });
    ts << ",\n";
    ts << "\"Date\":\"" << calendarTime(result->at).toString("yyyy-MM-dd HH:mm:ss") << "\"\n}\n";
    ts.flush();
    if (ts.status() != QTextStream::Ok) {
        qWarning().noquote() << "Cannot write wind profile file:" << path << file.errorString();
        return;
    }
    if (!file.flush())
        qWarning().noquote() << "Cannot flush wind profile file:" << path << file.errorString();
}
