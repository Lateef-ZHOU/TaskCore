#pragma once
#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <span>
#include <thread>
#include <utility>
#include <vector>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QSaveFile>
#include <QString>
#include <QVector>

#include "DAQCollector.h"
#include "VerticalWindProfile.h"

// ============================================================
// AcqSaver —— 采集卡族唯一的落盘格式与路径政策。写出只由本类负责：入参是族基 DAQCollector（通道数组
// 与上下文）和风廓线结果，官邸不参与；以后改写法只改这里。
//
//   · 每帧一个 JSON：<root>/<会话>/<帧时间>.json，功率谱、径向风与 CNR 合并；
//   · 风廓线同构：  <root>/<会话>/profile_<同步帧时间>.json。
//   键名英文；浮点数 6 位有效数字；缺测（NaN / 无朝向）写 null；通道顺序与名称取自通道规格。
//
// 入口起 detached 线程即返回，线程按值持只读份额；逐段流式写出，不在内存里整体构造 JSON。
// detached 线程不 join，进程退出时若仍在写可能截断（见待办）。存不存由调用方判（saveRoot 空 = 不落盘）。
// ============================================================
class AcqSaver
{
public:
    // sessionDir 只吃 run 级参数（一轮采集共用一个目录），由编排层算好经 Collector 上下文递下；
    // 帧级文件名由帧时刻现算。
    [[nodiscard]] static QString sessionDir(const QDateTime &sessionTime, const QString &tag)
    {
        return sessionTime.toString("yyyyMMdd_HHmmss") + "_" + tag;
    }
    [[nodiscard]] static QString fileStem(const RxTime frameTime)
    {
        return calendarTime(frameTime).toString("yyyyMMdd_HHmmss_zzz");
    }
    [[nodiscard]] static QString timeStem(const RxTime frameTime)
    {
        return calendarTime(frameTime).toString("yyyy-MM-dd HH:mm:ss.zzz");
    }

    // 一路通道的风廓线结果；名称取自通道规格。
    struct ProfileChannel
    {
        QString name;
        VerticalWindProfilePtr profile;
    };

    // 完成帧落盘；card 为设备的 ASCII 标识（objectName）。
    static void saveFrame(std::shared_ptr<const DAQCollector> frame, QString card)
    {
        if (!frame || !frame->parameters.saveRoot) return;
        std::thread(&AcqSaver::writeFrame, std::move(frame), std::move(card)).detach();
    }

    // 同步点的一组风廓线落盘；时刻取同步帧，会话取其上下文。
    static void saveWindProfiles(std::shared_ptr<const DAQCollector> frame, std::vector<ProfileChannel> channels)
    {
        if (!frame || !frame->parameters.saveRoot || channels.empty()) return;
        std::thread(&AcqSaver::writeWindProfiles, std::move(frame), std::move(channels)).detach();
    }

private:
    [[nodiscard]] static QDateTime calendarTime(const RxTime at)
    {
        return QDateTime::fromStdTimePoint(std::chrono::time_point_cast<std::chrono::milliseconds>(at)).toLocalTime();
    }

    // 逐段写入的 JSON 输出；只覆盖本格式用到的值类型。
    class Writer
    {
    public:
        explicit Writer(const QString &path) : file_(path) {}
        [[nodiscard]] bool open()
        {
            const QDir dir = QFileInfo(file_.fileName()).dir();
            return dir.mkpath(QStringLiteral(".")) && file_.open(QIODevice::WriteOnly);
        }
        void raw(const char *text) { file_.write(text); }
        void key(const char *name)
        {
            file_.write("\"");
            file_.write(name);
            file_.write("\":");
        }
        void text(const QString &value)
        {
            QByteArray escaped = value.toUtf8();
            escaped.replace("\\", "\\\\").replace("\"", "\\\"");
            file_.write("\"" + escaped + "\"");
        }
        void number(const double value)
        {
            file_.write(std::isfinite(value) ? QByteArray::number(value, 'g', 6) : QByteArrayLiteral("null"));
        }
        template<class Values>
        void numbers(const Values &values)
        {
            file_.write("[");
            bool first = true;
            for (const double value : values) {
                if (!first) file_.write(",");
                number(value);
                first = false;
            }
            file_.write("]");
        }
        void commit()
        {
            if (!file_.commit())
                qWarning().noquote() << "Cannot save acquisition file:" << file_.fileName() << file_.errorString();
        }
    private:
        QSaveFile file_;
    };

    // 帧级公共信息（与通道无关）。
    static void writeFrameInfo(Writer &out, const DAQCollector &frame)
    {
        const auto &context = frame.parameters;
        out.key("frame");
        out.raw("{");
        out.key("session"); out.text(context.sessionDir); out.raw(",");
        out.key("frameTime"); out.text(timeStem(frame.frameTime())); out.raw(",");
        out.key("azimuthDeg"); out.number(context.pose ? context.pose->azimuth() : std::nan("")); out.raw(",");
        out.key("elevationDeg"); out.number(context.pose ? context.pose->elevation() : std::nan("")); out.raw(",");
        out.key("sync"); out.raw(context.sync ? "true" : "false"); out.raw(",");
        out.key("accumSec"); out.number(context.accumSec);
    }

    // [独立线程] 每帧一个 JSON。
    static void writeFrame(const std::shared_ptr<const DAQCollector> frame, const QString card)
    {
        const auto &context = frame->parameters;
        Writer out(context.saveRoot->filePath(context.sessionDir + "/" + fileStem(frame->frameTime()) + ".json"));
        if (!out.open()) {
            qWarning().noquote() << "Cannot open acquisition frame file under:" << context.saveRoot->path();
            return;
        }
        out.raw("{");
        writeFrameInfo(out, *frame);
        out.raw(",");
        out.key("card"); out.text(card);
        out.raw("},");
        out.key("channels");
        out.raw("[");
        for (std::size_t i = 0; i < frame->channels().size(); ++i) {
            const auto &channel = frame->channels()[i];
            if (i) out.raw(",");
            out.raw("{");
            out.key("name"); out.text(channel.name); out.raw(",");
            const auto &spectrum = channel.spectrum;
            out.key("rangeStartM"); out.number(spectrum.rowStart); out.raw(",");
            out.key("rangeStepM"); out.number(spectrum.rowStep); out.raw(",");
            out.key("freqStartMHz"); out.number(spectrum.columnStart); out.raw(",");
            out.key("freqStepMHz"); out.number(spectrum.columnStep); out.raw(",");
            out.key("zeroWindMHz"); out.number(channel.zeroWindMHz); out.raw(",");
            out.key("spectrum");
            out.raw("[");
            for (int row = 0; spectrum.complete() && row < spectrum.rows; ++row) {
                if (row) out.raw(",");
                out.numbers(spectrum.row(row));
            }
            out.raw("],");
            out.key("radialWind"); out.numbers(channel.radialWind); out.raw(",");
            out.key("cnr"); out.numbers(channel.cnr);
            out.raw("}");
        }
        out.raw("]}");
        out.commit();
    }

    // [独立线程] 同步点一份风廓线 JSON；风向为气象来向，0–360°。
    static void writeWindProfiles(const std::shared_ptr<const DAQCollector> frame, const std::vector<ProfileChannel> channels)
    {
        const auto &context = frame->parameters;
        Writer out(context.saveRoot->filePath(context.sessionDir + "/profile_" + fileStem(frame->frameTime()) + ".json"));
        if (!out.open()) {
            qWarning().noquote() << "Cannot open wind profile file under:" << context.saveRoot->path();
            return;
        }
        out.raw("{");
        writeFrameInfo(out, *frame);
        out.raw("},");
        out.key("channels");
        out.raw("[");
        for (std::size_t i = 0; i < channels.size(); ++i) {
            const auto &profile = *channels[i].profile;
            std::vector<double> direction, horizontal, vertical;
            for (const auto &wind : profile.winds) {
                double from = std::atan2(-wind.x(), -wind.y()) * 180.0 / std::numbers::pi;
                if (from < 0.0) from += 360.0;
                if (from >= 360.0) from -= 360.0;
                direction.push_back(from);
                horizontal.push_back(std::hypot(wind.x(), wind.y()));
                vertical.push_back(wind.z());
            }
            if (i) out.raw(",");
            out.raw("{");
            out.key("name"); out.text(channels[i].name); out.raw(",");
            out.key("heightStartM"); out.number(profile.winds.start); out.raw(",");
            out.key("heightStepM"); out.number(profile.winds.step); out.raw(",");
            out.key("horizontalDirectionDeg"); out.numbers(direction); out.raw(",");
            out.key("horizontalSpeed"); out.numbers(horizontal); out.raw(",");
            out.key("verticalSpeed"); out.numbers(vertical);
            out.raw("}");
        }
        out.raw("]}");
        out.commit();
    }
};
