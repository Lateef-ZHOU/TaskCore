#pragma once
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <limits>
#include <numbers>
#include <thread>
#include <tuple>
#include <utility>
#include <ranges>
#include <QByteArray>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QIODevice>
#include <QSaveFile>
#include <QString>

#include "DAQFamily/DAQCollector.h"
#include "DAQFamily/DAQChannelSpec.h"

// ============================================================
// AcqSaver —— 采集卡族唯一的落盘格式与路径政策。写出只由本类负责：入参是具体收集器（通道元组与上下文）
// 和风廓线结果，官邸不参与；以后改写法只改这里。
//
//   · 每帧一个 JSON：<root>/<会话>/<帧时间>.json，功率谱、径向风与 CNR 合并；
//   · 风廓线同构：  <root>/<会话>/profile_<同步帧时间>.json。
//   键名英文；浮点数 6 位有效数字；缺测（NaN / 无朝向）写 null；通道顺序与名称取自通道规格。
//
// 入口起 detached 线程即返回，线程按值持只读份额；逐段流式写出，不在内存里整体构造 JSON。
// 保存入口统一检查 saveRoot，空时不启动写盘线程。detached 线程不 join，进程退出时若仍在写可能截断（见待办）。
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

    // 完成帧以只读份额交入；card 为设备的 ASCII 标识（objectName）。
    template <DAQCollectorLike C>
    static void saveFrame(std::shared_ptr<const C> frame, QString card)
    {
        if (!frame || !frame->parameters.saveRoot) return;
        std::thread(&AcqSaver::writeFrame<C>, std::move(frame), std::move(card)).detach();
    }

    // 同步点的一组风廓线落盘；线程只保留上下文、同步帧时刻和结果份额，不延长 Collector 寿命。
    template<DAQCollectorLike C, DAQChannelSpec... Specs>
    static void saveWindProfiles(std::shared_ptr<const C> frame,
        std::tuple<std::shared_ptr<const VerticalWindProfileUnit<Specs>>...> channels)
    {
        if (!frame || !frame->parameters.saveRoot
            || !std::apply([](const auto &...p) { return (bool(p) || ...); }, channels)) return;
        std::thread([context = frame->parameters, at = frame->frameTime(), channels = std::move(channels)] {
            writeWindProfiles(context, at, channels);
        }).detach();
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
    static void writeFrameInfo(Writer &out, const DAQParameters &context, const RxTime at)
    {
        out.key("frame");
        out.raw("{");
        out.key("session"); out.text(context.sessionDir); out.raw(",");
        out.key("frameTime"); out.text(timeStem(at)); out.raw(",");
        out.key("azimuthDeg"); out.number(context.pose ? context.pose->azimuth() : std::nan("")); out.raw(",");
        out.key("elevationDeg"); out.number(context.pose ? context.pose->elevation() : std::nan("")); out.raw(",");
        out.key("sync"); out.raw(context.sync ? "true" : "false"); out.raw(",");
        out.key("accumSec"); out.number(context.accumSec);
    }

    // [独立线程] 每帧一个 JSON；通道按元组顺序写出。
    template <DAQCollectorLike C>
    static void writeFrame(const std::shared_ptr<const C> frame, const QString card)
    {
        const auto &context = frame->parameters;
        Writer out(context.saveRoot->filePath(context.sessionDir + "/" + fileStem(frame->frameTime()) + ".json"));
        if (!out.open()) {
            qWarning().noquote() << "Cannot open acquisition frame file under:" << context.saveRoot->path();
            return;
        }
        out.raw("{");
        writeFrameInfo(out, context, frame->frameTime());
        out.raw(",");
        out.key("card"); out.text(card);
        out.raw("},");
        out.key("channels");
        out.raw("[");
        bool first = true;
        std::apply([&](const auto &...channel) { (writeChannel(out, channel, first), ...); }, frame->channels());
        out.raw("]}");
        out.commit();
    }

    // 一路通道：规格、功率谱、径向风与 CNR。
    template <DAQChannelLike Channel>
    static void writeChannel(Writer &out, const Channel &channel, bool &first)
    {
        if (!first) out.raw(",");
        first = false;
        out.raw("{");
        out.key("name"); out.text(QString::fromUtf8(Channel::name)); out.raw(",");
        const auto &spectrum = *channel.spectrum;
        out.key("rangeStartM"); out.number(spectrum.rowStart); out.raw(",");
        out.key("rangeStepM"); out.number(spectrum.rowStep); out.raw(",");
        out.key("freqStartMHz"); out.number(spectrum.columnStart); out.raw(",");
        out.key("freqStepMHz"); out.number(spectrum.columnStep); out.raw(",");
        out.key("zeroWindMHz"); out.number(Channel::zeroWindMHz); out.raw(",");
        out.key("spectrum");
        out.raw("[");
        for (int row = 0; spectrum.complete() && row < spectrum.rows; ++row) {
            if (row) out.raw(",");
            out.numbers(spectrum.row(row));
        }
        out.raw("],");
        out.key("radialWind"); out.numbers(*channel.radialWind); out.raw(",");
        out.key("cnr"); out.numbers(*channel.cnr);
        out.raw("}");
    }

    // [独立线程] 同步点一份风廓线 JSON；风向为气象来向，0–360°。
    template<DAQChannelSpec... Specs>
    static void writeWindProfiles(const DAQParameters &context, const RxTime at,
        const std::tuple<std::shared_ptr<const VerticalWindProfileUnit<Specs>>...> &channels)
    {
        Writer out(context.saveRoot->filePath(context.sessionDir + "/profile_" + fileStem(at) + ".json"));
        if (!out.open()) {
            qWarning().noquote() << "Cannot open wind profile file under:" << context.saveRoot->path();
            return;
        }
        out.raw("{");
        writeFrameInfo(out, context, at);
        out.raw("},");
        out.key("channels");
        out.raw("[");
        bool first = true;
        const auto writeChannel = [&]<DAQChannelSpec Spec>(const std::shared_ptr<const VerticalWindProfileUnit<Spec>> &pointer) {
            if (!pointer) return;
            const auto &profile = *pointer;
            if (!first) out.raw(",");
            first = false;
            out.raw("{");
            out.key("name"); out.text(QString::fromUtf8(Spec::name)); out.raw(",");
            out.key("heightStartM"); out.number(profile.heightStartM()); out.raw(",");
            out.key("heightStepM"); out.number(profile.heightStepM()); out.raw(",");
            out.key("horizontalDirectionDeg"); out.numbers(profile.values() | std::views::transform([](const auto &wind) {
                if (!wind) return std::numeric_limits<double>::quiet_NaN();
                double from = std::atan2(-wind->x(), -wind->y()) * 180.0 / std::numbers::pi;
                if (from < 0.0) from += 360.0;
                if (from >= 360.0) from -= 360.0;
                return from;
            })); out.raw(",");
            out.key("horizontalSpeed"); out.numbers(profile.values() | std::views::transform([](const auto &wind) {
                return wind ? std::hypot(wind->x(), wind->y()) : std::numeric_limits<double>::quiet_NaN();
            })); out.raw(",");
            out.key("verticalSpeed"); out.numbers(profile.values() | std::views::transform([](const auto &wind) {
                return wind ? wind->z() : std::numeric_limits<double>::quiet_NaN();
            }));
            out.raw("}");
        };
        std::apply([&](const auto &...p) { (writeChannel(p), ...); }, channels);
        out.raw("]}");
        out.commit();
    }
};
