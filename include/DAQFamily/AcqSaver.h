#pragma once
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <numbers>
#include <string>
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
#include <QStringList>

#include "DAQFamily/DAQCollector.h"
#include "DAQFamily/DAQChannelSpec.h"
#include "RunBase.h" // RxTime

// ============================================================
// AcqSaver —— 采集卡族唯一的落盘格式与路径政策。写出只由本类负责：入参是具体收集器（通道元组与上下文）
// 和风廓线结果，官邸不参与；以后改写法只改这里。
//
//   · 每帧一个 JSON：<root>/<会话>/<帧时间>.json，功率谱、径向风与 CNR 合并；
//   · 风廓线同构：  <root>/<会话>/profile_<同步帧时间>.json。
//   键名英文；浮点数 6 位有效数字；缺测（NaN / 无朝向）写 null；通道顺序与名称取自采集帧。
//
// 入口起 detached 线程即返回，线程按值持只读份额；逐段流式写出，不在内存里整体构造 JSON。
// 保存入口统一检查 saveRoot，空时不启动写盘线程。detached 线程不 join，进程退出时若仍在写可能截断（见待办）。
// ============================================================
class AcqSaver
{
public:
    // sessionDir 只吃 run 级参数（一轮采集共用一个目录），由编排层算好经 Collector 上下文递下；
    // 帧级文件名由帧时刻现算。
    [[nodiscard]] static std::string sessionDir(const QDateTime &sessionTime, const QString &tag)
    {
        return (sessionTime.toString("yyyyMMdd_HHmmss") + "_" + tag).toStdString();
    }
    [[nodiscard]] static QString fileStem(const RxTime frameTime)
    {
        return calendarTime(frameTime).toString("yyyyMMdd_HHmmss_zzz");
    }
    [[nodiscard]] static QString timeStem(const RxTime frameTime)
    {
        return calendarTime(frameTime).toString("yyyy-MM-dd HH:mm:ss.zzz");
    }

    // 完成帧以只读份额交入；线程只持上下文、帧时刻、设备标识与结果份额，不延长 Collector 寿命。
    template <DAQCollectorLike C>
    static void saveFrame(std::shared_ptr<const C> frame, QString card)
    {
        if (!frame || !frame->parameters.saveRoot) return;
        std::thread([context = frame->parameters, at = frame->frameTime(),
                     channels = frame->channels(), card = std::move(card)] {
            writeFrame(context, at, channels, card);
        }).detach();
    }

    // 同步点的一组风廓线落盘；线程只保留上下文、同步帧时刻和结果份额，不延长 Collector 寿命。
    template<DAQCollectorLike C, auto... Ps>
    static void saveWindProfiles(std::shared_ptr<const C> frame,
        std::tuple<std::shared_ptr<const VerticalWindProfileUnit<Ps>>...> channels)
    {
        if (!frame || !frame->parameters.saveRoot
            || !std::apply([](const auto &...p) { return (bool(p) || ...); }, channels)) return;
        QStringList names;
        std::apply([&](const auto &...ch) { ((names << QString::fromUtf8(ch.name ? ch.name : "")), ...); }, frame->channels());
        std::thread([context = frame->parameters, at = frame->frameTime(), channels = std::move(channels), names = std::move(names)] {
            writeWindProfiles(context, at, channels, names);
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
            if (!dir.mkpath(QStringLiteral("."))) {
                qWarning().noquote() << "Cannot create acquisition directory:" << dir.absolutePath()
                                     << "for file:" << file_.fileName();
                return false;
            }
            if (!file_.open(QIODevice::WriteOnly)) {
                qWarning().noquote() << "Cannot open acquisition file:" << file_.fileName() << file_.errorString();
                return false;
            }
            return true;
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
            QByteArray escaped;
            const auto utf8 = value.toUtf8();
            escaped.reserve(utf8.size());
            constexpr char hex[] = "0123456789abcdef";
            for (const char byte : utf8) {
                const auto code = static_cast<unsigned char>(byte);
                if (byte == '\\' || byte == '"') {
                    escaped.append('\\');
                    escaped.append(byte);
                } else if (code < 0x20) {
                    escaped.append("\\u00");
                    escaped.append(hex[code >> 4]);
                    escaped.append(hex[code & 0x0f]);
                } else {
                    escaped.append(byte);
                }
            }
            file_.write("\"" + escaped + "\"");
        }
        void number(const double value, const int precision = 6)
        {
            file_.write(std::isfinite(value) ? QByteArray::number(value, 'g', precision) : QByteArrayLiteral("null"));
        }
        template<class Values>
        void numbers(const Values &values, const int precision = 6)
        {
            file_.write("[");
            bool first = true;
            for (const double value : values) {
                if (!first) file_.write(",");
                number(value, precision);
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
    static void writeFrameInfo(Writer &out, const DAQParameters &context, const RxTime at, const QString &session)
    {
        out.key("frame");
        out.raw("{");
        out.key("session"); out.text(session); out.raw(",");
        out.key("frameTime"); out.text(timeStem(at)); out.raw(",");
        out.key("azimuthDeg"); out.number(context.pose ? context.pose->azimuth() : std::nan("")); out.raw(",");
        out.key("elevationDeg"); out.number(context.pose ? context.pose->elevation() : std::nan("")); out.raw(",");
        out.key("sync"); out.raw(context.sync ? "true" : "false"); out.raw(",");
        out.key("accumSec"); out.number(context.accumSec);
    }

    // [独立线程] 每帧一个 JSON；通道按元组顺序写出。
    template <DAQChannelLike... Channels>
    static void writeFrame(const DAQParameters &context, const RxTime at,
        const std::tuple<Channels...> &channels, const QString &card)
    {
        const auto session = QString::fromStdString(context.sessionDir);
        Writer out(QDir(*context.saveRoot).filePath(session + "/" + fileStem(at) + ".json"));
        if (!out.open()) return;
        out.raw("{");
        writeFrameInfo(out, context, at, session);
        out.raw(",");
        out.key("card"); out.text(card);
        out.raw("},");
        out.key("channels");
        out.raw("[");
        bool first = true;
        std::apply([&](const auto &...channel) { (writeChannel(out, channel, first), ...); }, channels);
        out.raw("]}");
        out.commit();
    }

    // 一路通道：规格、功率谱、径向风与 CNR。
    template <auto P>
    static void writeChannel(Writer &out, const DAQChannel<P> &channel, bool &first)
    {
        if (!first) out.raw(",");
        first = false;
        out.raw("{");
        out.key("name"); out.text(QString::fromUtf8(channel.name ? channel.name : "")); out.raw(",");
        const auto &spectrum = *channel.spectrum;
        if constexpr (P.uniformRange)
        {
            out.key("rangeStartM"); out.number(P.rangeStartM); out.raw(",");
            out.key("rangeStepM"); out.number(P.rangeStepM); out.raw(",");
        }
        else
        {
            constexpr int precision = std::numeric_limits<double>::max_digits10;
            out.key("rangeM"); out.numbers(P.range.edges.first(P.rangeGateCount), precision); out.raw(",");
            out.key("rangeStopM"); out.number(P.rangeStopM(), precision); out.raw(",");
        }
        out.key("freqStartMHz"); out.number(P.frequencyStartMHz); out.raw(",");
        out.key("freqStepMHz"); out.number(P.frequencyStepMHz); out.raw(",");
        out.key("zeroWindMHz"); out.number(P.zeroWindMHz); out.raw(",");
        out.key("spectrum");
        out.raw("[");
        for (int row = 0; row < P.rangeGateCount; ++row) {
            if (row) out.raw(",");
            out.numbers(spectrum.row(row));
        }
        out.raw("],");
        out.key("radialWind"); if (channel.radialWind) out.numbers(*channel.radialWind); else out.raw("[]"); out.raw(",");
        out.key("cnr"); if (channel.cnr) out.numbers(*channel.cnr); else out.raw("[]");
        out.raw("}");
    }

    // [独立线程] 同步点一份风廓线 JSON；风向为气象来向，0–360°。
    template<auto... Ps>
    static void writeWindProfiles(const DAQParameters &context, const RxTime at,
        const std::tuple<std::shared_ptr<const VerticalWindProfileUnit<Ps>>...> &channels,
        const QStringList &names)
    {
        const auto session = QString::fromStdString(context.sessionDir);
        Writer out(QDir(*context.saveRoot).filePath(session + "/profile_" + fileStem(at) + ".json"));
        if (!out.open()) return;
        out.raw("{");
        writeFrameInfo(out, context, at, session);
        out.raw("},");
        out.key("channels");
        out.raw("[");
        bool first = true;
        int nameIndex = 0;
        const auto writeChannel = [&]<auto P>(const std::shared_ptr<const VerticalWindProfileUnit<P>> &pointer) {
            const int index = nameIndex++;
            if (!pointer) return;
            const auto &profile = *pointer;
            if (!first) out.raw(",");
            first = false;
            out.raw("{");
            out.key("name"); out.text(names.value(index)); out.raw(",");
            if constexpr (P.uniformRange)
            {
                out.key("heightStartM"); out.number(profile.heightStartM()); out.raw(",");
                out.key("heightStepM"); out.number(profile.heightStepM()); out.raw(",");
            }
            else
            {
                constexpr int precision = std::numeric_limits<double>::max_digits10;
                const auto scale = profile.heightScale();
                out.key("heightM"); out.numbers(P.range.edges.first(P.rangeGateCount)
                    | std::views::transform([scale](double range) { return range * scale; }), precision); out.raw(",");
                out.key("heightStopM"); out.number(P.rangeStopM() * scale, precision); out.raw(",");
            }
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
