#pragma once
#include <optional>
#include <QVariantMap>
#include <PcapHandle.h>

// QML 表单适配：四台网口设备共用枚举快照与输入校验，Handle 只接受连接参数。
class PcapForm
{
public:
    // 选项与下标转换共用同一次枚举；表单只持名称和下标，插拔网卡后需重开程序。
    [[nodiscard]] static QVariantMap connectionParameters(const QString &hostIPv4, const int hostPort,
                                                          const QString &deviceIPv4, const int devicePort,
                                                          const std::optional<QString> &deviceMAC)
    {
        QStringList names, tips;
        for (const auto &n : interfaces()) { names << n.friendlyName; tips << n.description; }
        return {{"networkInterfaceOptions", names}, {"networkInterfaceTips", tips},
                {"networkInterfaceDefault", interfaces().isEmpty() ? -1 : 0},
                {"hostIPv4Default", hostIPv4}, {"hostPortDefault", hostPort},
                {"deviceIPv4Default", deviceIPv4}, {"devicePortDefault", devicePort},
                {"hasDeviceMAC", deviceMAC.has_value()}, {"deviceMACDefault", deviceMAC.value_or(QString{})},
                {"portMin", kPortMin}, {"portMax", kPortMax}};
    }

    // 先验证全部输入，再构造连接快照；失败不触碰正在运行的连接或重连种子。
    [[nodiscard]] static std::optional<PcapParams> parseParameters(
        const PcapHandle &source,
        const int interfaceIndex, const QString &hostIP, const int hostPort,
        const QString &deviceIP, const int devicePort, const std::optional<QString> &deviceMAC,
        const std::chrono::milliseconds readTimeout, const int kernelBufferBytes)
    {
        if (interfaceIndex < 0 || interfaceIndex >= interfaces().size())
        {
            Q_EMIT source.logMessage(QStringLiteral("未选择有效网卡，连接请求未提交"));
            return std::nullopt;
        }
        bool ok = false;
        const auto host = IPv4::fromString(hostIP, ok);
        if (!ok)
        {
            Q_EMIT source.logMessage(QStringLiteral("本机 IPv4 无效，连接请求未提交"));
            return std::nullopt;
        }
        const auto device = IPv4::fromString(deviceIP, ok);
        if (!ok)
        {
            Q_EMIT source.logMessage(QStringLiteral("设备 IPv4 无效，连接请求未提交"));
            return std::nullopt;
        }
        if (hostPort < kPortMin || hostPort > kPortMax || devicePort < kPortMin || devicePort > kPortMax)
        {
            Q_EMIT source.logMessage(QStringLiteral("UDP 端口超出 0~65535，连接请求未提交"));
            return std::nullopt;
        }
        MAC mac;
        if (deviceMAC)
        {
            mac = MAC::fromString(*deviceMAC, ok);
            if (!ok)
            {
                Q_EMIT source.logMessage(QStringLiteral("设备 MAC 无效，连接请求未提交"));
                return std::nullopt;
            }
        }
        return PcapParams{interfaces().at(interfaceIndex), host, device, mac,
                          static_cast<quint16>(hostPort), static_cast<quint16>(devicePort),
                          readTimeout, kernelBufferBytes};
    }

private:
    static constexpr int kPortMin = 0;
    static constexpr int kPortMax = 65535;
    [[nodiscard]] static const QList<NetworkInterfaceInfo> &interfaces()
    {
        static const auto list = availableInterfaces();
        return list;
    }

};
