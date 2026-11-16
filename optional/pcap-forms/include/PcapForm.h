#pragma once
#include <optional>
#include <QVariantMap>
#include <PcapHandle.h>

// QML 网卡选项与连接构参共用枚举快照；线路参数由型号提供，Handle 只接受连接参数。
class PcapForm
{
public:
    // 选项与下标转换共用同一次枚举；表单只持名称和下标，插拔网卡后需重开程序。
    [[nodiscard]] static QVariantMap connectionParameters()
    {
        QStringList names, tips;
        for (const auto &n : interfaces()) {
            names << QString::fromStdString(n.friendlyName);
            tips << QString::fromStdString(n.description);
        }
        return {{"networkInterfaceOptions", names}, {"networkInterfaceTips", tips},
                {"networkInterfaceDefault", interfaces().empty() ? -1 : 0}};
    }

    // 先验证全部输入，再构造连接快照；失败不触碰正在运行的连接或重连种子。
    [[nodiscard]] static std::optional<PcapParams> parseParameters(
        const PcapHandle &source,
        const int interfaceIndex, const QString &hostIP, const int hostPort,
        const QString &deviceIP, const int devicePort, const std::optional<QString> &deviceMAC,
        const std::chrono::milliseconds readTimeout, const int kernelBufferBytes)
    {
        if (interfaceIndex < 0 || static_cast<std::size_t>(interfaceIndex) >= interfaces().size())
        {
            Q_EMIT source.logMessage("未选择有效网卡，连接请求未提交");
            return std::nullopt;
        }
        const auto host = PacketProtocol::IPv4Header::Address::fromString(hostIP.toStdString());
        if (!host)
        {
            Q_EMIT source.logMessage("本机 IPv4 无效，连接请求未提交");
            return std::nullopt;
        }
        const auto device = PacketProtocol::IPv4Header::Address::fromString(deviceIP.toStdString());
        if (!device)
        {
            Q_EMIT source.logMessage("设备 IPv4 无效，连接请求未提交");
            return std::nullopt;
        }
        if (hostPort < kPortMin || hostPort > kPortMax || devicePort < kPortMin || devicePort > kPortMax)
        {
            Q_EMIT source.logMessage("UDP 端口超出 0~65535，连接请求未提交");
            return std::nullopt;
        }
        PacketProtocol::EthernetHeader::MAC mac;
        if (deviceMAC)
        {
            const auto parsed = PacketProtocol::EthernetHeader::MAC::fromString(deviceMAC->toStdString());
            if (!parsed)
            {
                Q_EMIT source.logMessage("设备 MAC 无效，连接请求未提交");
                return std::nullopt;
            }
            mac = *parsed;
        }
        const auto &network = interfaces().at(static_cast<std::size_t>(interfaceIndex));
        return PcapParams{network.pcapDeviceName, network.macAddress, *host, *device, mac,
                          static_cast<std::uint16_t>(hostPort), static_cast<std::uint16_t>(devicePort),
                          readTimeout, kernelBufferBytes};
    }

private:
    static constexpr int kPortMin = 0;
    static constexpr int kPortMax = 65535;
    [[nodiscard]] static const std::vector<NetworkInterfaceInfo> &interfaces()
    {
        static const auto list = availableInterfaces();
        return list;
    }

};
