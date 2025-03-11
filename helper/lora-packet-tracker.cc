/*
 * Copyright (c) 2018 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

#include "lora-packet-tracker.h"

#include "ns3/end-device-lora-phy.h"
#include "ns3/log.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/simulator.h"

#include <fstream>
#include <iostream>

namespace ns3
{
namespace lorawan
{
NS_LOG_COMPONENT_DEFINE("LoraPacketTracker");

DataAgeInformation LoraPacketTracker::m_dataAoi;
RetransmissionData LoraPacketTracker::m_reTransmissionTracker;
MacPacketData LoraPacketTracker::m_macPacketTracker; //!< Packet map of MAC layer metrics

LoraPacketTracker::LoraPacketTracker()
{
    NS_LOG_FUNCTION(this);
}

LoraPacketTracker::~LoraPacketTracker()
{
    NS_LOG_FUNCTION(this);
}

/////////////////
// MAC metrics //
/////////////////

void
LoraPacketTracker::MacTransmissionCallback(Ptr<const Packet> packet, uint8_t sf)
{
    if (IsUplink(packet))
    {
        NS_LOG_INFO("A new packet was sent by the MAC layer");

        MacPacketStatus status;
        status.packet = packet;
        status.sendTime = Simulator::Now();
        status.senderId = Simulator::GetContext();
        status.receivedTime = Time::Max();
        status.sf = sf;

        m_macPacketTracker.insert(std::pair<Ptr<const Packet>, MacPacketStatus>(packet, status));
    }
}

void
LoraPacketTracker::RequiredTransmissionsCallback(uint8_t reqTx,
                                                 uint8_t sf,
                                                 bool success,
                                                 Time firstAttempt,
                                                 Ptr<Packet> packet)
{
    NS_LOG_INFO("Finished retransmission attempts for a packet");
    NS_LOG_DEBUG("Packet: " << packet << "ReqTx " << unsigned(reqTx) << ", succ: " << success
                            << ", firstAttempt: " << firstAttempt.GetSeconds());

    RetransmissionStatus entry;
    entry.firstAttempt = firstAttempt;
    entry.finishTime = Simulator::Now();
    entry.sf = sf;
    entry.reTxAttempts = reqTx;
    entry.successful = success;

    if (packet != nullptr)
        m_reTransmissionTracker.insert(std::pair<Ptr<Packet>, RetransmissionStatus>(packet, entry));
}

void
LoraPacketTracker::MacGwReceptionCallback(Ptr<const Packet> packet)
{
    if (IsUplink(packet))
    {
        NS_LOG_INFO("A packet was successfully received at the MAC layer of gateway "
                    << Simulator::GetContext());

        // Find the received packet in the m_macPacketTracker
        auto it = m_macPacketTracker.find(packet);
        if (it != m_macPacketTracker.end())
        {
            (*it).second.receptionTimes.insert(
                std::pair<int, Time>(Simulator::GetContext(), Simulator::Now()));
        }
        else
        {
            NS_ABORT_MSG("Packet not found in tracker");
        }
    }
}

/////////////////
// AOI metric //
///////////////

/**
 * @brief Processa pacotes retransmitidos para análise de Age of Information (AoI).
 *
 * Filtra pacotes retransmitidos associados aos dispositivos listados em `AoIPlottingDevices`,
 * organiza-os em ordem cronológica pelo tempo de primeiro envio (`firstAttempt`) e
 * insere os dados preparados em uma estrutura para análise posterior.
 *
 * @param AoIPlottingDevices Mapa com os dispositivos monitorados (chave: endereço, valor:
 * identificador).
 *
 * @note Utiliza o sistema de logs do ns-3 para depuração detalhada.
 */

void
LoraPacketTracker::ProcessAndOrganizeAoiPacketsPlot(
    std::map<LoraDeviceAddress, uint8_t> AoIPlottingDevices)
{
    std::map<ns3::lorawan::LoraDeviceAddress, std::vector<RetransmissionStatus>> DataPackets;

    NS_LOG_INFO("ProcessAndOrganizeAoiPackets");
    NS_LOG_INFO("Iniciando o mapeamento de pacotes...");
    for (auto it = AoIPlottingDevices.begin(); it != AoIPlottingDevices.end(); ++it)
    {
        NS_LOG_DEBUG("Verificando dispositivo com endereço: " << it->first);
        for (auto j = m_reTransmissionTracker.begin(); j != m_reTransmissionTracker.end(); ++j)
        {
            LorawanMacHeader mHdr;
            LoraFrameHeader fHdr;
            Ptr<Packet> packetCopy = j->first->Copy();
            packetCopy->RemoveHeader(mHdr);
            packetCopy->RemoveHeader(fHdr);
            LoraDeviceAddress address = fHdr.GetAddress();

            if (address == it->first)
            {
                NS_LOG_DEBUG("Adicionando pacote para o dispositivo: " << address);
                DataPackets[address].push_back(j->second);
            }
        }
    }

    NS_LOG_DEBUG("Iniciando a ordenação dos pacotes...");
    for (auto& entry : DataPackets)
    {
        NS_LOG_DEBUG("Ordenando pacotes para o dispositivo: " << entry.first);
        std::sort(entry.second.begin(),
                  entry.second.end(),
                  [](const RetransmissionStatus& a, const RetransmissionStatus& b) {
                      return a.firstAttempt < b.firstAttempt;
                  });

        NS_LOG_DEBUG("Pacotes ordenados para o dispositivo: " << entry.first);

        for (const auto& status : entry.second)
        {
            NS_LOG_DEBUG("Address: " << entry.first << ", First Attempt: "
                                     << status.firstAttempt.GetSeconds() << " segundos");
        }
    }
    NS_LOG_INFO("Ordenação completa.");

    CalculateAndInsertAoiMetrics(DataPackets);
}

/**
 * @brief Processa e organiza pacotes retransmitidos para análise de Age of Information (AoI).
 *
 * Filtra pacotes retransmitidos de todos os dispositivos presentes em `m_reTransmissionTracker`,
 * organiza-os em ordem cronológica pelo tempo de primeiro envio (`firstAttempt`) e insere os dados
 * preparados em uma estrutura para análise posterior.
 *
 * @note Essa função processa todos os dispositivos registrados, sem restringir a análise a um
 * conjunto específico. Utiliza o sistema de logs do ns-3 para depuração detalhada.
 */

void
LoraPacketTracker::OrganizeRetransmittedPackets()
{
    std::map<ns3::lorawan::LoraDeviceAddress, std::vector<RetransmissionStatus>> DataPackets;

    NS_LOG_DEBUG("Iniciando o mapeamento de pacotes retransmitidos...");
    for (auto j = m_reTransmissionTracker.begin(); j != m_reTransmissionTracker.end(); ++j)
    {
        LorawanMacHeader mHdr;
        LoraFrameHeader fHdr;
        Ptr<Packet> packetCopy = j->first->Copy();
        packetCopy->RemoveHeader(mHdr);
        packetCopy->RemoveHeader(fHdr);
        LoraDeviceAddress address = fHdr.GetAddress();

        DataPackets[address].push_back(j->second);
    }

    NS_LOG_DEBUG("Iniciando a ordenação dos pacotes...");
    for (auto& entry : DataPackets)
    {
        NS_LOG_DEBUG("Ordenando pacotes para o dispositivo: " << entry.first);
        std::sort(entry.second.begin(),
                  entry.second.end(),
                  [](const RetransmissionStatus& a, const RetransmissionStatus& b) {
                      return a.firstAttempt < b.firstAttempt;
                  });
    }
    NS_LOG_DEBUG("Ordenação concluída. Total de dispositivos: " << DataPackets.size());

    CalculateAndInsertAoiMetrics(DataPackets);
}

/**
 * @brief Calcula e insere métricas de Age of Information (AoI) para dispositivos.
 *
 * Esta função processa pacotes retransmitidos agrupados por dispositivo, calcula as métricas de AoI
 * (delta e delta1) para cada pacote e as armazena em séries primárias e secundárias associadas a
 * diferentes fatores de espalhamento (SF).
 *
 * Métricas calculadas:
 * - **Delta1:** Diferença no tempo de primeiro envio entre pacotes consecutivos.
 * - **Delta:** Diferença no tempo de conclusão entre pacotes consecutivos.
 * - Resets são aplicados após cada cálculo para registrar os eventos de término.
 *
 * @param DataPackets Mapa contendo dispositivos (chave: `LoraDeviceAddress`) e seus pacotes
 * retransmitidos
 *                    (`std::vector<RetransmissionStatus`).
 *
 * @note Baseado na metodologia descrita em "Age of Information: An Introduction and Survey".
 */

void
LoraPacketTracker::CalculateAndInsertAoiMetrics(
    const std::map<ns3::lorawan::LoraDeviceAddress, std::vector<RetransmissionStatus>>& DataPackets)
{
    /* Reference:
     * Roy D. Yates, Yin Sun, D. Richard Brown, III, Sanjit K. Kaul, Eytan
     * Modiano, Sennur Ulukus, "Age of Information: An Introduction and Survey",
     * IEEE Journal, Fellow, IEEE.
     */

    NS_LOG_DEBUG("Iniciando processamento de AoI...");

    static double lastFirstAttempt = 0;
    static double lastFinishTime = 0;

    for (const auto& packetEntry : DataPackets)
    {
        NS_LOG_DEBUG("Processando dispositivo com endereço: " << packetEntry.first);

        for (const RetransmissionStatus& status : packetEntry.second)
        {
            double currenteFirsAttempt = status.firstAttempt.GetSeconds();
            double d1_xValue = status.firstAttempt.GetSeconds();
            double d1_yValue = currenteFirsAttempt - lastFirstAttempt;

            // Delta1
            NS_LOG_DEBUG("Delta1 -> x: " << d1_xValue << ", y: " << currenteFirsAttempt << " - "
                                         << lastFirstAttempt << " = " << d1_yValue);
            m_dataAoi[status.sf].primarySeries.push_back(std::make_pair(d1_xValue, d1_yValue));

            lastFirstAttempt = currenteFirsAttempt;

            // Reset Delta1
            NS_LOG_DEBUG("Reset Delta1 -> x: " << d1_xValue << ", y: 0");
            m_dataAoi[status.sf].primarySeries.push_back(
                std::make_pair(d1_xValue, Seconds(0).GetSeconds()));

            double currenteFinishTime = status.finishTime.GetSeconds();
            double d_xValue = status.finishTime.GetSeconds();
            double d_yValue = currenteFinishTime - lastFinishTime;

            // Delta
            NS_LOG_DEBUG("Delta -> x: " << d_xValue << ", y: " << d_yValue);
            m_dataAoi[status.sf].secondarySeries.push_back(std::make_pair(d_xValue, d_yValue));

            // Reset Delta
            NS_LOG_DEBUG("Reset Delta -> x: "
                         << d_xValue
                         << ", y: " << (status.finishTime - status.firstAttempt).GetSeconds());
            m_dataAoi[status.sf].secondarySeries.push_back(
                std::make_pair(d_xValue,
                               d_yValue - (status.finishTime - status.firstAttempt).GetSeconds()));

            lastFinishTime = currenteFinishTime;
        }
        lastFirstAttempt = 0;
        lastFinishTime = 0;
    }

    NS_LOG_DEBUG("Finalizando processamento de AoI...");
}

void
LoraPacketTracker::CountMetricAoi()
{
    std::map<uint8_t, MetricsAoi> metricsMap; // Map para armazenar as métricas por SF

    for (const auto& metricAoi : m_dataAoi)
    {
        double sum = 0, sumOfSquares = 0, media = 0;
        int totalCount = 0;
        double maxY =
            std::numeric_limits<double>::lowest();        // Inicializa com o menor valor possível
        double minY = std::numeric_limits<double>::max(); // Inicializa com o maior valor possível

        const auto& primarySeries = metricAoi.second.primarySeries;

        for (const auto& point : primarySeries)
        {
            double y = point.second;
            sum += y;
            sumOfSquares += y * y;
            totalCount++;

            // Atualiza o valor máximo e mínimo de y
            if (y > maxY)
                maxY = y;
            if (y < minY && y != 0)
                minY = y;

            // Debug: Imprime os valores processados
            /*  std::cout << "Processando ponto: y = " << y << std::endl;
             std::cout << "Soma acumulada: sum = " << sum << std::endl;
             std::cout << "Soma dos quadrados: sumOfSquares = " << sumOfSquares << std::endl;
             std::cout << "Contagem total: totalCount = " << totalCount << std::endl;
             std::cout << "Valor máximo atual: maxY = " << maxY << std::endl;
             std::cout << "Valor mínimo atual (diferente de 0): minY = " << minY << std::endl; */
        }

        // Calcula a média
        media = (totalCount > 0) ? (sum / totalCount) : 0;

        // Calcula a variância
        double variance =
            (totalCount > 1) ? (sumOfSquares - (sum * sum / totalCount)) / (totalCount - 1) : 0;

        // Calcula o desvio padrão
        double desvioPadrao = std::sqrt(variance);

        // Armazena as métricas no map
        metricsMap[metricAoi.first] = {media, desvioPadrao, maxY, minY};
    }

    // Exibe os resultados armazenados no map
    for (const auto& entry : metricsMap)
    {
        std::cout << "SF: " << static_cast<int>(entry.first) << std::endl;
        std::cout << "Média dos picos: " << entry.second.media << std::endl;
        std::cout << "Desvio padrão dos picos: " << entry.second.desvioPadrao << std::endl;
        std::cout << "Valor máximo de pico: " << entry.second.maxY << std::endl;
        std::cout << "Valor mínimo de pico: " << entry.second.minY << std::endl;
        std::cout << "-----------------------------------------" << std::endl;
    }
}

/////////////////
// PHY metrics //
/////////////////

void
LoraPacketTracker::TransmissionCallback(Ptr<const Packet> packet, uint32_t edId)
{
    if (IsUplink(packet))
    {
        NS_LOG_INFO("PHY packet " << packet << " was transmitted by device " << edId);
        // Create a packetStatus
        PacketStatus status;
        status.packet = packet;
        status.sendTime = Simulator::Now();
        status.senderId = edId;

        m_packetTracker.insert(std::pair<Ptr<const Packet>, PacketStatus>(packet, status));
    }
}

void
LoraPacketTracker::PacketReceptionCallback(Ptr<const Packet> packet, uint32_t gwId)
{
    if (IsUplink(packet))
    {
        // Remove the successfully received packet from the list of sent ones
        NS_LOG_INFO("PHY packet " << packet << " was successfully received at gateway " << gwId);

        auto it = m_packetTracker.find(packet);
        (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome>(gwId, RECEIVED));
    }
}

void
LoraPacketTracker::InterferenceCallback(Ptr<const Packet> packet, uint32_t gwId)
{
    if (IsUplink(packet))
    {
        NS_LOG_INFO("PHY packet " << packet << " was interfered at gateway " << gwId);

        auto it = m_packetTracker.find(packet);
        (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome>(gwId, INTERFERED));
    }
}

void
LoraPacketTracker::NoMoreReceiversCallback(Ptr<const Packet> packet, uint32_t gwId)
{
    if (IsUplink(packet))
    {
        NS_LOG_INFO("PHY packet " << packet << " was lost because no more receivers at gateway "
                                  << gwId);
        auto it = m_packetTracker.find(packet);
        (*it).second.outcomes.insert(
            std::pair<int, enum PhyPacketOutcome>(gwId, NO_MORE_RECEIVERS));
    }
}

void
LoraPacketTracker::UnderSensitivityCallback(Ptr<const Packet> packet, uint32_t gwId)
{
    if (IsUplink(packet))
    {
        NS_LOG_INFO("PHY packet " << packet << " was lost because under sensitivity at gateway "
                                  << gwId);

        auto it = m_packetTracker.find(packet);
        (*it).second.outcomes.insert(
            std::pair<int, enum PhyPacketOutcome>(gwId, UNDER_SENSITIVITY));
    }
}

void
LoraPacketTracker::LostBecauseTxCallback(Ptr<const Packet> packet, uint32_t gwId)
{
    if (IsUplink(packet))
    {
        NS_LOG_INFO(
            "PHY packet " << packet
                          << " was lost because of concurrent downlink transmission at gateway "
                          << gwId);

        auto it = m_packetTracker.find(packet);
        (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome>(gwId, LOST_BECAUSE_TX));
    }
}

bool
LoraPacketTracker::IsUplink(Ptr<const Packet> packet)
{
    NS_LOG_FUNCTION(this);

    LorawanMacHeader mHdr;
    Ptr<Packet> copy = packet->Copy();
    copy->RemoveHeader(mHdr);
    return mHdr.IsUplink();
}

////////////////////////
// Counting Functions //
////////////////////////

std::vector<int>
LoraPacketTracker::CountPhyPacketsPerGw(Time startTime, Time stopTime, int gwId)
{
    // Vector packetCounts will contain - for the interval given in the input of
    // the function, the following fields: totPacketsSent receivedPackets
    // interferedPackets noMoreGwPackets underSensitivityPackets lostBecauseTxPackets

    std::vector<int> packetCounts(6, 0);

    for (auto itPhy = m_packetTracker.begin(); itPhy != m_packetTracker.end(); ++itPhy)
    {
        if ((*itPhy).second.sendTime >= startTime && (*itPhy).second.sendTime <= stopTime)
        {
            packetCounts.at(0)++;

            NS_LOG_DEBUG("Dealing with packet " << (*itPhy).second.packet);
            NS_LOG_DEBUG("This packet was received by " << (*itPhy).second.outcomes.size()
                                                        << " gateways");

            if ((*itPhy).second.outcomes.count(gwId) > 0)
            {
                switch ((*itPhy).second.outcomes.at(gwId))
                {
                case RECEIVED: {
                    packetCounts.at(1)++;
                    break;
                }
                case INTERFERED: {
                    packetCounts.at(2)++;
                    break;
                }
                case NO_MORE_RECEIVERS: {
                    packetCounts.at(3)++;
                    break;
                }
                case UNDER_SENSITIVITY: {
                    packetCounts.at(4)++;
                    break;
                }
                case LOST_BECAUSE_TX: {
                    packetCounts.at(5)++;
                    break;
                }
                case UNSET: {
                    break;
                }
                }
            }
        }
    }

    return packetCounts;
}

std::string
LoraPacketTracker::PrintPhyPacketsPerGw(Time startTime, Time stopTime, int gwId)
{
    // Vector packetCounts will contain - for the interval given in the input of
    // the function, the following fields: totPacketsSent receivedPackets
    // interferedPackets noMoreGwPackets underSensitivityPackets lostBecauseTxPackets

    std::vector<int> packetCounts(6, 0);

    for (auto itPhy = m_packetTracker.begin(); itPhy != m_packetTracker.end(); ++itPhy)
    {
        if ((*itPhy).second.sendTime >= startTime && (*itPhy).second.sendTime <= stopTime)
        {
            packetCounts.at(0)++;

            NS_LOG_DEBUG("Dealing with packet " << (*itPhy).second.packet);
            NS_LOG_DEBUG("This packet was received by " << (*itPhy).second.outcomes.size()
                                                        << " gateways");

            if ((*itPhy).second.outcomes.count(gwId) > 0)
            {
                switch ((*itPhy).second.outcomes.at(gwId))
                {
                case RECEIVED: {
                    packetCounts.at(1)++;
                    break;
                }
                case INTERFERED: {
                    packetCounts.at(2)++;
                    break;
                }
                case NO_MORE_RECEIVERS: {
                    packetCounts.at(3)++;
                    break;
                }
                case UNDER_SENSITIVITY: {
                    packetCounts.at(4)++;
                    break;
                }
                case LOST_BECAUSE_TX: {
                    packetCounts.at(5)++;
                    break;
                }
                case UNSET: {
                    break;
                }
                }
            }
        }
    }

    std::string output("");
    for (int i = 0; i < 6; ++i)
    {
        output += std::to_string(packetCounts.at(i)) + " ";
    }

    return output;
}

std::string
LoraPacketTracker::CountMacPacketsGlobally(Time startTime, Time stopTime)
{
    NS_LOG_FUNCTION(this << startTime << stopTime);

    double sent = 0;
    double received = 0;
    for (auto it = m_macPacketTracker.begin(); it != m_macPacketTracker.end(); ++it)
    {
        if ((*it).second.sendTime >= startTime && (*it).second.sendTime <= stopTime)
        {
            sent++;
            if (!(*it).second.receptionTimes.empty())
            {
                received++;
            }
        }
    }

    return std::to_string(sent) + " " + std::to_string(received);
}

std::string
LoraPacketTracker::CountMacPacketsGlobally(Time startTime, Time stopTime, uint8_t sf)
{
    NS_LOG_FUNCTION(this << startTime << stopTime);

    double sent = 0;
    double received = 0;
    for (auto it = m_macPacketTracker.begin(); it != m_macPacketTracker.end(); ++it)
    {
        if ((*it).second.sf == sf)
        {
            if ((*it).second.sendTime >= startTime && (*it).second.sendTime <= stopTime)
            {
                sent++;
                if (!(*it).second.receptionTimes.empty())
                {
                    received++;
                }
            }
        }
    }

    return std::to_string(sent) + " " + std::to_string(received);
}

std::string
LoraPacketTracker::CountMacPacketsGlobally(Time startTime,
                                           Time stopTime,
                                           std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    NS_LOG_FUNCTION(this << startTime << stopTime);
    double sent = 0;
    double received = 0;
    for (auto it = m_macPacketTracker.begin(); it != m_macPacketTracker.end(); ++it)
    {
        Ptr<Packet> packetCopy = (*it).first->Copy();
        LorawanMacHeader mHdr;
        LoraFrameHeader fHdr;
        packetCopy->RemoveHeader(mHdr);
        packetCopy->RemoveHeader(fHdr);
        LoraDeviceAddress address = fHdr.GetAddress();
        if (mapDevices.find(address) != mapDevices.end())
        {
            if ((*it).second.sendTime >= startTime && (*it).second.sendTime <= stopTime)
            {
                sent++;
                if ((*it).second.receptionTimes.size())
                {
                    received++;
                }
            }
        }
    }

    return std::to_string(sent) + " " + std::to_string(received);
}

std::string
LoraPacketTracker::CountMacPacketsGlobally(Time startTime,
                                           Time stopTime,
                                           uint8_t sf,
                                           std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    NS_LOG_FUNCTION(this << startTime << stopTime);
    double sent = 0;
    double received = 0;

    for (auto it = m_macPacketTracker.begin(); it != m_macPacketTracker.end(); ++it)
    {
        if ((*it).second.sf == sf)
        {
            Ptr<Packet> packetCopy = (*it).first->Copy();
            LorawanMacHeader mHdr;
            LoraFrameHeader fHdr;
            packetCopy->RemoveHeader(mHdr);
            packetCopy->RemoveHeader(fHdr);
            LoraDeviceAddress address = fHdr.GetAddress();
            if (mapDevices.find(address) != mapDevices.end())
            {
                if ((*it).second.sendTime >= startTime && (*it).second.sendTime <= stopTime)
                {
                    sent++;
                    if ((*it).second.receptionTimes.size())
                    {
                        received++;
                    }
                }
            }
        }
    }

    return std::to_string(sent) + " " + std::to_string(received);
}

std::string
LoraPacketTracker::CountMacPacketsGloballyCpsr(Time startTime, Time stopTime)
{
    NS_LOG_FUNCTION(this << startTime << stopTime);

    double sent = 0;
    double received = 0;
    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        if ((*it).second.firstAttempt >= startTime && (*it).second.firstAttempt <= stopTime)
        {
            sent++;
            NS_LOG_DEBUG("Found a packet");
            NS_LOG_DEBUG("Number of attempts: " << unsigned(it->second.reTxAttempts)
                                                << ", successful: " << it->second.successful);
            if (it->second.successful)
            {
                received++;
            }
        }
    }

    return std::to_string(sent) + " " + std::to_string(received);
}

std::string
LoraPacketTracker::CountMacPacketsGloballyCpsr(Time startTime, Time stopTime, uint8_t sf)
{
    NS_LOG_FUNCTION(this << startTime << stopTime);
    double sent = 0;
    double received = 0;
    std::vector<double> rtxCounts(5, 0);
    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        if ((*it).second.sf == sf)
        {
            if ((*it).second.firstAttempt >= startTime && (*it).second.firstAttempt <= stopTime)
            {
                sent++;
                if ((*it).second.reTxAttempts >= 1 && (*it).second.reTxAttempts <= 4)
                {
                    rtxCounts.at((*it).second.reTxAttempts - 1) += 1;
                }

                NS_LOG_DEBUG("Found a packet");
                NS_LOG_DEBUG("Number of attempts: " << unsigned(it->second.reTxAttempts)
                                                    << ", successful: " << it->second.successful);

                if (((*it).second.reTxAttempts == 4) && (*it).second.successful == true)
                {
                    rtxCounts.at((*it).second.reTxAttempts) += 1;

                    NS_LOG_DEBUG("Success in the last retransmission");
                }

                if (it->second.successful)
                {
                    received++;
                }
            }
        }
    }
    std::string output("");
    for (int i = 0; i < 5; i++)
    {
        output += std::to_string(rtxCounts.at(i)) + " ";
    }
    return output;
}

std::string
LoraPacketTracker::CountSuccessfulRetransmissions(
    Time startTime,
    Time stopTime,
    std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    NS_LOG_FUNCTION(this << startTime << stopTime);
    double sent = 0;
    double received = 0;
    std::vector<double> rtxCounts(MAXRTX, 0);
    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        LorawanMacHeader mHdr;
        LoraFrameHeader fHdr;
        Ptr<Packet> packetCopy;
        packetCopy = (*it).first->Copy();

        packetCopy->RemoveHeader(mHdr);
        packetCopy->RemoveHeader(fHdr);
        LoraDeviceAddress address = fHdr.GetAddress();

        if (mapDevices.find(address) != mapDevices.end())
        {
            if ((*it).second.firstAttempt >= startTime && (*it).second.firstAttempt <= stopTime)
            {
                sent++;
                if ((*it).second.reTxAttempts >= 1 && (*it).second.reTxAttempts <= MAXRTX &&
                    it->second.successful)
                {
                    rtxCounts.at((*it).second.reTxAttempts - 1) += 1;
                }

                NS_LOG_DEBUG("Found a packet");
                NS_LOG_DEBUG("Number of attempts: " << unsigned(it->second.reTxAttempts)
                                                    << ", successful: " << it->second.successful);

                if (it->second.successful)
                {
                    received++;
                }
            }
        }
    }
    std::string output("");
    std::cout << "sent: " << sent << " received: " << received << std::endl;
    output = std::to_string(sent) + " " + std::to_string(received) + " ";
    for (std::size_t i = 0; i < rtxCounts.size(); i++)
    {
        output += std::to_string(rtxCounts.at(i)) + " ";
    }
    std::cout << "output: " << output << std::endl;
    return output;
}

std::string
LoraPacketTracker::CountSuccessfulRetransmissions(
    Time startTime,
    Time stopTime,
    uint8_t sf,
    std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    NS_LOG_FUNCTION(this << startTime << stopTime);
    double sent = 0;
    double received = 0;
    std::vector<double> rtxCounts(MAXRTX, 0);
    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        if ((*it).second.sf == sf)
        {
            LorawanMacHeader mHdr;
            LoraFrameHeader fHdr;
            Ptr<Packet> packetCopy;
            packetCopy = (*it).first->Copy();

            packetCopy->RemoveHeader(mHdr);
            packetCopy->RemoveHeader(fHdr);
            LoraDeviceAddress address = fHdr.GetAddress();

            if (mapDevices.find(address) != mapDevices.end())
            {
                if ((*it).second.firstAttempt >= startTime && (*it).second.firstAttempt <= stopTime)
                {
                    sent++;
                    if ((*it).second.reTxAttempts >= 1 && (*it).second.reTxAttempts <= MAXRTX &&
                        it->second.successful)
                    {
                        rtxCounts.at((*it).second.reTxAttempts - 1) += 1;
                    }

                    NS_LOG_DEBUG("Found a packet");
                    NS_LOG_DEBUG("Number of attempts: " << unsigned(it->second.reTxAttempts)
                                                        << ", successful: "
                                                        << it->second.successful);
                    /* if (((*it).second.reTxAttempts == MAXRTX) && (*it).second.successful == true)
                    {
                        std::cout << "(*it).second.reTxAttempts: "
                                  << static_cast<int>((*it).second.reTxAttempts) << std::endl;

                        rtxCounts.at((*it).second.reTxAttempts) += 1;
                        std::cout <<  rtxCounts.at((*it).second.reTxAttempts) << std::endl;

                        NS_LOG_DEBUG("Success in the last retransmission");
                    } */

                    if (it->second.successful)
                    {
                        received++;
                    }
                }
            }
        }
    }
    std::string output("");
    std::cout << "sent: " << sent << " received: " << received << std::endl;
    output = std::to_string(sent) + " " + std::to_string(received) + " ";
    for (std::size_t i = 0; i < rtxCounts.size(); i++)
    {
        output += std::to_string(rtxCounts.at(i)) + " ";
    }
    std::cout << "output: " << output << std::endl;
    return output;
}

std::string
LoraPacketTracker::CountMacPacketsGloballyDelay(Time startTime,
                                                Time stopTime,
                                                uint32_t gwId,
                                                uint32_t gwNum)
{
    Time delaySum = Seconds(0);
    double avgDelay = 0;
    int packetsOutsideTransient = 0;

    for (uint32_t i = gwId; i < (gwId + gwNum); i++)
    {
        for (auto itMac = m_macPacketTracker.begin(); itMac != m_macPacketTracker.end(); ++itMac)
        {
            NS_LOG_DEBUG("Dealing with packet " << (*itMac).first);

            if ((*itMac).second.sendTime > startTime && (*itMac).second.sendTime < stopTime)
            {
                packetsOutsideTransient++;

                // Compute delays
                /////////////////
                if ((*itMac).second.receptionTimes.find(gwId)->second == Time::Max() ||
                    (*itMac).second.receptionTimes.find(gwId)->second < (*itMac).second.sendTime)
                {
                    NS_LOG_DEBUG("Packet never received, ignoring it");
                    packetsOutsideTransient--;
                }
                else
                {
                    delaySum += (*itMac).second.receptionTimes.find(gwId)->second -
                                (*itMac).second.sendTime;
                }
            }
        }
    }
    // cout << "trans: " << packetsOutsideTransient << " d: " << delaySum.GetSeconds() << endl;

    if (packetsOutsideTransient != 0)
    {
        avgDelay = (delaySum / packetsOutsideTransient).GetSeconds();
    }

    return (std::to_string(avgDelay));
}

std::string
LoraPacketTracker::CountMacPacketsGloballyDelay(Time startTime,
                                                Time stopTime,
                                                uint32_t gwId,
                                                uint32_t gwNum,
                                                uint8_t sf)
{
    Time delaySum = Seconds(0);
    double avgDelay = 0;
    int packetsOutsideTransient = 0;

    for (uint32_t i = gwId; i < (gwId + gwNum); i++)
    {
        for (auto itMac = m_macPacketTracker.begin(); itMac != m_macPacketTracker.end(); ++itMac)
        {
            // NS_LOG_DEBUG ("Dealing with packet " << (*itMac).first);
            if ((*itMac).second.sf == sf)
            {
                if ((*itMac).second.sendTime > startTime && (*itMac).second.sendTime < stopTime)
                {
                    packetsOutsideTransient++;

                    // Compute delays
                    /////////////////
                    if ((*itMac).second.receptionTimes.find(gwId)->second == Time::Max() ||
                        (*itMac).second.receptionTimes.find(gwId)->second <
                            (*itMac).second.sendTime)
                    {
                        // NS_LOG_DEBUG ("Packet never received, ignoring it");
                        packetsOutsideTransient--;
                    }
                    else
                    {
                        delaySum += (*itMac).second.receptionTimes.find(gwId)->second -
                                    (*itMac).second.sendTime;
                    }
                }
            }
        }
    }

    if (packetsOutsideTransient != 0)
    {
        avgDelay = (delaySum / packetsOutsideTransient).GetSeconds();
    }

    return (std::to_string(avgDelay));
}

std::string
LoraPacketTracker::CountMacPacketsGloballyDelay(Time startTime,
                                                Time stopTime,
                                                uint32_t gwId,
                                                uint32_t gwNum,
                                                std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    Time delaySum = Seconds(0);
    double avgDelay = 0;
    int packetsOutsideTransient = 0;

    for (uint32_t i = gwId; i < (gwId + gwNum); i++)
    {
        for (auto itMac = m_macPacketTracker.begin(); itMac != m_macPacketTracker.end(); ++itMac)
        {
            NS_LOG_DEBUG("Dealing with packet " << (*itMac).first);

            Ptr<Packet> packetCopy = (*itMac).first->Copy();
            LorawanMacHeader mHdr;
            LoraFrameHeader fHdr;
            packetCopy->RemoveHeader(mHdr);
            packetCopy->RemoveHeader(fHdr);
            LoraDeviceAddress address = fHdr.GetAddress();
            if (mapDevices.find(address) != mapDevices.end())
            {
                if ((*itMac).second.receptionTimes.find(i) != (*itMac).second.receptionTimes.end())
                {
                    if ((*itMac).second.sendTime > startTime && (*itMac).second.sendTime < stopTime)
                    {
                        packetsOutsideTransient++;

                        // Compute delays
                        /////////////////
                        if ((*itMac).second.receptionTimes.find(i)->second == Time::Max() ||
                            (*itMac).second.receptionTimes.find(i)->second <
                                (*itMac).second.sendTime)
                        {
                            NS_LOG_DEBUG("Packet never received, ignoring it");
                            packetsOutsideTransient--;
                        }
                        else
                        {
                            delaySum += (*itMac).second.receptionTimes.find(i)->second -
                                        (*itMac).second.sendTime;
                        }
                    }
                }
            }
        }
    }

    if (packetsOutsideTransient != 0)
    {
        avgDelay = (delaySum / packetsOutsideTransient).GetSeconds();
    }

    return (std::to_string(avgDelay));
}

std::string
LoraPacketTracker::CountMacPacketsGloballyDelay(Time startTime,
                                                Time stopTime,
                                                uint32_t gwId,
                                                uint32_t gwNum,
                                                uint8_t sf,
                                                std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    Time delaySum = Seconds(0);
    double avgDelay = 0;
    int packetsOutsideTransient = 0;

    for (uint32_t i = gwId; i < (gwId + gwNum); i++)
    {
        for (auto itMac = m_macPacketTracker.begin(); itMac != m_macPacketTracker.end(); ++itMac)
        {
            // NS_LOG_DEBUG ("Dealing with packet " << (*itMac).first);
            if ((*itMac).second.sf == sf)
            {
                Ptr<Packet> packetCopy = (*itMac).first->Copy();
                LorawanMacHeader mHdr;
                LoraFrameHeader fHdr;
                packetCopy->RemoveHeader(mHdr);
                packetCopy->RemoveHeader(fHdr);
                LoraDeviceAddress address = fHdr.GetAddress();
                if (mapDevices.find(address) != mapDevices.end())
                {
                    if ((*itMac).second.sendTime > startTime && (*itMac).second.sendTime < stopTime)
                    {
                        // Compute delays
                        /////////////////
                        if ((*itMac).second.receptionTimes.find(i) !=
                            (*itMac).second.receptionTimes.end())
                        {
                            packetsOutsideTransient++;
                            if ((*itMac).second.receptionTimes.find(i)->second == Time::Max() ||
                                (*itMac).second.receptionTimes.find(i)->second <
                                    (*itMac).second.sendTime)
                            {
                                // NS_LOG_DEBUG ("Packet never received, ignoring it");
                                packetsOutsideTransient--;
                            }
                            else
                            {
                                // std::cout << "Tempo que o GW " <<
                                // (*itMac).second.receptionTimes.find(i)->first << " recebeu: " <<
                                // (*itMac).second.receptionTimes.find(gwId)->second << " Tempo que
                                // pacote foi enviado: " << (*itMac).second.sendTime << std::endl;
                                delaySum += (*itMac).second.receptionTimes.find(i)->second -
                                            (*itMac).second.sendTime;
                            }
                        }
                    }
                }
            }
        }
    }
    // std::cout << "trans: " << packetsOutsideTransient << " d: " << delaySum.GetSeconds() <<
    // std::endl;

    if (packetsOutsideTransient != 0)
    {
        avgDelay = (delaySum / packetsOutsideTransient).GetSeconds();
    }

    return (std::to_string(avgDelay));
}

std::string
LoraPacketTracker::CountMacPacketsGloballyDelayWithRetransmission(Time startTime,
                                                                  Time stopTime,
                                                                  uint32_t gwId,
                                                                  uint32_t gwNum)
{
    Time delaySum = Seconds(0);
    double avgDelay = 0;
    int packetsOutsideTransient = 0;

    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        RetransmissionStatus retransStatus = it->second;

        // Verifica se a primeira tentativa de envio está dentro do intervalo de tempo
        if (retransStatus.firstAttempt > startTime && retransStatus.firstAttempt < stopTime)
        {
            // Somente considera pacotes que foram recebidos com sucesso
            if (retransStatus.successful)
            {
                packetsOutsideTransient++;

                // Calcula o delay considerando a primeira tentativa e o tempo final da
                // retransmissão
                Time delay = retransStatus.finishTime - retransStatus.firstAttempt;
                delaySum += delay;
            }
        }
    }

    if (packetsOutsideTransient != 0)
    {
        avgDelay = (delaySum / packetsOutsideTransient).GetSeconds();
    }

    return std::to_string(avgDelay);
}

std::string
LoraPacketTracker::CountMacPacketsGloballyDelayWithRetransmission(Time startTime,
                                                                  Time stopTime,
                                                                  uint32_t gwId,
                                                                  uint32_t gwNum,
                                                                  uint8_t sf)
{
    Time delaySum = Seconds(0);
    double avgDelay = 0;
    int packetsOutsideTransient = 0;

    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        RetransmissionStatus retransStatus = it->second;

        // Verifica se o pacote pertence ao Spreading Factor correto
        if (retransStatus.sf == sf)
        {
            // Verifica se a primeira tentativa de envio está dentro do intervalo de tempo
            if (retransStatus.firstAttempt > startTime && retransStatus.firstAttempt < stopTime)
            {
                // Somente considera pacotes que foram recebidos com sucesso
                if (retransStatus.successful)
                {
                    packetsOutsideTransient++;

                    // Calcula o delay considerando a primeira tentativa e o tempo final da
                    // retransmissão
                    Time delay = retransStatus.finishTime - retransStatus.firstAttempt;
                    delaySum += delay;
                }
            }
        }
    }

    if (packetsOutsideTransient != 0)
    {
        avgDelay = (delaySum / packetsOutsideTransient).GetSeconds();
    }

    return std::to_string(avgDelay);
}

std::string
LoraPacketTracker::CountMacPacketsGloballyDelayWithRetransmission(
    Time startTime,
    Time stopTime,
    uint32_t gwId,
    uint32_t gwNum,
    std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    Time delaySum = Seconds(0);
    double avgDelay = 0;
    int packetsOutsideTransient = 0;

    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        RetransmissionStatus retransStatus = it->second;
        Ptr<Packet> packetCopy = (*it).first->Copy();
        LorawanMacHeader mHdr;
        LoraFrameHeader fHdr;
        packetCopy->RemoveHeader(mHdr);
        packetCopy->RemoveHeader(fHdr);
        LoraDeviceAddress address = fHdr.GetAddress();
        if (mapDevices.find(address) != mapDevices.end())
        {
            // Verifica se a primeira tentativa de envio está dentro do intervalo de tempo
            if (retransStatus.firstAttempt > startTime && retransStatus.firstAttempt < stopTime)
            {
                // Somente considera pacotes que foram recebidos com sucesso
                if (retransStatus.successful)
                {
                    packetsOutsideTransient++;

                    // Calcula o delay considerando a primeira tentativa e o tempo final da
                    // retransmissão
                    Time delay = retransStatus.finishTime - retransStatus.firstAttempt;
                    delaySum += delay;
                }
            }
        }
    }

    if (packetsOutsideTransient != 0)
    {
        avgDelay = (delaySum / packetsOutsideTransient).GetSeconds();
    }

    return std::to_string(avgDelay);
}

std::string
LoraPacketTracker::CountMacPacketsGloballyDelayWithRetransmission(
    Time startTime,
    Time stopTime,
    uint32_t gwId,
    uint32_t gwNum,
    uint8_t sf,
    std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    Time delaySum = Seconds(0);
    double avgDelay = 0;
    int packetsOutsideTransient = 0;

    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        RetransmissionStatus retransStatus = it->second;

        if (retransStatus.sf == sf)
        {
            Ptr<Packet> packetCopy = (*it).first->Copy();
            LorawanMacHeader mHdr;
            LoraFrameHeader fHdr;
            packetCopy->RemoveHeader(mHdr);
            packetCopy->RemoveHeader(fHdr);
            LoraDeviceAddress address = fHdr.GetAddress();
            if (mapDevices.find(address) != mapDevices.end())
            {
                // Verifica se a primeira tentativa de envio está dentro do intervalo de tempo
                if (retransStatus.firstAttempt > startTime && retransStatus.firstAttempt < stopTime)
                {
                    // Somente considera pacotes que foram recebidos com sucesso
                    if (retransStatus.successful)
                    {
                        packetsOutsideTransient++;

                        // Calcula o delay considerando a primeira tentativa e o tempo final da
                        // retransmissão
                        Time delay = retransStatus.finishTime - retransStatus.firstAttempt;
                        delaySum += delay;
                    }
                }
            }
        }
    }

    if (packetsOutsideTransient != 0)
    {
        avgDelay = (delaySum / packetsOutsideTransient).GetSeconds();
    }

    return std::to_string(avgDelay);
}

std::string
LoraPacketTracker::CountAgeOfInformationGlobally(Time startTime, Time stopTime, uint8_t sf)
{
    Time AOISum = Seconds(0);
    double avgAOI = 0;
    signed int received = 0;

    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        if ((*it).second.sf == sf)
        {
            if ((*it).second.firstAttempt >= startTime && (*it).second.firstAttempt <= stopTime)
            {
                NS_LOG_DEBUG("Found a packet");
                NS_LOG_DEBUG("Number of attempts: " << unsigned(it->second.reTxAttempts)
                                                    << ", successful: " << it->second.successful);
                if (it->second.successful)
                {
                    AOISum = it->second.finishTime - it->second.firstAttempt;
                    received++;
                }
                NS_LOG_INFO("sf: " << static_cast<int>((*it).second.sf));
                NS_LOG_INFO("enviado: " << double((*it).second.firstAttempt.GetSeconds()));
                NS_LOG_INFO("recebido: " << double((*it).second.finishTime.GetSeconds()));
            }
        }
    }
    NS_LOG_INFO("received: " << received << ", AOISum: " << AOISum.GetSeconds());

    if (received != 0)
    {
        avgAOI = (AOISum / received).GetSeconds();
    }

    return (std::to_string(avgAOI));
}

std::string
LoraPacketTracker::CountAgeOfInformationGlobally(Time startTime,
                                                 Time stopTime,
                                                 uint8_t sf,
                                                 std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    Time AOISum = Seconds(0);
    double avgAOI = 0;
    signed int received = 0;

    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        if ((*it).second.sf == sf)
        {
            if ((*it).second.firstAttempt >= startTime && (*it).second.firstAttempt <= stopTime)
            {
                NS_LOG_DEBUG("Found a packet");
                NS_LOG_DEBUG("Number of attempts: " << unsigned(it->second.reTxAttempts)
                                                    << ", successful: " << it->second.successful);
                if (it->second.successful)
                {
                    AOISum = it->second.finishTime - it->second.firstAttempt;
                    received++;
                }
                NS_LOG_INFO("sf: " << static_cast<int>((*it).second.sf));
                NS_LOG_INFO("enviado: " << double((*it).second.firstAttempt.GetSeconds()));
                NS_LOG_INFO("recebido: " << double((*it).second.finishTime.GetSeconds()));
            }
        }
    }
    NS_LOG_INFO("received: " << received << ", AOISum: " << AOISum.GetSeconds());

    if (received != 0)
    {
        avgAOI = (AOISum / received).GetSeconds();
    }

    return (std::to_string(avgAOI));
}

std::string
LoraPacketTracker::CountAgeOfInformationGlobally(Time startTime,
                                                 Time stopTime,
                                                 std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    Time AOISum = Seconds(0);
    double avgAOI = 0;
    signed int received = 0;

    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        if ((*it).second.firstAttempt >= startTime && (*it).second.firstAttempt <= stopTime)
        {
            NS_LOG_DEBUG("Found a packet");
            NS_LOG_DEBUG("Number of attempts: " << unsigned(it->second.reTxAttempts)
                                                << ", successful: " << it->second.successful);
            if (it->second.successful)
            {
                AOISum = it->second.finishTime - it->second.firstAttempt;
                received++;
            }
            NS_LOG_INFO("sf: " << static_cast<int>((*it).second.sf));
            NS_LOG_INFO("enviado: " << double((*it).second.firstAttempt.GetSeconds()));
            NS_LOG_INFO("recebido: " << double((*it).second.finishTime.GetSeconds()));
        }
    }
    NS_LOG_INFO("received: " << received << ", AOISum: " << AOISum.GetSeconds());

    if (received != 0)
    {
        avgAOI = (AOISum / received).GetSeconds();
    }

    return (std::to_string(avgAOI));
}

DataAgeInformation
LoraPacketTracker::GetDataAoi()
{
    return m_dataAoi;
}

void
LoraPacketTracker::PrintRetransmissionData()
{
    std::cout << "Retransmission Data:\n";
    for (auto& pair : m_reTransmissionTracker)
    {
        Ptr<const Packet> packet = pair.first;
        const RetransmissionStatus& status = pair.second;

        std::cout << "Packet ID: " << packet << " | "
                  << "reTxAttempts: " << static_cast<int>(status.reTxAttempts) << " | "
                  << "Successful: " << (status.successful ? "Yes" : "No") << "\n";
    }
}

void
LoraPacketTracker::PrintRetransmissionData2()
{
    std::cout << "m_macPacketTracker:\n";
    for (const auto& pair : m_macPacketTracker)
    {
        Ptr<const Packet> packet = pair.first;       // Pacote sendo rastreado
        const MacPacketStatus& status = pair.second; // Status do pacote

        // Exibe informações básicas do pacote
        std::cout << "Packet ID: " << packet << " | "
                  << "Sender ID: " << status.senderId << " | "
                  << "SF: " << static_cast<int>(status.sf) << " | "
                  << "Send Time: " << status.sendTime.As(Time::S) << "s | "
                  << "Received Time: " << status.receivedTime.As(Time::S) << "s\n";

        // Exibe os tempos de recepção para cada receptor
        std::cout << "Reception Times:\n";
        for (const auto& receptionPair : status.receptionTimes)
        {
            int receiverId = receptionPair.first;      // ID do receptor
            Time receptionTime = receptionPair.second; // Tempo de recepção

            std::cout << "  Receiver ID: " << receiverId << " | "
                      << "Reception Time: " << receptionTime.As(Time::S) << "s\n";
        }

        std::cout << "----------------------------\n";
    }
}

} // namespace lorawan
} // namespace ns3
