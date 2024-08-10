/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

#include "lora-packet-tracker.h"

#include "ns3/address.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/log.h"
#include "ns3/lora-frame-header.h"
#include "ns3/lora-helper.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/node-container.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"

#include <fstream>
#include <iostream>

namespace ns3
{
namespace lorawan
{
// Inicialização do membro estático
DataAgeInformation LoraPacketTracker::m_dataAoi;
std::map<Ptr<const Packet>, DataAoi> LoraPacketTracker::aoiMap;

NS_LOG_COMPONENT_DEFINE("LoraPacketTracker");

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

        if (packet != nullptr)
        {
            m_macPacketTracker.insert(
                std::pair<Ptr<const Packet>, MacPacketStatus>(packet, status));
        }
    }
}

void
LoraPacketTracker::RequiredTransmissionsCallback(uint8_t reqTx,
                                                 bool success,
                                                 ns3::Time firstAttempt,
                                                 ns3::Ptr<ns3::Packet> packet,
                                                 uint8_t sf,
                                                 bool ackFirstWindow)
{
    NS_LOG_INFO("Finished retransmission attempts for a packet");
    NS_LOG_DEBUG("Packet: " << packet << " ReqTx " << unsigned(reqTx) << ", succ: " << success
                            << ", firstAttempt: " << firstAttempt.GetSeconds());

    RetransmissionStatus entry;
    entry.firstAttempt = firstAttempt;
    entry.finishTime = ns3::Simulator::Now();
    entry.sf = sf;
    entry.reTxAttempts = reqTx;
    entry.successful = success;
    entry.ackFirstWindown = ackFirstWindow;

    // Variáveis estáticas para manter o estado entre chamadas
    static ns3::Time previousFirst = ns3::Seconds(0);

    // Imprimir os valores antes da subtração
    std::cout << "Current firstAttempt: " << firstAttempt.GetSeconds() << " seconds\n";
    std::cout << "Previous firstAttempt: " << previousFirst.GetSeconds() << " seconds\n";

    // Calcular delta1 garantindo que seja sempre positivo
    ns3::Time delta1Second =
        firstAttempt > previousFirst ? firstAttempt - previousFirst : previousFirst - firstAttempt;
    std::cout << "Delta1 (absolute difference): " << delta1Second.GetSeconds() << " seconds\n";

    DataAoi data;
    data.delta1 = std::make_pair(firstAttempt, delta1Second);
    data.reset1 = std::make_pair(firstAttempt, ns3::Seconds(0));
    data.delta = std::make_pair(entry.finishTime, entry.finishTime);
    data.reset = std::make_pair(entry.finishTime, entry.finishTime - firstAttempt);

    if (packet != nullptr)
    {
        m_reTransmissionTracker.insert(std::make_pair(packet, entry));
        aoiMap.insert(std::make_pair(packet, data));
    }

    // Atualizar variáveis estáticas para a próxima chamada
    previousFirst = firstAttempt;
}

DataAgeInformation
LoraPacketTracker::GetDataAoi()
{
    return m_dataAoi;
}

void
LoraPacketTracker::MacGwReceptionCallback(Ptr<const Packet> packet)
{
    if (IsUplink(packet))
    {
        NS_LOG_INFO("A packet was successfully received" << " at the MAC layer of gateway "
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

        std::map<Ptr<const Packet>, PacketStatus>::iterator it = m_packetTracker.find(packet);
        (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome>(gwId, RECEIVED));
    }
}

void
LoraPacketTracker::InterferenceCallback(Ptr<const Packet> packet, uint32_t gwId)
{
    if (IsUplink(packet))
    {
        NS_LOG_INFO("PHY packet " << packet << " was interfered at gateway " << gwId);

        std::map<Ptr<const Packet>, PacketStatus>::iterator it = m_packetTracker.find(packet);
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
        std::map<Ptr<const Packet>, PacketStatus>::iterator it = m_packetTracker.find(packet);
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

        std::map<Ptr<const Packet>, PacketStatus>::iterator it = m_packetTracker.find(packet);
        (*it).second.outcomes.insert(
            std::pair<int, enum PhyPacketOutcome>(gwId, UNDER_SENSITIVITY));
    }
}

void
LoraPacketTracker::LostBecauseTxCallback(Ptr<const Packet> packet, uint32_t gwId)
{
    if (IsUplink(packet))
    {
        NS_LOG_INFO("PHY packet " << packet << " was lost because of GW transmission at gateway "
                                  << gwId);

        std::map<Ptr<const Packet>, PacketStatus>::iterator it = m_packetTracker.find(packet);
        (*it).second.outcomes.insert(std::pair<int, enum PhyPacketOutcome>(gwId, LOST_BECAUSE_TX));
    }
}

DataAgeInformation
LoraPacketTracker::AgeOfInformationData(Time startTime,
                                        Time stopTime,
                                        std::map<LoraDeviceAddress, uint8_t> AoIPlottingDevices)
{
    for (auto it = AoIPlottingDevices.begin(); it != AoIPlottingDevices.end(); ++it)
    {
        for (auto j = aoiMap.begin(); j != aoiMap.end(); ++j)
        {
            LorawanMacHeader mHdr;
            LoraFrameHeader fHdr;
            Ptr<Packet> packetCopy = j->first->Copy();
            packetCopy->RemoveHeader(mHdr);
            packetCopy->RemoveHeader(fHdr);
            LoraDeviceAddress address = fHdr.GetAddress();

            if (it->first == address)
            {
                // std::cout<<"sf: "<< static_cast<int>(it->second)<<std::endl;
                /* Reference:
                 * Roy D. Yates, Yin Sun, D. Richard Brown, III, Sanjit K. Kaul, Eytan
                 * Modiano, Sennur Ulukus, "Age of Information: An Introduction and Survey",
                 * IEEE Journal, Fellow, IEEE.
                 */

                // Debug: Antes de adicionar os pontos
                /* std::cout << "endereço do pacote " << address << "endereço do dispositivo " <<
                   it->first
                          << std::endl; */

                // Adiciona ponto à primarySeries
                m_dataAoi[it->second].primarySeries.push_back(
                    std::make_pair(j->second.delta1.first.GetSeconds(),
                                   j->second.delta1.second.GetSeconds()));
                /* std::cout << "Added to primarySeries: (" << j->second.delta1.first.GetSeconds()
                          << ", " << j->second.delta1.second.GetSeconds() << ")" << std::endl; */

                // Reset delta1
                m_dataAoi[it->second].primarySeries.push_back(
                    std::make_pair(j->second.reset1.first.GetSeconds(), Seconds(0).GetSeconds()));
                /* std::cout << "Reset primarySeries: (" << j->second.reset1.first.GetSeconds()
                          << ", 0)" << std::endl; */

                // Adiciona ponto à secondarySeries
                m_dataAoi[it->second].secondarySeries.push_back(
                    std::make_pair(j->second.delta.first.GetSeconds(),
                                   j->second.delta.second.GetSeconds()));
                /* std::cout << "Added to secondarySeries: (" << j->second.delta.first.GetSeconds()
                          << ", " << j->second.delta.second.GetSeconds() << ")" << std::endl; */

                // Reset delta
                m_dataAoi[it->second].secondarySeries.push_back(
                    std::make_pair(j->second.reset.first.GetSeconds(),
                                   j->second.reset.second.GetSeconds()));
                /*  std::cout << "Reset secondarySeries: (" << j->second.reset.first.GetSeconds()
                           << ", " << j->second.reset.second.GetSeconds() << ")" << std::endl; */
            }
        }
    }
    return m_dataAoi;
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
            if ((*it).second.receptionTimes.size())
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
                if ((*it).first)
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
LoraPacketTracker::CountInformationOfAgeGlobally(Time startTime,
                                                 Time stopTime,
                                                 uint32_t gwId,
                                                 uint32_t gwNum,
                                                 uint8_t sf)
{
    Time delaySum = Seconds(0);
    double avgDelay = 0;
    double sent = 0;
    double received = 0;
    LoraTxParameters params;

    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        if ((*it).second.sf == sf)
        {
            params.sf = sf;
            if ((*it).second.firstAttempt >= startTime && (*it).second.firstAttempt <= stopTime)
            {
                sent++;

                NS_LOG_DEBUG("Found a packet");
                NS_LOG_DEBUG("Number of attempts: " << unsigned(it->second.reTxAttempts)
                                                    << ", successful: " << it->second.successful);
                if (it->second.successful)
                {
                    Time oat = LoraPhy::GetOnAirTime((*it).first->Copy(), params);
                    received++;
                    delaySum = it->second.finishTime - it->second.firstAttempt;
                    delaySum = Seconds(0);

                    /* if ((*it).second.ackFirstWindown &&
                        static_cast<int>((*it).second.reTxAttempts) == 2)
                    {
                        std::cout << "sf:" << static_cast<int>((*it).second.sf) << std::endl;
                        std::cout << "Age of information: " << delaySum.GetSeconds()
                                  << " J : " << (*it).second.ackFirstWindown << std::endl;

                        std::cout << "On air time: "
                                  << double(LoraPhy::GetOnAirTime((*it).first->Copy(), params)
                                                .GetSeconds())
                                  << std::endl
                                  << std::endl;
                    } */
                }
                /* std::cout << "sf:" << static_cast<int>((*it).second.sf) << std::endl;
                std::cout << "enviado: " << double((*it).second.firstAttempt.GetSeconds()) <<
                std::endl; std::cout << "recebido: " << double((*it).second.finishTime.GetSeconds())
                << std::endl << std::endl; */
            }
        }
    }
    /* std::cout << "sent: " << sent << " received: " << received << " d: " << delaySum.GetSeconds()
              << std::endl; */

    if (received != 0)
    {
        avgDelay = (delaySum / received).GetSeconds();
    }

    return (std::to_string(avgDelay));
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
    std::vector<double> rtxCounts(4, 0);

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
                if (it->second.successful)
                {
                    received++;
                }
            }
        }
    }

    std::string output("");
    for (int i = 0; i < 4; i++)
    {
        output += std::to_string(rtxCounts.at(i)) + " ";
    }
    return output;
}

std::string
LoraPacketTracker::CountMacPacketsGloballyCpsr(Time startTime,
                                               Time stopTime,
                                               uint8_t sf,
                                               std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    NS_LOG_FUNCTION(this << startTime << stopTime);
    double sent = 0;
    double received = 0;
    std::vector<double> rtxCounts(4, 0);
    for (auto it = m_reTransmissionTracker.begin(); it != m_reTransmissionTracker.end(); ++it)
    {
        if ((*it).second.sf == sf)
        {
            LorawanMacHeader mHdr;
            LoraFrameHeader fHdr;
            Ptr<Packet> packetCopy;
            if ((*it).first == nullptr)
            {
                NS_LOG_DEBUG(static_cast<int>((*it).second.sf));
            }
            else
            {
                packetCopy = (*it).first->Copy();
            }
            packetCopy->RemoveHeader(mHdr);
            packetCopy->RemoveHeader(fHdr);
            LoraDeviceAddress address = fHdr.GetAddress();

            if (mapDevices.find(address) != mapDevices.end())
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
                                                        << ", successful: "
                                                        << it->second.successful);
                    if (it->second.successful)
                    {
                        received++;
                    }
                }
            }
        }
    }
    std::string output("");
    for (int i = 0; i < 4; i++)
    {
        output += std::to_string(rtxCounts.at(i)) + " ";
    }
    return output;
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
                                                uint32_t gwNum)
{
    Time delaySum = Seconds(0);
    double avgDelay = 0;
    int packetsOutsideTransient = 0;

    for (uint32_t i = gwId; i < (gwId + gwNum); i++)
    {
        for (auto itMac = m_macPacketTracker.begin(); itMac != m_macPacketTracker.end(); ++itMac)
        {
            // NS_LOG_DEBUG ("Dealing with packet " << (*itMac).first);

            if ((*itMac).second.sendTime > startTime && (*itMac).second.sendTime < stopTime)
            {
                packetsOutsideTransient++;

                // Compute delays
                /////////////////
                if ((*itMac).second.receptionTimes.find(gwId)->second == Time::Max() ||
                    (*itMac).second.receptionTimes.find(gwId)->second < (*itMac).second.sendTime)
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
    // std::cout << "trans: " << packetsOutsideTransient << " d: " << delaySum.GetSeconds() <<
    // std::endl;

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

} // namespace lorawan
} // namespace ns3
