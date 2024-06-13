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
                                                 Time firstAttempt,
                                                 Ptr<Packet> packet,
                                                 uint8_t sf,
                                                 bool ackFirstWindow)
{
    NS_LOG_INFO("Finished retransmission attempts for a packet");
    if (packet == nullptr)
    {
        NS_LOG_DEBUG("PACOTE NULL");
    }
    NS_LOG_DEBUG("Packet: " << packet << " ReqTx " << unsigned(reqTx) << ", succ: " << success
                            << ", firstAttempt: " << firstAttempt.GetSeconds());

    RetransmissionStatus entry;
    entry.firstAttempt = firstAttempt;
    entry.finishTime = Simulator::Now();
    entry.sf = sf;
    entry.reTxAttempts = reqTx;
    entry.successful = success;
    entry.ackFirstWindown = ackFirstWindow;
    if (packet != nullptr)
    {
        m_reTransmissionTracker.insert(std::pair<Ptr<Packet>, RetransmissionStatus>(packet, entry));
    }
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
LoraPacketTracker::AvgPacketTimeOnAir(Time startTime,
                                      Time stopTime,
                                      uint32_t gwId,
                                      uint32_t gwNum,
                                      uint8_t sf)
{
    NS_LOG_FUNCTION(this << startTime << stopTime);
    Time timeOnAir = Seconds(0);
    std::map<double, int> timeOnAirFrequency;
    LoraTxParameters params;

    for (uint32_t i = gwId; i < (gwId + gwNum); i++)
    {
        for (auto it = m_macPacketTracker.begin(); it != m_macPacketTracker.end(); ++it)
        {
            if ((*it).second.sf == sf)
            {
                if ((*it).second.sendTime >= startTime && (*it).second.sendTime <= stopTime)
                {
                    if ((*it).second.receptionTimes.size())
                    {
                        if ((*it).second.receptionTimes.find(i) !=
                            (*it).second.receptionTimes.end())
                        {
                            timeOnAir = LoraPhy::GetOnAirTime((*it).first->Copy(), params);
                            double timeOnAirSeconds = timeOnAir.GetSeconds();

                            // Incrementa a frequência do tempo no ar
                            if (timeOnAirFrequency.find(timeOnAirSeconds) ==
                                timeOnAirFrequency.end())
                            {
                                timeOnAirFrequency[timeOnAirSeconds] = 1;
                            }
                            else
                            {
                                timeOnAirFrequency[timeOnAirSeconds]++;
                            }
                        }
                    }
                }
            }
        }
    }

    // Calcula a média ponderada
    double totalWeightedTimeOnAir = 0;
    int totalWeight = 0;
    for (const auto& pair : timeOnAirFrequency)
    {
        double timeOnAir = pair.first;
        int weight = pair.second;
        totalWeightedTimeOnAir += timeOnAir * weight;
        totalWeight += weight;
    }

    double avgTimeOnAir = totalWeightedTimeOnAir / totalWeight;

    return std::to_string(avgTimeOnAir);
}

std::string
LoraPacketTracker::AvgPacketTimeOnAir(Time startTime,
                                      Time stopTime,
                                      uint8_t sf,
                                      std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    NS_LOG_FUNCTION(this << startTime << stopTime);
    Time timeOnAir = Seconds(0);
    double avgTimeOnAir = 0;
    int count = 0;

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
                    if ((*it).second.receptionTimes.size())
                    {
                        timeOnAir += (*it).second.receivedTime - (*it).second.sendTime;
                        count++;
                    }
                }
            }
        }
    }
    avgTimeOnAir = (timeOnAir / count).GetSeconds();
    return std::to_string(avgTimeOnAir);
}

std::string
LoraPacketTracker::AvgPacketTimeOnAir(Time startTime,
                                      Time stopTime,
                                      uint32_t gwId,
                                      uint32_t gwNum,
                                      uint8_t sf,
                                      std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    Time timeOnAir = Seconds(0);
    std::map<double, int> timeOnAirFrequency;
    LoraTxParameters params;

    for (uint32_t i = gwId; i < (gwId + gwNum); i++)
    {
        // std::map<ns3::Ptr<ns3::Node>, deviceType>::iterator j = deviceTypeMap.begin();

        for (auto itMac = m_macPacketTracker.begin(); itMac != m_macPacketTracker.end(); ++itMac)
        {
            if ((*itMac).second.sf == sf)
            {
                params.sf = sf;
                Ptr<Packet> packetCopy =
                    (*itMac).first->Copy(); // Crie uma cópia não-constante do pacote
                LorawanMacHeader mHdr;
                LoraFrameHeader fHdr;
                packetCopy->RemoveHeader(mHdr);
                packetCopy->RemoveHeader(fHdr);
                LoraDeviceAddress address = fHdr.GetAddress();
                if (mapDevices.find(address) != mapDevices.end())
                {
                    if ((*itMac).second.sendTime > startTime && (*itMac).second.sendTime < stopTime)
                    {
                        if ((*itMac).second.receptionTimes.find(i) !=
                            (*itMac).second.receptionTimes.end())
                        {
                            timeOnAir = LoraPhy::GetOnAirTime((*itMac).first->Copy(), params);
                            double timeOnAirSeconds = timeOnAir.GetSeconds();

                            // Incrementa a frequência do tempo no ar
                            if (timeOnAirFrequency.find(timeOnAirSeconds) ==
                                timeOnAirFrequency.end())
                            {
                                timeOnAirFrequency[timeOnAirSeconds] = 1;
                            }
                            else
                            {
                                timeOnAirFrequency[timeOnAirSeconds]++;
                            }
                        }
                    }
                }
            }
        }
    }

    // Calcula a média ponderada
    double totalWeightedTimeOnAir = 0;
    int totalWeight = 0;
    for (const auto& pair : timeOnAirFrequency)
    {
        double timeOnAir = pair.first;
        int weight = pair.second;
        totalWeightedTimeOnAir += timeOnAir * weight;
        totalWeight += weight;
    }

    double avgTimeOnAir = totalWeightedTimeOnAir / totalWeight;

    return std::to_string(avgTimeOnAir);
}

std::string
LoraPacketTracker::AvgPacketTimeOnAirRtx(Time startTime,
                                         Time stopTime,
                                         uint32_t gwId,
                                         uint32_t gwNum,
                                         uint8_t sf,
                                         std::map<LoraDeviceAddress, deviceFCtn> mapDevices)
{
    Time timeOnAir = Seconds(0);
    std::map<double, int> timeOnAirFrequency;
    LoraTxParameters params;

    for (uint32_t i = gwId; i < (gwId + gwNum); i++)
    {
        for (auto itMac = m_reTransmissionTracker.begin(); itMac != m_reTransmissionTracker.end();
             ++itMac)
        {
            if ((*itMac).second.sf == sf)
            {
                params.sf = sf;
                Ptr<Packet> packetCopy =
                    (*itMac).first->Copy(); // Crie uma cópia não-constante do pacote
                LorawanMacHeader mHdr;
                LoraFrameHeader fHdr;
                packetCopy->RemoveHeader(mHdr);
                packetCopy->RemoveHeader(fHdr);
                LoraDeviceAddress address = fHdr.GetAddress();
                if (mapDevices.find(address) != mapDevices.end())
                {
                    if ((*itMac).second.firstAttempt >= startTime &&
                        (*itMac).second.firstAttempt <= stopTime)
                    {
                        timeOnAir = LoraPhy::GetOnAirTime((*itMac).first->Copy(), params);
                        double timeOnAirSeconds = timeOnAir.GetSeconds();

                        // Esquema Temporal

                        // 1. Envio do uplink: t = 0 s
                        // 2. Abertura da RX1: t = 1 s (m_receiveDelay1)
                        // 3. Escuta na RX1: t = 1 s a t = 2 s (1 segundo de escuta)
                        // 4. Fechamento da RX1: t = 2 s
                        // 5. Abertura da RX2: t = 2 s (após m_receiveDelay2 do envio do uplink)
                        // 6. Escuta na RX2: t = 2 s a t = 4 s (2 segundos de escuta)
                        
                        // Tempo total caso o pacote seja recebido na RX1: 2 segundos
                        if ((*itMac).second.ackFirstWindown)
                        {
                            timeOnAirSeconds = timeOnAirSeconds + Seconds(2).GetSeconds();
                        }
                        // Tempo total caso o pacote seja recebido na RX2:t_total_RX2 = 5 segundos
                        else{

                        }

                        // Incrementa a frequência do tempo no ar
                        if (timeOnAirFrequency.find(timeOnAirSeconds) == timeOnAirFrequency.end())
                        {
                            timeOnAirFrequency[timeOnAirSeconds] = 1;
                        }
                        else
                        {
                            timeOnAirFrequency[timeOnAirSeconds]++;
                        }
                    }
                }
            }
        }
    }

    // Calcula a média ponderada
    double totalWeightedTimeOnAir = 0;
    int totalWeight = 0;
    for (const auto& pair : timeOnAirFrequency)
    {
        double timeOnAir = pair.first;
        int weight = pair.second;
        totalWeightedTimeOnAir += timeOnAir * weight;
        totalWeight += weight;
    }

    double avgTimeOnAir = totalWeightedTimeOnAir / totalWeight;

    return std::to_string(avgTimeOnAir);
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
