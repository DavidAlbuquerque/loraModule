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

#ifndef LORA_PACKET_TRACKER_H
#define LORA_PACKET_TRACKER_H

#include "ns3/end-device-lora-phy.h"
#include "ns3/lora-device-address.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"

#include <map>
#include <string>

namespace ns3
{
namespace lorawan
{

enum PhyPacketOutcome
{
    RECEIVED,
    INTERFERED,
    NO_MORE_RECEIVERS,
    UNDER_SENSITIVITY,
    LOST_BECAUSE_TX,
    UNSET
};

struct PacketStatus
{
    Ptr<const Packet> packet;
    uint32_t senderId;
    Time sendTime;
    std::map<int, enum PhyPacketOutcome> outcomes;
};

struct deviceFCtn
{
    uint16_t id;
    uint16_t FCtn = 0;
};

enum deviceType
{
    ALARM_DEVICE = 0,
    REGULAR_DEVICE,
    ALL
};

struct MacPacketStatus
{
    Ptr<const Packet> packet;
    uint32_t senderId;
    uint8_t sf;
    Time sendTime;
    Time receivedTime;
    std::map<int, Time> receptionTimes;
};

struct RetransmissionStatus
{
    Time firstAttempt;
    Time finishTime;
    uint8_t sf;
    uint8_t reTxAttempts;
    bool successful;
};

typedef std::map<Ptr<const Packet>, MacPacketStatus> MacPacketData;
typedef std::map<Ptr<const Packet>, PacketStatus> PhyPacketData;
typedef std::map<Ptr<const Packet>, RetransmissionStatus> RetransmissionData;

class LoraPacketTracker
{
  public:
    LoraPacketTracker();
    ~LoraPacketTracker();

    void TransmissionCallback(Ptr<const Packet> packet, uint32_t systemId);
    void PacketReceptionCallback(Ptr<const Packet> packet, uint32_t systemId);
    void InterferenceCallback(Ptr<const Packet> packet, uint32_t systemId);
    void NoMoreReceiversCallback(Ptr<const Packet> packet, uint32_t systemId);
    void UnderSensitivityCallback(Ptr<const Packet> packet, uint32_t systemId);
    void LostBecauseTxCallback(Ptr<const Packet> packet, uint32_t systemId);

    void MacTransmissionCallback(Ptr<const Packet> packet, uint8_t sf);
    void RequiredTransmissionsCallback(uint8_t reqTx,
                                       bool success,
                                       Time firstAttempt,
                                       Ptr<Packet> packet,
                                       uint8_t sf);
    void MacGwReceptionCallback(Ptr<const Packet> packet);

    bool IsUplink(Ptr<const Packet> packet);

    std::vector<int> CountPhyPacketsPerGw(Time startTime, Time stopTime, int systemId);
    std::string PrintPhyPacketsPerGw(Time startTime, Time stopTime, int systemId);
    std::string CountMacPacketsPerGw(Time startTime, Time stopTime, int systemId);
    std::string PrintMacPacketsPerGw(Time startTime, Time stopTime, int systemId);
    std::string CountRetransmissions(Time startTime, Time stopTime);
    std::string CountMacPacketsGlobally(Time startTime, Time stopTime);

    std::string CountMacPacketsGlobally(Time startTime, Time stopTime, uint8_t sf);
    std::string CountMacPacketsGlobally(Time startTime,
                                        Time stopTime,
                                        std::map<LoraDeviceAddress, deviceFCtn> mapDevices);

    std::string CountMacPacketsGlobally(Time startTime,
                                        Time stopTime,
                                        uint8_t sf,
                                        std::map<LoraDeviceAddress, deviceFCtn> mapDevices);

    std::string AvgPacketTimeOnAir(Time startTime,
                                   Time stopTime,
                                   uint32_t gwId,
                                   uint32_t gwNum,
                                   uint8_t sf);

    std::string AvgPacketTimeOnAir(Time startTime,
                                   Time stopTime,
                                   uint8_t sf,
                                   std::map<LoraDeviceAddress, deviceFCtn> mapDevices);

    std::string AvgPacketTimeOnAir(Time startTime,
                                   Time stopTime,
                                   uint32_t gwId,
                                   uint32_t gwNum,
                                   uint8_t sf,
                                   std::map<LoraDeviceAddress, deviceFCtn> mapDevices);

    std::string CountMacPacketsGloballyCpsr(Time startTime, Time stopTime);

    std::string CountMacPacketsGloballyCpsr(Time startTime, Time stopTime, uint8_t sf);
    std::string CountMacPacketsGloballyCpsr(Time startTime,
                                            Time stopTime,
                                            uint8_t sf,
                                            std::map<LoraDeviceAddress, deviceFCtn> mapDevices);

    std::string CountMacPacketsGloballyDelay(Time startTime,
                                             Time stopTime,
                                             uint32_t gwId,
                                             uint32_t gwNum);

    std::string CountMacPacketsGloballyDelay(Time startTime,
                                             Time stopTime,
                                             uint32_t gwId,
                                             uint32_t gwNum,
                                             uint8_t sf);

    std::string CountMacPacketsGloballyDelay(Time startTime,
                                             Time stopTime,
                                             uint32_t gwId,
                                             uint32_t gwNum,
                                             std::map<LoraDeviceAddress, deviceFCtn> mapDevices);

    std::string CountMacPacketsGloballyDelay(Time startTime,
                                             Time stopTime,
                                             uint32_t gwId,
                                             uint32_t gwNum,
                                             uint8_t sf,
                                             std::map<LoraDeviceAddress, deviceFCtn> mapDevices);

  private:
    PhyPacketData m_packetTracker;
    MacPacketData m_macPacketTracker;
    RetransmissionData m_reTransmissionTracker;
};

} // namespace lorawan
} // namespace ns3
#endif
