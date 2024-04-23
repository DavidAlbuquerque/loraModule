/*
 * =====================================================================================
 *
 *       Filename:  lorawan-network-mClass-sim.cc
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  02/06/2020 22:42:04
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Francisco Helder (FHC), helderhdw@gmail.com
 *   Organization:  Federal University of Ceara
 *
 * =====================================================================================
 */

#include "ns3/building-allocator.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/buildings-helper.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/double.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/forwarder-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/log.h"
#include "ns3/lora-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/node-container.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/pointer.h"
#include "ns3/position-allocator.h"
#include "ns3/random-sender-helper.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>

using namespace ns3;
using namespace lorawan;
using namespace std;

NS_LOG_COMPONENT_DEFINE("LorawanNetworkSimulatorMClass");

/*Configuração da execução*/
#define RTX_ONLY_SF7 1
#define RTX_ONLY_DEVICE_TYPE 1
#define RTX_REGULAR 1
/*************************/

#define MAXRTX 4

#define CONFIGURE_LORAWAN_DEVICE_MAC(device)                                                       \
    auto mac = device->GetMac() -> GetObject<EndDeviceLorawanMac>();                               \
    mac->SetMaxNumberOfTransmissions(MAXRTX);                                                      \
    mac->SetMType(LorawanMacHeader::CONFIRMED_DATA_UP);

// File settings
// string fileData = "./scratch/result-STAs.dat";
// string fileMetric = "./scratch/mac-STAs-GW-1.txt";
string endDevRegularFile = "./scratch/TestResult/test";
string endDevAlarmeFile = "./scratch/TestResult/test";
string gwFile = "./scratch/TestResult/test";

bool connectTraceSoucer = false;
bool flagRtx = false; //, sizeStatus=0;
uint32_t nSeed = 1;
uint8_t trial = 1, numClass = 0; //, nCount=0, nClass1=0, nClass2=0, nClass3=0;
vector<uint16_t> sfQuantAlarm(6, 0);
vector<uint16_t> sfQuantRegular(6, 0);
vector<uint16_t> sfQuantAll(6, 0);
vector<double> rtxQuant(4, 0);
std::map<LoraDeviceAddress, deviceFCtn> mapEndAlarm, mapEndRegular;
uint16_t nDevicesAlarm;
uint16_t nDevicesRegular;

// Metrics
double packLoss = 0, sent = 0, received = 0, avgDelay = 0;
double angle = 0, sAngle = M_PI; //, radius1=4200; //, radius2=4900;
double throughput = 0, probSucc = 0, probLoss = 0;

// Network settings
uint16_t nDevicesTotally = 200;
uint16_t nGateways = 1;
double radius = 5600;
double gatewayRadius = 0;
uint16_t simulationTime = 600;
uint8_t pDevicesAlarm = 20;

// Channel model
bool realisticChannelModel = false;

uint16_t appPeriodSeconds = 600;

// Output control
bool printBuildings = false;
bool print = true;

enum SF
{
    SF7 = 7,
    SF8,
    SF9,
    SF10,
    SF11,
    SF12
};

enum deviceType
{
    ALARM_DEVICE = 0,
    REGULAR_DEVICE,
    ALL
};

void
writeRTXMetrics(ofstream& myfile,
                const string& fileMetric,
                uint8_t sf,
                uint16_t nDevices,
                double sent,
                const vector<double>& rtxQuant)
{
    vector<double> sortedRtxQuant = rtxQuant; // Cria uma cópia local do vetor
    sort(sortedRtxQuant.begin(), sortedRtxQuant.end(), greater<double>()); // Ordena a cópia
    myfile.open(fileMetric + "-RTX" + to_string(sf) + ".dat", ios::out | ios::app);
    myfile << nDevices << ", " << sent;
    for (uint8_t i = 0; i < sortedRtxQuant.size(); i++)
    {
        myfile << ", " << sortedRtxQuant[i];
    }
    myfile << "\n";
    myfile.close();
}

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  metricsResultFile
 *  Description:
 * =====================================================================================
 */
void
metricsResultFile(LoraPacketTracker& tracker,
                  enum deviceType device,
                  string fileData,
                  string fileMetric)
{
    Time appStopTime = Seconds(simulationTime);
    string deviceMetric;
    ofstream myfile;
    std::map<LoraDeviceAddress, deviceFCtn> mapDevices;
    vector<uint16_t> sfQuant;
    uint16_t nDevices;

    switch (device)
    {
    case ALARM_DEVICE:
        deviceMetric = "Alarm devices";
        sfQuant = sfQuantAlarm;
        nDevices = nDevicesAlarm;
        mapDevices = mapEndAlarm;
        fileMetric += "metricAlarm/result-STAs-alarm";
        break;

    case REGULAR_DEVICE:
        deviceMetric = "Regular devices";
        sfQuant = sfQuantRegular;
        nDevices = nDevicesRegular;
        mapDevices = mapEndRegular;
        fileMetric += "metricRegular/result-STAs-regular";
        break;

    case ALL:
        nDevices = nDevicesTotally;
        sfQuant = sfQuantAll;
        deviceMetric = "all devices";
        fileMetric += "metricAll/result-STAs-all";
        break;

    default:
        break;
    }

    NS_LOG_INFO(endl << "//////////////////////////////////////////////");
    NS_LOG_INFO("//  METRICS FOR " << deviceMetric << " DEVICES  //");
    NS_LOG_INFO("//////////////////////////////////////////////" << endl);
    NS_LOG_INFO("SF Allocation " << deviceMetric << " Devices: 6 -> "
                                 << "SF7=" << (unsigned)sfQuant.at(0) << " SF8="
                                 << (unsigned)sfQuant.at(1) << " SF9=" << (unsigned)sfQuant.at(2)
                                 << " SF10=" << (unsigned)sfQuant.at(3) << " SF11="
                                 << (unsigned)sfQuant.at(4) << " SF12=" << (unsigned)sfQuant.at(5));
    for (uint8_t i = SF7; i <= SF12; i++)
    {
        if (sfQuant.at(i - SF7))
        {
            NS_LOG_INFO("\n##################################################################");
            NS_LOG_INFO(endl << "//////////////////////////////////////////////");
            NS_LOG_INFO("//  Computing SF-" << (unsigned)i << " performance metrics  //");
            NS_LOG_INFO("//////////////////////////////////////////////" << endl);

            if (device == ALL)
            {
                stringstream(
                    tracker.CountMacPacketsGlobally(Seconds(0), appStopTime + Hours(2), i)) >>
                    sent >> received;
                stringstream(tracker.CountMacPacketsGloballyDelay(Seconds(0),
                                                                  appStopTime + Hours(1),
                                                                  (unsigned)nDevicesTotally,
                                                                  (unsigned)nGateways,
                                                                  i)) >>
                    avgDelay;
            }
            else
            {
                stringstream(tracker.CountMacPacketsGlobally(Seconds(0),
                                                             appStopTime + Hours(2),
                                                             i,
                                                             mapDevices)) >>
                    sent >> received;
                stringstream(tracker.CountMacPacketsGloballyDelay(Seconds(0),
                                                                  appStopTime + Hours(1),
                                                                  (unsigned)nDevicesTotally,
                                                                  (unsigned)nGateways,
                                                                  i,
                                                                  mapDevices)) >>
                    avgDelay;
            }

            NS_LOG_DEBUG("passou daqui");

            packLoss = sent - received;
            throughput = received / simulationTime;

            probSucc = received / sent;
            probLoss = packLoss / sent;

            NS_LOG_INFO("----------------------------------------------------------------");
            NS_LOG_INFO("nDevices"
                        << "  |  "
                        << "throughput"
                        << "  |  "
                        << "probSucc"
                        << "  |  "
                        << "probLoss"
                        << "  |  "
                        << "avgDelay");
            NS_LOG_INFO(nDevices << "       |  " << throughput << "    |  " << probSucc << "   |  "
                                 << probLoss << "   |  " << avgDelay);
            NS_LOG_INFO("----------------------------------------------------------------" << endl);

            myfile.open(fileMetric + "-SF" + to_string(i) + ".dat", ios::out | ios::app);
            myfile << nDevices << ", " << throughput << ", " << probSucc << ", " << probLoss << ", "
                   << avgDelay << "\n";
            myfile.close();

            if (flagRtx && (RTX_ONLY_SF7 || RTX_ONLY_DEVICE_TYPE))
            {
                stringstream(tracker.CountMacPacketsGloballyCpsr(Seconds(0),
                                                                 appStopTime + Hours(2),
                                                                 i,
                                                                 mapDevices)) >>
                    rtxQuant.at(0) >> rtxQuant.at(1) >> rtxQuant.at(2) >> rtxQuant.at(3);
                writeRTXMetrics(myfile, fileMetric, i, nDevices, sent, rtxQuant);
            }

            else if (flagRtx && !(RTX_ONLY_SF7 || RTX_ONLY_DEVICE_TYPE))
            {
                stringstream(
                    tracker.CountMacPacketsGloballyCpsr(Seconds(0), appStopTime + Hours(2), i)) >>
                    rtxQuant.at(0) >> rtxQuant.at(1) >> rtxQuant.at(2) >> rtxQuant.at(3);
                writeRTXMetrics(myfile, fileMetric, i, nDevices, sent, rtxQuant);
            }

            NS_LOG_INFO("numDev:" << nDevices << " numGW:" << nGateways
                                  << " simTime:" << simulationTime << " throughput:" << throughput);
            NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
            NS_LOG_INFO("sent:" << sent << "    succ:" << received << "     drop:" << packLoss
                                << "   delay:" << avgDelay);
            NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl);
            NS_LOG_INFO("##################################################################\n");

            myfile.open(fileData, ios::out | ios::app);
            myfile << "sent: " << sent << " succ: " << received << " drop: " << packLoss << "\n";
            myfile << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
                   << "\n";
            myfile << "numDev: " << nDevices << " numGat: " << nGateways
                   << " simTime: " << simulationTime << " throughput: " << throughput << "\n";
            myfile << "##################################################################"
                   << "\n\n";
            myfile.close();
        }
        else
        {
            NS_LOG_INFO("\n##################################################################");
            NS_LOG_INFO(endl << "//////////////////////////////////////////////");
            NS_LOG_INFO("//  No devices for SF-" << (unsigned)i << "  //");
            NS_LOG_INFO("//////////////////////////////////////////////" << endl);
            NS_LOG_INFO("##################################################################\n");
        }
    }

    NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    NS_LOG_INFO("Computing system performance metrics");

    if (device == ALL)
    {
        stringstream(tracker.CountMacPacketsGlobally(Seconds(0), appStopTime + Hours(1))) >> sent >>
            received;
        stringstream(tracker.CountMacPacketsGloballyDelay(Seconds(0),
                                                          appStopTime + Hours(1),
                                                          (unsigned)nDevicesTotally,
                                                          (unsigned)nGateways)) >>
            avgDelay;
    }
    else
    {
        stringstream(
            tracker.CountMacPacketsGlobally(Seconds(0), appStopTime + Hours(1), mapDevices)) >>
            sent >> received;
        stringstream(tracker.CountMacPacketsGloballyDelay(Seconds(0),
                                                          appStopTime + Hours(1),
                                                          (unsigned)nDevicesTotally,
                                                          (unsigned)nGateways,
                                                          mapDevices)) >>
            avgDelay;
    }

    packLoss = sent - received;
    throughput = received / simulationTime;

    probSucc = received / sent;
    probLoss = packLoss / sent;

    NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    NS_LOG_INFO("nDevices: " << nDevices);
    NS_LOG_INFO("throughput: " << throughput);
    NS_LOG_INFO("probSucc: " << probSucc << " (" << probSucc * 100 << "%)");
    NS_LOG_INFO("probLoss: " << probLoss << " (" << probLoss * 100 << "%)");
    NS_LOG_INFO("avgDelay: " << avgDelay);
    NS_LOG_INFO("----------------------------------" << endl);

    myfile.open(fileMetric + "-RTX" + ".dat", ios::out | ios::app);
    myfile << nDevices << ", " << sent;
    myfile << "\n";
    myfile.close();

    myfile.open(fileData + ".dat", ios::out | ios::app);
    myfile << nDevices << ", " << throughput << ", " << probSucc << ", " << probLoss << ", "
           << avgDelay << "\n";
    myfile.close();

    NS_LOG_INFO("numDev:" << nDevices << " numGW:" << unsigned(nGateways)
                          << " simTime:" << simulationTime << " throughput:" << throughput);
    NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    NS_LOG_INFO("sent:" << sent << "    succ:" << received << "     drop:" << packLoss
                        << "   delay:" << avgDelay);
    NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl);
}

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  OnReceivePacket
 *  Description:  Handles the reception of a packet, extracting and displaying relevant
 *                information such as the node ID, frame counter, and device address.
 *                Updates the frame counter for the corresponding device if found in
 *                the mapEndAlarm.
 * =====================================================================================
 */
void
OnReceivePacket(Ptr<const Packet> packet, uint32_t num)
{
    /* std::cout << "Receive Packet: " << packet << "\n"; */
    Ptr<Packet> packetCopy = packet->Copy();
    LorawanMacHeader mHdr;
    LoraFrameHeader fHdr;
    packetCopy->RemoveHeader(mHdr);
    packetCopy->RemoveHeader(fHdr);
    if (mHdr.IsUplink())
    {
        fHdr.SetAsUplink();
        LoraDeviceAddress address = fHdr.GetAddress();
        if (mapEndAlarm.find(address) != mapEndAlarm.end())
        {
            mapEndAlarm[address].FCtn = fHdr.GetFCnt();
            // NS_LOG_DEBUG("Frame Counter " << fHdr.GetFCnt() << " Node Addr: " << address <<
            // "\n");
            /* std::cout << "Node ID: " << mapEndAlarm[address].id
                      << ", Frame Counter: " << fHdr.GetFCnt() << ", Device Addr: " << address
                      << "\n"; */
        }
    }
}

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  OnReceivePacketMac
 *  Description:  Handles the reception of a MAC packet, extracting and displaying
 *                relevant information such as the node ID, frame counter, and
 *                device address. Updates the frame counter for the corresponding
 *                device if found in the mapEndAlarm.
 * =====================================================================================
 */
void
OnReceivePacketMac(Ptr<const Packet> packet)
{
    /* std::cout << "Receive Packet: " << packet << "\n"; */
    Ptr<Packet> packetCopy = packet->Copy();
    LorawanMacHeader mHdr;
    LoraFrameHeader fHdr;
    packetCopy->RemoveHeader(mHdr);
    packetCopy->RemoveHeader(fHdr);
    if (mHdr.IsUplink())
    {
        fHdr.SetAsUplink();
        LoraDeviceAddress address = fHdr.GetAddress();
        if (mapEndAlarm.find(address) != mapEndAlarm.end())
        {
            mapEndAlarm[address].FCtn = fHdr.GetFCnt();
            // NS_LOG_DEBUG("Frame Counter " << fHdr.GetFCnt() << " Node Addr: " << address <<
            // "\n");
            /* std::cout << "Node ID: " << mapEndAlarm[address].id
                      << ", Frame Counter: " << fHdr.GetFCnt() << ", Device Addr: " << address
                      << "\n"; */
        }
    }
}

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  OnStartingSendPacket
 *  Description:  Handles the start of sending a packet, extracting and displaying
 *                relevant information such as the node ID, frame counter, and
 *                device address. Updates the frame counter for the corresponding
 *                device if found in the mapEndAlarm.
 * =====================================================================================
 */
void
OnStartingSendPacket(Ptr<const Packet> packet, uint32_t num)
{
    Ptr<Packet> packetCopy = packet->Copy();
    LorawanMacHeader mHdr;
    LoraFrameHeader fHdr;
    packetCopy->RemoveHeader(mHdr);
    packetCopy->RemoveHeader(fHdr);
    if (mHdr.IsUplink())
    {
        fHdr.SetAsUplink();
        LoraDeviceAddress address = fHdr.GetAddress();
        if (mapEndAlarm.find(address) != mapEndAlarm.end())
        {
            mapEndAlarm[address].FCtn = fHdr.GetFCnt();
            // NS_LOG_DEBUG("Frame Counter " << fHdr.GetFCnt() << " Node Addr: " << address <<
            // "\n");
            /* std::cout << "Node ID: " << mapEndAlarm[address].id
                      << ", Frame Counter: " << fHdr.GetFCnt() << ", Device Addr: " << address
                      << "\n"; */
        }
    }
}

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  printEndDevices
 *  Description:
 * =====================================================================================
 */
void
PrintEndDevices(NodeContainer endDevices,
                NodeContainer gateways,
                std::string filename1,
                std::string filename2)
{
    const char* c = filename1.c_str();
    vector<int> countSF(6, 0);
    std::ofstream spreadingFactorFile;
    spreadingFactorFile.open(c);
    for (NodeContainer::Iterator j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position != NULL);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();
        NS_ASSERT(loraNetDevice != NULL);
        Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
        int sf = int(mac->GetSfFromDataRate(mac->GetDataRate()));
        countSF[sf - 7]++;
        Vector pos = position->GetPosition();
        spreadingFactorFile << pos.x << " " << pos.y << " " << sf << endl;
    }
    spreadingFactorFile.close();

    c = filename2.c_str();
    spreadingFactorFile.open(c);
    // Also print the gateways
    for (NodeContainer::Iterator j = gateways.Begin(); j != gateways.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        Vector pos = position->GetPosition();
        spreadingFactorFile << pos.x << " " << pos.y << " GW" << endl;
    }
    spreadingFactorFile.close();
}

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  printEndDevices with regular and alarm devices
 *  Description:
 * =====================================================================================
 */
void
PrintEndDevices(NodeContainer endDevicesRegular,
                NodeContainer endDevicesAlarm,
                NodeContainer gateways,
                std::string filename1,
                std::string filename2,
                std::string filename3)
{
    const char* c = filename1.c_str();
    vector<int> countSF(6, 0);
    std::ofstream regularFile;
    regularFile.open(c);

    // Print regular end devices
    for (NodeContainer::Iterator j = endDevicesRegular.Begin(); j != endDevicesRegular.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();
        Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
        int sf = int(mac->GetSfFromDataRate(mac->GetDataRate()));
        countSF[sf - 7]++;
        Vector pos = position->GetPosition();
        regularFile << pos.x << " " << pos.y << " " << sf << " R" << endl;
    }
    regularFile.close();

    c = filename2.c_str();
    std::ofstream alarmFile;
    alarmFile.open(c);

    // Print alarm end devices
    for (NodeContainer::Iterator j = endDevicesAlarm.Begin(); j != endDevicesAlarm.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();
        Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
        int sf = int(mac->GetSfFromDataRate(mac->GetDataRate()));
        countSF[sf - 7]++;
        Vector pos = position->GetPosition();
        alarmFile << pos.x << " " << pos.y << " " << sf << " A" << endl;
    }

    alarmFile.close();

    c = filename3.c_str();
    std::ofstream gatewayFile;
    gatewayFile.open(c);

    // Also print the gateways
    for (NodeContainer::Iterator j = gateways.Begin(); j != gateways.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        Vector pos = position->GetPosition();
        gatewayFile << pos.x << " " << pos.y << " GW" << endl;
    }

    gatewayFile.close();
}

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  buildingHandler
 *  Description:
 * =====================================================================================
 */
void
buildingHandler(NodeContainer endDevices, NodeContainer gateways)
{
    double xLength = 230;
    double deltaX = 80;
    double yLength = 164;
    double deltaY = 57;
    int gridWidth = 2 * radius / (xLength + deltaX);
    int gridHeight = 2 * radius / (yLength + deltaY);

    if (realisticChannelModel == false)
    {
        gridWidth = 0;
        gridHeight = 0;
    }

    Ptr<GridBuildingAllocator> gridBuildingAllocator;
    gridBuildingAllocator = CreateObject<GridBuildingAllocator>();
    gridBuildingAllocator->SetAttribute("GridWidth", UintegerValue(gridWidth));
    gridBuildingAllocator->SetAttribute("LengthX", DoubleValue(xLength));
    gridBuildingAllocator->SetAttribute("LengthY", DoubleValue(yLength));
    gridBuildingAllocator->SetAttribute("DeltaX", DoubleValue(deltaX));
    gridBuildingAllocator->SetAttribute("DeltaY", DoubleValue(deltaY));
    gridBuildingAllocator->SetAttribute("Height", DoubleValue(6));
    gridBuildingAllocator->SetBuildingAttribute("NRoomsX", UintegerValue(2));
    gridBuildingAllocator->SetBuildingAttribute("NRoomsY", UintegerValue(4));
    gridBuildingAllocator->SetBuildingAttribute("NFloors", UintegerValue(2));
    gridBuildingAllocator->SetAttribute(
        "MinX",
        DoubleValue(-gridWidth * (xLength + deltaX) / 2 + deltaX / 2));
    gridBuildingAllocator->SetAttribute(
        "MinY",
        DoubleValue(-gridHeight * (yLength + deltaY) / 2 + deltaY / 2));
    BuildingContainer bContainer = gridBuildingAllocator->Create(gridWidth * gridHeight);

    BuildingsHelper::Install(endDevices);
    BuildingsHelper::Install(gateways);
    // BuildingsHelper::MakeMobilityModelConsistent ();

    // Print the buildings
    if (printBuildings)
    {
        std::ofstream myfile;
        myfile.open("buildings.txt");
        std::vector<Ptr<Building>>::const_iterator it;
        int j = 1;
        for (it = bContainer.Begin(); it != bContainer.End(); ++it, ++j)
        {
            Box boundaries = (*it)->GetBoundaries();
            myfile << "set object " << j << " rect from " << boundaries.xMin << ","
                   << boundaries.yMin << " to " << boundaries.xMax << "," << boundaries.yMax
                   << std::endl;
        }
        myfile.close();
    }

} /* -----  end of function buildingHandler  ----- */

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  getPacketSizeFromSF
 *  Description:
 * =====================================================================================
 */
uint8_t
getPacketSizeFromSF(NodeContainer endDevices, int j, bool pDiff)
{
    uint8_t size = 90, sf = 0;

    Ptr<Node> object = endDevices.Get(j);
    Ptr<NetDevice> netDevice = object->GetDevice(0);
    Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();
    NS_ASSERT(loraNetDevice != NULL);
    Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
    sf = mac->GetSfFromDataRate(mac->GetDataRate());

    if (pDiff)
    {
        switch (sf)
        {
        case SF7:
            size = 90;
            break;
        case SF8:
            size = 35;
            break;
        case SF9:
            size = 5;
            break;
        default:
            break;
        } /* -----  end switch  ----- */
    }

    return (size);
} /* -----  end of function getRateSF  ----- */

/*
 * ===  FUNCTION  ======================================================================
 *         Name:  getShiftPosition
 *  Description:
 * =====================================================================================
 */
Vector
getShiftPosition(NodeContainer endDevices, int j, int base)
{
    double radius = 0, co = 0, si = 0;
    Ptr<Node> object = endDevices.Get(j);
    Ptr<MobilityModel> mobility = object->GetObject<MobilityModel>();
    NS_ASSERT(mobility != NULL);
    Vector position = mobility->GetPosition();

    cout << "x: " << position.x << " y: " << position.y << endl;
    cout << "mod: " << (int)position.x / base << " mod: " << (int)position.y / base << endl;

    radius = sqrt(pow(position.x, 2) + pow(position.y, 2));
    co = position.x / radius;
    si = position.y / radius;

    radius += base - (int)radius / 700 * 700;
    position.x = radius * co;
    position.y = radius * si;

    cout << "x: " << position.x << " y: " << position.y << endl;
    cout << sqrt(pow(position.x, 2) + pow(position.y, 2)) << endl;
    cout << endl;

    return (position);
} /* -----  end of function getRateSF  ----- */

int
main(int argc, char* argv[])
{
    string fileMetric = "./scratch/result-STAs.dat";
    string fileData = "./scratch/mac-STAs-GW-1.txt";
    CommandLine cmd;
    cmd.AddValue("nSeed", "Number of seed to position", nSeed);
    cmd.AddValue("nDevicesTotally",
                 "Number of end devices to include in the simulation",
                 nDevicesTotally);
    cmd.AddValue("pDevicesAlarm", "Percentage of alarm devices", pDevicesAlarm);
    cmd.AddValue("nGateways", "Number of gateway rings to include", nGateways);
    cmd.AddValue("radius", "The radius of the area to simulate", radius);
    cmd.AddValue("gatewayRadius", "The distance between gateways", gatewayRadius);
    cmd.AddValue("simulationTime", "The time for which to simulate", simulationTime);
    cmd.AddValue("appPeriod",
                 "The period in seconds to be used by periodically transmitting applications",
                 appPeriodSeconds);
    cmd.AddValue("fileMetric", "files containing result data", fileMetric);
    cmd.AddValue("fileData", "files containing result information", fileData);
    cmd.AddValue("flagRtx", "whether or not to activate retransmission", flagRtx);
    cmd.AddValue("print", "Whether or not to print various informations", print);
    cmd.AddValue("trial", "set trial parameter", trial);
    cmd.Parse(argc, argv);

    endDevRegularFile +=
        to_string(trial) + "/endDevicesRegular" + to_string(nDevicesTotally) + ".dat";
    endDevAlarmeFile += to_string(trial) + "/endDevicesAlarm" + to_string(nDevicesTotally) + ".dat";
    gwFile += to_string(trial) + "/GWs" + to_string(nGateways) + ".dat";

    // Set up logging
    LogComponentEnable("LorawanNetworkSimulatorMClass", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraPacketTracker", LOG_LEVEL_ALL);
    //  LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
    //  LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
    //  LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
    //  LogComponentEnable("SimpleEndDeviceLoraPhy", LOG_LEVEL_ALL);
    //  LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
    //  LogComponentEnable("SimpleGatewayLoraPhy", LOG_LEVEL_ALL);
    //  LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
    //  LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
    //  LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
    //  LogComponentEnable("EndDeviceStatus", LOG_LEVEL_ALL);
    //  LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
    //  LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
    //  LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
    //  LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
    //  LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
    //  LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
    //  LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
    //  LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
    //  LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
    //  LogComponentEnable("RandomSenderHelper", LOG_LEVEL_ALL);
    //  LogComponentEnable("MobilityHelper", LOG_LEVEL_ALL);
    //  LogComponentEnable("RandomSender", LOG_LEVEL_ALL);
    //  LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
    //  LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
    //  LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
    //  LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
    //  LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
    //  LogComponentEnable("NetworkController", LOG_LEVEL_ALL);

    /***********
     *  Setup  *
     ***********/
    ofstream myfile;

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(nSeed);

    // Create the time value from the period
    Time appPeriod = Seconds(appPeriodSeconds);

    // Mobility
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                  "rho",
                                  DoubleValue(radius),
                                  "X",
                                  DoubleValue(0.0),
                                  "Y",
                                  DoubleValue(0.0));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    /************************
     *  Create the channel  *
     ************************/

    // Create the lora channel object
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 7.7);

    if (realisticChannelModel)
    {
        // Create the correlated shadowing component
        Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
            CreateObject<CorrelatedShadowingPropagationLossModel>();

        // Aggregate shadowing to the logdistance loss
        loss->SetNext(shadowing);

        // Add the effect to the channel propagation loss
        Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss>();

        shadowing->SetNext(buildingLoss);
    }

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();

    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

    /************************
     *  Create the helpers  *
     ************************/

    // Create the LoraPhyHelper
    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);

    // Create the LorawanMacHelper
    LorawanMacHelper macHelper = LorawanMacHelper();

    // Create the LoraHelper
    LoraHelper helper = LoraHelper();
    helper.EnablePacketTracking(); // Output filename
    // helper.EnableSimulationTimePrinting ();

    // Create the NetworkServerHelper
    NetworkServerHelper nsHelper = NetworkServerHelper();

    // Create the ForwarderHelper
    ForwarderHelper forHelper = ForwarderHelper();

    // Create the PeriodicSenderHelper
    PeriodicSenderHelper periodicHelper = PeriodicSenderHelper();
    // Create the RandomSenderHelper
    RandomSenderHelper ramdomHelper = RandomSenderHelper();

    /************************
     *  Create End Devices  *
     ************************/

    // Create a set of nodes
    NodeContainer endDevicesRegular;
    NodeContainer endDevicesAlarm;

    nDevicesAlarm = ceil(nDevicesTotally * (pDevicesAlarm / 100.0));
    nDevicesRegular = nDevicesTotally - nDevicesAlarm;

    endDevicesRegular.Create(nDevicesRegular);
    endDevicesAlarm.Create(nDevicesAlarm);

    // Assign a mobility model to each node
    mobility.Install(endDevicesRegular);
    mobility.Install(endDevicesAlarm);

    // Installing sender type
    ramdomHelper.Install(endDevicesAlarm);
    periodicHelper.Install(endDevicesRegular);

    // Combine end devices into a single NodeContainer
    NodeContainer combinedEndDevices;
    std::map<ns3::Ptr<ns3::Node>, deviceType> deviceTypeMap;

    for (NodeContainer::Iterator it = endDevicesRegular.Begin(); it != endDevicesRegular.End();
         ++it)
    {
        deviceTypeMap[*it] = REGULAR_DEVICE;
        combinedEndDevices.Add(*it);
    }

    for (NodeContainer::Iterator it = endDevicesAlarm.Begin(); it != endDevicesAlarm.End(); ++it)
    {
        deviceTypeMap[*it] = ALARM_DEVICE;
        combinedEndDevices.Add(*it);
    }

    // Make it so that nodes are at a certain height > 0
    for (NodeContainer::Iterator j = combinedEndDevices.Begin(); j != combinedEndDevices.End(); ++j)
    {
        Ptr<Node> node = *j;
        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
        Vector position = mobility->GetPosition();
        position.z = 1.2;
        mobility->SetPosition(position);
    }

    // Create the LoraNetDevices of the end devices
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen =
        CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);

    // Create the LoraNetDevices of the end devices
    macHelper.SetAddressGenerator(addrGen);
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);

    // macHelper.SetRegion (LorawanMacHelper::SingleChannel);
    helper.Install(phyHelper, macHelper, endDevicesAlarm);
    helper.Install(phyHelper, macHelper, endDevicesRegular);

    // Now end devices are connected to the channel

    for (NodeContainer::Iterator j = endDevicesRegular.Begin(); j != endDevicesRegular.End(); ++j)
    {
        Ptr<Node> node = *j;
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
        mapEndRegular[mac->GetDeviceAddress()] = deviceFCtn{(uint16_t)node->GetId(), 0};
    }

    for (NodeContainer::Iterator j = endDevicesAlarm.Begin(); j != endDevicesAlarm.End(); ++j)
    {
        Ptr<Node> node = *j;
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
        mapEndAlarm[mac->GetDeviceAddress()] = deviceFCtn{(uint16_t)node->GetId(), 0};
    }

    if (connectTraceSoucer)
    {
        if (Config::ConnectWithoutContextFailSafe(
                "/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Phy/$ns3::LoraPhy/ReceivedPacket",
                MakeCallback(&OnReceivePacket)))
            NS_LOG_DEBUG("Conect Trace Source LoraPhy/ReceivedPacket!\n");
        else
            NS_LOG_DEBUG("Not Conect Trace Source LoraPhy/ReceivedPacket!\n");
        if (Config::ConnectWithoutContextFailSafe(
                "/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Phy/$ns3::LoraPhy/StartSending",
                MakeCallback(&OnStartingSendPacket)))
            NS_LOG_DEBUG("Conect Trace Source LoraPhy/StartSending!\n");
        else
            NS_LOG_DEBUG("Not Conect Trace Source LoraPhy/StartSending!\n");
        if (Config::ConnectWithoutContextFailSafe(
                "/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::LorawanMac/ReceivedPacket",
                MakeCallback(&OnReceivePacketMac)))
            NS_LOG_DEBUG("Conect Trace Source LorawanMac/ReceivedPacket!\n");
        else
            NS_LOG_DEBUG("Not Conect Trace Source LorawanMac/ReceivedPacket!\n");
    }

    /*********************
     *  Create Gateways  *
     *********************/

    // Create the gateway nodes (allocate them uniformely on the disc)
    NodeContainer gateways;
    gateways.Create(nGateways);

    sAngle = (2 * M_PI) / nGateways;

    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
    // Make it so that nodes are at a certain height > 0
    allocator->Add(Vector(0.0, 0.0, 0.0));
    mobility.SetPositionAllocator(allocator);
    mobility.Install(gateways);

    // Make it so that nodes are at a certain height > 0
    for (NodeContainer::Iterator j = gateways.Begin(); j != gateways.End(); ++j)
    {
        Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel>();
        Vector position = mobility->GetPosition();
        position.x = gatewayRadius * cos(angle);
        position.y = gatewayRadius * sin(angle);
        position.z = 15;
        mobility->SetPosition(position);
        angle += sAngle;
    }

    // Create a netdevice for each gateway
    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    /**********************
     *  Handle buildings  *
     **********************/
    buildingHandler(combinedEndDevices, gateways);

    /**********************************************
     *  Set up the end device's spreading factor  *
     **********************************************/

    sfQuantRegular = macHelper.SetSpreadingFactorsUp(endDevicesRegular, gateways, channel, flagRtx);
    sfQuantAlarm = macHelper.SetSpreadingFactorsUp(endDevicesAlarm, gateways, channel, flagRtx);
    // sfQuant = macHelper.SetSpreadingFactorsEIB (endDevices, radius);
    // sfQuant = macHelper.SetSpreadingFactorsEAB (endDevices, radius);
    // sfQuant = macHelper.SetSpreadingFactorsProp (endDevices, 0.4, 0, radius);
    // sfQuant = macHelper.SetSpreadingFactorsStrategies (endDevices, sfQuant, 0.76*nDevicesTotally,
    // 0*nDevicesTotally, nDevicesTotally, LorawanMacHelper::CLASS_TWO);

    for (uint16_t i = 0; i < sfQuantRegular.size(); i++)
        sfQuantRegular.at(i) ? numClass++ : numClass;

    for (uint16_t i = 0; i < sfQuantAlarm.size(); i++)
        sfQuantAlarm.at(i) ? numClass++ : numClass;

    for (uint16_t i = 0; i < sfQuantAll.size(); i++)
        sfQuantAll.at(i) = sfQuantAlarm.at(i) + sfQuantRegular.at(i);

    /**********************************************
     *  enabling relaying on end devices  *
     **********************************************/
    /* if (flagRtx)
    {
        for (NodeContainer::Iterator j = combinedEndDevices.Begin(); j != combinedEndDevices.End();
             ++j)
        {
            Ptr<Node> node = *j;
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<LoraPhy> phy = loraNetDevice->GetPhy();
            Ptr<EndDeviceLoraPhy> endDeviceLoraPhy = phy->GetObject<EndDeviceLoraPhy>();

            if ((static_cast<int>(endDeviceLoraPhy->GetSpreadingFactor()) == 7) && RTX_ONLY_SF7)
            {
                int mode = RTX_ONLY_SF7_device;
                switch (mode)
                {
                    bool isEndDeviceRegular = (j < endDevicesRegular.End());
                case 0:


                    if ((static_cast<int>(endDeviceLoraPhy->GetSpreadingFactor()) == 7) &&
                        RTX_ONLY_SF7 && isEndDeviceRegular)
                    {
                        Ptr<EndDeviceLorawanMac> mac =
                            loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
                        mac->SetMaxNumberOfTransmissions(MAXRTX);
                        mac->SetMType(LorawanMacHeader::CONFIRMED_DATA_UP);
                    }
                    break;

                case 1:


                    if ((static_cast<int>(endDeviceLoraPhy->GetSpreadingFactor()) == 7) &&
                        RTX_ONLY_SF7 && !isEndDeviceRegular)
                    {
                        Ptr<EndDeviceLorawanMac> mac =
                            loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
                        mac->SetMaxNumberOfTransmissions(MAXRTX);
                        mac->SetMType(LorawanMacHeader::CONFIRMED_DATA_UP);
                    }
                    break;
                default:
                    break;
                }
            }

            else if (!RTX_ONLY_SF7)
            {
                Ptr<EndDeviceLorawanMac> mac =
                    loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
                mac->SetMaxNumberOfTransmissions(MAXRTX);
                mac->SetMType(LorawanMacHeader::CONFIRMED_DATA_UP);
            }
        }
    } */

    if (flagRtx)
    {
        (std::map<ns3::Ptr<ns3::Node>, deviceType>::iterator j = deviceTypeMap.begin();
         j != deviceTypeMap.end();
         ++j)
        {
            Ptr<Node> node = *j;
            Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
            Ptr<LoraPhy> phy = loraNetDevice->GetPhy();
            Ptr<EndDeviceLoraPhy> endDeviceLoraPhy = phy->GetObject<EndDeviceLoraPhy>();

            bool isRtx = false;

            if (RTX_ONLY_DEVICE_TYPE)
            {
                bool isEndDeviceRegular = (*j).second == REGULAR_DEVICE;
                if (RTX_REGULAR && isEndDeviceRegular)
                {
                    /*rtx em dispositivos regular*/
                    isRtx = true;
                }
                else if (!RTX_REGULAR && !isEndDeviceRegular)
                {
                    /*rtx em dispositivos alarm*/
                    isRtx = true;
                }
            }

            if (RTX_ONLY_SF7)
            {
                isRtx = (static_cast<int>(endDeviceLoraPhy->GetSpreadingFactor()) == 7);
            }

            if (!RTX_ONLY_SF7 && !RTX_ONLY_DEVICE_TYPE)
            {
                /*rtx all*/
                isRtx = true;
            }

            if (isRtx)
            {
                Ptr<EndDeviceLorawanMac> mac =
                    loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
                mac->SetMaxNumberOfTransmissions(MAXRTX);
                mac->SetMType(LorawanMacHeader::CONFIRMED_DATA_UP);
            }
        }
    }

    NS_LOG_DEBUG("Completed configuration");

    /*********************************************
     *  Install applications on the end devices  *
     *********************************************/

    Time appStopTime = Seconds(simulationTime);

    periodicHelper.SetPeriod(Seconds(appPeriodSeconds));
    periodicHelper.SetPacketSize(19);
    ApplicationContainer appContainer = periodicHelper.Install(endDevicesRegular);

    ramdomHelper.SetMean(appPeriodSeconds);
    ramdomHelper.SetBound(appPeriodSeconds);
    ramdomHelper.SetPacketSize(19);
    appContainer.Add(ramdomHelper.Install(endDevicesAlarm));

    appContainer.Start(Seconds(0));
    appContainer.Stop(appStopTime);

    /**************************
     *  Create Network Server  *
     ***************************/

    // Create the NS node
    NodeContainer networkServer;
    networkServer.Create(1);

    // Create a NS for the network
    nsHelper.SetEndDevices(combinedEndDevices);
    nsHelper.SetGateways(gateways);
    nsHelper.Install(networkServer);

    // Create a forwarder for each gateway
    forHelper.Install(gateways);

    /**********************
     * Print output files *
     *********************/
    if (print)
    {
        PrintEndDevices(endDevicesRegular,
                        endDevicesAlarm,
                        gateways,
                        endDevRegularFile,
                        endDevAlarmeFile,
                        gwFile);
    }

    ////////////////
    // Simulation //
    ////////////////

    Simulator::Stop(appStopTime + Hours(1));

    NS_LOG_INFO("Running simulation...");
    Simulator::Run();

    Simulator::Destroy();

    metricsResultFile(helper.GetPacketTracker(), ALARM_DEVICE, fileData, fileMetric);
    metricsResultFile(helper.GetPacketTracker(), REGULAR_DEVICE, fileData, fileMetric);
    metricsResultFile(helper.GetPacketTracker(), ALL, fileData, fileMetric);

    return (0);
}
