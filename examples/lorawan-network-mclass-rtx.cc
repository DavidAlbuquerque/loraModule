/*
 * =====================================================================================
 *
 *       Filename:  lorawan-network-mclass-rtx.cc
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
#include "ns3/lora-packet-tracker.h"
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
#include <bitset>
#include <ctime>

using namespace ns3;
using namespace lorawan;
using namespace std;

NS_LOG_COMPONENT_DEFINE("LorawanNetworkSimulatorMClass");

// File settings
string endDevRegularFile = "./scratch/TestResult/test";
string endDevAlarmeFile = "./scratch/TestResult/test";
string gwFile = "./scratch/TestResult/test";

bool connectTraceSoucer = true;
bool flagRtx = false; //, sizeStatus=0;
uint32_t nSeed = 1;
uint8_t trial = 1, numClass = 0; //, nCount=0, nClass1=0, nClass2=0, nClass3=0;
vector<uint16_t> sfQuantAlarm(6, 0);
vector<uint16_t> sfQuantRegular(6, 0);
vector<uint16_t> sfQuantAll(6, 0);

std::map<LoraDeviceAddress, deviceFCtn> mapEndAlarm, mapEndRegular;
std::map<LoraDeviceAddress, uint8_t> AoIPlottingDevices;
std::map<ns3::Ptr<ns3::Node>, deviceType> deviceTypeMap;
std::map<LoraDeviceAddress, Ptr<EndDeviceLoraPhy>> devicePhyMapRegular, devicePhyMapAlarm;
uint16_t nDevicesAlarm;
uint8_t pDevicesAlarm = 20;
uint16_t nDevicesRegular;

// Metrics
double packLoss = 0, sent = 0, received = 0, avgDelay = 0, avgTimeOnAir = 0;
double angle = 0, sAngle = M_PI;
double throughput = 0, probSucc = 0, probLoss = 0;

// Network settings
uint16_t nDevicesTotally = 200;
uint16_t nGateways = 1;
double radius = 5600;
double gatewayRadius = 0;
uint16_t simulationTime = 600;

// Channel model
bool realisticChannelModel = false;

uint16_t appPeriodSeconds = 600;

uint8_t sfRetransmitFlagRegular = 0;
uint8_t sfRetransmitFlagAlarm = 0;
string sfFlagR = "";
string sfFlagA = "";

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

enum simulationType
{
    S_TODOS,
    S_SF7,
    S_SF7_SF8,
    S_SF12,
    S_REGULARES_SF7,
    S_ALARM_SF7
};

Ptr<ListPositionAllocator>
SetManualDevicePosition(double x, double y)
{
    Ptr<ListPositionAllocator> listAlloc = CreateObject<ListPositionAllocator>();

    double z = 0.0; // Mantemos a altura fixa
    listAlloc->Add(Vector(x, y, z));

    double distance = sqrt(x * x + y * y); // Distância ao gateway (0,0)

    std::cout << "--------------------------------------------------\n";
    std::cout << "   Posição Manual do Dispositivo   \n";
    std::cout << "--------------------------------------------------\n";
    std::cout << "  -> Coordenadas: (" << x << ", " << y << ", " << z << ")\n";
    std::cout << "  -> Distância do gateway: " << distance << "m\n";
    std::cout << "--------------------------------------------------\n";

    return listAlloc;
}

Ptr<ListPositionAllocator>
AllocateNodesBySF(uint16_t nDevices, double maxRadius)
{
    Ptr<ListPositionAllocator> listAlloc = CreateObject<ListPositionAllocator>();

    // Limites de alcance para cada SF
    std::vector<double> sfMaxDistances = {2900, 3500, 4200, 5000, 5700, maxRadius};

    uint16_t devicesPerSF = nDevices / 6;
    uint16_t excessDevices = nDevices % 6;

    std::vector<uint16_t> devicesDistribution(6, devicesPerSF);

    for (int i = 0; i < excessDevices; i++)
    {
        devicesDistribution[i]++;
    }

    for (int sf = 0; sf < 6; sf++)
    {
        double minRadius = (sf == 0) ? 0 : (sfMaxDistances[sf - 1] + 100);
        double maxRadius = sfMaxDistances[sf];

        /* std::cout << "SF" << (sf + 7) << ": "
                  << "Intervalo de distância [" << minRadius << "m - " << maxRadius << "m]\n"; */

        for (int i = 0; i < devicesDistribution[sf]; i++)
        {
            double angle = 2 * M_PI * ((double)rand() / RAND_MAX);
            double radius = minRadius + (maxRadius - minRadius) * ((double)rand() / RAND_MAX);

            // Coordenadas cartesianas
            double x = radius * cos(angle);
            double y = radius * sin(angle);
            double z = 0.0;

            listAlloc->Add(Vector(x, y, z));

            /* std::cout << "  -> Dispositivo " << (i + sf * devicesPerSF) << " | Posição: (" << x
                      << ", " << y << ")"
                      << " | Distância do gateway: " << radius << "m\n"; */
        }
    }

    /*     std::cout << "Distribuição de dispositivos por SF:" << std::endl;
        for (uint8_t sf = 0; sf < 6; sf++)
        {
            std::cout << "SF" << (sf + 7) << ": " << devicesDistribution[sf] << " dispositivos"
                      << std::endl;
        } */

    return listAlloc;
}

// Função para escrever dados em arquivos para um SF específico
void
WriteDataAoi(const std::string& filename, const DataAgeInformation& m_dataAoi, uint8_t SF)
{
    // Abre o arquivo para primarySeries
    std::string primaryFilename = filename + "P";
    std::string secondaryFilename = filename + "S";
    std::ofstream primaryFile(primaryFilename);
    if (!primaryFile.is_open())
    {
        std::cerr << "Error opening file: " << primaryFilename << std::endl;
        return;
    }

    // Abre o arquivo para secondarySeries
    std::ofstream secondaryFile(secondaryFilename);
    if (!secondaryFile.is_open())
    {
        std::cerr << "Error opening file: " << secondaryFilename << std::endl;
        primaryFile.close(); // Fecha o primaryFile se houver erro no secondaryFile
        return;
    }

    // Procura a série de dados correspondente ao SF especificado
    auto it = m_dataAoi.find(SF);
    if (it != m_dataAoi.end())
    {
        const auto& seriesData = it->second;

        // Calcula a média dos valores y em primarySeries
        double sumY = 0.0;
        for (const auto& point : seriesData.primarySeries)
        {
            sumY += point.second;
        }
        double meanY = sumY / seriesData.primarySeries.size();

        // Escreve o ponto (0, 0) no início do arquivo
        primaryFile << "0 0 " << meanY << std::endl;

        // Escreve os dados de primarySeries com a média apenas na primeira linha
        bool firstLine = true;
        for (const auto& point : seriesData.primarySeries)
        {
            if (firstLine)
            {
                primaryFile << point.first << " " << point.second << " " << meanY << std::endl;
                firstLine = false;
            }
            else
            {
                primaryFile << point.first << " " << point.second << std::endl;
            }
        }

        // Escreve os dados de secondarySeries
        for (const auto& point : seriesData.secondarySeries)
        {
            secondaryFile << point.first << " " << point.second << std::endl;
        }
    }
    else
    {
        std::cerr << "SF " << static_cast<int>(SF) << " not found in data." << std::endl;
    }

    // Fecha ambos os arquivos
    primaryFile.close();
    secondaryFile.close();
}

void
ConfigureDeviceForRetransmission(Ptr<LoraNetDevice> loraNetDevice)
{
    Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
    mac->SetMaxNumberOfTransmissions(kMaxMacTransmissionsPerPacket);
    mac->SetMType(LorawanMacHeader::CONFIRMED_DATA_UP);
}

// Verifica se o Spreading Factor (SF) pode retransmitir com base na flag binária
bool
CanRetransmit(uint8_t sf, uint8_t sfRetransmitFlag)
{
    if (sf < 7 || sf > 12)
    {
        NS_LOG_DEBUG("SF " << unsigned(sf) << " fora do intervalo válido (7-12).");
        return false;
    }

    int bitPosition = sf - 7; // SF7 → bit 0, SF8 → bit 1, ..., SF12 → bit 5
    bool result = (sfRetransmitFlag & (1 << bitPosition)) != 0; // Verifica se o bit está ativado

    return result;
}

#include <iostream>

void
simulationConfig(const std::map<Ptr<Node>, deviceType>& deviceTypeMap,
                 uint8_t sfRetransmitFlagRegular,
                 uint8_t sfRetransmitFlagAlarm)
{
    for (const auto& entry : deviceTypeMap)
    {
        Ptr<Node> node = entry.first;
        deviceType devType = entry.second;

        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<EndDeviceLorawanMac> endMac = loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
        Ptr<LoraPhy> phy = loraNetDevice->GetPhy();
        Ptr<EndDeviceLoraPhy> endDeviceLoraPhy = phy->GetObject<EndDeviceLoraPhy>();

        uint8_t spreadingFactor = endMac->GetSfFromDataRate(endMac->GetDataRate());

        // Seleciona a flag correta com base no tipo do dispositivo
        uint8_t selectedFlag =
            (devType == REGULAR_DEVICE) ? sfRetransmitFlagRegular : sfRetransmitFlagAlarm;

        if (CanRetransmit(spreadingFactor, selectedFlag))
        {
            ConfigureDeviceForRetransmission(loraNetDevice);
        }
    }
}

// Refatorada mantendo a mesma lógica e logs
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
                  string fileMetric,
                  uint8_t sfRetransmitFlagRegular,
                  uint8_t sfRetransmitFlagAlarm)
{
    Time appStopTime = Seconds(simulationTime);
    string deviceMetric;
    ofstream myfile;
    std::map<LoraDeviceAddress, deviceFCtn> mapDevices;
    vector<uint16_t> sfQuant;
    uint16_t nDevices = 0;
    vector<double> rtxQuant(kMaxMacTransmissionsPerPacket, 0);
    uint8_t sfRetransmitFlag = 0;

    // flagRtx = false;

    switch (device)
    {
    case ALARM_DEVICE:
        deviceMetric = "alarm";
        sfQuant = sfQuantAlarm;
        nDevices = nDevicesAlarm;
        mapDevices = mapEndAlarm;
        fileMetric += "metricAlarm/result-STAs-alarm";
        sfRetransmitFlag = sfRetransmitFlagAlarm;
        break;

    case REGULAR_DEVICE:
        deviceMetric = "regular";
        sfQuant = sfQuantRegular;
        nDevices = nDevicesRegular;
        mapDevices = mapEndRegular;
        fileMetric += "metricRegular/result-STAs-regular";
        sfRetransmitFlag = sfRetransmitFlagRegular;
        break;

    case ALL:
        nDevices = nDevicesTotally;
        sfQuant = sfQuantAll;
        deviceMetric = "all";
        fileMetric += "metricAll/result-STAs-all";
        mapDevices = mapEndAlarm;

        mapDevices.insert(mapEndRegular.begin(), mapEndRegular.end());
        // mapDevices.insert(mapEndAlarm.begin(), mapEndAlarm.end());

        break;

    default:
        break;
    }

    NS_LOG_INFO(endl << "//////////////////////////////////////////////");
    NS_LOG_INFO("//  METRICS FOR " << deviceMetric << " DEVICES  //");
    NS_LOG_INFO("//////////////////////////////////////////////" << endl);
    NS_LOG_INFO("SF Allocation " << deviceMetric << " Devices: -> " << "SF7="
                                 << (unsigned)sfQuant.at(0) << " SF8=" << (unsigned)sfQuant.at(1)
                                 << " SF9=" << (unsigned)sfQuant.at(2) << " SF10="
                                 << (unsigned)sfQuant.at(3) << " SF11=" << (unsigned)sfQuant.at(4)
                                 << " SF12=" << (unsigned)sfQuant.at(5));

    for (uint8_t i = SF7; i <= SF12; i++)
    {
        packLoss = 0, sent = 0, received = 0, avgDelay = 0, avgTimeOnAir = 0;
        throughput = 0, probSucc = 0, probLoss = 0;
        std::fill(rtxQuant.begin(), rtxQuant.end(), 0.0); // zerando o vetor de retransmissão

        if (sfQuant.at(i - SF7))
        {
            NS_LOG_INFO("\n##################################################################");
            NS_LOG_INFO(endl << "//////////////////////////////////////////////");
            NS_LOG_INFO("//  Computing SF-" << (unsigned)i << " performance metrics  //");
            NS_LOG_INFO("//////////////////////////////////////////////" << endl);

            if (device == ALL)
            {
                if (flagRtx && (sfRetransmitFlagRegular | sfRetransmitFlagAlarm) & (1 << (i - 7)))
                {
                    stringstream ss(tracker.CountSuccessfulRetransmissions(Seconds(0),
                                                                           appStopTime + Hours(1),
                                                                           i,
                                                                           mapDevices));

                    ss >> sent;
                    ss >> received;

                    // std::cout << "sent: " << sent << std::endl << "received: " << received <<
                    // std::endl;

                    double value = 0;
                    size_t index = 0;
                    while (ss >> value)
                    {
                        if (index < rtxQuant.size())
                        {
                            rtxQuant.at(index) = value;
                        }
                        else
                        {
                            std::cerr << "Erro: acessesando valor fora do limite do vetor\n";
                        }
                        index++;
                    }
                }
                stringstream(
                    tracker.CountAgeOfInformationGlobally(Seconds(0), appStopTime + Hours(1), i)) >>
                    avgTimeOnAir;

                stringstream(tracker.CountMacPacketsGloballyDelay(Seconds(0),
                                                                  appStopTime + Hours(1),
                                                                  (unsigned)nDevicesTotally,
                                                                  (unsigned)nGateways,
                                                                  i)) >>
                    avgDelay;

                stringstream(
                    tracker.CountMacPacketsGlobally(Seconds(0), appStopTime + Hours(1), i)) >>
                    sent >> received;
            }
            else
            {
                /* Contagem da quantidade de pacotes retransmitidos e salvo em arquivo */

                if (flagRtx && sfRetransmitFlag & (1 << (i - 7)))
                {
                    stringstream ss(tracker.CountSuccessfulRetransmissions(Seconds(0),
                                                                           appStopTime + Hours(1),
                                                                           i,
                                                                           mapDevices));

                    double value = 0;
                    size_t index = 0;

                    ss >> sent;
                    ss >> received;

                    while (ss >> value)
                    {
                        if (index < rtxQuant.size())
                        {
                            rtxQuant.at(index) = value;
                        }
                        else
                        {
                            std::cerr << "Erro: acessesando valor fora do limite do vetor\n";
                        }
                        index++;
                    }
                }
                stringstream(tracker.CountAgeOfInformationGlobally(Seconds(0),
                                                                   appStopTime + Hours(1),
                                                                   i,
                                                                   mapDevices)) >>
                    avgTimeOnAir;

                stringstream(tracker.CountMacPacketsGloballyDelay(Seconds(0),
                                                                  appStopTime + Hours(1),
                                                                  (unsigned)nDevicesTotally,
                                                                  (unsigned)nGateways,
                                                                  i,
                                                                  mapDevices)) >>
                    avgDelay;

                stringstream(tracker.CountMacPacketsGlobally(Seconds(0),
                                                             appStopTime + Hours(1),
                                                             i,
                                                             mapDevices)) >>
                    sent >> received;
            }

            packLoss = sent - received;
            throughput = received / simulationTime;

            probSucc = received / sent;
            probLoss = packLoss / sent;

            NS_LOG_INFO("----------------------------------------------------------------");
            NS_LOG_INFO("nDevices" << "  |  " << "throughput" << "  |  " << "probSucc" << "  |  "
                                   << "probLoss" << "  |  " << "avgDelay" << "  |  "
                                   << "avgTimeOnAir");
            NS_LOG_INFO(nDevices << "       |  " << throughput << "    |  " << probSucc << "   |  "
                                 << probLoss << "   |  " << avgDelay << "   |  " << avgTimeOnAir);
            NS_LOG_INFO("----------------------------------------------------------------" << endl);

            myfile.open(fileMetric + "-SF" + to_string(i) + ".dat", ios::out | ios::app);
            myfile << nDevices << ", " << throughput << ", " << probSucc << ", " << probLoss << ", "
                   << avgDelay << ", " << avgTimeOnAir << "\n";
            myfile.close();

            myfile.open(fileMetric + "-RTX" + to_string(i) + ".dat", ios::out | ios::app);
            myfile << nDevices << ", " << sent << ", " << received;
            for (uint8_t i = 0; i < rtxQuant.size(); i++)
            {
                myfile << ", " << rtxQuant[i];
            }
            myfile << "\n";
            myfile.close();

            NS_LOG_INFO("numDev:" << nDevices << " numGW:" << nGateways
                                  << " simTime:" << simulationTime << " throughput:" << throughput);
            NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
            NS_LOG_INFO("sent:" << sent << "    succ:" << received << "     drop:" << packLoss
                                << "   delay:" << avgDelay);
            NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl);
            NS_LOG_INFO("##################################################################\n");

            myfile.open(fileData, ios::out | ios::app);
            myfile << "sent: " << sent << " succ: " << received << " drop: " << packLoss << "\n";
            myfile << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << "\n";
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
        if (flagRtx)
        {
            stringstream ss(tracker.CountSuccessfulRetransmissions(Seconds(0),
                                                                   appStopTime + Hours(1),
                                                                   mapDevices));

            double value = 0;
            size_t index = 0;

            ss >> sent;
            ss >> received;

            while (ss >> value)
            {
                if (index < rtxQuant.size())
                {
                    rtxQuant.at(index) = value;
                }
                else
                {
                    std::cerr << "Erro: acessesando valor fora do limite do vetor\n";
                }
                index++;
            }
        }
        stringstream(tracker.CountAgeOfInformationGlobally(Seconds(0),
                                                           appStopTime + Hours(1),
                                                           mapDevices)) >>
            avgTimeOnAir;

        stringstream(tracker.CountMacPacketsGloballyDelay(Seconds(0),
                                                          appStopTime + Hours(1),
                                                          (unsigned)nDevicesTotally,
                                                          (unsigned)nGateways)) >>
            avgDelay;

        stringstream(
            tracker.CountMacPacketsGlobally(Seconds(0), appStopTime + Hours(1), mapDevices)) >>
            sent >> received;
    }
    else
    {
        if (flagRtx)
        {
            stringstream ss(tracker.CountSuccessfulRetransmissions(Seconds(0),
                                                                   appStopTime + Hours(1),
                                                                   mapDevices));

            double value = 0;
            size_t index = 0;

            ss >> sent;
            ss >> received;

            std::cout << "received1: " << received << std::endl;

            while (ss >> value)
            {
                if (index < rtxQuant.size())
                {
                    rtxQuant.at(index) = value;
                }
                else
                {
                    std::cerr << "Erro: acessesando valor fora do limite do vetor\n";
                }
                index++;
            }
        }
        stringstream(tracker.CountAgeOfInformationGlobally(Seconds(0),
                                                           appStopTime + Hours(1),
                                                           mapDevices)) >>
            avgTimeOnAir;

        stringstream(tracker.CountMacPacketsGloballyDelay(Seconds(0),
                                                          appStopTime + Hours(1),
                                                          (unsigned)nDevicesTotally,
                                                          (unsigned)nGateways,
                                                          mapDevices)) >>
            avgDelay;

        stringstream(
            tracker.CountMacPacketsGlobally(Seconds(0), appStopTime + Hours(1), mapDevices)) >>
            sent >> received;

        std::cout << "received2: " << received << std::endl;
    }

    packLoss = sent - received;
    throughput = received / simulationTime;

    std::cout << "throughput" << received << " / " << simulationTime << endl;

    probSucc = received / sent;
    probLoss = packLoss / sent;

    NS_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    NS_LOG_INFO("nDevices: " << nDevices);
    NS_LOG_INFO("throughput: " << throughput);
    NS_LOG_INFO("probSucc: " << probSucc << " (" << probSucc * 100 << "%)");
    NS_LOG_INFO("probLoss: " << probLoss << " (" << probLoss * 100 << "%)");
    NS_LOG_INFO("avgDelay: " << avgDelay);
    NS_LOG_INFO("Age of information: " << avgTimeOnAir);
    NS_LOG_INFO("----------------------------------" << endl);

    myfile.open(fileMetric + "-Gobally-RTX" + ".dat", ios::out | ios::app);
    myfile << nDevices << ", " << sent;
    myfile << "\n";
    myfile.close();

    myfile.open(fileData + "-" + deviceMetric + "-Gobally.dat", ios::out | ios::app);
    myfile << nDevices << ", " << throughput << ", " << probSucc << ", " << probLoss << ", "
           << avgDelay << ", " << avgTimeOnAir << "\n";
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
    NS_ASSERT(loraNetDevice);
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
    NS_ASSERT(mobility);
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
    cmd.AddValue("sfFlagR", "Retransmission flag for regular devices", sfFlagR);
    cmd.AddValue("sfFlagA", "Retransmission flag for alarm devices", sfFlagA);
    cmd.AddValue("print", "Whether or not to print various informations", print);
    cmd.AddValue("trial", "set trial parameter", trial);
    cmd.Parse(argc, argv);

    endDevRegularFile +=
        to_string(trial) + "/endDevicesRegular" + to_string(nDevicesTotally) + ".dat";
    endDevAlarmeFile += to_string(trial) + "/endDevicesAlarm" + to_string(nDevicesTotally) + ".dat";
    gwFile += to_string(trial) + "/GWs" + to_string(nGateways) + ".dat";

    std::cout << "numSeed: " << nSeed << endl;

    // Convertendo as strings binárias para valores inteiros (uint8_t)
    sfRetransmitFlagRegular = std::bitset<8>(sfFlagR.substr(2)).to_ulong();
    sfRetransmitFlagAlarm = std::bitset<8>(sfFlagA.substr(2)).to_ulong();


    // Set up logging
    LogComponentEnable("LorawanNetworkSimulatorMClass", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraPacketTracker", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
    // LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable("SimpleEndDeviceLoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable("SimpleGatewayLoraPhy", LOG_LEVEL_ALL);
    //LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
    // LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
    // LogComponentEnable("EndDeviceStatus", LOG_LEVEL_ALL);
    // LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
    // LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
    // LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
    // LogComponentEnable("RandomSenderHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("RandomSender", LOG_LEVEL_ALL);
    // LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
    // LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
    // LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
    // LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
    // LogComponentEnable("NetworkController", LOG_LEVEL_ALL);

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

    /* mobility.SetPositionAllocator(AllocateNodesBySF(nDevicesTotally, radius));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel"); */
    /*
        mobility.SetPositionAllocator(SetManualDevicePosition(5800, 0));
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel"); */

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
    PeriodicSenderHelper periodicHelper2 = PeriodicSenderHelper();

    // Create the RandomSenderHelper
    //RandomSenderHelper ramdomHelper = RandomSenderHelper();

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

    // Combine end devices into a single NodeContainer
    NodeContainer combinedEndDevices;
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

    // Installing sender type
    // ramdomHelper.Install(endDevicesAlarm);
    periodicHelper.Install(endDevicesRegular);

    periodicHelper2.Install(endDevicesAlarm);

    // Assign a mobility model to each node
    mobility.Install(endDevicesRegular);
    mobility.Install(endDevicesAlarm);

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
    helper.Install(phyHelper, macHelper, endDevicesAlarm);
    helper.Install(phyHelper, macHelper, endDevicesRegular);

    // Now end devices are connected to the channel

    for (NodeContainer::Iterator j = endDevicesRegular.Begin(); j != endDevicesRegular.End(); ++j)
    {
        Ptr<Node> node = *j;
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
        mapEndRegular[mac->GetDeviceAddress()] = deviceFCtn{(uint16_t)node->GetId(), 0};

        Ptr<LoraPhy> phy = loraNetDevice->GetPhy();
        Ptr<EndDeviceLoraPhy> endDeviceLoraPhy = phy->GetObject<EndDeviceLoraPhy>();
        devicePhyMapRegular[mac->GetDeviceAddress()] = endDeviceLoraPhy;
    }

    for (NodeContainer::Iterator j = endDevicesAlarm.Begin(); j != endDevicesAlarm.End(); ++j)
    {
        Ptr<Node> node = *j;
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<EndDeviceLorawanMac> mac = loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
        mapEndAlarm[mac->GetDeviceAddress()] = deviceFCtn{(uint16_t)node->GetId(), 0};

        Ptr<LoraPhy> phy = loraNetDevice->GetPhy();
        Ptr<EndDeviceLoraPhy> endDeviceLoraPhy = phy->GetObject<EndDeviceLoraPhy>();
        devicePhyMapAlarm[mac->GetDeviceAddress()] = endDeviceLoraPhy;
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

    sfQuantRegular = macHelper.SetSpreadingFactorsUp(endDevicesRegular, gateways, channel);
    sfQuantAlarm = macHelper.SetSpreadingFactorsUp(endDevicesAlarm, gateways, channel);
    // sfQuantRegular = macHelper.SetSpreadingFactorsEAB(endDevicesRegular, radius);
    // sfQuantAlarm = macHelper.SetSpreadingFactorsEAB(endDevicesAlarm, radius);
    //   sfQuant = macHelper.SetSpreadingFactorsEIB (endDevices, radius);
    //   sfQuant = macHelper.SetSpreadingFactorsEAB (endDevices, radius);
    //   sfQuant = macHelper.SetSpreadingFactorsProp (endDevices, 0.4, 0, radius);
    //   sfQuant = macHelper.SetSpreadingFactorsStrategies (endDevices, sfQuant, 0.76*nDevices,
    //   0*nDevices, nDevices, LorawanMacHelper::CLASS_TWO);

    for (uint16_t i = 0; i < sfQuantRegular.size(); i++)
        sfQuantRegular.at(i) ? numClass++ : numClass;

    for (uint16_t i = 0; i < sfQuantAlarm.size(); i++)
        sfQuantAlarm.at(i) ? numClass++ : numClass;

    for (uint16_t i = 0; i < sfQuantAll.size(); i++)
        sfQuantAll.at(i) = sfQuantAlarm.at(i) + sfQuantRegular.at(i);

    /*********************************************
     *  Retransmission  *

     * As flags sfRetransmitFlagRegular e sfRetransmitFlagAlarm definem quais SFs (7-12)
     * permitem retransmissão para dispositivos REGULAR e ALARM, respectivamente.
     * Cada bit representa um SF específico: Bit 0 → SF7, Bit 1 → SF8, ..., Bit 5 → SF12
     * Exemplo: 0b100101 (SF7, SF9 e SF12 ativados)
     *********************************************/
    // uint8_t sfRetransmitFlagRegular = 0b000011;
    // uint8_t sfRetransmitFlagAlarm = 0b000011;
    // uint8_t sfRetransmitFlagRegular =   0b111111;
    // uint8_t sfRetransmitFlagAlarm =     0b111111;

    if (flagRtx)
    {
        simulationConfig(deviceTypeMap, sfRetransmitFlagRegular, sfRetransmitFlagAlarm);
    }

    std::set<uint8_t> usedSFs; // Set to keep track of used SFs
    for (NodeContainer::Iterator j = combinedEndDevices.Begin(); j != combinedEndDevices.End(); ++j)
    {
        Ptr<Node> node = (*j);
        Ptr<LoraNetDevice> loraNetDevice = node->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<LoraPhy> phy = loraNetDevice->GetPhy();
        Ptr<EndDeviceLorawanMac> endMac = loraNetDevice->GetMac()->GetObject<EndDeviceLorawanMac>();
        Ptr<EndDeviceLoraPhy> endDeviceLoraPhy = phy->GetObject<EndDeviceLoraPhy>();

        uint8_t sf = endMac->GetSfFromDataRate(endMac->GetDataRate());
        LoraDeviceAddress address = endMac->GetDeviceAddress();

        // Check if the SF is already used
        if (usedSFs.find(sf) == usedSFs.end())
        {
            AoIPlottingDevices[address] = sf;
            usedSFs.insert(sf);
        }

        // Stop if we have filled all SF slots from 7 to 12
        if (usedSFs.size() >= 6)
        {
            break;
        }
    }

    /*     for (auto j = AoIPlottingDevices.begin(); j != AoIPlottingDevices.end(); ++j)
        {
            std::cout << "SF: " << static_cast<int>(j->second) << std::endl;
        } */

    NS_LOG_DEBUG("Completed configuration");

    /*********************************************
     *  Install applications on the end devices  *
     *********************************************/

    Time appStopTime = Seconds(simulationTime);

    periodicHelper.SetPeriod(Seconds(appPeriodSeconds));
    periodicHelper.SetPacketSize(19);
    ApplicationContainer appContainerPeriodic = periodicHelper.Install(endDevicesRegular);

    periodicHelper2.SetPeriod(Seconds(appPeriodSeconds));
    periodicHelper2.SetPacketSize(19);
    ApplicationContainer appContainerPeriodic2 = periodicHelper.Install(endDevicesAlarm);

    appContainerPeriodic2.Start(Seconds(0));
    appContainerPeriodic.Start(Seconds(0));
    appContainerPeriodic2.Stop(appStopTime);
    appContainerPeriodic.Stop(appStopTime);

    /* ramdomHelper.SetMean(1);
    ramdomHelper.SetBound(1);
    ramdomHelper.SetPacketSize(9);
    ApplicationContainer appContainerAlarm = ramdomHelper.Install(endDevicesAlarm); */

    /* appContainerAlarm.Start(Seconds(0));
    appContainerPeriodic.Start(Seconds(0));
    appContainerAlarm.Stop(appStopTime);
    appContainerPeriodic.Stop(appStopTime); */

    /**************************
     *  Create Network Server  *
     ***************************/
    // Create the network server node
    Ptr<Node> networkServer = CreateObject<Node>();

    // PointToPoint links between gateways and server
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));
    // Store network server app registration details for later
    P2PGwRegistration_t gwRegistration;
    for (auto gw = gateways.Begin(); gw != gateways.End(); ++gw)
    {
        auto container = p2p.Install(networkServer, *gw);
        auto serverP2PNetDev = DynamicCast<PointToPointNetDevice>(container.Get(0));
        gwRegistration.emplace_back(serverP2PNetDev, *gw);
    }

    // Create a network server for the network
    nsHelper.SetGatewaysP2P(gwRegistration);
    nsHelper.SetEndDevices(combinedEndDevices);
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

    /* print metrics */

    metricsResultFile(helper.GetPacketTracker(),
                      ALARM_DEVICE,
                      fileData,
                      fileMetric,
                      sfRetransmitFlagRegular,
                      sfRetransmitFlagAlarm);

    metricsResultFile(helper.GetPacketTracker(),
                      REGULAR_DEVICE,
                      fileData,
                      fileMetric,
                      sfRetransmitFlagRegular,
                      sfRetransmitFlagAlarm);

    metricsResultFile(helper.GetPacketTracker(),
                      ALL,
                      fileData,
                      fileMetric,
                      sfRetransmitFlagRegular,
                      sfRetransmitFlagAlarm);

    LoraPacketTracker::OrganizeRetransmittedPackets();

    DataAgeInformation dataAoi = LoraPacketTracker::GetDataAoi();
    std::cout << dataAoi.size() << std::endl;

    string fileAoi = fileMetric + "AoIData/";
    string fileDelay = fileMetric + "DelayData/";

    for (uint8_t i = SF7; i <= SF12; i++)
    {
        string fileMetricAoI = fileAoi + "SF" + to_string(i);
        WriteDataAoi(fileMetricAoI, dataAoi, i);
    }

    LoraPacketTracker& tracker = helper.GetPacketTracker();
    for (int SF = SF7; SF <= SF12; SF++)
    {
        tracker.CountAgeOfInformationGloballyPloting(Seconds(0),
                                                     appStopTime + Hours(1),
                                                     SF,
                                                     (unsigned)nDevicesTotally,
                                                     fileAoi);
        // Adicionando a coleta de dados de delay para boxplot
        tracker.CountDelayGloballyPlotting(Seconds(0),
                                           appStopTime + Hours(1),
                                           SF,
                                           (unsigned)nDevicesTotally,
                                           fileDelay);
    }

    // LoraPacketTracker::CountMetricAoi(nDevicesTotally, fileAoi);

    /*     LoraPacketTracker::PrintRetransmissionData();
        LoraPacketTracker::PrintRetransmissionData2(); */
    return (0);
}
