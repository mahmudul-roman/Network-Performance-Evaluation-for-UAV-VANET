// With out UAV
// Header File
#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/itu-r-1411-los-propagation-loss-model.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/config-store-module.h"
#include "ns3/integer.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/tag.h"
#include "ns3/seq-ts-header.h"
#include "ns3/ipv4-header.h"

// Namespace
using namespace ns3;
using namespace dsr;

//For colorful console printing
#define YELLOW_CODE "\033[33m"
#define TEAL_CODE "\033[36m"
#define BOLD_CODE "\033[1m"
#define RED_CODE "\033[91m"
#define GREEN_CODE "\033[32m"
#define END_CODE "\033[0m"

NS_LOG_COMPONENT_DEFINE ("Without-UAV");

/**
 * \ingroup wave
 * \brief The RoutingStats class manages collects statistics
 * on routing data (application-data packet and byte counts)
 * for the vehicular network
 */
// Routing Stats Class
class RoutingStats
{
public:
  /**
   * \brief Constructor
   * \return none
   */
  RoutingStats ();

  /**
   * \brief Returns the cumulative number of bytes received
   * \return the cumulative number of bytes received
   */
  uint32_t GetCumulativeRxBytes ();

  /**
   * \brief Returns the cumulative count of packets received
   * \return the cumulative count of packets received
   */
  uint32_t GetCumulativeRxPkts ();

  /**
   * \brief Increments the number of (application-data)
   * bytes received, not including MAC/PHY overhead
   * \param rxBytes the number of bytes received
   * \return none
   */
  void IncRxBytes (uint32_t rxBytes);

  /**
   * \brief Increments the count of packets received
   * \return none
   */
  void IncRxPkts ();

  /**
   * \brief Returns the cumulative number of bytes transmitted
   * \return none
   */
  uint32_t GetCumulativeTxBytes ();

  /**
   * \brief Returns the cumulative number of packets transmitted
   * \return the cumulative number of packets transmitted
   */
  uint32_t GetCumulativeTxPkts ();

  /**
   * \brief Increment the number of bytes transmitted
   * \param txBytes the number of additional bytes transmitted
   * \return none
   */
  void IncTxBytes (uint32_t txBytes);

  /**
   * \brief Increment the count of packets transmitted
   * \return none
   */
  void IncTxPkts ();

  /**
   * \brief Increment End to End delay
   * \return none
   */
  void IncEED (uint32_t txEED);

  /**
   * \brief Return the total end to end delay
   * \return the cumulative end to end delay
   */
  uint32_t GetTotalEED ();

  /**
   * \brief Increment the HOP count
   * \return none
   */
  void IncHOP (uint32_t hops);

  /**
   * \brief Return the average HOP number
   * \return the average HOP count
   */
  double GetAvgHOP ();

private:
  uint32_t m_RxBytes; ///< reeive bytes
  uint32_t m_cumulativeRxBytes; ///< cumulative receive bytes
  uint32_t m_RxPkts; ///< receive packets
  uint32_t m_cumulativeRxPkts; ///< cumulative receive packets
  uint32_t m_TxBytes; ///< transmit bytes
  uint32_t m_cumulativeTxBytes; ///< cumulative transmit bytes
  uint32_t m_TxPkts; ///< transmit packets
  uint32_t m_cumulativeTxPkts; ///< cumulative transmit packets
  uint32_t m_EED; ///< end to end delay
  uint32_t m_HOP;
  uint32_t m_TotalPacketHOP;
};

// RoutingStats Constructor
RoutingStats::RoutingStats ()
  : m_RxBytes (0),
    m_cumulativeRxBytes (0),
    m_RxPkts (0),
    m_cumulativeRxPkts (0),
    m_TxBytes (0),
    m_cumulativeTxBytes (0),
    m_TxPkts (0),
    m_cumulativeTxPkts (0),
    m_EED (0),
    m_HOP (0),
    m_TotalPacketHOP (0)
{
}

uint32_t
RoutingStats::GetCumulativeRxBytes ()
{
  return m_cumulativeRxBytes;
}

uint32_t
RoutingStats::GetCumulativeRxPkts ()
{
  return m_cumulativeRxPkts;
}

void
RoutingStats::IncRxBytes (uint32_t rxBytes)
{
  m_RxBytes += rxBytes;
  m_cumulativeRxBytes += rxBytes;
}

void
RoutingStats::IncRxPkts ()
{
  m_RxPkts++;
  m_cumulativeRxPkts++;
}

uint32_t
RoutingStats::GetCumulativeTxBytes ()
{
  return m_cumulativeTxBytes;
}

uint32_t
RoutingStats::GetCumulativeTxPkts ()
{
  return m_cumulativeTxPkts;
}

void
RoutingStats::IncTxBytes (uint32_t txBytes)
{
  m_TxBytes += txBytes;
  m_cumulativeTxBytes += txBytes;
}

void
RoutingStats::IncTxPkts ()
{
  m_TxPkts++;
  m_cumulativeTxPkts++;
}

// Added for calculating average end to end delay (added by me)
void
RoutingStats::IncEED (uint32_t txEED)
{
  m_EED += txEED;
}

// Will return the total end to end delay (added by me)
uint32_t
RoutingStats::GetTotalEED ()
{
  return m_EED;
}

void
RoutingStats::IncHOP (uint32_t hops)
{
  m_HOP += hops;
  m_TotalPacketHOP++;
}

double
RoutingStats::GetAvgHOP ()
{
  return (double)m_HOP/m_TotalPacketHOP;
}

/**
 * \ingroup wave
 * \brief The RoutingHelper class generates routing data between
 * nodes (vehicles) and uses the RoutingStats class to collect statistics
 * on routing data (application-data packet and byte counts).
 * A routing protocol is configured, and all nodes attempt to send
 * (i.e. route) small packets to another node, which acts as
 * data sinks.  Not all nodes act as data sinks.
 * for the vehicular network
 */
class RoutingHelper : public Object
{
public:
  /**
   * \brief Get class TypeId
   * \return the TypeId for the class
   */
  static TypeId GetTypeId (void);

  /**
   * \brief Constructor
   * \return none
   */
  RoutingHelper ();

  /**
   * \brief Destructor
   * \return none
   */
  virtual ~RoutingHelper ();

  /**
   * \brief Installs routing funcationality on nodes and their
   * devices and interfaces.
   * \param c node container
   * \param d net device container
   * \param i IPv4 interface container
   * \param totalTime the total time that nodes should attempt to
   * route data
   * \param protocol the routing protocol (1=OLSR;2=AODV;3=DSDV;4=DSR)
   * \param nSinks the number of nodes which will act as data sinks
   * \param routingTables dump routing tables at t=5 seconds (0=no;1=yes)
   * \return none
   */
  void Install (NodeContainer & c,
                NodeContainer & u, // Added for RSU nodes
                NetDeviceContainer & d, 
                NetDeviceContainer & ud, // Added for RSU NetDevices
                Ipv4InterfaceContainer & i,
                Ipv4InterfaceContainer & ui, // Added for RSU IP Interfaces
                double totalTime,
                int protocol,
                uint32_t nSinks,
                int routingTables);

  /**
   * \brief Trace the receipt of an on-off-application generated packet
   * \param context this object
   * \param packet a received packet
   * \return none
   */
  void OnOffTrace (std::string context, Ptr<const Packet> packet);

  void OnOffTraceWithAddress (std::string context, const Ptr< const Packet > packet, const Address &srcAddress, const Address &destAddress);

  /**
   * \brief Returns the RoutingStats instance
   * \return the RoutingStats instance
   */
  RoutingStats & GetRoutingStats ();

private:
  /**
   * \brief Sets up the protocol protocol on the nodes
   * \param c node container
   * \return none
   */
  void SetupRoutingProtocol (NodeContainer & c, NodeContainer & u); // u is for RSU nodes, v is for rsu nodes

  /**
   * \brief Assigns IPv4 addresses to net devices and their interfaces
   * \param d net device container
   * \param adhocTxInterfaces IPv4 interface container
   * \return none
   */
  void AssignIpAddresses (NetDeviceContainer & d, // vehicle
                          Ipv4InterfaceContainer & adhocTxInterfaces, // vehicle
                          NetDeviceContainer & ud, // rsu
                          Ipv4InterfaceContainer & ui); // rsu

  /**
   * \brief Sets up routing messages on the nodes and their interfaces
   * \param c node container
   * \param adhocTxInterfaces IPv4 interface container
   * \return none
   */
  void SetupRoutingMessages (NodeContainer & c,
                             Ipv4InterfaceContainer & adhocTxInterfaces, 
                             NodeContainer & d,
                             Ipv4InterfaceContainer & rsuTxInterfaces);

  /**
   * \brief Sets up a routing packet for tranmission
   * \param addr destination address
   * \param node source node
   * \return Socket to be used for sending/receiving a routed data packet
   */
  Ptr<Socket> SetupRoutingPacketReceive (Ipv4Address addr, Ptr<Node> node);

  /**
   * \brief Process a received routing packet
   * \param socket the receiving socket
   * \return none
   */
  void ReceiveRoutingPacket (Ptr<Socket> socket);

  double m_TotalSimTime;        ///< seconds
  uint32_t m_protocol;       ///< routing protocol; 0=NONE, 1=OLSR, 2=AODV, 3=DSDV, 4=DSR
  uint32_t m_port;           ///< port
  uint32_t m_nSinks;              ///< number of sink nodes (< all nodes)
  int m_routingTables;      ///< dump routing table (at t=5 sec).  0=No, 1=Yes
  RoutingStats routingStats; ///< routing statistics
  std::string m_protocolName; ///< protocol name
  int m_log; ///< log
};

NS_OBJECT_ENSURE_REGISTERED (RoutingHelper);

TypeId
RoutingHelper::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RoutingHelper")
    .SetParent<Object> ()
    .AddConstructor<RoutingHelper> ();
  return tid;
}

// Routing Helper Constructor
RoutingHelper::RoutingHelper ()
  : m_TotalSimTime (300.01),
    m_protocol (2), // set protocol
    m_port (9),
    m_nSinks (0),
    m_routingTables (0), // dump routing table (at t=5 sec).  0=No, 1=Yes
    m_log (0)
{
}

// Routing Helper Destructor
RoutingHelper::~RoutingHelper ()
{
}

// Setting up routing protocol, IP addresses and data messages
void
RoutingHelper::Install (NodeContainer & c,
                        NodeContainer & u, // Added for RSU node
                        NetDeviceContainer & d, 
                        NetDeviceContainer & ud, // Added for RSU NetDevice
                        Ipv4InterfaceContainer & i,
                        Ipv4InterfaceContainer & ui, // Added for RSU IP Interface
                        double totalTime,
                        int protocol,
                        uint32_t nSinks,
                        int routingTables)
{
  m_TotalSimTime = totalTime;
  m_protocol = protocol;
  m_nSinks = nSinks;
  m_routingTables = routingTables;

  SetupRoutingProtocol (c, u);
  AssignIpAddresses (d, i, ud, ui);
  //SetupRoutingMessages (c, i);
  SetupRoutingMessages (c, i, u, ui);
}

// Setting up routing protocol
void
RoutingHelper::SetupRoutingProtocol (NodeContainer & c, NodeContainer & u) // u is for rsu node
{
    AodvHelper aodv;
  OlsrHelper olsr;
  DsdvHelper dsdv;
  DsrHelper dsr;
  DsrMainHelper dsrMain;
  Ipv4ListRoutingHelper list;
  InternetStackHelper internet;

  Time rtt = Time (5.0);
  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> rtw = ascii.CreateFileStream ("routing_table");

  switch (m_protocol)
    {
    case 0:
      m_protocolName = "NONE";
      break;
    case 1:
      if (m_routingTables != 0)
        {
          olsr.PrintRoutingTableAllAt (rtt, rtw);
        }
      list.Add (olsr, 100);
      m_protocolName = "OLSR";
      break;
    case 2:
      if (m_routingTables != 0)
        {
          aodv.PrintRoutingTableAllAt (rtt, rtw);
        }
      list.Add (aodv, 100);
      m_protocolName = "AODV";
      break;
    case 3:
      if (m_routingTables != 0)
        {
          dsdv.PrintRoutingTableAllAt (rtt, rtw);
        }
      list.Add (dsdv, 100);
      m_protocolName = "DSDV";
      break;
    case 4:
      // setup is later
      m_protocolName = "DSR";
      break;
    default:
      NS_FATAL_ERROR ("No such protocol:" << m_protocol);
      break;
    }

  if (m_protocol < 4)
    {
      internet.SetRoutingHelper (list);
      internet.Install (c);
      internet.Install (u); // Installing Internet stack on RSU nodes
    }
  else if (m_protocol == 4)
    {
      internet.Install (c);
      dsrMain.Install (dsr, c);
    }

  if (m_log != 0)
    {
      NS_LOG_UNCOND ("Routing Setup for " << m_protocolName);
    }
}

// Assigning IP addresses to nodes
void
RoutingHelper::AssignIpAddresses (NetDeviceContainer & d, // vehicle
                                  Ipv4InterfaceContainer & adhocTxInterfaces, // vehicle
                                  NetDeviceContainer & ud, // rsu
                                  Ipv4InterfaceContainer & ui) // rsu
{
  NS_LOG_INFO ("Assigning IP addresses");
  Ipv4AddressHelper addressAdhoc;
  // we may have a lot of nodes, and want them all
  // in same subnet, to support broadcast
  addressAdhoc.SetBase ("10.1.0.0", "255.255.0.0");
  adhocTxInterfaces = addressAdhoc.Assign (d);
  ui = addressAdhoc.Assign (ud);
}

// Sending packet to nodes
// Setup Application
// void
// RoutingHelper::SetupRoutingMessages (NodeContainer & c,
//                                      Ipv4InterfaceContainer & adhocTxInterfaces)
void
RoutingHelper::SetupRoutingMessages (NodeContainer & c,
                                     Ipv4InterfaceContainer & adhocTxInterfaces, NodeContainer & d, Ipv4InterfaceContainer & rsuTxInterfaces)
{
  // Setup routing transmissions
  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());

  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
  int64_t stream = 2;
  var->SetStream (stream);
//   for (uint32_t i = 0; i < m_nSinks; i++) // if m_nsink is equal to 10 then first 10 nodes will be set as sink nodes
//     {
      // protocol == 0 means no routing data, WAVE BSM only
      // so do not set up sink
    for (uint32_t i = 0; i < 10; i++)
    {
      if (m_protocol != 0)
        {
          // Setting up Client (sink)
          Ptr<Socket> sink = SetupRoutingPacketReceive (adhocTxInterfaces.GetAddress (i)/*Destination IP address*/, c.Get (i)/*Destination node*/);
          //Ptr<Socket> sink = SetupRoutingPacketReceive (rsuTxInterfaces.GetAddress (i)/*Destination IP address*/, d.Get (i)/*Destination node*/);
        }

      // Setting up the destination address and port number
      AddressValue remoteAddress (InetSocketAddress (adhocTxInterfaces.GetAddress (i), m_port));
      onoff1.SetAttribute ("Remote", remoteAddress);

      // Setting up server(source node)
      //ApplicationContainer temp = onoff1.Install (c.Get (0));
      ApplicationContainer temp = onoff1.Install (d.Get (0));
      temp.Start (Seconds (var->GetValue (1.0,2.0)));
      temp.Stop (Seconds (m_TotalSimTime));
    }
}

// When packet received print on terminal
static inline std::string
PrintReceivedRoutingPacket (Ptr<Socket> socket, Ptr<Packet> packet, Address srcAddress, uint32_t h_txtime)
{
  std::ostringstream oss;

  oss << Simulator::Now ().GetSeconds () << " Node:" << socket->GetNode ()->GetId ();

  oss << "->IP:" << socket->GetNode ()->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetAddress (0).GetLocal ();
  
  //std::cout << socket->GetNode ()->GetObject<Ipv4RawSocketImpl> ()->GetIpTtl () << std::endl;

  if (InetSocketAddress::IsMatchingType (srcAddress))
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (srcAddress);
      oss << " received one packet from " << addr.GetIpv4 ();
      oss << " RxTime: " << Simulator::Now ().GetMilliSeconds () << "ms";
      uint32_t rx_time = Simulator::Now ().GetMilliSeconds ();
      oss << TEAL_CODE << " Delay: " << rx_time - h_txtime << "ms" << END_CODE;
    }
  else
    {
      oss << " received one packet!";
    }
  return oss.str ();
}

// Packet Receive Function
void
RoutingHelper::ReceiveRoutingPacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  // Ptr<Packet> ipPacket;
  Address srcAddress;
  while ((packet = socket->RecvFrom (srcAddress)))
    {      
      std::cout << GREEN_CODE << "A packet is received!" << END_CODE << std::ends;

      SocketIpTtlTag ttlTag;
      if (packet->RemovePacketTag (ttlTag))
      {
        uint32_t ttl = ttlTag.GetTtl ();
        routingStats.IncHOP (65-ttl);
        std::cout << GREEN_CODE << "\tTTL is Found!" << END_CODE << std::ends;
        std::cout << BOLD_CODE << GREEN_CODE << "\tTTL: " << ttl << "\tHop Count: " << 65-ttl << END_CODE << std::endl;
      }

      // application data, for goodput
      uint32_t RxRoutingBytes = packet->GetSize ();
      GetRoutingStats ().IncRxBytes (RxRoutingBytes);
      GetRoutingStats ().IncRxPkts ();

      // Extracting tx time from packet header
      SeqTsHeader txtime;
      packet->PeekHeader (txtime);
      uint32_t h_txtime = txtime.GetSeq ();

      // Get Rx time
      uint32_t rx_time = Simulator::Now ().GetMilliSeconds ();

      // Calculate Delay
      // Incrementing total EED
      routingStats.IncEED (rx_time - h_txtime);
      std::cout << RED_CODE << "\ttxtime " << h_txtime << "\teed: " << routingStats.GetTotalEED () 
                            << "\tRxRoutingBytes: " << RxRoutingBytes << END_CODE << std::endl;

      NS_LOG_UNCOND (m_protocolName + " " + PrintReceivedRoutingPacket (socket, packet, srcAddress, h_txtime));
    }
}

// Callback for Packet Receive
Ptr<Socket>
RoutingHelper::SetupRoutingPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, m_port);
  sink->Bind (local);
  // Enabling receiving of IP TTL
  sink->SetIpRecvTtl (true);
  sink->SetRecvCallback (MakeCallback (&RoutingHelper::ReceiveRoutingPacket, this));

  std::cout << GREEN_CODE << "Packet Received Function is Called." << END_CODE << std::endl;
  
  return sink;
}

// Tracing of application
void
RoutingHelper::OnOffTrace (std::string context, Ptr<const Packet> packet)
{
  //std::cout << TEAL_CODE << context << END_CODE << std::endl; // RnD, printing the trace source context
  uint32_t pktBytes = packet->GetSize ();
  //std::cout << "\tPacket Size: " << pktBytes << std::endl;
  routingStats.IncTxBytes (pktBytes);
  routingStats.IncTxPkts ();
}

// Tracing of application
void
RoutingHelper::OnOffTraceWithAddress (std::string context, const Ptr< const Packet > packet, const Address &srcAddress, const Address &destAddress)
{
    std::cout << YELLOW_CODE << context << " Time: "<< Simulator::Now ().GetMilliSeconds () << "ms"<< END_CODE << std::endl;
    std::cout << "\tPacket Size: " << packet->GetSize () 
                << " Source Address: " << InetSocketAddress::ConvertFrom (srcAddress).GetIpv4 ()
                << " Destination Address " << InetSocketAddress::ConvertFrom (destAddress).GetIpv4 () << std::endl;
}

RoutingStats &
RoutingHelper::GetRoutingStats ()
{
  return routingStats;
}

/**
 * \ingroup wave
 * \brief The VanetRoutingExperiment class implements a wifi app that
 * allows VANET routing experiments to be simulated
 */
class VanetRoutingExperiment
{
public:
  /**
   * \brief Constructor
   * \return none
   */
  VanetRoutingExperiment ();

  /**
   * \brief Enacts simulation of an ns-3 wifi application
   * \param argc program arguments count
   * \param argv program arguments
   * \return none
   */
  void Simulate (int argc, char **argv);

protected:

  /**
   * \brief Configure nodes
   * \return none
   */
  virtual void ConfigureNodes ();

  /**
   * \brief Configure applications
   * \return none
   */
  virtual void ConfigureApplications ();

  /**
   * \brief Process outputs
   * \return none
   */
  virtual void ProcessOutputs ();

private:
  /**
   * \brief Run the simulation
   * \return none
   */
  void Run ();

  /**
   * \brief Configure default attributes
   * \return none
   */
  void ConfigureDefaults ();

  /**
   * \brief Set up the adhoc mobility nodes
   * \return none
   */
  void SetupAdhocMobilityNodes ();

  /**
   * \brief Set up the adhoc devices
   * \return none
   */
  void SetupAdhocDevices ();

  /**
   * \brief Set up generation of packets to be routed
   * through the vehicular network
   * \return none
   */
  void SetupRoutingMessages ();

  /**
   * \brief Set up a prescribed scenario
   * \return none
   */
  void SetupScenario ();

  uint32_t m_port; ///< port
  uint32_t m_nSinks; ///< number of sinks
  std::string m_protocolName; ///< protocol name
  double m_txp; ///< distance
  bool m_traceMobility; ///< trace mobility
  uint32_t m_protocol; ///< protocol
  uint32_t m_lossModel; ///< loss model
  uint32_t m_fading; ///< fading
  std::string m_lossModelName; ///< loss model name
  std::string m_phyMode; ///< phy mode
  uint32_t m_80211mode; ///< 80211 mode
  std::string m_traceFile; ///< trace file 
  std::string m_logFile; ///< log file
  uint32_t m_mobility; ///< mobility
  uint32_t m_nNodes; ///< number of nodes
  uint32_t m_nrsuNodes; ///< number of rsu
  double m_TotalSimTime; ///< total sim time
  std::string m_rate; ///< rate
  std::string m_phyModeB; ///< phy mode
  std::string m_trName; ///< trace file name
  int m_nodeSpeed; ///< in m/s
  int m_nodePause; ///< in s
  uint32_t m_wavePacketSize; ///< bytes
  double m_waveInterval; ///< seconds
  int m_verbose; ///< verbose
  std::ofstream m_os; ///< output stream

  NetDeviceContainer m_adhocTxDevices; ///< adhoc transmit devices

  // Net Device for RSU
  NetDeviceContainer m_rsuTxDevices;

  Ipv4InterfaceContainer m_adhocTxInterfaces; ///< adhoc transmit interfaces

  // Ipv4 interface for RSU
  Ipv4InterfaceContainer m_rsuTxInterfaces;

  uint32_t m_scenario; ///< scenario
  double m_gpsAccuracyNs; ///< GPS accuracy
  double m_txMaxDelayMs; ///< transmit maximum delay
  int m_routingTables; ///< routing tables
  int m_asciiTrace; ///< ascii trace
  int m_pcap; ///< PCAP

  Ptr<RoutingHelper> m_routingHelper; ///< routing helper
  int m_log; ///< log
  /// used to get consistent random numbers across scenarios
  int64_t m_streamIndex;
  NodeContainer m_adhocTxNodes; ///< adhoc transmit nodes

  // Creating RSU nodes
  NodeContainer m_rsuNodes;

};

// Constructor --> Initializing values for simulation
VanetRoutingExperiment::VanetRoutingExperiment ()
  : m_port (9),
    m_nSinks (10), // number of sink nodes < all nodes: default was 10, now setting to 80
    m_protocolName ("protocol"),
    m_txp (11), // txp power 11 will cover upto 500 meters
    m_traceMobility (false),
    m_protocol (2), // set protocol // aodv
    m_lossModel (3), // Two-Ray ground
    m_fading (0),
    m_lossModelName (""),
    m_phyMode ("OfdmRate6MbpsBW10MHz"),
    m_80211mode (1), // 1=802.11p
    m_traceFile (""),
    m_logFile ("low99-ct-unterstrass-1day.filt.7.adj.log"),
    m_mobility (1),
    m_nNodes (156),
    m_TotalSimTime (300.01),
    m_rate ("10240bps"), // default 2048bps
    m_phyModeB ("DsssRate11Mbps"),
    m_trName ("vanet-routing-compare"),
    m_nodeSpeed (20),
    m_nodePause (0),
    m_wavePacketSize (200),
    m_waveInterval (0.1),
    m_verbose (0),
    m_scenario (2),
    m_gpsAccuracyNs (40),
    m_txMaxDelayMs (10),
    m_routingTables (0), // dump routing table (at t=5 sec).  0=No, 1=Yes
    m_asciiTrace (0),
    m_pcap (0),
    m_log (0),
    m_streamIndex (0),
    m_adhocTxNodes (),
    m_rsuNodes ()
{
  m_routingHelper = CreateObject<RoutingHelper> ();

  // set to non-zero value to enable
  // simply uncond logging during simulation run
  m_log = 1;
}


// Setting up the vehicular environment
void
VanetRoutingExperiment::SetupScenario ()
{
  // member variable parameter use
  // defaults or command line overrides,
  // except where scenario={1,2,3,...}
  // have been specified, in which case
  // specify parameters are overwritten
  // here to setup for specific scenarios

  // certain parameters may be further overridden
  // i.e. specify a scenario, override tx power.

  if (m_scenario == 1)
    {
      // 40 nodes in RWP 300 m x 1500 m synthetic highway, 10s
      m_traceFile = "";
      m_logFile = "";
      m_mobility = 2;
      if (m_nNodes == 156)
        {
          m_nNodes = 1;
        }
      if (m_TotalSimTime == 300.01)
        {
          m_TotalSimTime = 40.0;
        }
    }
  else if (m_scenario == 2)
    {
      // Realistic vehicular trace
      m_traceFile = "/home/roman/Documents/sumo/tools/2021-08-20-05-20-43/mobility.tcl";
      m_logFile = "low99-ct-unterstrass-1day.filt.7.adj.log";
      m_mobility = 1;
      m_nNodes = 29;
      m_TotalSimTime = 10.0;
      m_nodeSpeed = 0;
      m_nodePause = 0;

      // RSU information
      m_nrsuNodes = 1;
    }
}

// Configure Defaults
void
VanetRoutingExperiment::ConfigureDefaults ()
{
  Config::SetDefault ("ns3::OnOffApplication::PacketSize",StringValue ("512"));// original packetsize was 64, changed to 512
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (m_rate));

  //Set Non-unicastMode rate to unicast mode
  if (m_80211mode == 2)
    {
      Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (m_phyModeB));
    }
  else // default is m_80211mode = 1
    {
      Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (m_phyMode));
    }
}

// Configure Nodes
void
VanetRoutingExperiment::ConfigureNodes ()
{
  m_adhocTxNodes.Create (m_nNodes);
  m_rsuNodes.Create (m_nrsuNodes);
}

// Configuring Channels
void
VanetRoutingExperiment::SetupAdhocDevices ()
{
  if (m_lossModel == 1)
    {
      m_lossModelName = "ns3::FriisPropagationLossModel";
    }
  else if (m_lossModel == 2)
    {
      m_lossModelName = "ns3::ItuR1411LosPropagationLossModel";
    }
  else if (m_lossModel == 3) // Default is two ray ground (m_lossModel = 3)
    {
      m_lossModelName = "ns3::TwoRayGroundPropagationLossModel";
    }
  else if (m_lossModel == 4)
    {
      m_lossModelName = "ns3::LogDistancePropagationLossModel";
    }
  else
    {
      // Unsupported propagation loss model.
      // Treating as ERROR
      NS_LOG_ERROR ("Invalid propagation loss model specified.  Values must be [1-4], where 1=Friis;2=ItuR1411Los;3=TwoRayGround;4=LogDistance");
    }

  // frequency
  double freq = 0.0;
  if ((m_80211mode == 1) // default is m_80211mode = 1
      || (m_80211mode == 3))
    {
      // 802.11p 5.9 GHz
      freq = 5.9e9;
    }
  else
    {
      // 802.11b 2.4 GHz
      freq = 2.4e9;
    }

  // Setup propagation models
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  if (m_lossModel == 3) // Default is two ray ground (m_lossModel = 3)
    {
      // two-ray requires antenna height (else defaults to Friss)
      wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq), "HeightAboveZ", DoubleValue (1.5));
    }
  else
    {
      wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq));
    }

  // Propagation loss models are additive.
  if (m_fading != 0)
    {
      // if no obstacle model, then use Nakagami fading if requested
      wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
    }

  // the channel
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();

  // The below set of helpers will help us to put together the wifi NICs we want
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.SetChannel (channel);
  wavePhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

  // Setup WAVE PHY and MAC
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (m_verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
      // likewise, turn on WAVE PHY logging
      waveHelper.EnableLogComponents ();
    }

  WifiHelper wifi;

  // Setup 802.11b stuff
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (m_phyModeB),
                                "ControlMode",StringValue (m_phyModeB));

  // Setup 802.11p stuff
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (m_phyMode),
                                      "ControlMode",StringValue (m_phyMode));

  // Setup WAVE-PHY stuff
  waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (m_phyMode),
                                      "ControlMode",StringValue (m_phyMode));

  // Set Tx Power
  wifiPhy.Set ("TxPowerStart",DoubleValue (m_txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txp));

  wavePhy.Set ("TxPowerStart",DoubleValue (m_txp));
  wavePhy.Set ("TxPowerEnd", DoubleValue (m_txp));

  // Add an upper mac and disable rate control
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();

  // Setup net devices

  if (m_80211mode == 3)
    {
      m_adhocTxDevices = waveHelper.Install (wavePhy, waveMac, m_adhocTxNodes);
    }
  else if (m_80211mode == 1) // This one is default (802.11p) // default is m_80211mode = 1
    {
      m_adhocTxDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, m_adhocTxNodes);
      m_rsuTxDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, m_rsuNodes);
    }
  else
    {
      m_adhocTxDevices = wifi.Install (wifiPhy, wifiMac, m_adhocTxNodes);
    }

  if (m_asciiTrace != 0)
    {
      AsciiTraceHelper ascii;
      Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (m_trName + ".tr").c_str ());
      wifiPhy.EnableAsciiAll (osw);
      wavePhy.EnableAsciiAll (osw);
    }
  if (m_pcap != 0)
    {
      wifiPhy.EnablePcapAll ("vanet-routing-compare-pcap");
      wavePhy.EnablePcapAll ("vanet-routing-compare-pcap");
    }
}

// Setup mobility of the nodes
void
VanetRoutingExperiment::SetupAdhocMobilityNodes ()
{
  if (m_mobility == 1)
    {
      // Setting vehicles mobility
      // Create Ns2MobilityHelper with the specified trace log file as parameter
      Ns2MobilityHelper ns2 = Ns2MobilityHelper (m_traceFile);
      ns2.Install (); // configure movements for each node, while reading trace file

      // Setting RSU position
      MobilityHelper rsuMobility;
      Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
      positionAlloc->Add (Vector (2000.0, 1000.0, 1.5));
      rsuMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      rsuMobility.SetPositionAllocator (positionAlloc);
      rsuMobility.Install (m_rsuNodes);

      // initially assume all nodes are not moving
      //WaveBsmHelper::GetNodesMoving ().resize (m_nNodes, 0);
    }
  else if (m_mobility == 2)
    {
      // MobilityHelper mobilityAdhoc;

      // Default Random mobility model for vehicles
      // ObjectFactory pos;
      // pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
      // pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));
      // pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
      // // we need antenna height uniform [1.0 .. 2.0] for loss model
      // pos.Set ("Z", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"));

      // Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
      // m_streamIndex += taPositionAlloc->AssignStreams (m_streamIndex);

      // std::stringstream ssSpeed;
      // ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << m_nodeSpeed << "]";
      // std::stringstream ssPause;
      // ssPause << "ns3::ConstantRandomVariable[Constant=" << m_nodePause << "]";
      // mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
      //                                 "Speed", StringValue (ssSpeed.str ()),
      //                                 "Pause", StringValue (ssPause.str ()),
      //                                 "PositionAllocator", PointerValue (taPositionAlloc));
      // mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
      // mobilityAdhoc.Install (m_adhocTxNodes);
      // m_streamIndex += mobilityAdhoc.AssignStreams (m_adhocTxNodes, m_streamIndex);


      // initially assume all nodes are moving
      WaveBsmHelper::GetNodesMoving ().resize (m_nNodes, 1);
    }
}

// Passing the variables onto the Routing helper class function for Setup Routing Messages
void
VanetRoutingExperiment::SetupRoutingMessages ()
{
  m_routingHelper->Install (m_adhocTxNodes,
                            m_rsuNodes, // node for RSU
                            m_adhocTxDevices,
                            m_rsuTxDevices, // NetDevice for RSU node
                            m_adhocTxInterfaces,
                            m_rsuTxInterfaces, // IP interface for RSU node
                            m_TotalSimTime,
                            m_protocol,
                            m_nSinks,
                            m_routingTables);
}

// Configure Application
void
VanetRoutingExperiment::ConfigureApplications ()
{
  // Traffic mix consists of:
  // 1. routing data
  // 2. Broadcasting of Basic Safety Message (BSM)
  SetupRoutingMessages ();
  //SetupWaveMessages ();

  // config trace to capture app-data (bytes) for
  // routing data, subtracted and used for
  // routing overhead

  std::ostringstream oss;
  oss.str ("");
  oss << "/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx";
  Config::Connect (oss.str (), MakeCallback (&RoutingHelper::OnOffTrace, m_routingHelper));

  // trace source using tx with addresses
  Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/TxWithAddresses", MakeCallback (&RoutingHelper::OnOffTraceWithAddress, m_routingHelper));
}

// Start the simulation
void
VanetRoutingExperiment::Run ()
{
  NS_LOG_INFO ("Run Simulation.");

  Simulator::Stop (Seconds (m_TotalSimTime));
  // Adding code for netanim
  AnimationInterface anim("nouav.xml");
  Simulator::Run ();
  Simulator::Destroy ();
}

// Processing results
void
VanetRoutingExperiment::ProcessOutputs ()
{

  double averageRoutingGoodputKbps = 0.0;
  uint32_t totalBytesTotal = m_routingHelper->GetRoutingStats ().GetCumulativeRxBytes ();
  averageRoutingGoodputKbps = (((double) totalBytesTotal * 8.0) / m_TotalSimTime) / 1000.0;

  // calculate MAC/PHY overhead (mac-phy-oh)
  // total WAVE BSM bytes sent
  
  uint32_t totalPacketSend = m_routingHelper->GetRoutingStats ().GetCumulativeTxPkts (); // added by me
  uint32_t totalPacketRcvd = m_routingHelper->GetRoutingStats ().GetCumulativeRxPkts (); // added by me
  double pdr = (double)totalPacketRcvd/totalPacketSend; //added by me
  uint32_t totalEED = m_routingHelper->GetRoutingStats ().GetTotalEED (); // added by me
  double avgEED = (double)totalEED/totalPacketRcvd; // added by me
  double avgHOPCount = m_routingHelper->GetRoutingStats ().GetAvgHOP ();
  

  if (m_log != 0)
    {
      NS_LOG_UNCOND ("\nGoodput=" << averageRoutingGoodputKbps << "Kbps" << " T_PKT_SEND=" << totalPacketSend << " T_PKT_RCVD=" 
                      << totalPacketRcvd << " PDR=" << pdr << " AvgEED=" << avgEED << "ms" << " AvgHOPCounts=" << avgHOPCount << "\n");

    }
}

void
VanetRoutingExperiment::Simulate (int argc, char **argv)
{
  // Simulator Program Flow:
  //   (HandleProgramInputs:)
  //   SetDefaultAttributeValues
  //   ParseCommandLineArguments
  //   (ConfigureTopology:)
  //   ConfigureNodes
  //   ConfigureChannels
  //   ConfigureDevices
  //   ConfigureMobility
  //   ConfigureApplications
  //     e.g AddInternetStackToNodes
  //         ConfigureIpAddressingAndRouting
  //         configureSendMessages
  //   ConfigureTracing
  //   RunSimulation
  //   ProcessOutputs

  SetupScenario ();
  ConfigureDefaults ();
  ConfigureNodes ();
  SetupAdhocDevices ();
  SetupAdhocMobilityNodes ();
  ConfigureApplications ();
  Run ();
  ProcessOutputs ();
}

int
main (int argc, char *argv[])
{
  VanetRoutingExperiment experiment;
  experiment.Simulate (argc, argv);
}
