#include <queue>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <limits.h>
#include <iostream>

#define M_PI 3.1415926

#define numOfRelayNodes       50       // 測試用 待改
#define numOfSinkNodes        1
#define numOfSourceNodes      1
#define numOfNodes            (numOfRelayNodes+numOfSinkNodes+numOfSourceNodes)
#define numOfPackets          150      // 測試用 待改
#define numOfReferredPackets  100      // 測試用 待改

#define packetSize            3000     // unit in bytes
#define packetDataLength      2988 

#define dimensionX            1000     // unit in meters
#define dimensionY            1000     // 測試用 待改
#define dimensionZ            1000     // 實際考慮 1000-3000m

#define nodeSpeedMin          0.0001   // unit in meters per ms
#define nodeSpeedMax          0.0005        

#define powerIdling           0.000008 // unit in watts (J/ms) 8mW
#define powerReceiving        0.00075  // 0.75W
#define powerSending          0.002    // 2W

#define energy_MAX            1000     // unit in Joule (J) (節點初始電量)

#define transmissionRangeMax  350      // unit in meters (重要參數)
#define propogationSpeed      1.5      // unit in meters per ms
#define bitRate               10       // unit in bits per ms

#define simulationTime        6000000  // unit in ms (10 min)
#define extendedTime          600000   // 最後一個封包送出後再延長一小段時間
#define timeslotLength        100      // unit in ms
#define positionUpdateSlot    1000     // unit in ms

#define sourcePacketRate      1000     // unit in ms per packet (constant bit rate, CBR)

#define event_ARRIVAL         0
#define event_DEPARTURE       1

#define node_type_SOURCE      0
#define node_type_SINK        1
#define node_type_RELAY       2

#define msg_type_DATA         0        // "0"為 data 之後可定義不同 type (保留)

#define collisionEnabled      1        // 不做collision時將此設為"0"

#define traceFileName_0         "SPR_0_channel_model.log"             // channel model (transmitter power, node distance, bit error rate)
#define traceFileName_1         "SPR_1_neighborhood.log"              // neighbors of each node
#define traceFileName_2         "SPR_2_events_and_collisions.log"     // number of events and number of collisions
#define traceFileName_3         "SPR_3_tree_table_for_all_nodes.log"  // shortest path tree table for "all nodes"
#define traceFileName_4         "SPR_4_max_number_collisions.log"     // max number of collisions of one slot
#define traceFileName_5         "SPR_5_info_departure_event.log"      // information of departure events
#define traceFileName_6         "SPR_6_info_arrival_event.log"        // information of arrival events
#define traceFileName_7         "SPR_7_tree_table_for_all_sinks.log"  // shortest path tree table for "all sinks"
#define traceFileName_8         "SPR_8_hop_count.log"                 // hop count of each packet
#define traceFileName_9         "SPR_9_energy_consumption.log"        // energy consumption of each node
#define traceFileName_10        "SPR_10_node_position.log"            // position of source, sink, relay nodes
#define traceFileName_11        "SPR_11_node_position_mobile.log"     // position of nodes (different time slots)
#define traceFileName_12        "SPR_12.log"                          // 保留(需要時 可自行定義)

FILE *tFile0 = fopen(traceFileName_0, "w");  
FILE *tFile1 = fopen(traceFileName_1, "w");
FILE *tFile2 = fopen(traceFileName_2, "w");
FILE *tFile3 = fopen(traceFileName_3, "w");
FILE *tFile4 = fopen(traceFileName_4, "w");
FILE *tFile5 = fopen(traceFileName_5, "w");
FILE *tFile6 = fopen(traceFileName_6, "w");
FILE *tFile7 = fopen(traceFileName_7, "w");
FILE *tFile8 = fopen(traceFileName_8, "w");  
FILE *tFile9 = fopen(traceFileName_9, "w");  
FILE *tFile10 = fopen(traceFileName_10, "w"); 
FILE *tFile11 = fopen(traceFileName_11, "w"); 
FILE *tFile12 = fopen(traceFileName_12, "w"); // 保留(需要時 可自行定義)

using namespace std;

typedef struct SPR_Packet  // Packet format
{
	int messageType;                        // "0"為 data 之後可定義不同 type (保留)
    int senderNodeID, sourceNodeID;         // SPR 的 header 記錄 sender 和 source 
	int packetSN;
    int hop_count;                          // number of hops (from source to sink)
    //char packetData[packetDataLength];    // 佔記憶體空間故不模擬

    SPR_Packet() {}
    SPR_Packet(int mt, int hc, int psn, int sdid, int srcid) {         // DATA packet
        messageType = mt;
		hop_count = hc;
		packetSN = psn;
		senderNodeID = sdid;
		sourceNodeID = srcid;
        //strcpy(packetData, pd);          // 佔記憶體空間故不模擬
    }
} SPRpacket;

typedef struct PHB_Item   // Packet history buffer (Q2) item
{
    //int senderNodeID, packetSN;          
    bool bBuffered;

    PHB_Item() {
        bBuffered = false;
    }
} Q2_Item;

typedef struct SPR_Node  // Data structure for node
{
    float nX, nY, nZ;                      // 2017.04.10 fixed bug (從 int 改為 float) 
    int nodeType;
    float currentEnergy, consumedEnergy;   // energy of node
	bool bIsTreePathNode;                  // 是否為 tree 的節點 (是否在 shortest path)

    SPR_Node() {}
    SPR_Node(int x, int y, int z, int t) {
        nX = x;
        nY = y;
        nZ = z;
        nodeType = t;  
        currentEnergy = energy_MAX;    // 初始電量
        consumedEnergy = 0;            // 初始耗電
		bIsTreePathNode = false;
    }
} SPR;

typedef struct Neighbor_Node  // Data structure for recording neighbor node
{
    int nodeID;
    int nodeDepth;
    float nodeDistance;
    float propogationDelay;
    float transmissionDelay;
    float totalDelay;

    Neighbor_Node() {}
    Neighbor_Node(int i, int dep, float dis) {
        nodeID = i;
        nodeDepth = dep;
        nodeDistance = dis;
        propogationDelay = nodeDistance / propogationSpeed;
        transmissionDelay = (float)(packetSize*8) / bitRate;
        totalDelay = propogationDelay + transmissionDelay;
    }
} NeighborNode;

struct Neighbor_Sort_Distance
{
    // sort by distance
    inline bool operator() (const Neighbor_Node& struct1, const Neighbor_Node& struct2)
    {
        return (struct1.nodeDistance < struct2.nodeDistance);
    }
} NeighborSortDistance;

struct Neighbor_Sort_Depth
{
    // sort by Depth
    inline bool operator() (const Neighbor_Node& struct1, const Neighbor_Node& struct2)
    {
        return (struct1.nodeDepth < struct2.nodeDepth);
    }
} NeighborSortDepth;

typedef struct Event_Item  // Data structure for event item
{
    int eventType;
    int occurringTime;
    /*
	   Arrival Event: nodeID表示destination 
       Departure Event: nodeID表示source
    */
    int nodeID; 
	int sourceNodeID;          // 記錄 source node ID 

    SPRpacket packet;

    Event_Item() {}
    Event_Item(int t, int o, int i, int ss, SPRpacket p) {
        eventType = t;
        occurringTime = o;
        nodeID = i;
		sourceNodeID = ss;
        packet = p;
    }
} EventItem;

struct Event_Sort_OccurringTime
{
    // sort by occurring time
    inline bool operator() (const Event_Item& struct1, const Event_Item& struct2)
    {
        return (struct1.occurringTime < struct2.occurringTime);
    }
} EventSortOccurringTime;

struct Event_Sort_PacketSN
{
    // sort by PacketSN
    inline bool operator() (const Event_Item& struct1, const Event_Item& struct2)
    {
        return (struct1.packet.packetSN < struct2.packet.packetSN);
    }
} EventSortPacketSN;

typedef struct Packet_Record_Item  // Packet format for packet record table (最後統計時使用)
{
    int generatedTime, arrivedTime, delay, hopCount;
    bool isArrived;                // 是否到達sink
        
    Packet_Record_Item() {
        generatedTime = INT_MAX;
        arrivedTime = INT_MAX;
        delay = INT_MAX;
        hopCount = 0;             // 記錄 hop count of each packet
        isArrived = false;
    }
} PacketRecordItem;

//-----------------------------------------------------------
// 宣告 1) sensor nodes 2) event queue 3) packet record table
SPR sensorNode[numOfNodes];
vector<EventItem> eventQueue;                                
PacketRecordItem packetRecordTable[numOfPackets];       // data packet table
//-----------------------------------------------------------
//-----------------------------------------------------------
// 宣告 history buffer (for each node)
Q2_Item HistoryBuffer[numOfNodes][numOfPackets];       // history buffer (Q2)
// 記錄node i是否有過去(Q2)的packet Sequence Number
//-----------------------------------------------------------
                                           
int packetSN; 
int numOfArrivals;
int numOfDepartures;
int nPositionUpdated;
int totalCollisions;                   
int sourceSendingTime[numOfSourceNodes];   // unit in ms
int currentTime;                           // unit in ms
bool bEndWithTime;                         // 是否因模擬時間到而結束
int SinkPositionOption;                    // sink positions是預設/手動/自動
int SourcePositionOption;                  // source positions是預設/手動/自動
int executionTime;                         // 模擬程式預計執行結束的時間
int max_num_of_collision_in_one_slot;      // just for debug
int collision_count;                       // just for debug

void printToCheckNeighborNodes();                 
void printToCheckChannelModelValue();             
void collisionPrinting(int nCollisions);   
void endingMessage();                              
void centralizedPreprocessing();
void establishSensorNetworkGraph();
void generateInitialDepartureEventsForSource();
int getNumberOfEventsInTimeslot(int currentTime);
void dataPacketProcessing(int iEvent);     
bool checkPacketArriveSink(int eIndex);
void SPR_PacketForwardingAlgorithm(int eIndex);
void departurePacketProcessing(int iEvent);
void generateArrivalEventsForNeighbors(int eIndex);
vector<NeighborNode> scanNeighbors(int nodeID, int txRange);
bool determineSendingSuccessByModels(float nodeDistance, bool printValue);
bool checkReceivedPacketHeader(EventItem currentEvent);
void removeEvents(int numberOfProcessedEvents);
int removeCollisionEvents(int nEvents);
bool generateNewDepartureEventsForSource(int iNode);
void updateRelayNodePosition();
void calculateStatistics();
float calculateDeliveryRatio();
float calculateEnergyConsumption();
float calculateTotalDelay();

void printPacketHopCount();                              
void printNodeEnergyCurrentTimeslot(int timeslot);
float get_rand(float lower, float upper);
//-----------------------------------------------------------
void idleEnergyConsumption();      
void checkAndGenerateNewDataPacket();        
void checkNodePositionUpdate();    
//-----------------------------------------------------------
float calculateNumOfArrived();
float calculateDelayOfNonArrived();
void updatePriorityQueue(int nodeID, int packetSN, int occuringTime);
void updateHistoryBuffer(int nodeID, int packetSN);
//-----------------------------------------------------------
void closeLogFiles();      
//-----------------------------------------------------------
void initialMessageAndInsert();            
void initializeParameters();               
void initializeNodePosition();          
void setSinkDefaultPositions(); 

//-----------------------------------------------------------
// For Dijkstra algorithm
//-----------------------------------------------------------
void dijkstraPreprocessing();                         
void establishSensorNetworkGraph();                   
void Dijkstra_Algorithm();                            
void dijkstra(float graph[numOfNodes][numOfNodes], int src);
int Dijkstra_MinDistance(float dist[], bool sptSet[]);  
void Dijkstra_InsertPath(int src, int u, int v);        
bool Dijkstra_InsertPathSink(int src, int &numOfPaths);     
void printGeneratedTreeTable_v2();                         // by mickey 

float sensorNetworkGraph[numOfNodes][numOfNodes];          // adjacency matrix (兩節點距離小於 transmissionRangeMax 才有connection)
int shortestPathTreeAll[numOfSourceNodes][numOfNodes];     // hortestPathTreeAll 表示到所有node的 shortest path tree
int shortestPathTreeSink[numOfSourceNodes][numOfNodes];    // shortestPathTreeSink 表示到所有"sink"的 shortest path tree
                                                           // 註: 實際 SPR 是使用 shortestPathTreeSink 做 forwarding 唷 (很重要!!)
//-----------------------------------------------------------