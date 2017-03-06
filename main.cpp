/* 
   Simulation of the shortest path routing (SPR) for Underwater Sensor Networks
*/
//#pragma comment(linker, "/STACK:36777216")
//#pragma comment(linker, "/HEAP:36777216")

#pragma warning(disable:4996)

#include "main.h"

void main()
{
    srand(0); // initialize random seed
              // parameter = 0, 1, 2, ... or time(NULL)
    
    initialMessageAndInsert();          
    initializeNodePosition();
    initializeParameters();             
    printToCheckNeighborNodes();       
	printToCheckChannelModelValue();   

	dijkstraPreprocessing();                     // 第一次建立 shortest path tree (from source to all the "sinks")
    generateInitialDepartureEventsForSource();   

    while(currentTime < executionTime) {
        
        int nEvents = getNumberOfEventsInTimeslot(currentTime);  // nEvents: 在此timeslot裡有幾個events
        fprintf(tFile2, "before: (number of events, time slot): (%d, %d)\n", nEvents, currentTime);
        
        int nCollisions = 0;                                     
        if(collisionEnabled) {
            nCollisions = removeCollisionEvents(nEvents);        // 在此timeslot的collision判斷與處理
        }
        totalCollisions += nCollisions;      
        fprintf(tFile2, "mid: (number of collisions, total collisions, time slot): (%d, %d, %d)\n", nCollisions, totalCollisions, currentTime);
        
		// 除掉collision events後，重新再取一次此timeslot裡有幾個events
        nEvents = getNumberOfEventsInTimeslot(currentTime);
        fprintf(tFile2, "after: (number of events, time slot): (%d, %d)\n", nEvents, currentTime);
        
        collisionPrinting(nCollisions);        // 檢查collisions (just for debugging)

        for(int i=0; i<nEvents; i++) {
            switch(eventQueue[i].eventType) {
                case event_ARRIVAL:
					if(eventQueue[i].packet.messageType == msg_type_DATA) dataPacketProcessing(i);   // data packet 的處理
                    break;
                case event_DEPARTURE:
                    departurePacketProcessing(i);     
                    break;
                default:
                    break;
            }
        }
        removeEvents(nEvents); // 處理完arrival和departure直接remove events

		printNodeEnergyCurrentTimeslot(currentTime);   
        idleEnergyConsumption();                       
        checkAndGenerateNewDataPacket();                   
        checkNodePositionUpdate();  
      
        currentTime += timeslotLength;
    }
    calculateStatistics();                         
    endingMessage();  
    closeLogFiles();  

    system("pause");                                   // 暫停畫面用
}

void initialMessageAndInsert()
{
	printf("WitLab: Simulation of Shortest-Path Routing (SPR) for Underwater Sensor Networks\n");
	printf("Number of nodes (source, sink, relay) = (%d, %d, %d)\n", numOfSourceNodes, numOfSinkNodes, numOfRelayNodes);

	// Dimensions: 進行模擬的海域空間size
	printf("Dimensions = (%d, %d, %d)\n", dimensionX, dimensionY, dimensionZ);

	printf("----------------------------------------------------------------------\n");
	printf("Source positions setting (0:default, 1:manual, 2:random) ");
	scanf("%d", &SourcePositionOption);
	printf("Sink positions setting (0:default, 1:manual, 2:random) ");
	scanf("%d", &SinkPositionOption);
	printf("----------------------------------------------------------------------\n");
}

void initializeParameters()
{
	packetSN = 0;
	numOfArrivals = 0;
	numOfDepartures = 0;
	nPositionUpdated = 0;
	totalCollisions = 0;
	currentTime = 0;
	bEndWithTime = true;
	SinkPositionOption = -1;
	SourcePositionOption = -1;
	executionTime = simulationTime;
	max_num_of_collision_in_one_slot = 0;
	collision_count = 0;

	for (int i = 0; i<numOfNodes; i++) {
		sensorNode[i].consumedEnergy = 0;
		sensorNode[i].currentEnergy = energy_MAX;
	}
}

void printToCheckNeighborNodes()
{
    for (int i = 0; i < numOfNodes; i++) {
        vector<NeighborNode> neighbors = scanNeighbors(i, transmissionRangeMax);
        fprintf(tFile1, "neighbors of node %d : \n", i);
        for (int j = 0; j < neighbors.size(); j++) {
            fprintf(tFile1, "node %d \n", neighbors[j].nodeID);
        }
    }
	fflush(tFile1);
}

void printToCheckChannelModelValue()
{
	for (float d = 100; d <= 1000; d += 100) {
		determineSendingSuccessByModels(d, true);
	}
	fflush(tFile0);
}

void generateInitialDepartureEventsForSource()
{
	// Generate the first packet departure event from the source nodes and push events into queue
	for (int i = 0; i<numOfSourceNodes; i++) {

		// packet format: message type, hop count, packet serial number, sender node ID, source node ID
		SPRpacket newPacket = SPRpacket(msg_type_DATA, 0, packetSN, i, i); 

		// source產生的第一個封包的時間點介於0到packet rate之間，因CBR時間不會超過packet rate
		sourceSendingTime[i] = get_rand(0.0, (float)sourcePacketRate);

		// 記錄packet產生的時間
		packetRecordTable[packetSN].generatedTime = sourceSendingTime[i];

		// 為source node產生第一個封包departure event
		// event format: event type, occurring time, current node ID, source node ID, packet
		EventItem newItem = EventItem(event_DEPARTURE, sourceSendingTime[i], i, i, newPacket);
		eventQueue.push_back(newItem);

		updateHistoryBuffer(i, packetSN);

		packetSN++;
	}
}

bool generateNewDepartureEventsForSource(int iNode)
{
	// 若已到預設的上限numOfPackets，就不再產生，作為「程式終止」的判斷條件
	if (packetSN >= numOfPackets) return false;

	// packet format: message type, hop count, packet serial number, sender node ID, source node ID
	SPRpacket newPacket = SPRpacket(msg_type_DATA, 0, packetSN, iNode, iNode);
	sourceSendingTime[iNode] += sourcePacketRate;
	packetRecordTable[packetSN].generatedTime = sourceSendingTime[iNode];   // 記錄產生的時間，最後統計時使用

	// event format: event type, occurring time, current node ID, source node ID, packet
	EventItem newItem = EventItem(event_DEPARTURE, sourceSendingTime[iNode], iNode, iNode, newPacket);
	eventQueue.push_back(newItem);

	updateHistoryBuffer(iNode, packetSN);

	packetSN++;

	return true;
}

// 2016.11.28: 新增updateHistoryBuffer()
void updateHistoryBuffer(int nodeID, int packetSN)
{
	// if it is the first time that the node sends out the packet
	if (HistoryBuffer[nodeID][packetSN].bBuffered == false) {
		// update history buffer at this node (with this packet)
		HistoryBuffer[nodeID][packetSN].bBuffered = true;
	}
}

int getNumberOfEventsInTimeslot(int currentTime)
{
	// sort items in the event queue
	sort(eventQueue.begin(), eventQueue.end(), EventSortOccurringTime);

	// check the number of events will be processed in this timeslot
	int iQueueSize = eventQueue.size();
	int numOfProcessingEvents = 0;

	// calculate how many events in this timeslot
	for (int i = 0; i<iQueueSize; i++) {
		if (eventQueue[i].occurringTime <= currentTime) {
			numOfProcessingEvents++;
			//eventQueue[i].occurringTime = currentTime;
		}
	}
	return numOfProcessingEvents;
}

int removeCollisionEvents(int nEvents)
{
	bool *bEventErase = (bool*)malloc(nEvents * sizeof(bool));
	for (int i = 0; i<nEvents; i++) {
		bEventErase[i] = false;
	}
	for (int i = 0; i<nEvents; i++) {
		for (int j = i + 1; j<nEvents; j++) {                      // j起始值不能等於0否則i=0,j=0時if將成立  
			if (eventQueue[i].nodeID == eventQueue[j].nodeID &&
				eventQueue[i].eventType == event_ARRIVAL &&
				eventQueue[j].eventType == event_ARRIVAL &&
				eventQueue[i].occurringTime <= currentTime &&
				eventQueue[j].occurringTime <= currentTime) {      // 條件都成立時表示發生collision

				bEventErase[i] = true;
				bEventErase[j] = true;
			}
		}
	}
	int nCollisions = 0;                           // number of collisions
	for (int i = 0; i<nEvents; i++) {
		if (bEventErase[i]) {
			nCollisions++;
			eventQueue[i].occurringTime = -1;      // 設發生collision的events的時間為-1
		}
	}
	sort(eventQueue.begin(), eventQueue.end(), EventSortOccurringTime);
	int iQueueSize = eventQueue.size();
	int tempIndex = -1;

	for (int i = 0; i<iQueueSize-1; i++) {
		if (eventQueue[i].occurringTime == -1 &&
			eventQueue[i+1].occurringTime != -1) {
			tempIndex = i + 1;                     // 要刪除的events數量
			break;
		}
	}
	// 將發生collision的events刪掉
	if (tempIndex != -1) {
		// 一次刪除多個collision events
		eventQueue.erase(eventQueue.begin(), eventQueue.begin() + tempIndex);
	}
	free(bEventErase);

	return nCollisions;
}

void collisionPrinting(int nCollisions)
{
	if (nCollisions > 0)
		collision_count++;                              // 記錄有多少個time slot發生collision
	if (nCollisions > max_num_of_collision_in_one_slot) {
		max_num_of_collision_in_one_slot = nCollisions;
		fprintf(tFile4, "Max # of collisions/slot: %d \n", max_num_of_collision_in_one_slot);
	}
	fflush(tFile4);
}

void dataPacketProcessing(int iEvent)
{
    if (eventQueue[iEvent].packet.packetSN == 0)   // 觀察 packet #1 的走向 (just for debugging)
        fprintf(tFile7, "packet %d (from node %d) arrived at node %d when %d \n", 
		        eventQueue[iEvent].packet.packetSN, eventQueue[iEvent].packet.senderNodeID, 
				eventQueue[iEvent].nodeID, eventQueue[iEvent].occurringTime);

	// 確認此node剩餘電力是否足夠接收該封包，若足夠則接著進行封包的處理
	float remainingEnergy = sensorNode[eventQueue[iEvent].nodeID].currentEnergy;
	if(remainingEnergy >= (powerReceiving - powerIdling)*timeslotLength) {

		// Energy consumption of receiving packet
		sensorNode[eventQueue[iEvent].nodeID].consumedEnergy += (powerReceiving - powerIdling)*timeslotLength;
		sensorNode[eventQueue[iEvent].nodeID].currentEnergy -= (powerReceiving - powerIdling)*timeslotLength;

		// 統計number of arrivals
		numOfArrivals++;

		// 接收封包後hop數增加一次
		eventQueue[iEvent].packet.hop_count++;

		// 若此event不是到達sink的arrival event，則執行SPR的forwarding algorithm
		if(!checkPacketArriveSink(iEvent)) {          
			SPR_PacketForwardingAlgorithm(iEvent);
		}
	}
	else {
		printf("Node[%d]'s energy is exhausted.\n", eventQueue[iEvent].nodeID);
		sensorNode[eventQueue[iEvent].nodeID].currentEnergy = -1;
	}
}

bool checkPacketArriveSink(int eIndex)
{
	if (sensorNode[eventQueue[eIndex].nodeID].nodeType == node_type_SINK) {  // 若為到達sink的arrival event
		int SN = eventQueue[eIndex].packet.packetSN;
		if (!packetRecordTable[SN].isArrived) {                             // 若無記錄表示此編號的封包第一次到sink，則記錄之
			packetRecordTable[SN].isArrived = true;                         // (note:僅記錄最早到達sink的那一次)
			packetRecordTable[SN].arrivedTime = eventQueue[eIndex].occurringTime;
			packetRecordTable[SN].delay = packetRecordTable[SN].arrivedTime - packetRecordTable[SN].generatedTime;
			packetRecordTable[SN].hopCount = eventQueue[eIndex].packet.hop_count;  

			// note: 也可以print"generated time"和"arrived time"等檢查
			fprintf(tFile3, "Packet %d arrived node %d\n", eventQueue[eIndex].packet.packetSN, eventQueue[eIndex].nodeID);
		}
		return true;
	}
	else return false;
}

void SPR_PacketForwardingAlgorithm(int eIndex)
{
	// 檢查packet內容決定此packet要被forward或drop
	bool bDropPacket = checkReceivedPacketHeader(eventQueue[eIndex]); //檢查 header (重要!)
	if (!bDropPacket) {  // 若通過檢查(非drop)

		// packet format: message type, hop count, packet serial number, sender node ID, source node ID
		SPRpacket newPacket = SPRpacket(msg_type_DATA, eventQueue[eIndex].packet.hop_count, 
			                            eventQueue[eIndex].packet.packetSN, eventQueue[eIndex].nodeID, 
						     			eventQueue[eIndex].packet.sourceNodeID);

		int HT = rand()%10 + 1;        // holding time 為 random 1~10 個 time slots 
		int ST = currentTime + HT;     // sending time

		// 根據sending time ST產生departure event
		// event format: event type, occurring time, current node ID, source node ID, packet
		EventItem newItem = EventItem(event_DEPARTURE, ST, eventQueue[eIndex].nodeID, eventQueue[eIndex].sourceNodeID, newPacket);
		eventQueue.push_back(newItem);

		// record the packet in history buffer
		updateHistoryBuffer(eventQueue[eIndex].nodeID, eventQueue[eIndex].packet.packetSN);

		if (newPacket.packetSN == 10) // just for debugging
			fprintf(tFile5, "Departure Event: sending time, from which node, sequence number: (%d, %d, %d) \n", ST, eventQueue[eIndex].nodeID, newPacket.packetSN);
	}
}

bool checkReceivedPacketHeader(EventItem currentEvent)
{
	// 1.若非routing tree 上的node則回傳true --> drop packet (不在 shortestPathTreeSink 上, 就 drop)
	if(!sensorNode[currentEvent.nodeID].bIsTreePathNode) return true;

	// 2.若sender非nodeID的parent則回傳true --> drop packet (只有收到來自 parent 的 packet 才做 forward, 是檢查 shortestPathTreeSink, 不是 shortestPathTreeAll 唷, 很重要!)
	if(currentEvent.packet.senderNodeID != 
	   shortestPathTreeSink[currentEvent.sourceNodeID][currentEvent.nodeID]) return true; 
	
	// 3.已轉送此封包過則回傳true --> drop packet (檢查 HistoryBuffer 避免重覆送同樣的封包 浪費資源)
	if(HistoryBuffer[currentEvent.nodeID][currentEvent.packet.packetSN].bBuffered) return true;

	return false;
}

void departurePacketProcessing(int iEvent)
{
	if (eventQueue[iEvent].packet.packetSN == 1)      // 觀察packet #1的走向 (just for debugging)
		fprintf(tFile1, "packet %d (from node %d) departured at node %d when %d \n", eventQueue[iEvent].packet.packetSN, eventQueue[iEvent].packet.senderNodeID, eventQueue[iEvent].nodeID, eventQueue[iEvent].occurringTime);

	// check the remaining energy of node
	float remainingEnergy = sensorNode[eventQueue[iEvent].nodeID].currentEnergy;
	if (remainingEnergy >= (powerSending - powerIdling)*timeslotLength) {

		// Energy consumption of forwarding packet
		sensorNode[eventQueue[iEvent].nodeID].consumedEnergy += (powerSending - powerIdling)*timeslotLength;   
		sensorNode[eventQueue[iEvent].nodeID].currentEnergy -= (powerSending - powerIdling)*timeslotLength;   

		// 統計number of departures
		numOfDepartures++;

		// departure event發生，產生鄰近節點的arrival events
		generateArrivalEventsForNeighbors(iEvent);
	}
	else {
		printf("Node[%d]'s energy is exhausted.\n", eventQueue[iEvent].nodeID);
		sensorNode[eventQueue[iEvent].nodeID].currentEnergy = -1;
	}
}

void generateArrivalEventsForNeighbors(int eIndex)
{
	// 在傳輸半徑內的節點為鄰近節點
	vector<NeighborNode> neighbors = scanNeighbors(eventQueue[eIndex].nodeID, transmissionRangeMax);
	int iNeighborSize = neighbors.size();
	for (int n = 0; n<iNeighborSize; n++) {
		// check sending success by the probability from the channel model
		bool sendingSuccess = determineSendingSuccessByModels(neighbors[n].nodeDistance, false);
		if (sendingSuccess) {
			// generate arrival events of successful sending for neighbor nodes
			int delay = neighbors[n].propogationDelay + neighbors[n].transmissionDelay;
			int AT = currentTime + delay;

			// 根據arrival time AT產生鄰近節點的arrival event
			SPRpacket newPacket;
			if(eventQueue[eIndex].packet.messageType == msg_type_DATA) {
				// packet format: message type, hop count, packet serial number, sender node ID, source node ID
				newPacket = SPRpacket(msg_type_DATA, eventQueue[eIndex].packet.hop_count, eventQueue[eIndex].packet.packetSN, 
					                  eventQueue[eIndex].nodeID, eventQueue[eIndex].packet.sourceNodeID); 
			}
			// event format: event type, occurring time, current node ID, source node ID, packet
			EventItem newItem = EventItem(event_ARRIVAL, AT, neighbors[n].nodeID, eventQueue[eIndex].packet.sourceNodeID, newPacket);
			eventQueue.push_back(newItem);

			if (newPacket.packetSN == 10) // just for debugging
				fprintf(tFile6, "Arrival Event: arrival time, from which node, to which node, sequence number: (%d, %d, %d, %d) \n", AT, eventQueue[eIndex].nodeID, neighbors[n].nodeID, newPacket.packetSN);
		}
	}
}

vector<NeighborNode> scanNeighbors(int nodeID, int txRange)
{
    // index: node serial number
    vector<NeighborNode> neighborTable;
    SPR tempNode = sensorNode[nodeID];

    for (int n = 0; n<numOfNodes; n++) {
        if(n == nodeID) continue;    // 若為本身則跳過並執行下一輪迴圈
        int d2_X = pow(abs(tempNode.nX - sensorNode[n].nX), 2.0);
        int d2_Y = pow(abs(tempNode.nY - sensorNode[n].nY), 2.0);
        int d2_Z = pow(abs(tempNode.nZ - sensorNode[n].nZ), 2.0);
        int d2 = d2_X + d2_Y + d2_Z; // x^2 + y^2 + z^2
        int R2 = pow(txRange, 2.0);
        float d = pow(d2, 0.5);      // 開根號

        // if the node is within the transmission range, push it as item in table 
        if (d2 <= R2) {
            neighborTable.push_back(NeighborNode(n, sensorNode[n].nZ, d));
        }
    }
    return neighborTable;
}

// ------------------------------------------------------------------
void removeEvents(int numberOfProcessedEvents)
{
	// erase the events that already processed
	if (numberOfProcessedEvents == 0) return;
	else eventQueue.erase(eventQueue.begin(), eventQueue.begin() + numberOfProcessedEvents);
}
// ------------------------------------------------------------------

void printNodeEnergyCurrentTimeslot(int timeslot)
{
	fprintf(tFile9, "timeslot = %d\n", timeslot);
	for (int i = 0; i<numOfNodes; i++) {
		fprintf(tFile9, "Node %d consumed energy = %e, current energy = %e\n", i, sensorNode[i].consumedEnergy, sensorNode[i].currentEnergy);
	}
	fflush(tFile9);
}

void idleEnergyConsumption()
{
	// Energy consumption of node idling
	for (int i = 0; i<numOfNodes; i++) {
		float remainingEnergy = sensorNode[i].currentEnergy;
		if (remainingEnergy > (powerIdling*timeslotLength)) {
			sensorNode[i].consumedEnergy += (powerIdling*timeslotLength);   
			sensorNode[i].currentEnergy -= (powerIdling*timeslotLength);    
		}
		else {
			printf("Node[%d]'s energy exhausted.\n", i);
			sensorNode[i].currentEnergy = -1;
		}
	}
}

void checkAndGenerateNewDataPacket()
{
	if (!bEndWithTime) return;

	// check every source node which should generate a new data packet
	for (int i = 0; i<numOfSourceNodes; i++) {
		if (sourceSendingTime[i] + sourcePacketRate <= currentTime) {    
			if (!generateNewDepartureEventsForSource(i)) {
				// 從source node產生新的departure event，只有在「封包數量到達上限」時才回傳false，否則回傳true
				// 即只有在最後一個封包送出時，才會進入
				bEndWithTime = false;
				executionTime = currentTime + extendedTime;
			}
		}
	}
}

void checkNodePositionUpdate()
{
	// update node positions periodically
	if (currentTime == (nPositionUpdated * positionUpdateSlot)) {
		updateRelayNodePosition();
		nPositionUpdated++;      // 第n次update nodes' position
	}
}

void updateRelayNodePosition()   // 各方向按照速率 random 移動
{
	// 註: 假設sink和source nodes都不會移動 
    for(int n = numOfSinkNodes + numOfSourceNodes; n<numOfNodes; n++) {

        // calculate node move distance based on the move speed
        float speed_x = get_rand(nodeSpeedMin, nodeSpeedMax);
        float speed_y = get_rand(nodeSpeedMin, nodeSpeedMax);
        float speed_z = get_rand(nodeSpeedMin, nodeSpeedMax);
        float detX = speed_x * positionUpdateSlot;
        float detY = speed_y * positionUpdateSlot;
        float detZ = speed_z * positionUpdateSlot;

        // update the node position
        bool bPositiveDirection = rand() % 2;  // 先決定方向是"正"或"負"
        if(bPositiveDirection) {
            if(sensorNode[n].nX + detX <= dimensionX) sensorNode[n].nX += detX;
            else sensorNode[n].nX = dimensionX;
        }
        else {
            if(sensorNode[n].nX - detX > 0) sensorNode[n].nX -= detX;
            else sensorNode[n].nX = 0;
        }

        bPositiveDirection = rand() % 2;
        if(bPositiveDirection) {
            if(sensorNode[n].nY + detY <= dimensionY) sensorNode[n].nY += detY;
            else sensorNode[n].nY = dimensionY;
        }
        else {
            if(sensorNode[n].nY - detY > 0) sensorNode[n].nY -= detY;
            else sensorNode[n].nY = 0;
        }

        bPositiveDirection = rand() % 2;
        if(bPositiveDirection) {
            if(sensorNode[n].nZ + detZ <= dimensionZ) sensorNode[n].nZ += detZ;
            else sensorNode[n].nZ = dimensionZ;
        }
        else {
            if(sensorNode[n].nZ - detZ > 0) sensorNode[n].nZ -= detZ;
            else sensorNode[n].nZ = 0;
        }
    }
}

void calculateStatistics()
{
	printf("[Data packet]\n");
    printf("Number of arrived packets = %f\n", calculateNumOfArrived());
    printf("Delivery ratio = %f\n", calculateDeliveryRatio());
	printf("Total delay of arrived packets = %f (ms)\n", calculateTotalDelay());
	printf("Total Collisions = %d (times)\n", totalCollisions);
	printf("Delay of non-arrived packets = %f\n", calculateDelayOfNonArrived());
	printf("Total delay of both kinds of packets = %f\n", calculateTotalDelay() + calculateDelayOfNonArrived());
	printf("Average delay of both kinds of packets = %f\n", (calculateTotalDelay() + calculateDelayOfNonArrived()) / calculateNumOfArrived());
	printPacketHopCount();
	printf("------------------------\n");
	printf("Energy consumption = %f (J)\n", calculateEnergyConsumption());

    fprintf(tFile4, "Total Collisions = %d (times)\n", totalCollisions);
    fprintf(tFile4, "number of time slots that Collisions happen = %d (times)\n", collision_count);
	fflush(tFile4);
}

float calculateNumOfArrived()
{
    int numOfArrived = 0;
    for (int n = 0; n<numOfReferredPackets; n++) {
        if (packetRecordTable[n].isArrived) numOfArrived++;
    }
    return (float)numOfArrived;
}

float calculateDeliveryRatio()
{
    int numOfDelivery = 0;
    for(int n=0; n<numOfReferredPackets; n++) {
        if(packetRecordTable[n].isArrived) numOfDelivery++;
    }
    return (float)numOfDelivery/numOfReferredPackets;
}

float calculateEnergyConsumption()
{
    float idleConsumptionRelayNodes = numOfRelayNodes*powerIdling*executionTime; 
	printf("idleConsumptionRelayNodes = %f (J)\n", idleConsumptionRelayNodes);
    
	float idleConsumptionAllNodes = numOfNodes*powerIdling*executionTime; 
	printf("idleConsumptionAllNodes = %f (J)\n", idleConsumptionAllNodes);

    float sendingConsumption = numOfDepartures*(powerSending - powerIdling)*timeslotLength;
    float receivingConsumption = numOfArrivals*(powerReceiving - powerIdling)*timeslotLength;

    return (idleConsumptionRelayNodes + sendingConsumption + receivingConsumption);
}

float calculateTotalDelay()
{
    float totalDelay = 0;
    for(int d=0; d<numOfReferredPackets; d++) {
        if(packetRecordTable[d].delay != INT_MAX) {
            totalDelay += packetRecordTable[d].delay;
        }
    }  
    return totalDelay;
}

float calculateDelayOfNonArrived()
{
    int totalDelayofNonArrived = 0;

    for (int n = 0; n<numOfReferredPackets; n++) {
        if (!packetRecordTable[n].isArrived) {
            // method 1 (not for short simulation time)
            totalDelayofNonArrived += (currentTime - packetRecordTable[n].generatedTime); 
        }
    }
    return (float)totalDelayofNonArrived;
}

void printPacketHopCount()
{
	for (int i = 0; i<numOfPackets; i++) {
		fprintf(tFile8, "Packet %d hop count = %d\n", i, packetRecordTable[i].hopCount);
	}
	fflush(tFile8);
}

void endingMessage()
{
	printf("executionTime = %d (ms)\n", executionTime);
	printf("currentTime = %d (ms)\n", currentTime);

	if (bEndWithTime) printf("Simulation ended with time.\n");
	else printf("Simulation ended with the last packet.\n");
}

// close log files~!!
void closeLogFiles()
{
	fclose(tFile0);         // 若有開檔 記得關檔 (麻煩幫DBR版本補上)
	fclose(tFile1);
	fclose(tFile2);
	fclose(tFile3);
	fclose(tFile4);
	fclose(tFile5);
	fclose(tFile6);
	fclose(tFile7);
	fclose(tFile8);
	fclose(tFile9);
	fclose(tFile10);
	fclose(tFile11);
	fclose(tFile12);
}

// ------------------------------------------------------------------
// 隨機產生介於lower和upper之間的值
// ------------------------------------------------------------------
float get_rand(float lower, float upper)
{
    return rand() * (upper-lower) / RAND_MAX + lower;
}

//--------------------------------------------
// 因為 position function 較長, 所以放在較後面
//--------------------------------------------
void initializeNodePosition()
{
	int n = 0;
	for (; n<numOfSourceNodes; n++) {                   // for source nodes
		if (SourcePositionOption == 0) {                // case 0: default setting
			if (numOfSourceNodes == 1){
				sensorNode[0].nX = (dimensionX / 2);
				sensorNode[0].nY = (dimensionY / 2);
			}
			/*else if (numOfSourceNodes == 2){
			sensorNode[0].nX = (dimensionX / 2);
			sensorNode[0].nY = (dimensionY / 2);
			sensorNode[1].nX = (dimensionX / 4);
			sensorNode[1].nY = (dimensionY / 4);
			}*/
			// 以此可類推到 numOfSourceNodes == 9 (目前先用1個source node即可)
		}
		else if (SourcePositionOption == 1) {           // case 1: manual input
			int sourceID = n;
			printf("Enter the position X of source node %d = ", sourceID);
			scanf("%d", &sensorNode[n].nX);
			printf("Enter the position Y of source node %d = ", sourceID);
			scanf("%d", &sensorNode[n].nY);
		}
		else if (SourcePositionOption == 2){            // case 2: random
			sensorNode[n].nX = rand() % (dimensionX + 1);
			sensorNode[n].nY = rand() % (dimensionY + 1);
		}
		sensorNode[n].nZ = dimensionZ;                  // bottom of the sea
		sensorNode[n].nodeType = node_type_SOURCE;

		fprintf(tFile10, "source node id_%d = (%d,%d,%d)\n", n, sensorNode[n].nX, sensorNode[n].nY, sensorNode[n].nZ);
	}
	for (; n<numOfSourceNodes + numOfSinkNodes; n++) {     // for sink nodes
		if (SinkPositionOption == 0) {                  // case 0: default setting
			setSinkDefaultPositions();
		}
		else if (SinkPositionOption == 1) {              // case 1: manual input
			int sinkID = n - numOfSourceNodes;
			printf("Enter the position X of sink node %d = ", sinkID);
			scanf("%d", &sensorNode[n].nX);
			printf("Enter the position Y of sink node %d = ", sinkID);
			scanf("%d", &sensorNode[n].nY);
		}
		else if (SinkPositionOption == 2){               // case 2: random
			sensorNode[n].nX = rand() % (dimensionX + 1);
			sensorNode[n].nY = rand() % (dimensionY + 1);
		}
		sensorNode[n].nZ = 0;                           // surface of the sea
		sensorNode[n].nodeType = node_type_SINK;

		fprintf(tFile10, "sink node id_%d = (%d,%d,%d)\n", n, sensorNode[n].nX, sensorNode[n].nY, sensorNode[n].nZ);
	}
	for (; n < numOfSourceNodes + numOfSinkNodes + numOfRelayNodes; n++) {    // for remaining relay nodes
		sensorNode[n].nX = rand() % (dimensionX + 1);
		sensorNode[n].nY = rand() % (dimensionY + 1);
		sensorNode[n].nZ = rand() % (dimensionZ + 1);
		sensorNode[n].nodeType = node_type_RELAY;

		fprintf(tFile10, "relay node id_%d = (%d,%d,%d)\n", n, sensorNode[n].nX, sensorNode[n].nY, sensorNode[n].nZ);
	}
	nPositionUpdated++;
}

void setSinkDefaultPositions()     // by chu 
{                                  // by jerry wu
	if (numOfSinkNodes == 1){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
	}
	else if (numOfSinkNodes == 2){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
	}
	else if (numOfSinkNodes == 3){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
	}
	else if (numOfSinkNodes == 4){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 3].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 3].nY = (dimensionY / 4);
	}
	else if (numOfSinkNodes == 5){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 3].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 3].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 4].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 4].nY = 3 * (dimensionY / 4);
	}
	else if (numOfSinkNodes == 6){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 3].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 3].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 4].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 4].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 5].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 5].nY = 3 * (dimensionY / 4);
	}
	else if (numOfSinkNodes == 7){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 3].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 3].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 4].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 4].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 5].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 5].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 6].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 6].nY = (dimensionY / 4);
	}
	else if (numOfSinkNodes == 8){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 3].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 3].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 4].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 4].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 5].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 5].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 6].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 6].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 7].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 7].nY = (dimensionY / 4);
	}
	else if (numOfSinkNodes == 9){
		sensorNode[numOfSourceNodes].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 1].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 1].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 2].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 2].nY = (dimensionY / 2);
		sensorNode[numOfSourceNodes + 3].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 3].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 4].nX = (dimensionX / 2);
		sensorNode[numOfSourceNodes + 4].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 5].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 5].nY = 3 * (dimensionY / 4);
		sensorNode[numOfSourceNodes + 6].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 6].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 7].nX = 3 * (dimensionX / 4);
		sensorNode[numOfSourceNodes + 7].nY = (dimensionY / 4);
		sensorNode[numOfSourceNodes + 8].nX = (dimensionX / 4);
		sensorNode[numOfSourceNodes + 8].nY = 3 * (dimensionY / 4);
	}
}

//----------------------------------------------------
// channel model為獨立的一部分(重要)
//----------------------------------------------------
bool determineSendingSuccessByModels(float nodeDistance, bool printValue)
{
	// powerSending (unit: J/ms) ---> transmitter_power (unit: J/s)
	float transmitter_power = powerSending * 1000;

	// frequency = 10kHz
	float f = 10;

	// a(f): absorption coefficient in dB/km
	float a_f = 0.11*pow(f, 2) / (1 + pow(f, 2)) + 44 * pow(f, 2) / (4100 + pow(f, 2)) + 2.75*pow(10.0, -4)*pow(f, 2) + 0.003; // 吸收係數

	// node distance, radius of an imaginary sphere
	float d = nodeDistance;
	float r = d;

	// source level, transmission loss, noise level, directivity index
	float T_loss = 20 * log10(d) + a_f*d*pow(10.0, -3);
	float S_level = 10 * (log10(transmitter_power) - log10(4 * M_PI*pow(r, 2)) - log10(0.67*pow(10.0, -18)));
	float N_level = 50 - 18 * log10(f);
	float D_index = 0;

	// Signal to Noise Ratio (SNR)
	float gamma = S_level - T_loss - N_level + D_index;

	// Bit Error Rate (BER)
	float BER = 0.5*(1 - sqrt(pow(10, (float)gamma / 10) / (1 + pow(10, (float)gamma / 10))));
	if (printValue) {
		fprintf(tFile0, "transmitter_power = %f, nodeDistance = %f, BER = %e\n", transmitter_power, nodeDistance, BER);
	}

	// successProbability ~= 50% (1W, 300m, 49.5% / 2W, 350m, 51.6%)
	float successProbability = pow((float)1 - BER, (float)packetSize * 8);

	// 隨機取0~1之間的值
	float _rand = get_rand(0.0, 1.0);

	if (_rand <= successProbability)
		return true;             // 傳輸成功
	else
		return false;            // 傳輸失敗
}

// ------------------------------------------------------------------
// For Dijkstra use
// [Reference] http://www.geeksforgeeks.org/greedy-algorithms-set-6-dijkstras-shortest-path-algorithm/
// ------------------------------------------------------------------
void dijkstraPreprocessing()
{
	establishSensorNetworkGraph(); // 先建出 adjacency matrix (兩節點距離小於 transmissionRangeMax 才有connection)
	Dijkstra_Algorithm();          // 先後建出 1) shortestPathTreeAll 和 2) shortestPathTreeSink (註:shortestPathTreeSink才是SPR用的tree)
	printGeneratedTreeTable_v2();  // 印出 tree 於 log 檔檢查
}

// modified by YS
// generate a graph G(V,E) to represent the sensor network
// G(V,E): V is a set of sensor nodes, E is a set of the distance between nodes
// 注意: 距離小於 transmissionRangeMax 才有 edge (E)
void establishSensorNetworkGraph()
{
	for (int i = 0; i<numOfNodes; i++) {
		sensorNode[i].bIsTreePathNode = false;    // 初始:所有的節點都還不是tree上的node
		for (int j = 0; j<numOfNodes; j++) {
			sensorNetworkGraph[i][j] = 0.0;       // 註: 0.0 表示沒有 edge 唷
		}
	}
	for (int i = 0; i<numOfNodes; i++) {
		for (int j = 0; j<numOfNodes; j++) {
			if (sensorNetworkGraph[i][j] == 0.0) {
				float xSquare = pow((float)abs(sensorNode[i].nX - sensorNode[j].nX), (float)2.0);
				float ySquare = pow((float)abs(sensorNode[i].nY - sensorNode[j].nY), (float)2.0);
				float zSquare = pow((float)abs(sensorNode[i].nZ - sensorNode[j].nZ), (float)2.0);
				float distance = pow((float)(xSquare + ySquare + zSquare), (float)0.5);

				if (distance <= transmissionRangeMax) { //距離小於 transmissionRangeMax 才有 edge (E)
					sensorNetworkGraph[i][j] = distance;
					sensorNetworkGraph[j][i] = distance;
				}
			}
		}
	}
}

// modified by YS
// run the Dijkstra algorithm and construct the shortest path tree for each source node
void Dijkstra_Algorithm()
{
	for (int i = 0; i<numOfSourceNodes; i++) {
		for (int j = 0; j<numOfNodes; j++) {
			shortestPathTreeAll[i][j] = -1;       
			// shortestPathTreeAll 表示 "到所有node" 的shortest path tree, 初始所有的"parent"都設為 -1
			shortestPathTreeSink[i][j] = -1;      
			// shortestPathTreeSink 表示 "到所有sink" 的 shortest path tree, 初始所有的"parent"都設為 -1
			// 註: 實際 SPR 是使用 shortestPathTreeSink 做 forwarding 唷 (很重要!!)
		}
	}
	for (int i = 0; i<numOfSourceNodes; i++) {
		dijkstra(sensorNetworkGraph, i);
	}
}

// Function that implements Dijkstra's single source shortest path algorithm for a graph using adjacency matrix 
void dijkstra(float graph[numOfNodes][numOfNodes], int src)
{
	float dist[numOfNodes]; // The output array. dist[i] 表示從 node i 到 source 的最短距離
	bool sptSet[numOfNodes]; // sptSet[i] 表示 node i 是否在 shortest path tree 上

	for (int i = 0; i < numOfNodes; i++) // Initialize all distances as INFINITE and stpSet[] as false
	{
		dist[i] = FLT_MAX; //初始: 每一個 node i 到 source 的距離 一開始都設為"無限大"
		sptSet[i] = false; //初始: 每一個 node i 一開始都不在 shortest path tree 上
	}

	// Distance of source vertex from itself is always 0
	dist[src] = 0.0;

	// Find shortest path for all vertices
	for (int count = 0; count < numOfNodes - 1; count++)
	{
		// Pick the minimum distance vertex from the set of vertices not yet processed. u is always equal to src in first iteration.
		int u = Dijkstra_MinDistance(dist, sptSet);

		// Mark the picked vertex as processed
		sptSet[u] = true;

		// Update dist value of the adjacent vertices of the picked vertex.
		for (int v = 0; v < numOfNodes; v++)
			// Update dist[v] only if is not in sptSet, there is an edge from u to v, and total weight of path from src to v through u is smaller than current value of dist[v]
			if (!sptSet[v] && graph[u][v] && dist[u] != FLT_MAX && dist[u] + graph[u][v] < dist[v]) {
					
				dist[v] = dist[u] + graph[u][v]; // update dist[v]
				Dijkstra_InsertPath(src, u, v);  // modified by YS
		}
	}

	int numOfPaths = 0;
	if (!Dijkstra_InsertPathSink(src, numOfPaths)) {
		printf("No path between the source node %d and sinks.\n", src);
	}
	printf("Totally %d paths between the source node %d and sinks.\n", numOfPaths, src);
	// Dijkstra_InsertPathSink() 是根據 shortestPathTreeAll 重新建出 shortestPathTreeSink (此tree僅包含到達所有sinks的paths)
}

// A utility function to find the vertex with minimum distance value, from the set of vertices not yet included in shortest path tree
int Dijkstra_MinDistance(float dist[], bool sptSet[])
{
	float min = FLT_MAX; // Initialize min value
	int min_index;

	for (int v = 0; v < numOfNodes; v++)
		if (sptSet[v] == false && dist[v] <= min)
			min = dist[v], min_index = v;

	return min_index;
}

// modified by YS, insert the path to construct the shortest path tree (source to all nodes)
void Dijkstra_InsertPath(int src, int u, int v)
{
	// src: source node ID, v: current vertex, u: the parent of v
	shortestPathTreeAll[src][v] = u;
}

// modified by YS, insert the path to construct the shortest path tree (source to all sinks) by reverse tracing
bool Dijkstra_InsertPathSink(int src, int &numOfPaths)
{
	bool bPathCompleted = false;                    // 判斷path是否完成(是否有path可到任一sink)
	sensorNode[src].bIsTreePathNode = true;         // 標記src在SPR的tree上
	numOfPaths = 0;

	// for each sink
	for (int s = numOfSourceNodes; s<numOfSourceNodes + numOfSinkNodes; s++) {
		int v = s;
		while (v != src) {
			// src: source node ID, v: current vertex, u: the parent of v
			int u = shortestPathTreeAll[src][v];
			
			// No path between the source node and the sink node (break from the while loop)
			if (u < 0) break;                        
			
			// 標記node v在tree上 (shortestPathTreeSink為packet forwarding參照tree)
			shortestPathTreeSink[src][v] = u;
			sensorNode[v].bIsTreePathNode = true;    
			v = u;
		}

		if (v == src) {                             // 若迴圈結束後v指到src (即sink與source node間有path)       
			bPathCompleted = true;                  // bPathCompleted flag設為true (只要存在path到任一sink即為true)
			numOfPaths++;                           // 累計number of path
		}
	}
	return bPathCompleted;
}

// A utility function to print the constructed tree, version_2, by mickey
void printGeneratedTreeTable_v2()
{
	for (int i = 0; i<numOfSourceNodes; i++) {
		fprintf(tFile3, "src = %d\n", i);
		fprintf(tFile7, "src = %d\n", i);
		for (int j = 0; j<numOfNodes; j++) {
			fprintf(tFile3, "%d ", shortestPathTreeAll[i][j]);  // 印出 shortestPathTreeAll 整個 array (即到所有node的tree)
			fprintf(tFile7, "%d ", shortestPathTreeSink[i][j]); // 印出 shortestPathTreeSink 整個 array (即到所有sink的tree)
		}
		fprintf(tFile3, "\n");
		fprintf(tFile7, "\n");
	}

	// 印出 shortestPathTreeSink 的 paths
	int temp_1;
	fprintf(tFile7, "tree-begin (shortestPathTreeSink)\n");
	for (int i = 0; i < numOfSourceNodes; i++) {
		for (int j = numOfSourceNodes; j < (numOfSourceNodes + numOfSinkNodes); j++){ // for 每一個 sink
			temp_1 = j; // 用 temp 避免改變到 "j" (會影響for迴圈)
			fprintf(tFile7, "Path: %d -> ", temp_1); //起點 (即sink)
			while (shortestPathTreeSink[i][temp_1] != i && shortestPathTreeSink[i][temp_1] != -1){ // temp_1 的 parent 不是 source 也不是 -1
				fprintf(tFile7, "%d ->", shortestPathTreeSink[i][temp_1]); //把 temp_1 的 parent 印出來
				temp_1 = shortestPathTreeSink[i][temp_1]; // temp_1 移動到自己的 parent
			}
			fprintf(tFile7, "%d (end of path) \n", shortestPathTreeSink[i][temp_1]); //把 temp_1 的 parent 印出來 (終點)
		}
	}
	fprintf(tFile7, "tree-end (shortestPathTreeSink)\n");

	// 印出 shortestPathTreeAll 的 paths
	int temp_2;
	fprintf(tFile3, "tree-begin (shortestPathTreeAll)\n");
	for (int i = 0; i < numOfSourceNodes; i++) {
		for (int j = 0; j < numOfNodes; j++){ // for 每一個 node
			temp_2 = j; // 用 temp 避免改變到 "j" (會影響for迴圈)
			fprintf(tFile3, "Path: %d -> ", temp_2); //起點 
			while (shortestPathTreeAll [i][temp_2] != i && shortestPathTreeAll[i][temp_2] != -1){ // temp_2 的 parent 不是 source 也不是 -1
				fprintf(tFile3, "%d ->", shortestPathTreeAll[i][temp_2]); //把 temp_2 的 parent 印出來
				temp_2 = shortestPathTreeAll[i][temp_2]; // temp_2 移動到自己的 parent
			}
			fprintf(tFile3, "%d (end of path) \n", shortestPathTreeAll[i][temp_2]); //把 temp_2 的 parent 印出來
		}
	}
	fprintf(tFile3, "tree-end (shortestPathTreeAll)\n");
	
	fflush(tFile3);
	fflush(tFile7);
}