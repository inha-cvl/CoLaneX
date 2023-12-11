
/* include */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include "sys/time.h"
#include "time.h"
#include <signal.h>
#include <pthread.h>
#include <iostream>
#include <vector>

/* include for v2x */
#include "v2x_defs.h"
#include "v2x_ext_type.h"
#include "db_v2x.h"
#include "math.h"

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "novatel_oem7_msgs/INSPVA.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "visualization_msgs/Marker.h"

#include <json-c/json.h>

#define SAMPLE_V2X_API_VER 0x0001
#define SAMPLE_V2X_IP_ADDR "192.168.1.11"
#define SAMPLE_LOCAL "127.0.0.1"

#define SAMPLE_V2X_PORT_ADDR 47347
#define MAX_UPER_SIZE  4096

volatile bool is_running = true;

/* Global Variable Value */
V2xAction_t e_action_g = eV2xAction_ADD;
V2xPayloadType_t e_payload_type_g = eRaw;
V2xPsid_t psid_g = 5271;
V2XCommType_t e_comm_type_g = eV2XCommType_5GNRV2X;
V2xPowerDbm_t tx_power_g = 20;
V2xSignerId_t e_signer_id_g = eV2xSignerId_UNSECURED;
V2xMsgPriority_t e_priority_g = eV2xPriority_CV2X_PPPP_0;
uint32_t tx_cnt_g = 1000000;
uint32_t tx_delay_g = 100;
V2xFrequency_t freq_g = 5900;
V2xDataRate_t e_data_rate_g = eV2xDataRate_6MBPS;
V2xTimeSlot_t e_time_slot_g = eV2xTimeSlot_Continuous;
uint8_t peer_mac_addr_g[MAC_EUI48_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t transmitter_profile_id_g = 100;
uint32_t peer_l2id_g = 0;

uint32_t delay_time_sec_g = 100000;

int sock_g = -1;

std_msgs::Float32MultiArray hlv_system;
ros::Publisher pub_hlv_system;
geometry_msgs::Pose tlv_pose;
ros::Publisher pub_tlv_pose;
visualization_msgs::Marker tlv_path;
ros::Publisher pub_tlv_path;

unsigned long state;
unsigned long _signal;
unsigned long latitude;
unsigned long longitude;
unsigned long heading;
unsigned long velocity; //mps

std::vector<std::pair<double, double>> path;
std::vector<std::pair<int, time_t>> ts_list;
int rx_msg_cnt;
int path_len;
bool show_result = true;
bool same_machine = false;
int hz = 5;
int tx_packet_count = 0;
int rx_packet_count = 0;

int connect_v2x_socket(void);
void close_v2x_socket(void);
int v2x_wsr_cmd_process(void);
void *v2x_tx_cmd_process(void *);
void *v2x_rx_cmd_process(void *);
int process_commands(void);
void pubTLV(long, long, long, long, long, std::vector<std::pair<double, double>>);
void* check_packet_rates(void *);
void check_rtt(int);
void check_distance(long, long);

void poseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
	latitude = msg->position.x * pow(10, 7);
 	longitude = msg->position.y * pow(10, 7);
	if(latitude != 0 && longitude !=0 ){
		hlv_system.data[1] = 1;
	}
	//Input ) degree
	//BSM->heading : LSB of 0.0125 degrees (A range of 0 to 359.9875 degrees)
	int _heading = (msg->position.z <= 0 && msg->position.z >= -180) ? msg->position.z + 360 : msg->position.z;
 	heading = int(_heading / 0.0125);
	//Input ) m/s
	//BSM->velocity : integer, Units of 0.02 m/s
	velocity = msg->orientation.x / 0.02;
}

void pathCallback(const visualization_msgs::Marker::ConstPtr &msg){
	path.clear();
	path_len = atoi(msg->text.c_str());
	for (size_t i = 0; i < path_len; i++)
	{

		path.push_back(std::make_pair(msg->points[i].x, msg->points[i].y));
	}
}

void stateCallback(const std_msgs::Int8::ConstPtr &msg){
	state = msg->data;
}

void signalCallback(const std_msgs::Int8::ConstPtr &msg){
	_signal = msg->data;
}

void sigint_handler(int sig) {
    is_running = false; // 스레드 종료 플래그 설정
}

/* function : Main(Entry point of this program) */
int main(int argc, char *argv[])
{
	signal(SIGINT, sigint_handler);
	int res;
	ros::init(argc,argv, "HLV_V2X");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);

	ros::Subscriber sub_pose = n.subscribe("/car/hlv_pose", 100, poseCallback);
	ros::Subscriber sub_path = n.subscribe("/planning/hlv_path", 100, pathCallback);
	ros::Subscriber sub_state = n.subscribe("/hlv_state", 100, stateCallback);
	ros::Subscriber sub_signal = n.subscribe("/hlv_signal", 100, signalCallback);

	spinner.start();
	pub_hlv_system = n.advertise<std_msgs::Float32MultiArray>("/hlv_system", 100);
	hlv_system.data.resize(7);
	hlv_system.data = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
	pub_tlv_pose = n.advertise<geometry_msgs::Pose>("/v2x/tlv_pose", 100);
	tlv_pose = geometry_msgs::Pose();
	pub_tlv_path = n.advertise<visualization_msgs::Marker>("/v2x/tlv_path", 100);
	tlv_path = visualization_msgs::Marker();

	res = connect_v2x_socket();
	v2x_wsr_cmd_process();
	res = process_commands();

	ros::Rate r(10);
	while (is_running)
	{
		ros::spinOnce();
		r.sleep();
	}
	close_v2x_socket();
	return res;
}

/* function : V2X TX processing */
void *v2x_tx_cmd_process(void *arg)
{
	(void)arg;
	unsigned long cnt = 0;
	ssize_t n;
	time_t start_time = time(NULL);

	while (is_running)
	{
		time_t timer;
		struct tm *t;
		timer = time(NULL);
		t = localtime(&timer);
		struct timeval tv;
		gettimeofday(&tv, NULL);
		char filename_tx[512];
		long sec_in_mill = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000;
		sprintf(filename_tx, "/home/inha/catkin_ws/src/CoLaneX/v2x/log/hlv/tx_%ld.json", sec_in_mill);
		char datetime[30];
		snprintf(datetime, sizeof(datetime), "%04d.%02d.%02d.%02d:%02d.%02d%06ld",
             t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);

		hlv_system.data[0] = state;
		pub_hlv_system.publish(hlv_system);

		MessageFrame_t msg = {0};
		msg.messageId = 20;
		msg.value.present = MessageFrame__value_PR_BasicSafetyMessage;
		BasicSafetyMessage_t *ptrBSM = &msg.value.choice.BasicSafetyMessage;	
		ptrBSM->coreData.msgCnt = cnt;
		ptrBSM->coreData.secMark = rx_msg_cnt;
		ptrBSM->coreData.lat = latitude;
		ptrBSM->coreData.Long = longitude;
		ptrBSM->coreData.elev = _signal;
		ptrBSM->coreData.heading = heading;
		ptrBSM->coreData.speed = velocity;
		
		struct path paths[path_len];
		if (path_len > 0){
			for (int i = 0; i < path_len; i++)
			{
				paths[i].x = path[i].first;
				paths[i].y = path[i].second;
			}
		}

		int paths_size = path_len * sizeof(struct path);

		int db_v2x_tmp_size = sizeof(DB_V2X_T)+paths_size;
		int v2x_tx_pdu_size = sizeof(Ext_V2X_TxPDU_t)+db_v2x_tmp_size;
		char packet[MAX_UPER_SIZE];

		DB_V2X_T *db_v2x_tmp_p = (DB_V2X_T *)&packet[0];

		db_v2x_tmp_p->eDeviceType = DB_V2X_DEVICE_TYPE_OBU;
		db_v2x_tmp_p->eTeleCommType = DB_V2X_TELECOMM_TYPE_5G_PC5;
		db_v2x_tmp_p->unDeviceId =htonl(72);
		db_v2x_tmp_p->ulTimeStamp = 0ULL;
		db_v2x_tmp_p->eServiceId = DB_V2X_SERVICE_ID_ADVANCED_DRIVING;
		db_v2x_tmp_p->eActionType = DB_V2X_ACTION_TYPE_REQUEST;
		db_v2x_tmp_p->eRegionId = DB_V2X_REGION_ID_SEOUL;
		db_v2x_tmp_p->ePayloadType = DB_V2X_PAYLOAD_TYPE_SAE_J2735_BSM;
		db_v2x_tmp_p->eCommId = DB_V2X_COMM_ID_V2V;
		db_v2x_tmp_p->ulPayloadLength = htonl(sizeof(MessageFrame_t)+paths_size);
		db_v2x_tmp_p->messageFrame = msg;

		Ext_V2X_TxPDU_t *v2x_tx_pdu_p = (Ext_V2X_TxPDU_t *)malloc(v2x_tx_pdu_size);
		memset(v2x_tx_pdu_p, 0, v2x_tx_pdu_size);
		v2x_tx_pdu_p->ver = htons(SAMPLE_V2X_API_VER);
		v2x_tx_pdu_p->e_payload_type = e_payload_type_g;
		v2x_tx_pdu_p->psid = htonl(psid_g);
		v2x_tx_pdu_p->tx_power = tx_power_g;
		v2x_tx_pdu_p->e_signer_id = e_signer_id_g;
		v2x_tx_pdu_p->e_priority = e_priority_g;
		if (e_comm_type_g == eV2XCommType_LTEV2X || e_comm_type_g == eV2XCommType_5GNRV2X)
		{
			v2x_tx_pdu_p->magic_num = htons(MAGIC_CV2X_TX_PDU);
			v2x_tx_pdu_p->u.config_cv2x.transmitter_profile_id = htonl(transmitter_profile_id_g);
			v2x_tx_pdu_p->u.config_cv2x.peer_l2id = htonl(peer_l2id_g);
		}
		else if (e_comm_type_g == eV2XCommType_DSRC)
		{
			v2x_tx_pdu_p->magic_num = htons(MAGIC_DSRC_TX_PDU);
			v2x_tx_pdu_p->u.config_wave.freq = htons(freq_g);
			v2x_tx_pdu_p->u.config_wave.e_data_rate = htons(e_data_rate_g);
			v2x_tx_pdu_p->u.config_wave.e_time_slot = e_time_slot_g;
			memcpy(v2x_tx_pdu_p->u.config_wave.peer_mac_addr, peer_mac_addr_g, MAC_EUI48_LEN);
		}
		
		memcpy(packet + sizeof(DB_V2X_T), paths, paths_size);
		v2x_tx_pdu_p->v2x_msg.length = htons(db_v2x_tmp_size);
		memcpy(v2x_tx_pdu_p->v2x_msg.data, packet, db_v2x_tmp_size);
		n = write(sock_g, v2x_tx_pdu_p, v2x_tx_pdu_size);

		if (n < 0)
		{
			perror("send() failed");
			break;
		}
		else if (n != v2x_tx_pdu_size)
		{
			fprintf(stderr, "send() sent a different number of bytes than expected\n");
			break;
		}
		else
		{
			tx_packet_count++;
			time_t current_time = time(NULL);
			ts_list.push_back(std::make_pair(cnt, current_time));
			double send_time_s = (difftime(current_time, start_time));
			double mbps = ( n / send_time_s )/1000000.0;
			if(isinf(mbps)){
				mbps = 0.0;
			}
			hlv_system.data[4] = mbps;
			start_time = current_time;
			cnt += 1;

			json_object *Tx = json_object_new_object();
			json_object_object_add(Tx, "timestamp", json_object_new_string(datetime));

			json_object *extV2XTxPDU = json_object_new_object();
			json_object_object_add(extV2XTxPDU, "ver", json_object_new_int(SAMPLE_V2X_API_VER));
			json_object_object_add(extV2XTxPDU, "e_payload_type", json_object_new_int(e_payload_type_g));
			json_object_object_add(extV2XTxPDU, "psid", json_object_new_int(psid_g));
			json_object_object_add(extV2XTxPDU, "tx_power", json_object_new_int(tx_power_g));
			json_object_object_add(extV2XTxPDU, "e_signer_id", json_object_new_int(e_signer_id_g));
			json_object_object_add(extV2XTxPDU, "e_priority", json_object_new_int(e_priority_g));

			if (e_comm_type_g == eV2XCommType_LTEV2X || e_comm_type_g == eV2XCommType_5GNRV2X) {
				json_object_object_add(extV2XTxPDU, "magic_num", json_object_new_int(MAGIC_CV2X_TX_PDU));
				json_object *configCV2X = json_object_new_object();
				json_object_object_add(configCV2X, "transmitter_profile_id", json_object_new_int(transmitter_profile_id_g));
				json_object_object_add(configCV2X, "peer_l2id", json_object_new_int(peer_l2id_g));
				json_object_object_add(extV2XTxPDU, "u.config_cv2x", configCV2X);
			} else if (e_comm_type_g == eV2XCommType_DSRC) {
				json_object_object_add(extV2XTxPDU, "magic_num", json_object_new_int(MAGIC_DSRC_TX_PDU));
				json_object *configWave = json_object_new_object();
				json_object_object_add(configWave, "freq", json_object_new_int(freq_g));
				json_object_object_add(configWave, "e_data_rate", json_object_new_int(e_data_rate_g));
				json_object_object_add(configWave, "e_time_slot", json_object_new_int(e_time_slot_g));
				char macAddress[18]; // 충분한 크기의 문자열 배열 선언 (12문자 + 5개의 ':'와 널 문자)

				sprintf(macAddress, "%02X:%02X:%02X:%02X:%02X:%02X",
						peer_mac_addr_g[0], peer_mac_addr_g[1], peer_mac_addr_g[2],
						peer_mac_addr_g[3], peer_mac_addr_g[4], peer_mac_addr_g[5]);

				json_object_object_add(configWave, "peer_mac_addr", json_object_new_string(macAddress));
				json_object_object_add(extV2XTxPDU, "u.config_wave", configWave);
			} 
			

			json_object *v2xMsgData = json_object_new_object();
			json_object_object_add(v2xMsgData, "length",  json_object_new_int(db_v2x_tmp_size));
			json_object *dbV2XData = json_object_new_object();


			json_object_object_add(dbV2XData, "eDeviceType", json_object_new_int(DB_V2X_DEVICE_TYPE_OBU));
			json_object_object_add(dbV2XData, "eTeleCommType", json_object_new_int(DB_V2X_TELECOMM_TYPE_5G_PC5));
			json_object_object_add(dbV2XData, "unDeviceId", json_object_new_int(72));
			json_object_object_add(dbV2XData, "ulTimeStamp", json_object_new_int64(0ULL));
			json_object_object_add(dbV2XData, "eServiceId", json_object_new_int(DB_V2X_SERVICE_ID_ADVANCED_DRIVING));
			json_object_object_add(dbV2XData, "eActionType", json_object_new_int(DB_V2X_ACTION_TYPE_REQUEST));
			json_object_object_add(dbV2XData, "eRegionId", json_object_new_int(DB_V2X_REGION_ID_SEOUL));
			json_object_object_add(dbV2XData, "ePayloadType", json_object_new_int(DB_V2X_PAYLOAD_TYPE_SAE_J2735_BSM));
			json_object_object_add(dbV2XData, "eCommId", json_object_new_int(DB_V2X_COMM_ID_V2V));
			json_object_object_add(dbV2XData, "ulPayloadLength", json_object_new_int(sizeof(MessageFrame_t) + paths_size));


			json_object *messageFrame = json_object_new_object();
			json_object *coreData = json_object_new_object();

			json_object_object_add(coreData, "msgCnt", json_object_new_int(cnt));
			json_object_object_add(coreData, "rx_msg_cnt", json_object_new_int(rx_msg_cnt));
			json_object_object_add(coreData, "lat", json_object_new_double(latitude));
			json_object_object_add(coreData, "long", json_object_new_double(longitude));
			json_object_object_add(coreData, "signal", json_object_new_double(_signal));
			json_object_object_add(coreData, "heading", json_object_new_double(heading));
			json_object_object_add(coreData, "velocity", json_object_new_double(velocity));

			json_object_object_add(messageFrame, "coreData", coreData);
			json_object_object_add(dbV2XData, "messageFrame", messageFrame);

			// 경로 데이터를 JSON 배열로 추가
			json_object *pathArray = json_object_new_array();
			for (int i = 0; i < path_len; i++) {
				json_object *pathObj = json_object_new_object();
				json_object_object_add(pathObj, "x", json_object_new_double(path[i].first));
				json_object_object_add(pathObj, "y", json_object_new_double(path[i].second));
				json_object_array_add(pathArray, pathObj);
			}



			json_object_object_add(dbV2XData, "path", pathArray);
			json_object_object_add(v2xMsgData, "data", dbV2XData);
			json_object_object_add(extV2XTxPDU, "v2x_msg", v2xMsgData);
			json_object_object_add(Tx, "Ext_V2X_Tx_PDU", extV2XTxPDU);

			json_object_to_file_ext(filename_tx, json_object_get(Tx), JSON_C_TO_STRING_PRETTY);
			json_object_put(Tx);
			


			if (show_result){
				char _test[MAX_UPER_SIZE];
				memcpy(_test, v2x_tx_pdu_p, n);
				int db_v2x_len = ntohs(v2x_tx_pdu_p->v2x_msg.length);
				int read_len = sizeof(Ext_V2X_TxPDU_t);
				DB_V2X_T test_db_v2x_t;
				memcpy(&test_db_v2x_t, _test+read_len,db_v2x_len);
				read_len += (sizeof(DB_V2X_T)-sizeof(MessageFrame_t));
				int msgFrame_len = ntohl(test_db_v2x_t.ulPayloadLength);
				MessageFrame_t test_msgFrame_t;
				memcpy(&test_msgFrame_t, _test+read_len,msgFrame_len);
				BasicSafetyMessage_t *test_bsm = &test_msgFrame_t.value.choice.BasicSafetyMessage;

				int test_path_len = (msgFrame_len - sizeof(MessageFrame_t))/sizeof(struct path);
				read_len += sizeof(MessageFrame_t);

				struct path received_paths[test_path_len];
				memcpy(received_paths, _test+read_len, test_path_len*sizeof(struct path));

				printf("\nV2X Tx Test Msg>>\n"
					   "  Send Size   : %ld\n"
					   "  CNT        :  %ld\n", 
					   n,
					   test_bsm->coreData.msgCnt);
			}
		}
		free(v2x_tx_pdu_p);
		usleep(1e6 / hz);
	}

	return NULL;
}


void *v2x_rx_cmd_process(void *arg)
{
	
	(void)arg;
	uint8_t recvbuf[MAX_UPER_SIZE] = {0};
	int n = -1;
	time_t start_time = time(NULL);


	while (is_running)
	{
		time_t timer;
		struct tm *t;
		timer = time(NULL);
		t = localtime(&timer);
		struct timeval tv;
		gettimeofday(&tv, NULL);
		char filename_tx[512];
		long sec_in_mill = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000;
		sprintf(filename_tx, "/home/inha/catkin_ws/src/CoLaneX/v2x/log/hlv/rx_%ld.json", sec_in_mill);
		char datetime[30];
		snprintf(datetime, sizeof(datetime), "%04d.%02d.%02d.%02d:%02d.%02d%06ld",
             t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);
			 
		n = recv(sock_g, recvbuf, sizeof(recvbuf), 0);
		
		if (n < 0)
		{
			if (errno != EAGAIN && errno != EWOULDBLOCK)
			{
				perror("recv() failed");
				break;
			}
			else
			{
				usleep(1000);
			}
		}
		else if (n == 0)
		{
			fprintf(stderr, "recv() connection closed by peer\n");
			break;
		}
		else
		{
			rx_packet_count++;

			Ext_V2X_RxPDU_t ext_v2x_rx_pdu_p;
			memcpy(&ext_v2x_rx_pdu_p, recvbuf, sizeof(Ext_V2X_RxPDU_t));
			int read_len = sizeof(Ext_V2X_RxPDU_t);
			DB_V2X_T db_v2x_tmp;

			memcpy(&db_v2x_tmp, recvbuf+read_len, sizeof(DB_V2X_T)-sizeof(MessageFrame_t));
			read_len += (sizeof(DB_V2X_T)- sizeof(MessageFrame_t));
			int msgFrame_len = ntohl(db_v2x_tmp.ulPayloadLength);

			MessageFrame_t msgFrame;
			memcpy(&msgFrame, recvbuf+read_len, msgFrame_len);
			BasicSafetyMessage_t *ptrBSM = &msgFrame.value.choice.BasicSafetyMessage;

			rx_msg_cnt = ptrBSM->coreData.msgCnt;
			int rx_dsecond = ptrBSM->coreData.secMark;
			check_rtt(rx_dsecond);
			check_distance(ptrBSM->coreData.lat, ptrBSM->coreData.Long);

			int get_path_len = (msgFrame_len - sizeof(MessageFrame_t))/sizeof(struct path);
			read_len += sizeof(MessageFrame_t);

			struct path received_paths[get_path_len];
			memcpy(received_paths, recvbuf + read_len, get_path_len * sizeof(struct path));

			std::vector<std::pair<double, double>> _t_path;
			for (int i = 0; i < get_path_len-1; ++i)
			{
				_t_path.push_back(std::make_pair(received_paths[i].x, received_paths[i].y));
			}

			pubTLV(ptrBSM->coreData.lat, ptrBSM->coreData.Long, ptrBSM->coreData.heading, ptrBSM->coreData.speed, ptrBSM->coreData.elev, _t_path);


			json_object *Rx = json_object_new_object();
			json_object_object_add(Rx, "timestamp", json_object_new_string(datetime));

			json_object *extV2XRxPDU = json_object_new_object();
			json_object_object_add(extV2XRxPDU, "magic_num", json_object_new_int(ntohs(ext_v2x_rx_pdu_p.magic_num)));
			json_object_object_add(extV2XRxPDU, "ver", json_object_new_int(ntohs(ext_v2x_rx_pdu_p.ver)));
			json_object_object_add(extV2XRxPDU, "psid", json_object_new_int(ntohl(ext_v2x_rx_pdu_p.psid)));
			json_object_object_add(extV2XRxPDU, "e_v2x_comm_type", json_object_new_int(ext_v2x_rx_pdu_p.e_v2x_comm_type));
			json_object_object_add(extV2XRxPDU, "e_payload_type", json_object_new_int(ext_v2x_rx_pdu_p.e_payload_type));
			json_object_object_add(extV2XRxPDU, "freq", json_object_new_int(ntohs(ext_v2x_rx_pdu_p.freq)));
			json_object_object_add(extV2XRxPDU, "rssi", json_object_new_int(ntohs(ext_v2x_rx_pdu_p.rssi)));
			json_object_object_add(extV2XRxPDU, "is_signed", json_object_new_int(ext_v2x_rx_pdu_p.is_signed));
			// json_object_object_add(extV2XRxPDU, "peer_mac_addr", json_object_new_string(ext_v2x_rx_pdu_p.u.peer_mac_addr));
			// json_object_object_add(extV2XRxPDU, "peer_l2id", json_object_new_int(ntohl(ext_v2x_rx_pdu_p.u.peer_l2id)));
		
			json_object *v2xMsgData = json_object_new_object();
			json_object_object_add(v2xMsgData, "length",  json_object_new_int(ntohs(ext_v2x_rx_pdu_p.v2x_msg.length)));
			json_object *dbV2XData = json_object_new_object();

			json_object_object_add(dbV2XData, "eDeviceType", json_object_new_int(db_v2x_tmp.eDeviceType));
			json_object_object_add(dbV2XData, "eTeleCommType", json_object_new_int(db_v2x_tmp.eTeleCommType));
			json_object_object_add(dbV2XData, "unDeviceId", json_object_new_int(ntohl(db_v2x_tmp.unDeviceId)));
			json_object_object_add(dbV2XData, "ulTimeStamp", json_object_new_int64(db_v2x_tmp.ulTimeStamp));
			json_object_object_add(dbV2XData, "eServiceId", json_object_new_int(db_v2x_tmp.eServiceId));
			json_object_object_add(dbV2XData, "eActionType", json_object_new_int(db_v2x_tmp.eActionType));
			json_object_object_add(dbV2XData, "eRegionId", json_object_new_int(db_v2x_tmp.eRegionId));
			json_object_object_add(dbV2XData, "ePayloadType", json_object_new_int(db_v2x_tmp.ePayloadType));
			json_object_object_add(dbV2XData, "eCommId", json_object_new_int(db_v2x_tmp.eCommId));
			json_object_object_add(dbV2XData, "ulPayloadLength", json_object_new_int(ntohl(db_v2x_tmp.ulPayloadLength)));

			json_object *messageFrame = json_object_new_object();
			json_object *coreData = json_object_new_object();

			json_object_object_add(coreData, "msgCnt", json_object_new_int(ptrBSM->coreData.msgCnt));
			json_object_object_add(coreData, "rx_msg_cnt", json_object_new_int(ptrBSM->coreData.secMark));
			json_object_object_add(coreData, "lat", json_object_new_double(ptrBSM->coreData.lat));
			json_object_object_add(coreData, "long", json_object_new_double(ptrBSM->coreData.Long));
			json_object_object_add(coreData, "signal", json_object_new_double(ptrBSM->coreData.elev));
			json_object_object_add(coreData, "heading", json_object_new_double(ptrBSM->coreData.heading));
			json_object_object_add(coreData, "velocity", json_object_new_double(ptrBSM->coreData.speed));

			json_object_object_add(messageFrame, "coreData", coreData);
			json_object_object_add(dbV2XData, "messageFrame", messageFrame);

			// 경로 데이터를 JSON 배열로 추가
			json_object *pathArray = json_object_new_array();
			for (int i = 0; i < _t_path.size(); i++) {
				json_object *pathObj = json_object_new_object();
				json_object_object_add(pathObj, "x", json_object_new_double(_t_path[i].first));
				json_object_object_add(pathObj, "y", json_object_new_double(_t_path[i].second));
				json_object_array_add(pathArray, pathObj);
			}


			json_object_object_add(dbV2XData, "path", pathArray);
			json_object_object_add(v2xMsgData, "data", dbV2XData);
			json_object_object_add(extV2XRxPDU, "v2x_msg", v2xMsgData);
			json_object_object_add(Rx, "Ext_V2X_Rx_PDU", extV2XRxPDU);

			json_object_to_file_ext(filename_tx, json_object_get(Rx), JSON_C_TO_STRING_PRETTY);
			json_object_put(Rx);
			

		}
		usleep(1e6/hz);
	}

	return NULL;
}




/////////////////////////////////////////////////////////////////////////////////////////
int connect_v2x_socket(void)
{
	int res = -1; // failure

	// Create the socket
	int sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0)
	{
		perror("socket() failed");
		res = sock;
		return res;
	}

	// Connect to the server
	struct sockaddr_in server_addr;
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(SAMPLE_V2X_IP_ADDR);
	// server_addr.sin_addr.s_addr = inet_addr(SAMPLE_LOCAL);
	server_addr.sin_port = htons(SAMPLE_V2X_PORT_ADDR);

	if(same_machine){
		const char *interface_name = "enx5ca6e6fb9ab2";
		if (setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, interface_name, strlen(interface_name)) < 0)
		{
			perror("Error binding socket to the interface");
			close(sock);
			exit(EXIT_FAILURE);
		}
		if (inet_pton(AF_INET, "192.168.1.11", &server_addr.sin_addr) <= 0) {
			perror("Invalid address");
			close(sock);
			exit(EXIT_FAILURE);
		}
	}

	if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
	{
		perror("connect() failed");
		return res;
	}
#if 1
	int flags = fcntl(sock, F_GETFL, 0);
	if (flags == -1)
	{
		perror("fcntl F_GETFL failed");
		return res;
	}

	flags |= O_NONBLOCK;
	if (fcntl(sock, F_SETFL, flags) == -1)
	{
		perror("fcntl F_SETFL failed");
		return res;
	}
#endif
	printf("connect socket success\n");

	sock_g = sock;

	res = 0;
	return res;
}

void close_v2x_socket(void)
{
	if (sock_g >= 0)
	{
		close(sock_g);
	}
}

int v2x_wsr_cmd_process(void)
{
	int res = -1;

	Ext_WSReq_t ws_req;
	memset(&ws_req, 0, sizeof(ws_req));
	ws_req.magic_num = htons(MAGIC_WSREQ);
	ws_req.ver = htons(SAMPLE_V2X_API_VER);
	ws_req.e_action = eV2xAction_ADD;
	ws_req.e_payload_type = e_payload_type_g;
	ws_req.psid = htonl(psid_g);

	ssize_t n = send(sock_g, &ws_req, sizeof(ws_req), 0);
	if (n < 0)
	{
		perror("send() failed");
		return res;
	}
	else if (n != sizeof(ws_req))
	{
		fprintf(stderr, "send() sent a different number of bytes than expected\n");
		return res;
	}else{
		printf("WSR Send Sucess\n");
	}

	Ext_WSResp_t ws_resp;
	memset(&ws_resp, 0, sizeof(ws_resp));
	n = -1;
	while (n <= 0)
	{
		n = recv(sock_g, &ws_resp, sizeof(ws_resp), 0);
		if (n < 0)
		{
			if (errno != EAGAIN && errno != EWOULDBLOCK)
			{
				perror("recv() failed");
				break;
			}
		}
		else if (n == 0)
		{
			fprintf(stderr, "recv() connection closed by peer\n");
		}
		else if (n != sizeof(ws_resp))
		{
			fprintf(stderr, "recv() received a different number of bytes than expected\n");
		}

		usleep(1000);
	}
	printf("WSR RECV Sucess\n");
	res = 0;
	hlv_system.data[2] = 1;
	return res;
}

int process_commands(void)
{
	signal(SIGINT, sigint_handler); 
	pthread_t tx_thread;
	pthread_t rx_thread;
	pthread_t rate_thread;
	pthread_create(&tx_thread, NULL, v2x_tx_cmd_process, NULL);
	pthread_create(&rx_thread, NULL, v2x_rx_cmd_process, NULL);
	pthread_create(&rate_thread,NULL,check_packet_rates,NULL );
	pthread_join(tx_thread, NULL);
	pthread_join(rx_thread, NULL);
	pthread_join(rate_thread, NULL);
	return -1;
}

void pubTLV(long lat, long lng, long yaw, long vel, long sig, std::vector<std::pair<double, double>> _path){
	tlv_pose.position.x = lat / pow(10, 7);
	tlv_pose.position.y = lng / pow(10, 7);
	tlv_pose.position.z = yaw * 0.0125;
	tlv_pose.orientation.x = vel * 0.02;
	tlv_pose.orientation.y = sig;
	pub_tlv_pose.publish(tlv_pose);

	std::vector<geometry_msgs::Point> points_msg;
    for (const auto& point_pair : _path) {
        geometry_msgs::Point point_msg;
        point_msg.x = point_pair.first;
        point_msg.y = point_pair.second;
        point_msg.z = 0.0; // z 축은 0으로 설정 (2D 공간에서는 필요하지 않음)
        points_msg.push_back(point_msg);
    }

	tlv_path.type = visualization_msgs::Marker::LINE_STRIP;
	tlv_path.action = visualization_msgs::Marker::ADD;
	tlv_path.header.frame_id = "world";
	tlv_path.ns = "tlv_path";
	tlv_path.id = 999;
	tlv_path.text = std::to_string(_path.size());
	tlv_path.lifetime = ros::Duration(0);
	tlv_path.points = points_msg;
	tlv_path.scale.x = 1.5;
	tlv_path.color.r = 94.0/255.0;
	tlv_path.color.g = 204.0/255.0;
	tlv_path.color.b = 243.0/255.0;
	tlv_path.color.a = 0.5;
	tlv_path.pose.orientation.x = 0;
	tlv_path.pose.orientation.y = 0;
	tlv_path.pose.orientation.z= 0;
	tlv_path.pose.orientation.w = 1;

	pub_tlv_path.publish(tlv_path);
}


void* check_packet_rates(void *){
	while(is_running){
		double tx_rate = static_cast<double>(tx_packet_count) / 1.0;
		double rx_rate = static_cast<double>(rx_packet_count) / 1.0;
		tx_packet_count = 0;
		rx_packet_count = 0;
		hlv_system.data[5] = (rx_rate/hz)*100;
		usleep(1000000);
	}
	return NULL;
}

void check_rtt(int rx_ds){
	double rtt = 0.0;
	for(const auto& msg : ts_list){
		if(msg.first == rx_ds){
			rtt = round((difftime(time(NULL), msg.second)) * 1000);
			hlv_system.data[3] = rtt;
			break;
		}
	}
	auto it = std::remove_if(ts_list.begin(), ts_list.end(),
                                 [&](const std::pair<int, time_t>& msg) {
                                     return msg.first < rx_ds;
                                 });
    ts_list.erase(it, ts_list.end());
}


double toRadians(double v2x_degree) {
    return (v2x_degree/pow(10,7)) * M_PI / 180.0;
}

//두 지점 간의 거리를 계산하는 함수 (Haversine formula)
void check_distance(long lat, long lng){
	double earthRadius = 6371000.0; // 지구의 반지름 (미터 단위)

    // 라디안으로 변환
    double lat1Rad = toRadians(latitude);
    double lon1Rad = toRadians(longitude);
    double lat2Rad = toRadians(lat);
    double lon2Rad = toRadians(lng);

    // Haversine formula 적용
    double dLat = lat2Rad - lat1Rad;
    double dLon = lon2Rad - lon1Rad;
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1Rad) * cos(lat2Rad) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = earthRadius * c;
	hlv_system.data[6] = distance;
}
