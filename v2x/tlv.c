
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
#include <time.h>
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


#define SAMPLE_V2X_API_VER 0x0001
#define SAMPLE_V2X_IP_ADDR "192.168.1.11"
#define SAMPLE_V2X_PORT_ADDR 47347

#define SAMPLE_V2X_MSG_LEN 2000

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

std_msgs::Float32MultiArray tlv_system;
ros::Publisher pub_tlv_system;
geometry_msgs::Pose hlv_pose;
ros::Publisher pub_hlv_pose;
visualization_msgs::Marker hlv_path;
ros::Publisher pub_hlv_path;

unsigned long state;
unsigned long _signal;
unsigned long latitude;
unsigned long longitude;
unsigned long heading;
unsigned long velocity; //mps

std::vector<std::pair<double, double>> path;
int path_len;
bool show_result = true;

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
	server_addr.sin_port = htons(SAMPLE_V2X_PORT_ADDR);
	
	if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
	{
		perror("connect() failed");
		return res;
	}
#if 1
	// Change to NON-BLOCK socket
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


/* function : WSR Send/Response processing */
int v2x_wsr_cmd_process(void)
{
	int res = -1; // failure

	// Prepare the Ext_WSReq_t structure
	Ext_WSReq_t ws_req;
	memset(&ws_req, 0, sizeof(ws_req));
	ws_req.magic_num = htons(MAGIC_WSREQ);
	ws_req.ver = htons(SAMPLE_V2X_API_VER);
	ws_req.e_action = eV2xAction_ADD;
	ws_req.e_payload_type = e_payload_type_g;
	ws_req.psid = htonl(psid_g);

	// Send the request
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
		printf("WSR Send Sucess");
	}

	// Wait for the response
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
	printf("WSR RECV Sucess");
	res = 0;
	tlv_system.data[2] = 1;
	return res;
}

/* function : V2X TX processing */
void *v2x_tx_cmd_process(void *arg)
{
	(void)arg;

	// Prepare the Ext_WSReq_t structure
	int db_v2x_tmp_size = sizeof(DB_V2X_T) + sizeof(MessageFrame_t);//SAMPLE_V2X_MSG_LEN;
	int v2x_tx_pdu_size = sizeof(Ext_V2X_TxPDU_t) + db_v2x_tmp_size;

	Ext_V2X_TxPDU_t *v2x_tx_pdu_p = NULL;
	DB_V2X_T *db_v2x_tmp_p = NULL;

	v2x_tx_pdu_p = (Ext_V2X_TxPDU_t *)malloc(v2x_tx_pdu_size);
	memset(v2x_tx_pdu_p, 0, sizeof(Ext_V2X_TxPDU_t));

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

	// Payload = KETI Format
	v2x_tx_pdu_p->v2x_msg.length = htons(db_v2x_tmp_size);
	db_v2x_tmp_p = (DB_V2X_T *)malloc(db_v2x_tmp_size); //DB_V2X_T
	memset(db_v2x_tmp_p, 0, db_v2x_tmp_size);

	db_v2x_tmp_p->eDeviceType = DB_V2X_DEVICE_TYPE_OBU;
	db_v2x_tmp_p->eTeleCommType = DB_V2X_TELECOMM_TYPE_5G_PC5;
	db_v2x_tmp_p->unDeviceId =htonl(71);
	db_v2x_tmp_p->ulTimeStamp = 0ULL;
	db_v2x_tmp_p->eServiceId = DB_V2X_SERVICE_ID_PLATOONING;
	db_v2x_tmp_p->eActionType = DB_V2X_ACTION_TYPE_REQUEST;
	db_v2x_tmp_p->eRegionId = DB_V2X_REGION_ID_SEOUL;
	db_v2x_tmp_p->ePayloadType = DB_V2X_PAYLOAD_TYPE_SAE_J2735_BSM;
	db_v2x_tmp_p->eCommId = DB_V2X_COMM_ID_V2V;
	db_v2x_tmp_p->usDbVer = 0;
	db_v2x_tmp_p->usHwVer = 0;
	db_v2x_tmp_p->usSwVer = 0;
	db_v2x_tmp_p->ulPayloadLength = htonl(sizeof(MessageFrame_t));

	//Subscribe
	
	unsigned long cnt = 0;

	ssize_t n;
	time_t start_time = time(NULL);
	int period = 1000000 / 10;
	while (is_running)
	{
		tlv_system.data[0] = state;
		pub_tlv_system.publish(tlv_system);

		MessageFrame_t msg = {0};
		msg.messageId = 20;
		msg.value.present = MessageFrame__value_PR_BasicSafetyMessage;

		BasicSafetyMessage_t *ptrBSM = &msg.value.choice.BasicSafetyMessage;
		
		ptrBSM->coreData.id.buf = (uint8_t *)malloc(1);
		ptrBSM->coreData.id.size = 1;
		ptrBSM->coreData.id.buf[0] = 0x71;
		ptrBSM->coreData.msgCnt = cnt;
		ptrBSM->coreData.lat = latitude;
		ptrBSM->coreData.Long = longitude;
		ptrBSM->coreData.heading = heading;
		ptrBSM->coreData.speed = velocity;
		for (int i = 0; i < path_len; ++i)
		{
			Path *bsmPath = (Path *)calloc(1, sizeof(Path));
			bsmPath->x = path[i].first;
			bsmPath->y = path[i].second;
			ASN_SEQUENCE_ADD(&ptrBSM->path, bsmPath);
		}

		if(path_len>0){
			printf("sequence len %d\n", ptrBSM->path.count);
		}

		db_v2x_tmp_p->data = msg;
		memcpy(v2x_tx_pdu_p->v2x_msg.data, db_v2x_tmp_p, db_v2x_tmp_size); //(dst, src, length)
		n = send(sock_g, v2x_tx_pdu_p, v2x_tx_pdu_size, 0);

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
			printf(" \n\ntx send success(%ld bytes)\n", n);
			time_t current_time = time(NULL);
			double send_time_s = (difftime(current_time, start_time));
			double mbps = ( n / send_time_s )/1000000.0;
			if(isinf(mbps)){
				mbps = 0.0;
			}
			tlv_system.data[4] = mbps;
			start_time = current_time;
			cnt += 1;

			//For Test
			if (show_result){
				DB_V2X_T *test = NULL;
				test = (DB_V2X_T *)malloc(v2x_tx_pdu_p->v2x_msg.length);
				memcpy(test, v2x_tx_pdu_p->v2x_msg.data, v2x_tx_pdu_p->v2x_msg.length);
				MessageFrame_t *test_msg = NULL;
				test_msg = (MessageFrame_t *)malloc(ntohl(test->ulPayloadLength));
				memcpy(test_msg, &test->data, ntohl(test->ulPayloadLength));

				printf("\nV2X Tx Test Msg>>\n"
					"  ID         :  0x%02x\n"
					"  CNT        :  %ld\n"
					"  latitude   :  %ld\n"
					"  longitude  :  %ld\n"
					"  heading    :  %ld\n"
					"  velocity   :  %ld\n"
					"  path len   :  %d\n",
					test_msg->value.choice.BasicSafetyMessage.coreData.id.buf[0],
					test_msg->value.choice.BasicSafetyMessage.coreData.msgCnt,
					test_msg->value.choice.BasicSafetyMessage.coreData.lat,
					test_msg->value.choice.BasicSafetyMessage.coreData.Long,
					test_msg->value.choice.BasicSafetyMessage.coreData.heading, 
					test_msg->value.choice.BasicSafetyMessage.coreData.speed,
					test_msg->value.choice.BasicSafetyMessage.path.count);
			}
		}
		usleep(period);
	}

	free(v2x_tx_pdu_p);
	free(db_v2x_tmp_p);
	
	return NULL;
}


void pubHLV(long lat, long lng, long yaw, long vel, std::vector<std::pair<double, double>> _path){
	hlv_pose.position.x = lat / pow(10, 7);
	hlv_pose.position.y = lng / pow(10, 7);
	hlv_pose.position.x = yaw * 0.0125;
	hlv_pose.orientation.x = vel * 0.02;
	pub_hlv_pose.publish(hlv_pose);


	std::vector<geometry_msgs::Point> points_msg;
    for (const auto& point_pair : _path) {
        geometry_msgs::Point point_msg;
        point_msg.x = point_pair.first;
        point_msg.y = point_pair.second;
        point_msg.z = 0.0; // z 축은 0으로 설정 (2D 공간에서는 필요하지 않음)
        points_msg.push_back(point_msg);
    }

	hlv_path.type = visualization_msgs::Marker::LINE_STRIP;
	hlv_path.action = visualization_msgs::Marker::ADD;
	hlv_path.header.frame_id = "world";
	hlv_path.ns = "hlv_path";
	hlv_path.id = 999;
	hlv_path.text = std::to_string(_path.size());
	hlv_path.lifetime = ros::Duration(0);
	hlv_path.points = points_msg;
	hlv_path.scale.x = 0.4;
	hlv_path.color.r = 241 / 255;
	hlv_path.color.g = 76 / 255;
	hlv_path.color.b = 152 / 255;
	hlv_path.color.a = 1;
	hlv_path.pose.orientation.x = 0;
	hlv_path.pose.orientation.y = 0;
	hlv_path.pose.orientation.z= 0;
	hlv_path.pose.orientation.w = 1;

	pub_hlv_path.publish(hlv_path);
}


/* function : V2X RX processing */
void *v2x_rx_cmd_process(void *arg)
{
	(void)arg;
	uint8_t buf[4096] = {0};
	int n = -1;
	time_t start_time = time(NULL);

	DB_V2X_T *db_v2x_tmp_p = NULL;
	MessageFrame_t *msgFrame = NULL;
	Ext_V2X_RxPDU_t *v2x_rx_pdu_p = NULL;
	int period = 1000000 / 10;
	while (is_running)
	{
		n = recv(sock_g, buf, sizeof(buf), 0);
		
		if (n < 0)
		{
			if (errno != EAGAIN && errno != EWOULDBLOCK)
			{
				perror("recv() failed");
				break;
			}
			else
			{
				printf("wait. . . \n");
				usleep(10000);
			}
		}
		else if (n == 0)
		{
			fprintf(stderr, "recv() connection closed by peer\n");
			break;
		}
		else
		{
			printf("\n\nrecv() success : len[%u]\n", n);
			time_t current_time = time(NULL);
			double delay_time_ms = round((difftime(current_time, start_time))*1000);
			tlv_system.data[3] = delay_time_ms;
			start_time = current_time;
			
			v2x_rx_pdu_p = (Ext_V2X_RxPDU_t *)malloc(n);
			memcpy(v2x_rx_pdu_p, buf, n);
			printf("\nV2X RX PDU>>\n"
				   "  magic_num        : 0x%04X\n"
				   "  ver              : 0x%04X\n"
				   "  e_payload_type   : 0x%04X\n"
				   "  psid             : %u\n"
				   "  v2x length       : %d\n",
				   ntohs(v2x_rx_pdu_p->magic_num),
				   ntohs(v2x_rx_pdu_p->ver),
				   v2x_rx_pdu_p->e_payload_type,
				   ntohl(v2x_rx_pdu_p->psid),
				   ntohs(v2x_rx_pdu_p->v2x_msg.length));

			int v2x_msg_length = ntohs(v2x_rx_pdu_p->v2x_msg.length);
			db_v2x_tmp_p = (DB_V2X_T *)malloc(v2x_msg_length);
			memcpy(db_v2x_tmp_p, v2x_rx_pdu_p->v2x_msg.data, v2x_msg_length);

			printf("\nV2X RX Data>>\n"
				   "  deivce ID    :  %u\n"
				   "  Payload Type :  0x%04X\n"
				   "  Payload Length :  %u\n"
				   "  Region ID    :  0x%04x\n"
				   "  data Size    :  %ld\n",
				   ntohl(db_v2x_tmp_p->unDeviceId),
				   db_v2x_tmp_p->ePayloadType,
				   ntohl(db_v2x_tmp_p->ulPayloadLength),
				   ntohs(db_v2x_tmp_p->eRegionId),
				   sizeof(db_v2x_tmp_p->data));

			int payload_length = sizeof(db_v2x_tmp_p->data);
			msgFrame = (MessageFrame_t *)malloc(payload_length);
			memcpy(msgFrame, &db_v2x_tmp_p->data, payload_length);

			BasicSafetyMessage_t *ptrBSM = &msgFrame->value.choice.BasicSafetyMessage;

			printf("\nV2X RX Test Msg>>\n"
				   "  CNT        :  %ld\n"
				   "  latitude   :  %ld\n"
				   "  longitude  :  %ld\n"
				   "  heading    :  %ld\n"
				   "  velocity   :  %ld\n",
				   ptrBSM->coreData.msgCnt,
				   ptrBSM->coreData.lat,
				   ptrBSM->coreData.Long,
				   ptrBSM->coreData.heading, 
				   ptrBSM->coreData.speed);
			std::vector<std::pair<double, double>> _h_path;
			for (int i = 0; i < ptrBSM->path.count; ++i)
			{
				_h_path.push_back(std::make_pair(ptrBSM->path.array[i]->x, ptrBSM->path.array[i]->y));
			}
			pubHLV(ptrBSM->coreData.lat, ptrBSM->coreData.Long, ptrBSM->coreData.heading, ptrBSM->coreData.speed, _h_path);
		}
		usleep(period);	
	}
	free(v2x_rx_pdu_p);
	free(db_v2x_tmp_p);
	free(msgFrame);

	return NULL;
}

/* function : Process Commands */
int process_commands(void)
{
	pthread_t tx_thread;
	pthread_t rx_thread;

	pthread_create(&tx_thread, NULL, v2x_tx_cmd_process, NULL);
	pthread_create(&rx_thread, NULL, v2x_rx_cmd_process, NULL);
	pthread_join(tx_thread, NULL);
	pthread_join(rx_thread, NULL);

	return -1;
}


void poseCallback(const geometry_msgs::Pose::ConstPtr &msg){
	latitude = msg->position.x * pow(10, 7);
 	longitude = msg->position.y * pow(10, 7);
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
	for (size_t i = 0; i < path_len; i++){
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
	ros::init(argc,argv, "v2x");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);


	ros::Subscriber sub_pose = n.subscribe("/car/tlv_pose", 100, poseCallback);
	ros::Subscriber sub_path = n.subscribe("/planning/tlv_path", 100, pathCallback);
	ros::Subscriber sub_state = n.subscribe("/tlv_state", 100, stateCallback);
	ros::Subscriber sub_signal = n.subscribe("/tlv_signal", 100, signalCallback);

	spinner.start();
	pub_tlv_system = n.advertise<std_msgs::Float32MultiArray>("/tlv_system", 100);
	tlv_system.data.resize(5);
	tlv_system.data = {{0.0, 0.0, 0.0, 0.0, 0.0}};
	pub_hlv_pose = n.advertise<geometry_msgs::Pose>("/v2x/hlv_pose", 100);
	hlv_pose = geometry_msgs::Pose();
	pub_hlv_path = n.advertise<visualization_msgs::Marker>("/v2x/hlv_path", 100);
	hlv_path = visualization_msgs::Marker();

	res = connect_v2x_socket();
	v2x_wsr_cmd_process();
	res = process_commands();

	ros::Rate r(10);
	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}

	close_v2x_socket();
	return res;
}

/////////////////////////////////////////////////////////////////////////////////////////
