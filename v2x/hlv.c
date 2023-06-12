
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
#include <pthread.h>

/* include for v2x */
#include "v2x_defs.h"
#include "v2x_ext_type.h"
#include "db_v2x.h"
#include "math.h"

#include <ros/ros.h>

/////////////////////////////////////////////////////////////////////////////////////////
/* MACRO - for modify */

#define SAMPLE_V2X_API_VER 0x0001
#define SAMPLE_V2X_IP_ADDR "192.168.1.11"
#define SAMPLE_V2X_PORT_ADDR 47347

#define SAMPLE_V2X_MSG_LEN 2000


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


int connect_v2x_socket(void)
{
	printf("Cooectigomn");
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

	printf("\nWSM Service REQ>>\n"
		   "  magic_num        : 0x%04X\n"
		   "  ver              : 0x%04X\n"
		   "  e_action         : %d\n"
		   "  e_payload_type   : %d\n"
		   "  psid             : %u\n",
		   ntohs(ws_req.magic_num),
		   ntohs(ws_req.ver),
		   ws_req.e_action,
		   ws_req.e_payload_type,
		   ntohl(ws_req.psid));

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

	// Print the response
	printf("\nWSM Service RESP>>\n"
		   "  magic_num      : 0x%04X\n"
		   "  ver            : 0x%04X\n"
		   "  e_action       : %d\n"
		   "  is_confirmed   : %d\n"
		   "  psid           : %u\n",
		   ntohs(ws_resp.magic_num),
		   ntohs(ws_resp.ver),
		   ws_resp.e_action,
		   ws_resp.is_confirmed,
		   ntohl(ws_resp.psid));

	res = 0;
	
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
	printf("0x%04x <-> 0x%04x \n",v2x_tx_pdu_p->e_payload_type, e_payload_type_g);
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
	printf("%d\n", db_v2x_tmp_size);
	db_v2x_tmp_p = (DB_V2X_T *)malloc(db_v2x_tmp_size); //DB_V2X_T
	memset(db_v2x_tmp_p, 0, db_v2x_tmp_size);

	db_v2x_tmp_p->eDeviceType = DB_V2X_DEVICE_TYPE_OBU;
	db_v2x_tmp_p->eTeleCommType = DB_V2X_TELECOMM_TYPE_5G_PC5;
	db_v2x_tmp_p->unDeviceId =htonl(2);
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

	unsigned long latitude = 37.383797*pow(10,7);
	unsigned long longitude = 126.656788*pow(10,7);
	unsigned long heading = 0;
	unsigned long velocity = 5; //mps
	unsigned long cnt = 0;

	MessageFrame_t msg;
	msg.messageId = 20;
	msg.value.present = MessageFrame__value_PR_BasicSafetyMessage;

	if (e_comm_type_g == eV2XCommType_LTEV2X || e_comm_type_g == eV2XCommType_5GNRV2X)
	{
		printf("  u.config_cv2x.transmitter_profile_id : %u\n"
			   "  u.config_cv2x.peer_l2id              : %u\n",
			   ntohl(v2x_tx_pdu_p->u.config_cv2x.transmitter_profile_id),
			   ntohl(v2x_tx_pdu_p->u.config_cv2x.peer_l2id));
	}
	else if (e_comm_type_g == eV2XCommType_DSRC)
	{
		printf("  u.config_wave.freq                  : %d\n"
			   "  u.config_wave.e_data_rate           : %d\n"
			   "  u.config_wave.e_time_slot           : %d\n"
			   "  u.config_wave.peer_mac_addr         : %s\n",
			   ntohs(v2x_tx_pdu_p->u.config_wave.freq),
			   ntohs(v2x_tx_pdu_p->u.config_wave.e_data_rate),
			   v2x_tx_pdu_p->u.config_wave.e_time_slot,
			   v2x_tx_pdu_p->u.config_wave.peer_mac_addr);
	}

	uint32_t i;
	ssize_t n;
	//for (i = 0; i < tx_cnt_g; i++)
	while( 1 )
	{
		// Send the request
		BasicSafetyMessage_t *ptrBSM = &msg.value.choice.BasicSafetyMessage;

		ptrBSM->coreData.id.buf = (uint8_t *)malloc(1);
		ptrBSM->coreData.id.size = 1;
		ptrBSM->coreData.id.buf[0] = 0x02;
		ptrBSM->coreData.msgCnt = cnt;
		ptrBSM->coreData.lat = latitude;
		ptrBSM->coreData.Long = longitude;
		ptrBSM->coreData.heading = heading;
		ptrBSM->coreData.speed = velocity;
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
			//printf(" tx send success(%ld bytes) : [%u/%u]\n", n, i + 1, tx_cnt_g);
			printf(" \n\ntx send success(%ld bytes)\n", n);


			printf("\nV2X TX PDU>>\n"
				"  magic_num        : 0x%04X\n"
				"  ver              : 0x%04X\n"
				"  e_payload_type   : 0x%04X\n"
				"  psid             : %u\n"
				"  v2x length       : %d\n",
				ntohs(v2x_tx_pdu_p->magic_num),
				ntohs(v2x_tx_pdu_p->ver),
				v2x_tx_pdu_p->e_payload_type,
				ntohl(v2x_tx_pdu_p->psid),
				ntohs(v2x_tx_pdu_p->v2x_msg.length));	

			DB_V2X_T *test = NULL;
			test =(DB_V2X_T *)malloc(v2x_tx_pdu_p->v2x_msg.length);
			memcpy(test, v2x_tx_pdu_p->v2x_msg.data, v2x_tx_pdu_p->v2x_msg.length);

			printf("\nV2X TX Data>>\n"
				   "  deivce ID    :  %d\n"
				   "  Db Ver       :  %d\n"
				   "  Hw Ver       :  %d\n"
				   "  Sw Ver       :  %d\n"
				   "  Payload Type :  0x%04X\n"
				   "  Payload Length :  %d\n"
				   "  Region ID    :  0x%04x\n",
				  
				   ntohl(test->unDeviceId),
				   test->usDbVer,
				   test->usHwVer,
				   test->usSwVer,
				   test->ePayloadType,
				   ntohl(test->ulPayloadLength),
				   ntohs(test->eRegionId));

			MessageFrame_t *test_msg = NULL;
			test_msg = (MessageFrame_t *)(ntohl(test->ulPayloadLength));
			memcpy(test_msg, &test->data, ntohl(test->ulPayloadLength));

			printf("\nV2X RX Test Msg>>\n"
				   "  ID         :  %ld\n"
				   "  CNT        :  %ld\n"
				   "  latitude   :  %ld\n"
				   "  longitude  :  %ld\n"
				   "  heading    :  %ld\n"
				   "  velocity   :  %ld\n",
				   test_msg->value.choice.BasicSafetyMessage.coreData.id.buf,
				   test_msg->value.choice.BasicSafetyMessage.coreData.msgCnt,
				   test_msg->value.choice.BasicSafetyMessage.coreData.lat,
				   test_msg->value.choice.BasicSafetyMessage.coreData.Long,
				   test_msg->value.choice.BasicSafetyMessage.coreData.heading, 
				   test_msg->value.choice.BasicSafetyMessage.coreData.speed);
			cnt += 1;
			free(test);
			free(test_msg);
		}

		usleep((1000 * tx_delay_g));
	}

	free(v2x_tx_pdu_p);
	free(db_v2x_tmp_p);
	
	return NULL;
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

	while (1)
	{
		
		time_t current_time = time(NULL);
		if (current_time - start_time >= delay_time_sec_g)
		{
			break;
		}

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
				continue;
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
				   "  Db Ver       :  %d\n"
				   "  Hw Ver       :  %d\n"
				   "  Sw Ver       :  %d\n"
				   "  Payload Type :  0x%04X\n"
				   "  Payload Length :  %u\n"
				   "  Region ID    :  0x%04x\n"
				   "  data Size    :  %ld\n",
				   ntohl(db_v2x_tmp_p->unDeviceId),
				   db_v2x_tmp_p->usDbVer,
				   db_v2x_tmp_p->usHwVer,
				   db_v2x_tmp_p->usSwVer,
				   db_v2x_tmp_p->ePayloadType,
				   ntohl(db_v2x_tmp_p->ulPayloadLength),
				   ntohs(db_v2x_tmp_p->eRegionId),
				   sizeof(db_v2x_tmp_p->data));

			int payload_length = sizeof(db_v2x_tmp_p->data);//ntohl(db_v2x_tmp_p->ulPayloadLength);
			msgFrame = (MessageFrame_t *)malloc(payload_length);
			memcpy(msgFrame, &db_v2x_tmp_p->data, payload_length);

			BasicSafetyMessage_t *ptrBSM = &msgFrame->value.choice.BasicSafetyMessage;

			printf("\nV2X RX Test Msg>>\n"
				   "  ID         :  %ld\n"
				   "  CNT        :  %ld\n"
				   "  latitude   :  %ld\n"
				   "  longitude  :  %ld\n"
				   "  heading    :  %ld\n"
				   "  velocity   :  %ld\n",
				   ptrBSM->coreData.id.buf,
				   ptrBSM->coreData.msgCnt,
				   ptrBSM->coreData.lat,
				   ptrBSM->coreData.Long,
				   ptrBSM->coreData.heading, 
				   ptrBSM->coreData.speed);
		}
		usleep((1000 * tx_delay_g));
	}
	free(v2x_rx_pdu_p);
	free(db_v2x_tmp_p);
	free(msgFrame);
}

/* function : Process Commands */
int process_commands(void)
{
	v2x_rx_cmd_process(NULL);
	// pthread_t tx_thread;
	// pthread_t rx_thread;
	// void *tx_thread_ret;
	// void *rx_thread_ret;

	// pthread_create(&tx_thread, NULL, v2x_tx_cmd_process, NULL);
	// pthread_create(&rx_thread, NULL, v2x_rx_cmd_process, NULL);
	// pthread_join(tx_thread, &tx_thread_ret);
	// pthread_join(rx_thread, &rx_thread_ret);
	return -1;
}

/* function : Main(Entry point of this program) */
int main(int argc, char *argv[])
{
	int res;
	ros::init(argc,argv, "v2x");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);
	printf("Hello\n");
	do
	{
		if ((res = connect_v2x_socket()) < 0)
		{
			break;
		}
		v2x_wsr_cmd_process();
		if ((res = process_commands()) < 0)
		{
			break;
		}
	} while (ros::ok());

	close_v2x_socket();
	return res;
}

/////////////////////////////////////////////////////////////////////////////////////////
