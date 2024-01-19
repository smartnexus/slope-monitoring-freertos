/* Private functions to implement MQTT app */
/* aluque 2022-12-21 */

#include <core_mqtt.h>
#include <mqtt_priv.h>
#include <main.h>
//#include "stm32l475e_iot01.h"

static uint8_t ucSharedBuffer[NETWORK_BUFFER_SIZE];
/** @brief Static buffer used to hold MQTT messages being sent and received. */
static MQTTFixedBuffer_t xBuffer = { ucSharedBuffer,
NETWORK_BUFFER_SIZE };

typedef struct topicFilterContext {
	const char *pcTopicFilter;
	MQTTSubAckStatus_t xSubAckStatus;
} topicFilterContext_t;

static topicFilterContext_t xTopicFilterContext[TOPIC_COUNT] = { {
topicModoOperacion, MQTTSubAckFailure } };
static uint16_t usSubscribePacketIdentifier;
/**
 * @brief Global entry time into the application to use as a reference timestamp
 * in the #prvGetTimeMs function. #prvGetTimeMs will always return the difference
 * between the current time and the global entry time. This will reduce the chances
 * of overflow for the 32 bit unsigned integer used for holding the timestamp.
 */
static uint32_t ulGlobalEntryTimeMs;

TransportStatus_t prvConnectToServer(NetworkContext_t *pxNetworkContext) {
	TransportStatus_t xNetworkStatus;
	uint8_t ret;
	uint8_t ipaddr[4] = MQTT_BROKER_ENDPOINT_IP;

	/* Attempt to connect to MQTT broker. */
	do {
		/* Establish a TCP connection with the MQTT broker. */
		printf("Create a TCP connection to %s:%d.\n", MQTT_BROKER_ENDPOINT,
		MQTT_BROKER_PORT);
		ret = WIFI_OpenClientConnection(SOCKET, WIFI_TCP_PROTOCOL, "mqtt", ipaddr, MQTT_BROKER_PORT, 0);
		if (ret != WIFI_STATUS_OK) {
			printf("Error in opening MQTT connection: %d\n", ret);
			osDelay(pdMS_TO_TICKS(10000));
		} else {
			pxNetworkContext->socket = SOCKET;
			pxNetworkContext->socket_open = 1;
			memcpy(pxNetworkContext->ipaddr, ipaddr, 4 * sizeof(uint8_t));
			pxNetworkContext->remote_port = MQTT_BROKER_PORT;
			xNetworkStatus = PLAINTEXT_TRANSPORT_SUCCESS;
		}

	} while ((xNetworkStatus != PLAINTEXT_TRANSPORT_SUCCESS));

	return PLAINTEXT_TRANSPORT_SUCCESS;
}

void prvCreateMQTTConnectionWithBroker(MQTTContext_t *pxMQTTContext, NetworkContext_t *pxNetworkContext) {
	MQTTStatus_t xResult;
	MQTTConnectInfo_t xConnectInfo;
	bool xSessionPresent;
	TransportInterface_t xTransport;

	/* Fill in Transport Interface send and receive function pointers. */
	init_transport_from_socket(pxNetworkContext->socket, 1, pxNetworkContext, &xTransport);
	/* Initialize MQTT library. */
	xResult = MQTT_Init(pxMQTTContext, &xTransport, prvGetTimeMs, prvEventCallback, &xBuffer);

	configASSERT(xResult == MQTTSuccess);
	printf("MQTT initialized\n");

	/* Many fields not used in this demo so start with everything at 0. */
	(void) memset((void*) &xConnectInfo, 0x00, sizeof(xConnectInfo));

	/* Start with a clean session i.e. direct the MQTT broker to discard any
	 * previous session data. Also, establishing a connection with clean
	 * session will ensure that the broker does not store any data when this
	 * client gets disconnected. */
	xConnectInfo.cleanSession = true;

	/* The client identifier is used to uniquely identify this MQTT client to
	 * the MQTT broker. In a production device the identifier can be something
	 * unique, such as a device serial number. */
	xConnectInfo.pClientIdentifier = MQTTCLIENT_IDENTIFIER;
	xConnectInfo.clientIdentifierLength = (uint16_t) strlen(
	MQTTCLIENT_IDENTIFIER);
//	xConnectInfo.pUserName = mqttUserName;
//	xConnectInfo.userNameLength = strlen(mqttUserName);
//	xConnectInfo.pPassword = mqttPass;
//	xConnectInfo.passwordLength = strlen(mqttPass);

	/* Set MQTT keep-alive period. It is the responsibility of the application
	 * to ensure that the interval between Control Packets being sent does not
	 * exceed the Keep Alive value.  In the absence of sending any other
	 * Control Packets, the Client MUST send a PINGREQ Packet. */
	xConnectInfo.keepAliveSeconds = 60U;
	;

	/* Send MQTT CONNECT packet to broker. LWT is not used in this demo, so it
	 * is passed as NULL. */
	xResult = MQTT_Connect(pxMQTTContext, &xConnectInfo,
	NULL, 1000U, &xSessionPresent);
	configASSERT(xResult == MQTTSuccess);
	printf("MQTT connected to broker\n");

}

void prvMQTTPublishToTopic(MQTTContext_t *pxMQTTContext, char *topic, void *payload) {
	MQTTStatus_t xResult;
	MQTTPublishInfo_t xMQTTPublishInfo;

	/* Some fields are not used by this demo so start with everything at 0. */
	(void) memset((void*) &xMQTTPublishInfo, 0x00, sizeof(xMQTTPublishInfo));

	/* This demo uses QoS0. */
	xMQTTPublishInfo.qos = MQTTQoS0;
	xMQTTPublishInfo.retain = false;
	xMQTTPublishInfo.pTopicName = topic;
	xMQTTPublishInfo.topicNameLength = (uint16_t) strlen(topic);
	xMQTTPublishInfo.pPayload = payload;
	xMQTTPublishInfo.payloadLength = strlen(payload);

	/* Send PUBLISH packet. Packet ID is not used for a QoS0 publish. */
	xResult = MQTT_Publish(pxMQTTContext, &xMQTTPublishInfo, 0U);
	if (xResult == MQTTSuccess)
		printf("Published to topic %s\n", topic);
	//configASSERT( xResult == MQTTSuccess );
}

void prvMQTTSubscribeToTopic(MQTTContext_t *pxMQTTContext, char *topic) {
	MQTTStatus_t xResult = MQTTSuccess;
	MQTTSubscribeInfo_t xMQTTSubscription[TOPIC_COUNT];
	bool xFailedSubscribeToTopic = false;

	/* Some fields not used by this demo so start with everything at 0. */
	(void) memset((void*) &xMQTTSubscription, 0x00, sizeof(xMQTTSubscription));

	/* Each packet requires a unique ID. */
	usSubscribePacketIdentifier = MQTT_GetPacketId(pxMQTTContext);

	/* Subscribe to the pcExampleTopic topic filter. This example subscribes
	 * to only one topic and uses QoS0. */
	xMQTTSubscription[0].qos = MQTTQoS0;
	xMQTTSubscription[0].pTopicFilter = topic;
	xMQTTSubscription[0].topicFilterLength = strlen(topic);

	do {
		/* The client is already connected to the broker. Subscribe to the topic
		 * as specified in pcExampleTopic by sending a subscribe packet then
		 * waiting for a subscribe acknowledgment (SUBACK). */
		xResult = MQTT_Subscribe(pxMQTTContext, xMQTTSubscription, 1, /* Only subscribing to one topic. */
		usSubscribePacketIdentifier);
		if (xResult == MQTTSuccess)
			printf("Subscription to %s, result: %d, success\n", topic, xResult);
		else
			printf("Subscription to %s, result: %d, failed\n", topic, xResult);
		//configASSERT( xResult == MQTTSuccess );

		/* Process incoming packet from the broker. After sending the
		 * subscribe, the client may receive a publish before it receives a
		 * subscribe ack. Therefore, call generic incoming packet processing
		 * function. Since this demo is subscribing to the topic to which no
		 * one is publishing, probability of receiving Publish message before
		 * subscribe ack is zero; but application must be ready to receive any
		 * packet.  This demo uses the generic packet processing function
		 * everywhere to highlight this fact. Note there is a separate demo that
		 * shows how to use coreMQTT in a thread safe way – in which case the
		 * MQTT protocol runs in the background and this call is not required. */
		/* For version 1.1.0: xResult = MQTT_ProcessLoop( pxMQTTContext, 1000 ); */
		xResult = MQTT_ProcessLoop(pxMQTTContext);
		//configASSERT( xResult == MQTTSuccess );

		/* Reset flag before checking suback responses. */
		xFailedSubscribeToTopic = false;

		/* Check if recent subscription request has been rejected.
		 * #xTopicFilterContext is updated in the event callback (shown in a
		 * code block below) to reflect the status of the SUBACK sent by the
		 * broker. It represents either the QoS level granted by the server upon
		 * subscription, or acknowledgment of server rejection of the
		 * subscription request. */
		if (xTopicFilterContext[0].xSubAckStatus == MQTTSubAckFailure) {
			xFailedSubscribeToTopic = true;
			break;
		}

	} while (xFailedSubscribeToTopic == true);
}

void prvMQTTProcessIncomingPublish(MQTTPublishInfo_t *pxPublishInfo) {
	char buffer1[128];
	char buffer2[128];

	// pPayload no termina en \0, hay que copiarlo en un buffer para imprimirlo. Lo mismo con pTopicName
	memcpy(buffer1, pxPublishInfo->pPayload, min(127, pxPublishInfo->payloadLength));
	buffer1[min(1023, pxPublishInfo->payloadLength)] = '\0';
	memcpy(buffer2, pxPublishInfo->pTopicName, min(127, pxPublishInfo->topicNameLength));
	buffer2[min(1023, pxPublishInfo->topicNameLength)] = '\0';

	printf("Topic \"%s\": publicado \"%s\"\n", buffer2, buffer1);
	MQTTSubscribeCallback(buffer2, buffer1);
}

uint32_t prvGetTimeMs(void) {
#define MILLISECONDS_PER_TICK ( 1000 / configTICK_RATE_HZ )
	TickType_t xTickCount = 0;
	uint32_t ulTimeMs = 0UL;

	/* Get the current tick count. */
	xTickCount = xTaskGetTickCount();

	/* Convert the ticks to milliseconds. */
	ulTimeMs = (uint32_t) xTickCount * MILLISECONDS_PER_TICK;

	/* Reduce ulGlobalEntryTimeMs from obtained time so as to always return the
	 * elapsed time in the application. */
	ulTimeMs = (uint32_t) (ulTimeMs - ulGlobalEntryTimeMs);

	return ulTimeMs;
}

void prvEventCallback(MQTTContext_t *pxMQTTContext, MQTTPacketInfo_t *pxPacketInfo, MQTTDeserializedInfo_t *pxDeserializedInfo) {
	/* The MQTT context is not used for this demo. */
	(void) pxMQTTContext;

	if ((pxPacketInfo->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH) {
		// procesar un paquete PUBLISH recibido,
		prvMQTTProcessIncomingPublish(pxDeserializedInfo->pPublishInfo);
	} else {
		// también se podría hacer algo con otros paquetes si fuera necesario
		//prvMQTTProcessResponse(pxPacketInfo, pxDeserializedInfo->packetIdentifier);
	}

}

//static void prvMQTTProcessResponse(MQTTPacketInfo_t *pxIncomingPacket, uint16_t usPacketId) {
//	MQTTStatus_t xResult = MQTTSuccess;
//	uint8_t *pucPayload = NULL;
//	size_t ulSize = 0;
//
//	switch (pxIncomingPacket->type) {
//		case MQTT_PACKET_TYPE_SUBACK:
//
//			/* A SUBACK from the broker, containing the server response to our
//			 * subscription request, has been received.  It contains the status
//			 * code indicating server approval/rejection for the subscription to
//			 * the single topic requested. The SUBACK will be parsed to obtain
//			 * the status code, and this status code will be stored in
//			 * #xTopicFilterContext. */
//			xResult = MQTT_GetSubAckStatusCodes(pxIncomingPacket, &pucPayload, &ulSize);
//
//			/* MQTT_GetSubAckStatusCodes always returns success if called with
//			 * packet info from the event callback and non-NULL parameters. */
//			configASSERT(xResult == MQTTSuccess)
//			;
//
//			/* This should be the QOS leve, 0 in this case. */
//			xTopicFilterContext[0].xSubAckStatus = *pucPayload;
//
//			/* Make sure ACK packet identifier matches with Request packet
//			 * identifier. */
//			configASSERT(usSubscribePacketIdentifier == usPacketId)
//			;
//			break;
//
//		case MQTT_PACKET_TYPE_UNSUBACK:
//			printf("Unsubscribed from the topic.");
//			/* Make sure ACK packet identifier matches with Request packet
//			 * identifier. */
//			//configASSERT( usUnsubscribePacketIdentifier == usPacketId );
//			break;
//
//		case MQTT_PACKET_TYPE_PINGRESP:
//
//			/* Nothing to be done from application as library handles
//			 * PINGRESP with the use of MQTT_ProcessLoop API function. */
//			printf("PINGRESP should not be handled by the application " "callback when using MQTT_ProcessLoop.\n");
//			break;
//
//			/* Any other packet type is invalid. */
//		default:
//			printf("prvMQTTProcessResponse() called with unknown packet type:(%02X).", pxIncomingPacket->type);
//	}
//}
