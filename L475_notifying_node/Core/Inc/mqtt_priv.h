/* Private functions for coreMQTT */
/* aluque 2022-12-21 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MQTT_PRIV_H
#define __MQTT_PRIV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include <wifi.h>
#include <core_mqtt.h>
#include <mqtt_priv_config.h>

#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

#define NETWORK_BUFFER_SIZE (1000U)
#ifndef SOCKET
#define SOCKET 0
#endif

TransportStatus_t prvConnectToServer(NetworkContext_t*);
void prvCreateMQTTConnectionWithBroker(MQTTContext_t*, NetworkContext_t*);
void prvMQTTPublishToTopic(MQTTContext_t*, char*, void*);
void prvMQTTSubscribeToTopic(MQTTContext_t*, char*);
uint32_t prvGetTimeMs(void);
void prvEventCallback(MQTTContext_t*, MQTTPacketInfo_t*, MQTTDeserializedInfo_t*);
void init_transport_from_socket(uint8_t, uint8_t, NetworkContext_t*, TransportInterface_t*);
void prvMQTTProcessIncomingPublish(MQTTPublishInfo_t*);
//static void prvMQTTProcessResponse(MQTTPacketInfo_t *pxIncomingPacket, uint16_t usPacketId);

#ifdef __cplusplus
extern "C" {
#endif

#endif /* __MQTT_PRIV_H */
