/* Private functions for coreMQTT */
/* aluque 2022-12-21 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MQTT_PRIV_CONFIG_H
#define __MQTT_PRIV_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define MQTT_BROKER_ENDPOINT "test.mosquitto.org"
#define MQTT_BROKER_ENDPOINT_IP {91,121,93,94}
#define MQTT_BROKER_PORT (1883)
#define MQTTCLIENT_IDENTIFIER "piCscpTZp8M68HfCY67BwN"
#define TOPIC_COUNT (1)

#define topicModoOperacion "Monitor_Node/modo_operacion"

// Define strings for these parameters or set them to NULL. Do not use empty string ("")
#define mqttUserName NULL
#define clientID NULL
#define mqttPass NULL

#ifdef __cplusplus
extern "C" {
#endif

#endif /* __MQTT_PRIV_CONFIG_H */
