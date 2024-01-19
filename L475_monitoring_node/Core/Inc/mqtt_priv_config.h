/* Private functions for coreMQTT */
/* aluque 2022-12-21 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MQTT_PRIV_CONFIG_H
#define __MQTT_PRIV_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define MQTT_BROKER_ENDPOINT "industrial.api.ubidots.com"
#define MQTT_BROKER_ENDPOINT_IP {169,55,61,243}
#define MQTT_BROKER_PORT (1883)
#define MQTTCLIENT_IDENTIFIER "pNaajmx3by49ZTdMuNWMmV"
#define TOPIC_COUNT (1)

#define topicModoOperacion "/v1.6/devices/monitor_node/modo_operacion/lv"
#define topicTemperatura "/v1.6/devices/monitor_node/temperatura/lv"
#define topicAceleraciones "/v1.6/devices/monitor_node/aceleraciones/lv"
#define topicHumedad "/v1.6/devices/monitor_node/humedad/lv"
#define topicGeneral "/v1.6/devices/monitor_node"

// Define strings for these parameters or set them to NULL. Do not use empty string ("")
#define mqttUserName "BBUS-JiJQzKGd62TAdDpe4pQ5kDgKpH9YzZ"
#define clientID NULL
#define mqttPass "Monitor_Node"


#ifdef __cplusplus
extern "C" {
#endif

#endif /* __MQTT_PRIV_CONFIG_H */
