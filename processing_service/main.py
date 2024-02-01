import json
import paho.mqtt.client as mqtt
import math

MQTT_BROKER_ENDPOINT = "industrial.api.ubidots.com"
MQTT_BROKER_PORT = 1883

mqttUserName = "BBUS-JiJQzKGd62TAdDpe4pQ5kDgKpH9YzZ"
mqttPass = "Procesing_Service"

topicTemperatura = "/v1.6/devices/monitor_node/temperatura/lv"
topicAceleraciones = "/v1.6/devices/monitor_node/aceleraciones"
topicHumedad = "/v1.6/devices/monitor_node/humedad/lv"

topicAnguloX = "/v1.6/devices/monitor_node/angulo_x"
topicAnguloY = "/v1.6/devices/monitor_node/angulo_y"
topicEstado = "/v1.6/devices/monitor_node/estado"
topicModoOperacion = "/v1.6/devices/monitor_node/modo_operacion"

MODO_OPERACION_NORMAL = 0
MODO_OPERACION_ALARMA = 1

TALUD_SALUDABLE = 0
TALUD_SINIESTRO = 1

client = mqtt.Client()

##### HIGH LEVEL API #####
def callbackTemperatura(value):
    print("(RX)~[temperatura] valor nuevo:", value)

def callbackHumedad(value):
    print("(RX)~[humedad] valor nuevo:", value)
    if float(value) >= 0.00:#50.00:
        publishModoOperacion(MODO_OPERACION_ALARMA)
    else:
        publishModoOperacion(MODO_OPERACION_NORMAL)

def callbackAceleraciones(x,y,z):
    print("(RX)~[aceleraciones] valores nuevos: x={},y={},z={}".format(x,y,z))
    
    ang_x = 90-(math.atan(x/math.sqrt(math.pow(y,2)+math.pow(z,2)))*180)/math.pi
    ang_y = 90-(math.atan(y/math.sqrt(math.pow(x,2)+math.pow(z,2)))*180)/math.pi
    publishAngulos(ang_x, ang_y)

def calulateModeLogic(ang_x,ang_y):
    diff_x = abs(90-ang_x)
    diff_y = abs(90-ang_y)

    if diff_x>=5 or diff_y>=5:
        publishModoOperacion(MODO_OPERACION_ALARMA)
    else:
        publishModoOperacion(MODO_OPERACION_NORMAL)

    if diff_x>=25 or diff_y>=25:
        publishEstado(TALUD_SINIESTRO)
    else:
        publishEstado(TALUD_SALUDABLE)

def publishModoOperacion(modo):
    if modo == MODO_OPERACION_NORMAL or modo == MODO_OPERACION_ALARMA:
        client.publish(topicModoOperacion, json.dumps({'value': modo}))
    else:
        print("(TX)~[modo_operacion] formato incorrecto al actualizar el valor")

def publishEstado(estado):
    if estado == TALUD_SALUDABLE or estado == TALUD_SINIESTRO:
        client.publish(topicEstado, json.dumps({'value': estado}))
    else:
        print("(TX)~[estado] formato incorrecto al actualizar el valor")
    
def publishAngulos(ang_x, ang_y):
    client.publish(topicAnguloX, json.dumps({'value': ang_x}))
    client.publish(topicAnguloY, json.dumps({'value': ang_y}))
    print("(TX)~[angulos] valores nuevos: x={},y={}".format(round(ang_x,0),round(ang_y,0)))
    
    calulateModeLogic(ang_x,ang_y)

##### CONNECTION HANDLE #####

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(topicTemperatura)
    client.subscribe(topicAceleraciones)
    client.subscribe(topicHumedad)

def on_message(client, userdata, msg):
    if msg.topic == topicAceleraciones:
        body = json.loads(msg.payload)
        callbackAceleraciones(body["context"]["accel_x"], body["context"]["accel_y"], body["context"]["accel_z"])
    elif msg.topic == topicHumedad:
        callbackHumedad(msg.payload)
    elif msg.topic == topicTemperatura:
        callbackTemperatura(msg.payload)

client.on_connect = on_connect
client.on_message = on_message

client.username_pw_set(username=mqttUserName, password=mqttPass)
client.connect(MQTT_BROKER_ENDPOINT, MQTT_BROKER_PORT, 60)

client.loop_forever()