import json
import paho.mqtt.client as mqtt
import math
import time
import threading

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

HUMIDITY_COOLDOWN = 1
ANGLE_COOLDOWN = 2

client = mqtt.Client()
waiting_thread = 0

humedad = 0
ang_x = 0
ang_y = 0

##### HIGH LEVEL API #####
def callbackTemperatura(value):
    print("(RX)~[temperatura] valor nuevo:", value)

def callbackHumedad(value):
    global humedad
    print("(RX)~[humedad] valor nuevo:", value)
    humedad = float(value)

def callbackAceleraciones(x,y,z):
    global ang_x,ang_y
    print("(RX)~[aceleraciones] valores nuevos: x={},y={},z={}".format(x,y,z))
    
    ang_x = 90-(math.atan(x/math.sqrt(math.pow(y,2)+math.pow(z,2)))*180)/math.pi
    ang_y = 90-(math.atan(y/math.sqrt(math.pow(x,2)+math.pow(z,2)))*180)/math.pi

    publishAngulos(ang_x,ang_y)
    
def publishModoOperacion(modo):
    if modo == MODO_OPERACION_NORMAL or modo == MODO_OPERACION_ALARMA:
        print("(TX)~[modo_operacion] actualizando el valor:", modo)
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
    
def calulateModeLogic():
    global ang_x, ang_y, humedad, waiting_thread
    print('(*) Calculating with values: ang_x={},ang_y={},humedad={}'.format(round(ang_x,0), round(ang_y,0), humedad))
    diff_x = abs(90-ang_x)
    diff_y = abs(90-ang_y)

    print('(*) diff_x={},diff_y={}'.format(round(diff_x,0),round(diff_y,0)))

    if diff_x>=5 or diff_y>=5 or humedad >= 50.0:
        publishModoOperacion(MODO_OPERACION_ALARMA)
    else:
        publishModoOperacion(MODO_OPERACION_NORMAL)

    if diff_x>=25 or diff_y>=25:
        publishEstado(TALUD_SINIESTRO)
    else:
        publishEstado(TALUD_SALUDABLE)

    # Reset to default
    ang_x = 0
    ang_y = 0
    humedad = 0
    waiting_thread = 0

##### TRANSPORT HANDLE #####
def wait_until(condition, interval=0.1, timeout=1):
  start = time.time()
  while not condition and time.time() - start < timeout:
    time.sleep(interval)

def waiting_loop(initTopic):
    print('(*) start waiting')

    if initTopic == topicAceleraciones:
        if humedad == 0:
            print('(*) sleeping until humidity received')
            wait_until(humedad!=0, timeout=HUMIDITY_COOLDOWN)
            print('(*) end waiting')
            calulateModeLogic()
        else:
            print('(*) end waiting')
            calulateModeLogic()
    elif initTopic == topicHumedad:
        print('(*) sleeping until humidity received')
        wait_until(ang_x!=0 and ang_y!=0, timeout=ANGLE_COOLDOWN)
        print('(*) end waiting')
        calulateModeLogic()

##### CONNECTION HANDLE #####

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(topicTemperatura)
    client.subscribe(topicAceleraciones)
    client.subscribe(topicHumedad)

def on_message(client, userdata, msg):
    global waiting_thread
    if waiting_thread == 0 and msg.topic != topicTemperatura:
        waiting_thread = threading.Thread(target=waiting_loop, args=(msg.topic,))
        waiting_thread.start()
    
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