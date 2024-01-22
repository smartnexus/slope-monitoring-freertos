# Sistemas Ciberfísicos y Seguridad Hardware:  Trabajo de curso: slope-monitoring-freertos

Descripción de funcionalidad del trabajo.
## Monitoring Node (MN)
- **Objetivo**: monitorizar variables de interés
- **Funcionalidades**:
1. Monitorizar ```temperatura``` (sensor HTS21)
2. Monitorizar ```humedad``` (sensor HTS21)
3. Monitorizar ```ángulo base``` (acelerómetro LSM6DSL)
4. Modos de funcionamiento (controlados por el bróker MQTT):
    - **Normal**: monitoriza aceleraciones cada minuto --> Cada valor será la media de 10 aceleraciones.
    `Nota` Se busca que se midan 10 muestras totalmente seguidas y hagamos la media. Se cogen con la interrupcion del DRDY a la frecuencia que sea. A partir de ahí, espero.
    - **Alarma**: monitoriza aceleraciones cada 20 segundos --> Cada valor será la media de 10 aceleraciones
  
- **Tareas**:
  - **NetworkSetup**
    - Descripción: se encarga de establecer la conexión Wifi y se subscribe al topic MQTT modo_operacion.
    - StackSize: 512
    - Prioridad: BelowNormal
  - **StopWatch**
    - Descripción: temporiza según el modo de operación y notifica a NetworkSetup al finalizar.
    - StackSize: 128
    - Prioridad: High
  - **CollectMeasures**
    - Descripción: se encarga de medir la temperatura, la humedad y la aceleración. Al finalizar, añade todas las medidas a la cola MeasuresQueue.
    - StackSize: 512
    - Prioridad: AboveNormal
  - **MQTTPublish**
    - Descripción: Lee de la cola las medidas y las publica a los topics correspondientes mediante MQTT.
    - StackSize: 512
    - Prioridad: Low
- **Colas** 
  - MeasuresQueue
    - Tamaño: 3
    - Tipo de dato: uintptr_t
- **Periféricos**
    - **Configuración de cada periférico**
- **Interrupciones**
  - Acelerómetro
- **Notificaciones**
    - La tarea StopWatch envía notificación con flag 0x0001 para avisar a la tarea CollectMeasures que se ha finalizado la temporización y que puede comenzar a recolectar las medidas.
- **Librerías adicionales**
  - Sensor de temperatura
  - Sensor de humedad
  - Acelerómetro
  - Giróscopo
  - MQTT
  - Wifi

## Notifying Node (NN)
- **Objetivo**: sistema de notificaciones (mediante leds) local
- **Funcionalidades**: 
1. Si modo Normal --> Parpadea Led1 100ms cada 3 segundos
1. Si modo Alarma --> Parpadea Led2 500ms cada 1 segundos
- **Tareas**:
  - **NetworkSetup**
    - Descripción: se encarga de establecer la conexión Wifi y se subscribe al topic MQTT modo_operacion.
    - StackSize: 1024 
    - Prioridad: Normal
  - **LedsBlink**
    - Descripción: hacer parpadear los leds según los modos de operación.
    - StackSize: 1024 
    - Prioridad: AboveNormal
- **Colas**
  - No procede.
- **Periféricos**
    - **Configuración de cada periférico**
      - **LED 1:** PA5 - GPIO_OUT - LED1
      - **LED 2:** PB14 - GPIO_OUT - LED2
- **Interrupciones**
  - No procede.
- **Notificaciones**
  - No procede.

## MQTT Broker (MB)
- **Objetivo**: interconexiona todas las partes del sistema
- Se usará el [bróker de Ubidots](https://es.ubidots.com/)
- **Estructura de topics**:
    * /Monitor_Node/aceleraciones (publica MN, suscrito PS)
    * /Monitor_Node/temperatura (publica MN, suscrito PS)
    * /Monitor_Node/humedad (publica MN, suscrito PS)
    * /Monitor_Node/modo_operacion (publica PS, suscrito NN-MN)
    * /Monitor_Node/angulo_x (publica PS)
    * /Monitor_Node/angulo_y (publica PS)
    * /Monitor_Node/estado (publica PS)
**Nota:**
## Dashboard (ubidots)
- **Objetivo**: visualización remota de la información monitorizada/calculada
- Se usará el ofrecido por [Ubidots](https://es.ubidots.com/)
- **Método de representación de la información recibida y procesada**
## Processing service (PS)
-  **Objetivo**: servicio local que procesa la información existente en el bróker MQTT
- **Software**: Python
- **Algoritmo del servidor de procesamiento**:

    1º Subscripción a los topics de ```temperatura```, ```aceleracion``` y ```humedad```
    
    2º Cálculo del ```ángulo X/Y``` con respecto a Z y publicación en los topics correspondientes
    
    3º Publicar el modo de operación (0:normal,1:alarma) según el valor del ```ángulo X/Y``` 
    - Si ```humedad```>umbral fijado por los alumnos
    - Si ```ángulo X/Y``` ha variado 5 o más grados respecto al valor de referencia (90/90)

    4º Publicar el estado del talud (0:saludable,1:dañado) según el ángulo X/Y
    - Si ```ángulo X/Y``` ha variado 25 o más grados respecto al valor de referencia (90/90)
