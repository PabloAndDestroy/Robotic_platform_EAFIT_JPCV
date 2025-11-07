#!/usr/bin/env python3
import canopen
import time
import sys
import numpy as np
import termios
import tty
# Conexión MQTT
import paho.mqtt.client as mqtt
import Jetson.GPIO as GPIO
import json
# Configuración


LEFT_ID = 3
RIGHT_ID = 127

BROKER_EAFIT = "mqtt.dis.eafit.edu.co"          # label del dispositivo
DEVICE_LABEL = "Antonia_thing"
ADMIN = "admin"
PASSWORD = "semillero"
ON_LABEL = "Encender_base"          # label de la variable (API label del switch)
UP_LABEL = "Mover_adelante"       # label de la variable (API label del switch)
DOWN_LABEL = "Mover_atras"
RIGHT_LABEL = "Mover_derecha"
LEFT_LABEL = "Mover_izquierda"
VEL_LABEL = "Velocidad_motor"
SQUARE_LABEL = "Dibujar_cuadrado"
# Configuración de puerto CAN
CHANNEL = 'can0' # Canal en el que se conectan los Drivers
BAUDRATE = 125000 # Velocidad de comunicación
EDS_FILE = "BGE.eds" # Archivo EDS del Driver
ENCODER = "Encoder_1"
SENSOR_1 = "Sensor_1"
#-----------------------------Pines sensores----------------------------------------------------------#
ECHO_PIN = 11 
TRIG_PIN = 7



# CONFIGURACION INTERNA DEL PUERTO CAN PARA CONTROLAR LOS MOTORES

def reset_node(node):
    try:
        node.sdo['Controlword'].raw = 0x80  # Reset command
        print(f"🔄 Nodo {node.id} reseteado")
    except Exception as e:
        print(f"❌ Error reseteando nodo {node.id}: {e}")
# Control de teclado
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# Funciones de control
def enable_node(node):
    # Esta función pone los drivers en modo operativo (Leer EDS), aparte activa la lectura de los sensores.
    try:
        #node.nmt.state = 'OPERATIONAL'   # asegurar estado operativo
        #time.sleep(0.2)
        node.sdo['Power enable'].raw = 1
        node.sdo['Device mode'].raw=3
        node.sdo['Controlword'].raw=1
        node.sdo['Actual position'].raw=0
        node.sdo['Velocity feedback'][0].raw = 2410
        print(f"✅ Nodo {node.id} habilitado")

    except Exception as e:
        print(f"❌ Error habilitando nodo {node.id}: {e}")
# Apaga los drivers
def disable_node(node):
    try:
        node.sdo['Velocity - desired value'].raw = 0
        node.sdo['Power enable'].raw = 0
        print(f"⏹️ Nodo {node.id} detenido")
    except Exception as e:
        print(f"❌ Error deteniendo nodo {node.id}: {e}")

def set_velocity(node, rpm):
    try:
        node.sdo['Velocity - desired value'].raw = rpm

    except Exception as e:
        print(f"❌ Error enviando velocidad al nodo {node.id}: {e}")

#-------------------------------------------------------------------------------------------------#

# LECTURA DE VARIABLES DEL SISTEMA 

def get_velocity(node):
    try:
        # feedback for controller
        node.sdo['Velocity feedback'][0].raw = 1
        vel = node.sdo['Profile velocity'][0].raw
        return vel
    except Exception as e:
        print(f"❌ Error leyendo velocidad del nodo {node.id}: {e}")
        return 0
def print_velocity(node):
    try:
        vel = node.sdo['Measured velocity in increments'][0].raw
        print(f"Velocidad nodo {node.id}: {vel}")
    except Exception as e:
        print(f"❌ Error leyendo velocidad del nodo {node.id}: {e}")
def set_control_pid(node):
    node.sdo['PID-Controller - gain'].raw = 200
    node.sdo['PID-Controller - integral factor'].raw = 30
    node.sdo['PID-Controller - differential factor'].raw = 100
    node.sdo['PID-Controller - integral limit'].raw = 300
    node.sdo['PID-Controller - velocity feed forward'].raw = 1000
def giro_90(node_left, node_right, speed):
    while True:
        angulo = get_angle_degrees(node_right)
        while angulo < 90:
            angulo = get_angle_degrees(node_right)
            set_velocity(node_left, speed*0.8)
            set_velocity(node_right, -speed*2.8)

            break
def locate(node):
    try:
        # Leer posición cruda del encoder
        position = node.sdo['Actual encoder position'].raw

        # Leer la resolución configurada en el EDS (ej: 2000 counts/vuelta)
        resolution = node.sdo['Encoder resolution in counts'].raw

        # Normalizar la cuenta en una vuelta
        counts_in_turn = position % resolution

        return counts_in_turn
    except Exception as e:
        print(f"❌ Error leyendo posición del nodo {node.id}: {e}")
        return 0
def get_angle_degrees(node):
    """
    Devuelve el ángulo en grados [0, 360).
    """
    try:
        counts = locate(node)
        resolution = node.sdo['Encoder resolution in counts'].raw

        angle = (counts / resolution) * 360.0

        # Asegurar que quede en el rango [0, 360)
        angle = angle % 360.0
      #  topic_ang = f"/Thingworx/{DEVICE_LABEL}/Enconder_1"
     #   client.publish(topic_ang, json.dumps({"Encoder_1": angle}))
        return angle
    except Exception as e:
        print(f"❌ Error convirtiendo a grados nodo {node.id}: {e}")
        return 0
def print_velocity(node):
    try:
        vel = node.sdo['Measured velocity in increments'][0].raw
        print(f"Velocidad nodo {node.id}: {vel}")
    except Exception as e:
        print(f"❌ Error leyendo velocidad del nodo {node.id}: {e}")

def get_distance(echo, trig):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    
    # Asegurar que el trigger está en bajo
    GPIO.output(trig, False)
    time.sleep(0.05)  # Tiempo de estabilización
    
    # Enviar pulso ultrasónico
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    
    # Medir respuesta con timeouts para evitar bucles infinitos
    timeout_start = time.time()
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if pulse_start - timeout_start > 0.1:  # Timeout de 100ms
            return -1  # Error: no se detectó el pulso
    
    timeout_start = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end - timeout_start > 0.1:  # Timeout de 100ms
            return -1  # Error: pulso demasiado largo
    
    duracion = pulse_end - pulse_start
    
    # Calcular distancia (velocidad del sonido ~343 m/s a 20°C)
    # distancia = (duracion * 34300) / 2  # en cm
    distancia = (duracion * 34300) / 2
    
    return distancia
    
#-----------------------------------------------------------------------------------------------#

# Acciones de movimiento


def mover_linea_recta(node_left, node_right, distance, speed):
    #rpm a velocidad lineal
    #relacion de reduccion 1:80
    posicion_i = locate(node_left)
    posicion_d = locate(node_right)
    print(f"Nodo izq en: {posicion_i}, Node der en: {posicion_d}")
    time.sleep(3)
    diametro_rueda = 0.15
    t =  distance*diametro_rueda/(2*np.pi*speed)
    set_velocity(node_left, speed*0.8)
    set_velocity(node_right, speed*2.8)
    set_control_pid(node_left)
    set_control_pid(node_right)
    print(t)
    time.sleep(8.59)
    set_velocity(node_right, 0)
    set_velocity(node_left, 0)
    posicion_i = locate(node_left)
    posicion_d = locate(node_right)
    print(f"Nodo izq en: {posicion_i}, Node der en: {posicion_d}")

def mover_cuadrado_m(node_left, node_right, side_length, speed, R=0.15/2):
    """
    Mueve el robot en forma cuadrada usando control por encoders.

    node_left, node_right : nodos de los motores
    side_length           : longitud del lado del cuadrado (m)
    speed                 : velocidad base (RPM o equivalente)
    R                     : radio de la rueda (m)
    """

    print("🟢 Iniciando dibujo del cuadrado...")
    sides_completed = 0
    state = 0
    start_time = time.time()
    speed = 200
    tiempo_avance = side_length/0.169    # tiempo para avanzar un lado
    tiempo_giro = 2.2     # tiempo estimado para girar 90°

    while sides_completed < 4:
        if state == 0:
            print(f"➡️ Avanzando lado {sides_completed + 1}")
            set_velocity(node_left, speed * 0.8)
            set_velocity(node_right, speed * 2.8)
            start_time = time.time()
            state = 1

        elif state == 1:
            if time.time() - start_time >= tiempo_avance:
                set_velocity(node_left, 0)
                set_velocity(node_right, 0)
                sides_completed += 1
                print(f"✅ Lado {sides_completed} completado — girando...")
                time.sleep(1)
                start_time = time.time()
                state = 2

        elif state == 2:
            if time.time() - start_time < tiempo_giro:
                set_velocity(node_left, speed * 0.8)
                set_velocity(node_right, -speed * 2.8)
            else:
                set_velocity(node_left, 0)
                set_velocity(node_right, 0)
                state = 0
                time.sleep(1)

        time.sleep(0.05)

    set_velocity(node_left, 0)
    set_velocity(node_right, 0)
    print("🏁 Cuadro completado ✅")



def girar_rueda_360(node_left, node_right, speed):
    angulo = 0
    while True:
        angulo = get_angle_degrees(node_left)
        set_velocity(node_left, speed*0.8)
        set_velocity(node_right, -speed*2.8)
        if angulo >= 360:
            set_velocity(node_left, 0)
            set_velocity(node_right, 0)
            break


class Estado_vel:
    rpm = 0


def on_message(client, userdata, msg):
    left_node = userdata["left"]
    right_node = userdata["right"]

    if msg.topic == f"/Thingworx/{DEVICE_LABEL}/{ON_LABEL}":
        payload = json.loads(msg.payload)
        if payload == 1:
            enable_node(left_node)
            enable_node(right_node)
            #set_control_pid(left_node)
            #set_control_pid(right_node)
        elif payload == 0:
            disable_node(left_node)
            disable_node(right_node)

    elif msg.topic == f"/Thingworx/{DEVICE_LABEL}/{VEL_LABEL}":
        payload = json.loads(msg.payload)

        print(f"📩 Mensaje recibido en {VEL_LABEL}: {payload}")
        Estado_vel.rpm = float((payload))
        Estado_vel.rpm = int(Estado_vel.rpm)

    elif msg.topic == f"/Thingworx/{DEVICE_LABEL}/{UP_LABEL}":
        payload = json.loads(msg.payload)
        print(f"📩 Mensaje recibido en {UP_LABEL}: {payload}")
        if payload ==1:
            set_velocity(left_node, Estado_vel.rpm*0.8)
            set_velocity(right_node, Estado_vel.rpm*2.8)
            #time.sleep(0.5)
        elif payload==0:
            set_velocity(left_node, 0)
            set_velocity(right_node, 0)
    elif msg.topic == f"/Thingworx/{DEVICE_LABEL}/{DOWN_LABEL}":
        payload = json.loads(msg.payload)
        print(f"📩 Mensaje recibido en {DOWN_LABEL}: {payload}")
        if payload == 1:
            set_velocity(left_node, -1*Estado_vel.rpm*0.8)
            set_velocity(right_node, -1*Estado_vel.rpm*2.8)
            #time.sleep(0.5)
        elif payload==0:
            set_velocity(left_node, 0)
            set_velocity(right_node, 0)
    elif msg.topic == f"/Thingworx/{DEVICE_LABEL}/{RIGHT_LABEL}":
        payload = json.loads(msg.payload)
        print(f"📩 Mensaje recibido en {RIGHT_LABEL}: {payload}")
        if payload == 1:
            set_velocity(left_node, Estado_vel.rpm*0.8)
            set_velocity(right_node, -Estado_vel.rpm*2.8)
        elif payload==0:
            set_velocity(left_node, 0)
            set_velocity(right_node, 0)
    elif msg.topic == f"/Thingworx/{DEVICE_LABEL}/{LEFT_LABEL}":
        payload = json.loads(msg.payload)
        print(f"📩 Mensaje recibido en {LEFT_LABEL}: {payload}")
        if payload == 1:
            set_velocity(left_node, -Estado_vel.rpm*0.8)
            set_velocity(right_node, Estado_vel.rpm*2.8)
        elif payload==0:
            set_velocity(left_node, 0)
            set_velocity(right_node, 0)
    elif msg.topic == f"/Thingworx/{DEVICE_LABEL}/{SQUARE_LABEL}":
        payload = json.loads(msg.payload)
        print(f"📩 Mensaje recibido en {SQUARE_LABEL}: {payload}")
        if payload == 1:
            mover_cuadrado_m(left_node, right_node, 1, 200, R=0.15/2)


def on_connect(client, userdata, flags, rc):

    print("🟢 Conectado al broker MQTT con código:", rc)
    on_topic = f"/Thingworx/{DEVICE_LABEL}/{ON_LABEL}"
    up_topic = f"/Thingworx/{DEVICE_LABEL}/{UP_LABEL}"
    down_topic = f"/Thingworx/{DEVICE_LABEL}/{DOWN_LABEL}"
    right_topic = f"/Thingworx/{DEVICE_LABEL}/{RIGHT_LABEL}"
    left_topic = f"/Thingworx/{DEVICE_LABEL}/{LEFT_LABEL}"
    vel_topic = f"/Thingworx/{DEVICE_LABEL}/{VEL_LABEL}"
    square_topic = f"/Thingworx/{DEVICE_LABEL}/{SQUARE_LABEL}"
    enconder_topic = f"/Thingworx/{DEVICE_LABEL}/{ENCODER}"
    ultrasonic_topic = f"/Thingworx/{DEVICE_LABEL}/{SENSOR_1}"
    client.subscribe(on_topic)
    client.subscribe(up_topic)
    client.subscribe(down_topic)
    client.subscribe(right_topic)
    client.subscribe(left_topic)
    client.subscribe(vel_topic)
    client.subscribe(square_topic)
    #angulo = get_angle_degrees(userdata["right"])
    #client.publish(enconder_topic, json.dumps({f"{ENCODER}": angulo}))
    #client.publish(ultrasonic_topic, json.dumps({f"{SENSOR_1}": get_distance(ECHO_PIN, TRIG_PIN)}))
    print(f"📡 Suscrito al topic: {on_topic}")
    print(f"📡 Suscrito al topic: {up_topic}\n")
    print(f"📡 Suscrito al topic: {down_topic}\n")
    print(f"📡 Suscrito al topic: {right_topic}\n")
    print(f"📡 Suscrito al topic: {left_topic}\n")
    print(f"📡 Suscrito al topic: {vel_topic}\n")
    print(f"📡 Suscrito al topic: {square_topic}\n")
    
def enviar_datos(client, left_node, right_node):
    ultrasonic_topic = f"/Thingworx/{DEVICE_LABEL}/{SENSOR_1}"
    encoder_topic = f"/Thingworx/{DEVICE_LABEL}/{ENCODER}"

    
    try:
        distancia = get_distance(ECHO_PIN, TRIG_PIN)
        angulo = get_angle_degrees(right_node)

        client.publish(ultrasonic_topic, json.dumps({SENSOR_1: distancia}))
        client.publish(encoder_topic, json.dumps({ENCODER: angulo}))

        print(f"📤 Sensor_1 = {distancia:.2f} cm | Encoder_1 = {angulo:.2f}°")
        time.sleep(1)

    except Exception as e:
        print(f"⚠️ Error enviando datos: {e}")
        time.sleep(2)
def main():
    network = canopen.Network()
    network.connect(channel=CHANNEL, bustype='socketcan', bitrate=BAUDRATE)
    left_node = network.add_node(LEFT_ID, EDS_FILE)
    right_node = network.add_node(RIGHT_ID, EDS_FILE)
    #enable_node(left_node)
    #enable_node(right_node)
    #set_velocity(left_node, 0)
    #set_velocity(right_node, 0)
    """try:
        network.connect(channel=CHANNEL, bustype='socketcan', bitrate=BAUDRATE)

        client = mqtt.Client(userdata ={"left": left_node, "right": right_node})
        client.username_pw_set(ADMIN, PASSWORD)
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(BROKER_EAFIT, 1883, 60)
    except Exception as e:
        print(f"❌ Error de conexión: {e}")
        return"""

    # Hilo para procesar mensajes CAN

    # Loop MQTT (solo comunicación)
    # Selección: control por teclado o MQTT
    modo = int(input("Seleccione modo de control (1: teclado, 2: Thingworx): "))
    if modo ==1:
        enable_node(left_node)
        enable_node(right_node)
        set_velocity(left_node, 0)
        set_velocity(right_node, 0)
        print("➡️ Modo control por teclado seleccionado")
        print("Controles:")
        print(" W: Avanzar")
        print(" S: Retroceder")
        print(" A: Girar izquierda")
        print(" D: Girar derecha")
        print(" +: Aumentar RPM")
        print(" -: Disminuir RPM")
        print(" J: Mover en línea recta (m)")
        print(" Q: Salir")
        try:
            rpm=0
            while True:
                print_velocity(left_node)
                print_velocity(right_node)
                pos_counts = locate(right_node)
                pos_deg = get_angle_degrees(right_node)

                print(f"Posición nodo der (counts): {pos_counts}")
                print(f"Posición nodo der (grados): {pos_deg:.2f}°")
                key = get_key().lower()
                if key == "q":
                    break
                elif key == "w":
                    set_velocity(right_node, rpm*2.8)
                    set_control_pid(right_node)
                    set_velocity(left_node, rpm*0.7)
                    print(f"Posición nodo der (grados): {pos_deg:.2f}°")


                elif key == "s":
                    set_velocity(left_node, -rpm*0.7)
                    set_velocity(right_node, -rpm*2.8)
                    print_velocity(left_node)
                    print_velocity(right_node)
                elif key == "a":
                    set_velocity(right_node, rpm*2.8)
                    set_velocity(left_node, -rpm*0.8)
                elif key == "d":
                    set_velocity(left_node, rpm*0.7)
                    set_velocity(right_node, -rpm*2.5)
                elif key == "+":
                    rpm += 10
                    print(f"⚡ RPM = {rpm}")
                elif key == "-":
                    rpm = max(0, rpm - 10)
                    print(f"⚡ RPM = {rpm}")
                elif key == "j":
                    d = float(input("Distancia a mover (m): "))

                    if d > 5:
                        print("❌ Distancia demasiado grande")
                        break

                    mover_cuadrado_m(left_node, right_node, d, 200, R=0.15/2)
                else:
                # Soltar teclas = detener
                    set_velocity(left_node, 0)
                    set_velocity(right_node, 0)




      #  client.loop_forever()
        except KeyboardInterrupt:
            print("🔴 Programa terminado por el usuario")
        finally:
        # Apagar nodos al salir
            disable_node(left_node)
            disable_node(right_node)
            network.disconnect()
            print("✅ Motores apagados y red desconectada")
    elif modo ==2:
        print("➡️ Modo Thingworx seleccionado")
        client = mqtt.Client(userdata ={"left": left_node, "right": right_node})
        client.username_pw_set(ADMIN, PASSWORD)
        client.on_connect = on_connect
        client.on_message = on_message
        distancia = get_distance(ECHO_PIN, TRIG_PIN)
        print(f"distancia sensor: {distancia}")
        

        try:
            client.connect(BROKER_EAFIT, 1883, 60)
            enviar_datos(client, left_node, right_node)
            client.loop_forever()
        except KeyboardInterrupt:
            print("🔴 Programa terminado por el usuario")
        finally:
        # Apagar nodos al salir
            disable_node(left_node)
            disable_node(right_node)
            network.disconnect()
            print("✅ Motores apagados y red desconectada")



if __name__ == "__main__":
    main()