#!/usr/bin/env python3
import canopen
import time
import sys
import numpy as np
import termios
import tty
# Conexión MQTT
import paho.mqtt.client as mqtt
import json
# Configuración

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

LEFT_ID = 3
RIGHT_ID = 127
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
        msg = can.Message(arbitration_id=0x000,  data=[0x01, 0x00],is_extended_id=False)
        node.network.send_message(msg.arbitration_id, msg.data)
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
        topic_ang = f"/Thingworx/{DEVICE_LABEL}/Enconder_1"
        client.publish(topic_ang, json.dumps({"Encoder_1": angle}))
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

    state = 0
    sides_completed = 0
    angulo = get_angle_degrees(node_left)

    # Reiniciar encoders si es posible
    # Mover hasta enconder == 0
    state = 0
    sides_completed = 0
    start_angle = 0
    start_time = 0
    set_velocity(node_left, speed * 0.8)
    set_velocity(node_right, speed * 2.8)
    time.sleep(5.91)
    set_velocity(node_left, 0)
    set_velocity(node_right, 0)
    print("🟢 Iniciando dibujo del cuadrado...")
    

    """while sides_completed < 4:
        angulo = get_angle_degrees(node_left)  # Se asume función que devuelve grados del encoder

        # ───────────────────────────────────────────────
        # Estado 0: Alinear (corrección inicial de ángulo)
        # ───────────────────────────────────────────────
        if state == 0:
            if angulo > 5:
                set_velocity(node_left, speed * 0.8)
                set_velocity(node_right, speed * 2.8)
            else:
                set_velocity(node_left, 0)
                set_velocity(node_right, 0)
                state = 1
                start_angle = angulo
                print(f"➡️  Estado 1: avanzar lado {sides_completed + 1}")

        # ───────────────────────────────────────────────
        # Estado 1: Avanzar un lado del cuadrado
        # ───────────────────────────────────────────────
        elif state == 1:
            distancia = 0.075 * np.deg2rad(abs(angulo - start_angle))  # 0.075 ≈ radio de rueda (m)
            if distancia < side_length:
                set_velocity(node_left, speed * 0.8)
                set_velocity(node_right, speed * 2.8)
            else:
                set_velocity(node_left, 0)
                set_velocity(node_right, 0)
                sides_completed += 1
                state = 2
                start_time = time.time()
                print(f"✅ Lado {sides_completed} completado — girando...")

        # ───────────────────────────────────────────────
        # Estado 2: Giro de 90 grados
        # ───────────────────────────────────────────────
        elif state == 2:
            # Ajusta este tiempo según la velocidad del robot para un giro de 90°
            if time.time() - start_time < 1.15:
                set_velocity(node_left, speed * 0.8)
                set_velocity(node_right, -speed * 2.8)
            else:
                set_velocity(node_left, 0)
                set_velocity(node_right, 0)
                state = 3
                start_time = time.time()
                print("↪️ Giro completado — avanzando corto...")

        # ───────────────────────────────────────────────
        # Estado 3: Avance corto antes del siguiente lado
        # ───────────────────────────────────────────────
        elif state == 3:
            duracion = 8.8 if speed == 200 else 5.9
            if time.time() - start_time < duracion:
                set_velocity(node_left, speed * 0.8)
                set_velocity(node_right, speed * 2.8)
            else:
                set_velocity(node_left, 0)
                set_velocity(node_right, 0)
                state = 0  # 🔁 reiniciar ciclo
                print(f"🔁 Listo para siguiente lado ({sides_completed + 1}/4)\n")

        # ───────────────────────────────────────────────
        # Pequeña pausa para no saturar el bus CAN
        # ───────────────────────────────────────────────
        time.sleep(0.01)

    # Fin del cuadrado
    set_velocity(node_left, 0)
    set_velocity(node_right, 0)
    print("🏁 Cuadro completado exitosamente ✅")"""

        
            
   
            
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
            set_control_pid(left_node)
            set_control_pid(right_node)
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
            mover_cuadrado_m(left_node, right_node, side_length=1.0, speed=200)

def on_connect(client, userdata, flags, rc): 
    
    print("🟢 Conectado al broker MQTT con código:", rc) 
    on_topic = f"/Thingworx/{DEVICE_LABEL}/{ON_LABEL}" 
    up_topic = f"/Thingworx/{DEVICE_LABEL}/{UP_LABEL}" 
    down_topic = f"/Thingworx/{DEVICE_LABEL}/{DOWN_LABEL}" 
    right_topic = f"/Thingworx/{DEVICE_LABEL}/{RIGHT_LABEL}" 
    left_topic = f"/Thingworx/{DEVICE_LABEL}/{LEFT_LABEL}" 
    vel_topic = f"/Thingworx/{DEVICE_LABEL}/{VEL_LABEL}" 
    square_topic = f"/Thingworx/{DEVICE_LABEL}/{SQUARE_LABEL}"
    client.subscribe(on_topic) 
    client.subscribe(up_topic) 
    client.subscribe(down_topic) 
    client.subscribe(right_topic) 
    client.subscribe(left_topic) 
    client.subscribe(vel_topic) 
    print(f"📡 Suscrito al topic: {on_topic}") 
    print(f"📡 Suscrito al topic: {up_topic}\n") 
    print(f"📡 Suscrito al topic: {down_topic}\n") 
    print(f"📡 Suscrito al topic: {right_topic}\n") 
    print(f"📡 Suscrito al topic: {left_topic}\n") 
    print(f"📡 Suscrito al topic: {vel_topic}\n")
    print(f"📡 Suscrito al topic: {square_topic}\n")
def main():
    network = canopen.Network()
    
    network.connect(channel=CHANNEL, bustype='socketcan', bitrate=BAUDRATE)
    left_node = network.add_node(LEFT_ID, EDS_FILE)
    right_node = network.add_node(RIGHT_ID, EDS_FILE)
    client = mqtt.Client(userdata ={"left": left_node, "right": right_node})
    client.username_pw_set(ADMIN, PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(BROKER_EAFIT, 1883, 60)

    # Hilo para procesar mensajes CAN

    # Loop MQTT (solo comunicación)
    client.loop_start()


if __name__ == "__main__":
    main()
