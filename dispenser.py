from umqtt.simple import MQTTClient
from hcsr04 import HCSR04
import utime
import machine
from machine import Pin, PWM
import network
import micropython
import time
from utime import sleep

ssid= "" #nombre de la red wifi
wifipassword = "" #contraseña wifi

# Configuración de MQTT
mqtt_server = 'io.adafruit.com'
port = 1883
user=''
password = ''
client_id='Dispenser'
topic_SENSORDEPO='micacorvera/feeds/sensor HC-SR04'
topic_SENSORPORCION='micacorvera/feeds/sensor2 HC-SR04'
topic_TAMANIO='micacorvera/feeds/tamanioComida'
topic_SERVIR ='micacorvera/feeds/servo'
topic_TEMPORIZADOR='micacorvera/feeds/temporizador'
topic_MINUTOS='micacorvera/feeds/minutos'
topic_HORAS='micacorvera/feeds/horas'
topic_CONECTADO='micacorvera/feeds/conexion'


#variables de entrada en adafruit
temporizador_activo = False
usar_temporizador = temporizador_activo
boton_presionado = 0
servirComida = boton_presionado
slider_comida = 0
cantidad_tiempo = 0
establecer_horas = 0
establecer_minutos = 0
conectado = 0

#modo station
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)
#conectar a wifi
sta_if.connect(ssid, wifipassword)
print("Conectando\n")
while not sta_if.isconnected():
    print(".", end="")
    time.sleep(0.1)
print("\nConectado a wifi\n")
print(sta_if.ifconfig())

def callback(topic, msg):    
    global usar_temporizador, temporizador_activo, boton_presionado, servirComida, slider_comida, cantidad_tiempo, establecer_horas, establecer_minutos
    # Decodificar el mensaje
    dato = msg.decode('utf-8').strip().upper()
    topicrec = topic.decode('utf-8')
    print(f"Mensaje en tópico {topicrec}: {dato}")
    
    if topicrec == topic_TEMPORIZADOR:
        if dato == "OFF":
            temporizador_activo = False
        elif dato == "ON":
            temporizador_activo = True
        usar_temporizador = temporizador_activo 

    if topicrec == topic_SERVIR:
        if dato == '0':
            boton_presionado = 0
        elif dato == '1':
            boton_presionado = 1
        servirComida = boton_presionado

    if topicrec == topic_TAMANIO:
        slider_comida = int(dato)

    if topicrec == topic_HORAS:
        establecer_horas = int(dato)
        print(str(establecer_horas))

    if topicrec == topic_MINUTOS:
        establecer_minutos = int(dato)   
        print(str(establecer_minutos))

try:
    conexionMQTT = MQTTClient(client_id, mqtt_server, user=user, password=password, port=int(port))
    conexionMQTT.set_callback(callback)
    conexionMQTT.connect()
    conexionMQTT.subscribe(topic_TEMPORIZADOR)
    conexionMQTT.subscribe(topic_SERVIR)
    conexionMQTT.subscribe(topic_TAMANIO)
    conexionMQTT.subscribe(topic_HORAS)
    conexionMQTT.subscribe(topic_MINUTOS)

    print("\nConectado con Broker MQTT\n")
    esperando_respuesta = True
    conectado = 1
    conexionMQTT.publish(topic_CONECTADO, str(conectado))
    #tiempo de espera para poder leer el mensaje del dashboard
    tiempo_espera = time.time() + 10 
    while esperando_respuesta and time.time() < tiempo_espera:
        conexionMQTT.check_msg()
        time.sleep(5) 
        if usar_temporizador:
            esperando_respuesta = False
            
    print("Temporizador:"+str(usar_temporizador))
    print("Servir:"+str(servirComida))
    print("Horas:"+str(establecer_horas))
    print("Minutos:"+str(establecer_minutos))

except OSError as e:
    print("Fallo la conexion al broker, reiniciando...")
    time.sleep(5)
    machine.reset()


# Inicializar componentes

sliderComida = machine.ADC(1)

#inicializacion de variables
lectura_inicial = 0
tiempo_a_transcurrir = 0
seg_max_Comida= 4
gr_max_Comida = 50
distancia_lleno = 0
tiempo_inicial = 0
intervalo_comida_hs = 0
intervalo_comida_min = 0

#configuracion del servo
servo_pin = machine.PWM(machine.Pin(2))
servo_pin.freq(50)

#configuracion del sensor
sensorDepo = HCSR04(trigger_pin=5, echo_pin=4)
sensorPorcion = HCSR04(trigger_pin=7, echo_pin=6)

#preguntarle al usuario si quiere configurar un temporizador
def programar_temporizador():
    conexionMQTT.check_msg()
    time.sleep(2)
    global establecer_horas, establecer_minutos, intervalo_comida_hs, intervalo_comida_min
    if(usar_temporizador == True):
        intervalo_comida_hs = establecer_horas
        intervalo_comida_min = establecer_minutos
        print(str(intervalo_comida_hs)+","+str(intervalo_comida_min))

def leer_sensor(sensor):
    distancia = sensor.distance_cm()
    return distancia

print('Espere antes de llenar el depósito...')
time.sleep(0.5)
distancia_vacio_depo= leer_sensor(sensorDepo)
print('capacidad maxima:'+str(distancia_vacio_depo)+'cms')
print('Ya puede llenar el depósito')
time.sleep(5)
distancia_actual_depo = leer_sensor(sensorDepo)
print('Distancia actual: '+str(distancia_actual_depo)+'cms')

#Accionar el servo con botones
'''
def servo(degrees):
    # Checkear si el ángulo buscado está dentro del rango de trabajo
    if degrees > 360:
        degrees = 360
    if degrees < 0:
        degrees = 0
    
    # Valores min y max del duty cycle
    maxDuty = 10000
    minDuty = 0
    
    # Fórmula para pasar de grados a PWM/duty cycle
    # Duty Cycle = minimo + rango * proporción de ángulo
    newDuty = minDuty + (maxDuty - minDuty) * (degrees / 180)
    
    # Pasar el valor de duty al servo
    servo_pin.duty_u16(int(newDuty))

#función para hacer girar al servomotor, y dejar la valvula abierta por cierto tiempo según el tamaño de porción
def girar_servo():
    # Girar 180 grados
        for degree in range(0, 360, 1):
            servo(degree)
            sleep(0.02)
        print("Válvula cerrada")
'''

def ejecutar_por_tiempo(tiempo_espera, funcion):
    inicio = time.time()
    while time.time() - inicio < tiempo_espera:
        funcion()
        # Esperar el tiempo especificado
    

def capacidad_depo():
    distancia_actual_depo = leer_sensor(sensorDepo)
    porcentaje_actual=((distancia_vacio_depo-distancia_actual_depo)/(distancia_vacio_depo))*100
    print('EL dispenser está un '+str(porcentaje_actual)+'% lleno')

    if(porcentaje_actual <= 100 and porcentaje_actual >= 50):
        print("Hay suficiente comida")

    elif(porcentaje_actual <50 and porcentaje_actual>=20):
        print("Cuidado, podrías quedarte sin comida")

    elif(porcentaje_actual < 20 and porcentaje_actual >0):
        print("Te estás quedando sin comida! Llena antes de que tu mascota tenga hambre!")

    elif(porcentaje_actual <= 0):
        print("Te quedaste sin comida! Recarga el dispenser para que tu mascota no tenga hambre")

    return porcentaje_actual

def calcular_tiempo_llenado(distancia, seg):
   
    if distancia <= distancia_lleno:
        return 0
    # calcula el tiempo que falta si hay comida
    else: 
        diferencia = distancia - distancia_lleno
        tiempo_necesario = min(diferencia, seg)
        print(f"Diferencia: {diferencia:.1f} cm -> Tiempo: {tiempo_necesario:.1f}s")
        return tiempo_necesario

def rellenar_plato(seg, distancia):
    
    distancia_actual = leer_sensor(sensorPorcion)

    if distancia_actual > distancia:
        tiempo_llenado = calcular_tiempo_llenado(distancia_actual, seg)
        print(f"Rellenando plato por {tiempo_llenado:.1f} segundos...")
        ejecutar_por_tiempo(tiempo_llenado, girar_servo)
        sleep(1)
        
        nueva_distancia = leer_sensor(sensorPorcion)
        print(f"Nivel después del rellenado: {nueva_distancia:.1f} cm")
    elif distancia_actual == distancia_vacio:
        tiempo_llenado = calcular_tiempo_llenado(distancia_vacio, seg)
        print(f"Rellenando plato por {tiempo_llenado:.1f} segundos...")
        
        sleep(1)
    else:
        print("El plato ya está lleno")
        
    porcentaje_actual=(((distancia_vacio)-distancia_actual)/(distancia_vacio))*100
    conexionMQTT.publish(topic_SENSORPORCION, str(porcentaje_actual))
    
    return True

def detectar_comida_por_distancia(sensor):
    distancia = leer_sensor(sensor)
    # Si la distancia es menor o igual a lo que consideramos "vacio", hay comida
    if distancia < distancia_vacio:
        print(f"Comida detectada por distancia: {distancia} cm")
        return True
    else:
        print(f"No hay comida detectada. Distancia actual: {distancia} cm")
        return False

def servir_faltante(tiempo, distancia):
    hay_comida = detectar_comida_por_distancia(sensorPorcion)
    distancia_ahora = leer_sensor(sensorPorcion)
    print(f"Distancia actual: {distancia_ahora} cm")
        
    if hay_comida:
        print("Se detectó comida restante. Rellenando plato...")
        rellenar_plato(tiempo, distancia)
    elif distancia_ahora == distancia_vacio:
        print("El plato está vacío, sirviendo porción completa...")
        rellenar_plato(tiempo, distancia)
    else:
        print("Sirviendo porción completa...")
        ejecutar_por_tiempo(tiempo, girar_servo) 


print("Sistema de alimentación iniciado")

estado_depo = capacidad_depo()
tiempo = programar_temporizador()
(input('Ingrese el plato vacio para calcular la distancia: '))
distancia_vacio = leer_sensor(sensorPorcion)
print(f"La distancia de vacío es de {distancia_vacio}cm")


while True:
    try:
        # Verificar mensajes MQTT
        conexionMQTT.check_msg()
        time.sleep_ms(500)
    
        #define la porción la primera vez
        lectura_comida = slider_comida
        estado_depo = capacidad_depo()
        conexionMQTT.publish(topic_SENSORDEPO, str(estado_depo))

        if estado_depo>0:
            if lectura_comida != lectura_inicial :#and estado_depo>0
                lectura_inicial = lectura_comida
                tiempo_a_transcurrir = (lectura_comida * seg_max_Comida ) / gr_max_Comida
                ejecutar_por_tiempo(tiempo_a_transcurrir, girar_servo)
                time.sleep(2)
                distancia_lleno = leer_sensor(sensorPorcion)
                print(f"Nivel después del rellenado: {distancia_lleno} cm")

            if(servirComida==1 ):#and estado_depo>0
                servir_faltante(tiempo_a_transcurrir, distancia_lleno)
                servirComida=0
                time.sleep(5)

            programar_temporizador()
            print(str(intervalo_comida_hs))
            print(str(intervalo_comida_min))
            if (intervalo_comida_hs != 0 or intervalo_comida_min !=0): #and estado_depo>0
                print("La comida se servirá en "+str(36*intervalo_comida_hs + 6*intervalo_comida_min))
                time.sleep(36*intervalo_comida_hs + 6*intervalo_comida_min)
                print("Hora de comer!")
                servir_faltante(tiempo_a_transcurrir, distancia_lleno)
                estado_depo = capacidad_depo()
        
        elif estado_depo<=0:
            print('El dispenser no tiene comida! Recargar para que continue con su funcionamiento')
            estado_depo = capacidad_depo()
            time.sleep(1)
        
        sleep(1)  # Esperar 1 segundo entre iteraciones

    except OSError as e:
        print(f"Error en check_msg: {e}")
        time.sleep(1)
        configurar_mqtt()
        machine.reset()
    sleep(0.1)  