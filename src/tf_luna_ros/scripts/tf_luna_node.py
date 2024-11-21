#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import serial
import rospy
from std_msgs.msg import String

# Configuración de los pines GPIO para UART
GPIO.setwarnings(False)
TX_PIN = 14  # GPIO14 para TX
RX_PIN = 15  # GPIO15 para RX

# Configuración de GPIO
def setup_gpio():
    GPIO.setmode(GPIO.BCM)  # Usar el esquema BCM para los números de GPIO
    GPIO.setup(TX_PIN, GPIO.OUT)  # TX_PIN como salida
    GPIO.setup(RX_PIN, GPIO.IN)   # RX_PIN como entrada

# Función para enviar y recibir datos
def talker():
    setup_gpio()
    
    uart = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)  # Cambia a /dev/serial0 si es necesario
    pub = rospy.Publisher('lidar_data', String, queue_size=10)
    rospy.init_node('tf_luna_node', anonymous=True)
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        if uart.in_waiting > 0:
            data = uart.readline().decode('utf-8').strip()
            rospy.loginfo(f"LIDAR data: {data}")
            rospy.loginfo("Publicando datos en el topic lidar_data")
            pub.publish(data)  # Publicar los datos
        else:
            rospy.loginfo("Esperando datos del UART...")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

