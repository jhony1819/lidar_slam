#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Float32

def talker():
    # Configuración de la conexión serial
    ser = serial.Serial(
        port='/dev/ttyAMA0',  # Reemplaza con el puerto serial correcto
        baudrate=115200,
        timeout=1
    )

    pub = rospy.Publisher('/tf_luna_distance', Float32, queue_size=10)
    rospy.init_node('lidar_node', anonymous=True)
    rate = rospy.Rate(10)  # Frecuencia de publicación en Hz

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            data = ser.read(9)  # Lee los 9 bytes de datos del sensor
            if data[0] == 0x59 and data[1] == 0x59:  # Cabecera de datos
                distance = data[2] + data[3] * 256  # Calcula la distancia
                rospy.loginfo(f"Distancia medida: {distance} cm")
                pub.publish(distance / 100.0)  # Publica en metros

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

