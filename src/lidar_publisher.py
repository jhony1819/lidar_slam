#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String  # Cambia el tipo de mensaje si es necesario

def lidar_publisher():
    rospy.init_node('lidar_publisher', anonymous=True)
    pub = rospy.Publisher('/lidar_data', String, queue_size=10)  # Cambia 'String' si es necesario

    # Configura el puerto serial para AMA0
    try:
        ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)  # Cambia la velocidad de baudios si es necesario
    except serial.SerialException:
        rospy.logerr("Error al conectar con el puerto serial /dev/ttyAMA0")
        return

    rate = rospy.Rate(10)  # Frecuencia de publicación de 10 Hz

    while not rospy.is_shutdown():
        try:
            # Lee una línea del puerto serial
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                rospy.loginfo(f"Data recibida del LIDAR: {data}")
                pub.publish(data)
        except Exception as e:
            rospy.logerr(f"Error leyendo desde el puerto serial: {e}")

        rate.sleep()

    ser.close()

if __name__ == '__main__':
    try:
        lidar_publisher()
    except rospy.ROSInterruptException:
        pass

