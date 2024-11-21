#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import serial
import math

# Función para leer y procesar los datos del LiDAR
def parse_lidar_data(data):
    if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
        distance = data[2] | (data[3] << 8)  # Distancia en mm
        strength = data[4] | (data[5] << 8)  # Intensidad de señal
        checksum = sum(data[:8]) & 0xFF      # Verificación de checksum
        if checksum == data[8]:
            return distance / 1000.0, strength  # Devuelve distancia en metros
    return None, None

def lidar_publisher():
    # Inicializar nodo ROS
    rospy.init_node('lidar_publisher', anonymous=True)

    # Publicador para LiDAR
    lidar_pub = rospy.Publisher('scan', LaserScan, queue_size=10)

    # Configuración del puerto serial para el LiDAR
    LIDAR_PORT = "/dev/ttyUSB0"
    LIDAR_BAUDRATE = 115200
    ser = serial.Serial(LIDAR_PORT, LIDAR_BAUDRATE, timeout=1)

    # Crear el mensaje LaserScan para el LiDAR
    scan_msg = LaserScan()
    scan_msg.angle_min = -math.pi / 4
    scan_msg.angle_max = math.pi / 4
    scan_msg.angle_increment = math.pi / 180
    scan_msg.time_increment = 0.0
    scan_msg.scan_time = 0.1
    scan_msg.range_min = 0.0
    scan_msg.range_max = 10.0
    distances = []

    rate = rospy.Rate(100)  # Frecuencia de publicación para el LiDAR

    try:
        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                raw_data = ser.read(9)
                dist, strength = parse_lidar_data(list(raw_data))
                if dist is not None:
                    rospy.loginfo(f"Distancia: {dist:.3f} m")

                    # Llenar los datos en el mensaje LaserScan
                    scan_msg.header.stamp = rospy.Time.now()
                    scan_msg.header.frame_id = "laser_frame"
                    distances = [dist] * int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
                    scan_msg.ranges = distances

                    # Publicar los datos del LiDAR
                    lidar_pub.publish(scan_msg)
                else:
                    rospy.logwarn("Datos inválidos")

            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("Lectura interrumpida")
    finally:
        ser.close()

if __name__ == '__main__':
    try:
        lidar_publisher()
    except rospy.ROSInterruptException:
        pass

