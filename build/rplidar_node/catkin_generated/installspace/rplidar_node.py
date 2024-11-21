#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar

def lidar_scan():
    # Configura el nodo de ROS
    rospy.init_node('rplidar_node', anonymous=True)

    # Crea el objeto LIDAR para la comunicación con el puerto UART
    # Usamos '/dev/ttyAMA0' para UART en Raspberry Pi
    lidar = RPLidar('/dev/ttyAMA0')

    # Publicador de los datos de escaneo
    scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

    # Espera 2 segundos antes de empezar
    rospy.sleep(2)

    try:
        for scan in lidar.iter_scans():
            # Aquí puedes publicar los datos del escaneo en un tópico de ROS
            scan_msg = LaserScan()
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.header.frame_id = "laser_frame"
            # Agrega el resto de los datos de escaneo a `scan_msg`
            # Ejemplo: scan_msg.ranges = scan[1]
            scan_pub.publish(scan_msg)
    except rospy.ROSInterruptException:
        print("Nodo interrumpido.")
    finally:
        lidar.stop()
        lidar.disconnect()

if __name__ == '__main__':
    try:
        lidar_scan()
    except rospy.ROSInterruptException:
        pass

